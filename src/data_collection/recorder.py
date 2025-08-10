# data_collection/recorder.py

import os
import time
import cv2
import numpy as np
import re # 【新增】导入正则表达式库，用于清理文件名

class DataRecorder:
    """
    负责在内存中缓冲机器人和相机的数据，
    并在指令下一次性保存所有数据。
    【修改】现在会同时缓存动作和状态数据，并支持自定义文件夹结构。
    """
    def __init__(self, ur5_controller, realsense_cam, zed_cam, base_path="./robot_dataset"):
        self.ur5e = ur5_controller
        self.d435_cam = realsense_cam
        self.zed_cam = zed_cam
        self.base_path = base_path
        os.makedirs(self.base_path, exist_ok=True)
        
        self.session_path = ""
        self.action_buffer = []
        self.state_buffer = []
        self.is_recording = False
        self.task_instruction = "No instruction provided."
        
        # --- 【新增】用于存储任务主文件夹的路径 ---
        self.task_folder_path = ""

    def set_task_instruction(self, instruction):
        """
        接收任务指令，并根据指令创建主任务文件夹。
        """
        self.task_instruction = instruction
        
        # --- 【核心修改】根据指令创建主文件夹 ---
        # 1. 清理指令字符串，使其成为一个有效的文件名
        #    (替换空格为下划线，移除所有非字母、数字、下划线或连字符的字符)
        sane_filename = re.sub(r'[^a-zA-Z0-9_-]', '', instruction.replace(' ', '_'))
        if not sane_filename: # 如果清理后文件名为空，则使用时间戳
            sane_filename = f"task_{time.strftime('%Y%m%d_%H%M%S')}"

        # 2. 创建主任务文件夹路径
        self.task_folder_path = os.path.join(self.base_path, sane_filename)
        os.makedirs(self.task_folder_path, exist_ok=True)
        print(f"所有 '{instruction}' 任务的样本将被保存在: {self.task_folder_path}")

    def start_recording_session(self):
        """
        在主任务文件夹下，创建一个新的session子文件夹。
        """
        if self.is_recording:
            print("警告：已在录制中，无法开始新的会话。")
            return

        # 1. 【修改】在主任务文件夹下创建 session 子文件夹
        session_name = f"session_{time.strftime('%Y%m%d_%H%M%S')}"
        self.session_path = os.path.join(self.task_folder_path, session_name)
        os.makedirs(self.session_path, exist_ok=True)
        
        # 2. 创建图像子文件夹
        os.makedirs(os.path.join(self.session_path, "d435_color"), exist_ok=True)
        os.makedirs(os.path.join(self.session_path, "d435_depth"), exist_ok=True)
        os.makedirs(os.path.join(self.session_path, "zed_color"), exist_ok=True)
        
        # 3. 创建并写入 language.txt 文件
        lang_filepath = os.path.join(self.session_path, "language.txt")
        try:
            with open(lang_filepath, 'w', encoding='utf-8') as f:
                f.write(self.task_instruction)
        except Exception as e:
            print(f"错误：无法写入 language.txt 文件: {e}")
        
        # 4. 清空缓冲区并设置标志位
        self.action_buffer = []
        self.state_buffer = []
        self.is_recording = True
        print(f"录制开始... 数据将保存至: {os.path.join(os.path.basename(self.task_folder_path), session_name)}")

    def stop_and_save_session(self):
        """
        停止录制，并将缓存的所有数据一次性保存到文件中。
        """
        if not self.is_recording:
            print("警告：当前未在录制，无需保存。")
            return
            
        self.is_recording = False
        print("录制结束，正在处理并保存数据...")
        
        if not self.state_buffer:
            print("没有采集到任何数据，已取消保存。")
            return

        # 分别保存为两个 .npy 文件
        state_data_array = np.array(self.state_buffer)
        state_npy_filename = os.path.join(self.session_path, "robot_state.npy")
        np.save(state_npy_filename, state_data_array)
        
        action_data_array = np.array(self.action_buffer)
        action_npy_filename = os.path.join(self.session_path, "robot_action.npy")
        np.save(action_npy_filename, action_data_array)
        
        print(f"数据保存完毕！共 {len(self.state_buffer)} 帧。")
        print(f"  - 机器人状态数据已保存为: {state_npy_filename}")
        print(f"  - 机器人动作数据已保存为: {action_npy_filename}")
        print(f"  - 图像数据保存在: {self.session_path}")
        
        self.action_buffer = []
        self.state_buffer = []

    def record_step(self, action_command):
        """
        执行单次数据采集，将数据存入内存缓冲区。
        """
        if not self.is_recording or action_command is None:
            return

        timestamp = time.time()
        
        # 获取机器人状态数据 (State / Proprioception)
        tcp_pose = self.ur5e.rtde_r.getActualTCPPose()
        gripper_state = 1 if self.ur5e.gripper_is_open else 0
        
        # 获取相机图像
        d435_color, d435_depth = self.d435_cam.get_frames()
        zed_color = self.zed_cam.get_frames()
        
        if tcp_pose is None or d435_color is None or zed_color is None:
            print("警告: 数据采集失败，部分数据源返回None。跳过此帧。")
            return
            
        # 实时保存图像
        ts_str = f"{timestamp:.4f}"
        cv2.imwrite(os.path.join(self.session_path, "d435_color", f"{ts_str}.png"), d435_color)
        cv2.imwrite(os.path.join(self.session_path, "d435_depth", f"{ts_str}.png"), d435_depth)
        cv2.imwrite(os.path.join(self.session_path, "zed_color", f"{ts_str}.png"), zed_color)

        # 将两种数据分别存入对应的内存缓冲区
        robot_state_vector = tcp_pose + [gripper_state]
        self.action_buffer.append(action_command)
        self.state_buffer.append(robot_state_vector)