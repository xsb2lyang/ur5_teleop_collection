# data_collection/recorder.py (V3 - 仿照LIBERO格式)

import os
import time
import cv2
import numpy as np
import re

class DataRecorder:
    """
    负责采集数据并以仿LIBERO数据集的格式进行保存。
    """
    def __init__(self, ur5_controller, realsense_cam, zed_cam, base_path="./robot_dataset"):
        self.ur5e = ur5_controller
        self.d435_cam = realsense_cam
        self.zed_cam = zed_cam
        self.base_path = base_path
        os.makedirs(self.base_path, exist_ok=True)
        
        self.traj_path = ""
        self.action_buffer = []
        self.state_buffer = []
        self.is_recording = False
        self.task_instruction = "No instruction provided."
        self.task_folder_path = ""
        
        # --- 【新增】用于为每个任务下的样本进行独立编号 ---
        self.demo_counter = 0

    def set_task_instruction(self, instruction):
        """
        接收任务指令，并根据指令创建任务主文件夹。
        """
        self.task_instruction = instruction
        
        # --- 【核心修改】文件夹命名仿照LIBERO，用_demo结尾 ---
        sane_filename = re.sub(r'[^a-zA-Z0-9_]', '', instruction.replace(' ', '_'))
        if not sane_filename:
            sane_filename = f"task_{time.strftime('%Y%m%d_%H%M%S')}"
        
        # 加上 _demo 后缀
        self.task_folder_path = os.path.join(self.base_path, f"{sane_filename}_demo")
        os.makedirs(self.task_folder_path, exist_ok=True)
        print(f"所有 '{instruction}' 任务的样本将被保存在: {self.task_folder_path}")
        
        # 重置demo计数器，因为开始了新任务
        self.demo_counter = 0

    def start_recording_session(self):
        """
        在任务主文件夹下，创建一个新的 demo_X 子文件夹。
        """
        if self.is_recording:
            print("警告：已在录制中，无法开始新的会话。")
            return

        # --- 【核心修改】文件夹命名为 demo_X ---
        demo_name = f"demo_{self.demo_counter}"
        self.traj_path = os.path.join(self.task_folder_path, demo_name)
        os.makedirs(self.traj_path, exist_ok=True)
        
        # --- 【核心修改】创建 image0, image1, image3 文件夹 ---
        os.makedirs(os.path.join(self.traj_path, "image0"), exist_ok=True) # 第三方称视角 (zed_color)
        os.makedirs(os.path.join(self.traj_path, "image1"), exist_ok=True) # 第一人称视角 (d435_color)
        os.makedirs(os.path.join(self.traj_path, "image3"), exist_ok=True) # 深度图 (d435_depth)
        
        # 创建并写入 lang.txt 文件
        lang_filepath = os.path.join(self.traj_path, "lang.txt")
        try:
            with open(lang_filepath, 'w', encoding='utf-8') as f:
                f.write(self.task_instruction)
        except Exception as e:
            print(f"错误：无法写入 lang.txt 文件: {e}")
        
        self.action_buffer = []
        self.state_buffer = []
        self.is_recording = True
        self.demo_counter += 1 # 样本计数器加一
        print(f"录制开始... 数据将保存至: {os.path.join(os.path.basename(self.task_folder_path), demo_name)}")

    def stop_and_save_session(self):
        """
        停止录制，并将数据保存到对应的 demo_X 文件夹中。
        """
        if not self.is_recording:
            return
            
        self.is_recording = False
        if not self.state_buffer:
            print("没有采集到任何数据，已取消保存。")
            return

        # --- 【核心修改】
        state_npy_filename = os.path.join(self.traj_path, "state.npy")
        action_npy_filename = os.path.join(self.traj_path, "action.npy")
        
        np.save(state_npy_filename, np.array(self.state_buffer))
        np.save(action_npy_filename, np.array(self.action_buffer))
        
        print(f"录制结束，数据保存完毕！共 {len(self.state_buffer)} 帧。")
        print(f"  - 动作/状态/指令/图像 已全部保存至: {self.traj_path}")
        
        self.action_buffer = []
        self.state_buffer = []

    def record_step(self, action_command):
        """
        执行单次数据采集，并将图像按帧序号保存。
        """
        if not self.is_recording or action_command is None:
            return

        # 获取机器人状态和相机图像
        tcp_pose = self.ur5e.rtde_r.getActualTCPPose()
        gripper_state = 1 if self.ur5e.gripper_is_open else 0
        d435_color, d435_depth = self.d435_cam.get_frames()
        zed_color = self.zed_cam.get_frames()
        
        if tcp_pose is None or d435_color is None or zed_color is None:
            return
            
        # --- 【核心修改】图像按帧序号命名，并保存到对应的 imageX 文件夹 ---
        frame_index = len(self.action_buffer) # 使用缓冲区长度作为帧序号 (0, 1, 2, ...)
        
        # ZED (第三人称) -> image0
        cv2.imwrite(os.path.join(self.traj_path, "image0", f"{frame_index}.jpg"), zed_color)
        # D435 Color (第一人称) -> image1
        cv2.imwrite(os.path.join(self.traj_path, "image1", f"{frame_index}.jpg"), d435_color)
        # D435 Depth -> image3
        cv2.imwrite(os.path.join(self.traj_path, "image3", f"{frame_index}.png"), d435_depth)

        # 将数据存入缓冲区
        robot_state_vector = tcp_pose + [gripper_state]
        self.action_buffer.append(action_command)
        self.state_buffer.append(robot_state_vector)