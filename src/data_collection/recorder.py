# data_collection/recorder.py

import os
import time
import cv2
import numpy as np

class DataRecorder:
    """
    负责在内存中缓冲机器人和相机的数据，
    并在指令下一次性保存所有数据。
    """
    def __init__(self, ur5_controller, realsense_cam, zed_cam, base_path="./robot_dataset"):
        self.ur5e = ur5_controller
        self.d435_cam = realsense_cam
        self.zed_cam = zed_cam
        self.base_path = base_path
        os.makedirs(self.base_path, exist_ok=True)
        
        # 用于缓存数据的变量
        self.session_path = ""
        self.metadata_buffer = [] # 在内存中缓存机器人和夹爪数据
        self.is_recording = False

    def start_recording_session(self):
        """
        准备一个新的采集会话，创建文件夹结构，并清空缓冲区。
        """
        if self.is_recording:
            print("警告：已在录制中，无法开始新的会话。")
            return

        # 1. 创建会话主文件夹
        session_name = f"session_{time.strftime('%Y%m%d_%H%M%S')}"
        self.session_path = os.path.join(self.base_path, session_name)
        os.makedirs(self.session_path, exist_ok=True)
        
        # 2. 创建图像子文件夹
        os.makedirs(os.path.join(self.session_path, "d435_color"), exist_ok=True)
        os.makedirs(os.path.join(self.session_path, "d435_depth"), exist_ok=True)
        os.makedirs(os.path.join(self.session_path, "zed_color"), exist_ok=True)
        
        # 3. 清空缓冲区并设置标志位
        self.metadata_buffer = []
        self.is_recording = True
        print(f"录制开始... 数据将保存至: {session_name}")

    def stop_and_save_session(self):
        """
        停止录制，并将缓存的所有数据一次性保存到文件中。
        """
        if not self.is_recording:
            print("警告：当前未在录制，无需保存。")
            return
            
        self.is_recording = False
        print("录制结束，正在处理并保存数据...")
        
        if not self.metadata_buffer:
            print("没有采集到任何数据，已取消保存。")
            return

        # 1. 将机器人元数据列表转换为Numpy数组
        # 数组维度为 (N, 7)，N是采集的帧数
        robot_data_array = np.array(self.metadata_buffer)
        
        # 2. 保存为 .npy 文件
        npy_filename = os.path.join(self.session_path, "robot_state.npy")
        np.save(npy_filename, robot_data_array)
        
        print(f"数据保存完毕！共 {len(self.metadata_buffer)} 帧。")
        print(f"机器人状态数据已保存为: {npy_filename}")
        print(f"图像数据保存在: {self.session_path}")
        
        # 清空缓冲区以备下次使用
        self.metadata_buffer = []

    def record_step(self):
        """
        执行单次数据采集，将数据存入内存缓冲区。
        图像仍然实时保存以避免占用过多内存，但元数据被缓存。
        """
        if not self.is_recording:
            return

        timestamp = time.time()
        
        # 1. 获取机器人和夹爪数据
        tcp_pose = self.ur5e.rtde_r.getActualTCPPose()
        # 将布尔值转换为0或1
        gripper_state = 1 if self.ur5e.gripper_is_open else 0
        
        # 2. 获取相机图像
        d435_color, d435_depth = self.d435_cam.get_frames()
        zed_color = self.zed_cam.get_frames()
        
        # 3. 检查所有数据是否有效
        if tcp_pose is None or d435_color is None or zed_color is None:
            print("警告: 数据采集失败，部分数据源返回None。跳过此帧。")
            return
            
        # 4. 实时保存图像
        ts_str = f"{timestamp:.4f}"
        cv2.imwrite(os.path.join(self.session_path, "d435_color", f"{ts_str}.png"), d435_color)
        cv2.imwrite(os.path.join(self.session_path, "d435_depth", f"{ts_str}.png"), d435_depth)
        cv2.imwrite(os.path.join(self.session_path, "zed_color", f"{ts_str}.png"), zed_color)

        # 5. 将机器人数据存入内存缓冲区
        # 组织成一个7维向量
        robot_state_vector = [
            tcp_pose[0], tcp_pose[1], tcp_pose[2],
            tcp_pose[3], tcp_pose[4], tcp_pose[5],
            gripper_state
        ]
        self.metadata_buffer.append(robot_state_vector)