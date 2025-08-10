# main.py

import time
import threading
import os

from ur5_control.ur5_controller import UR5Controller
from xbox.xbox_control import XboxController
from camera_control.realsense_handler import RealSenseCamera
from camera_control.zed_handler import ZEDCamera
from data_collection.recorder import DataRecorder

class UR5JoyControl:
    """
    主控制类，负责初始化所有硬件模块，并编排各个线程的运行。
    """
    def __init__(self):
        print("正在初始化系统...")
        self.ur5e = UR5Controller(robot_ip="192.168.192.7")
        self.joy = XboxController()
        self.d435_cam = RealSenseCamera()
        self.zed_cam = ZEDCamera()
        self.recorder = DataRecorder(self.ur5e, self.d435_cam, self.zed_cam)
        
        # --- 【核心修改】获取语言指令和采集频率 ---
        # 1. 获取语言指令
        self.task_instruction = input(">>> (1/2) 请输入当前任务的语言指令，然后按 Enter: ")
        self.recorder.set_task_instruction(self.task_instruction)
        print(f"任务指令已设置: '{self.task_instruction}'")
        
        # 2. 获取并设置采集频率
        while True:
            try:
                freq_str = input(">>> (2/2) 请输入数据采集频率 (Hz, 推荐 10-20)，然后按 Enter: ")
                self.recording_frequency = int(freq_str)
                if self.recording_frequency > 0:
                    break
                else:
                    print("错误：频率必须是正整数。")
            except ValueError:
                print("错误：请输入一个有效的整数。")

        # 计算控制循环的延时间隔
        self.control_loop_interval = 1.0 / self.recording_frequency
        print(f"采集频率已设置为: {self.recording_frequency} Hz (每帧间隔约 {self.control_loop_interval:.3f} 秒)")
        
        # --- 初始化状态变量 ---
        self.motion_input = (0,) * 14
        self.is_recording = False
        self.last_start_button_state = 0
        self.last_back_button_state = 0
        self.running = True
        
        print("系统初始化完成。")

    def read_joystick(self):
        """
        此线程持续从手柄读取数据。
        """
        while self.running:
            self.motion_input = self.joy.read()
            time.sleep(0.02)

    def control_robot(self):
        """
        此线程现在以用户自定义的频率运行，并负责触发数据记录。
        """
        try:
            while self.running:
                # 1. 从控制器获取计算出的动作指令
                action_command = self.ur5e.move_robot(self.motion_input)
                
                # 2. 如果正在录制，则将该指令传递给记录器
                if self.is_recording:
                    self.recorder.record_step(action_command)
                    
                # 3. 使用计算出的延时间隔来控制循环频率
                time.sleep(self.control_loop_interval)
                
        except KeyboardInterrupt:
            self.stop()

    def data_collection_manager(self):
        """
        此线程只负责监听开始/停止信号。
        """
        while self.running:
            current_start_button_state = self.motion_input[12]
            current_back_button_state = self.motion_input[13]
            
            # 检查是否按下Start键来开始录制
            if current_start_button_state == 1 and self.last_start_button_state == 0 and not self.is_recording:
                self.is_recording = True
                self.recorder.start_recording_session()
            
            # 检查是否按下Back键来结束录制
            if current_back_button_state == 1 and self.last_back_button_state == 0 and self.is_recording:
                self.is_recording = False
                self.recorder.stop_and_save_session()
            
            self.last_start_button_state = current_start_button_state
            self.last_back_button_state = current_back_button_state

            # 以较低频率检查按键即可
            time.sleep(0.1)

    def start(self):
        joystick_thread = threading.Thread(target=self.read_joystick, daemon=True)
        robot_thread = threading.Thread(target=self.control_robot, daemon=True)
        collection_manager_thread = threading.Thread(target=self.data_collection_manager, daemon=True)
        
        print("正在启动所有控制线程...")
        joystick_thread.start()
        robot_thread.start()
        collection_manager_thread.start()
        
        print("\n系统已启动！现在您可以开始操作了。")
        print(f"  - 当前任务指令: '{self.task_instruction}'")
        print(f"  - 采集频率: {self.recording_frequency} Hz")
        print("  - 按下手柄 'Start' 键开始录制。")
        print("  - 在录制过程中自由操作机械臂。")
        print("  - 按下手柄 'Back' 键结束录制并保存数据。")
        print("  - 在终端中按 Ctrl+C 来安全退出程序。")
        
        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()
            
    def stop(self):
        print("\n程序中断！正在安全停止...")
        if self.running:
            self.running = False
            
            if self.is_recording:
                print("程序退出时仍在录制，将自动保存数据...")
                self.recorder.stop_and_save_session()

            self.ur5e.rtde_c.servoStop()
            self.ur5e.rtde_c.stopScript()
            self.d435_cam.release()
            self.zed_cam.release()
            
            print("所有硬件已安全停止。程序退出。")

if __name__ == "__main__":
    controller = UR5JoyControl()
    controller.start()