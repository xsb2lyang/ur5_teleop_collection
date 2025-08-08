# main.py

# 导入标准库
import time       # 用于添加延时，控制循环频率
import threading  # 用于创建和管理多线程，实现并行处理
import os         # 在此项目中未直接使用，但通常用于处理文件路径

# 从项目内的其他模块导入我们自己编写的类
from ur5_control.ur5_controller import UR5Controller          # 导入UR5机器人控制器类
from xbox.xbox_control import XboxController                # 导入Xbox手柄控制器类
from camera_control.realsense_handler import RealSenseCamera    # 导入D435相机处理类
from camera_control.zed_handler import ZEDCamera              # 导入ZED2i相机处理类
from data_collection.recorder import DataRecorder           # 导入数据记录器类

# --- 全局常量 ---
# 定义数据采集的频率 (单位: Hz)，例如10代表每秒采集10次
RECORDING_FREQUENCY = 10 

class UR5JoyControl:
    """
    主控制类，负责初始化所有硬件模块，并编排各个线程的运行。
    它是整个应用程序的“指挥中心”。
    """
    def __init__(self):
        """
        构造函数，在创建UR5JoyControl对象时被调用。
        负责实例化所有硬件控制类和数据记录器。
        """
        print("正在初始化系统...")
        # --- 实例化核心对象 ---
        self.ur5e = UR5Controller(robot_ip="192.168.192.7")  # 初始化机器人控制器
        self.joy = XboxController()                          # 初始化Xbox手柄
        self.d435_cam = RealSenseCamera()                    # 初始化D435相机
        self.zed_cam = ZEDCamera()                           # 初始化ZED相机
        # 初始化数据记录器，并将机器人和相机的对象传递给它
        self.recorder = DataRecorder(self.ur5e, self.d435_cam, self.zed_cam)
        
        # --- 初始化状态变量 ---
        # motion_input用于存储从手柄读取的最新输入值，14个元素对应手柄的14个状态
        self.motion_input = (0,) * 14
        self.is_recording = False  # 一个布尔标志位，用于控制是否处于数据采集中
        # 以下两个变量用于检测手柄按键的“按下”瞬间（边沿检测）
        self.last_start_button_state = 0
        self.last_back_button_state = 0
        # 主循环运行标志位，当它变为False时，所有线程都将退出
        self.running = True
        
        print("系统初始化完成。")

    def read_joystick(self):
        """
        这个方法将在一个独立的线程中运行。
        它的任务是持续地从手柄读取数据，并更新到共享变量 self.motion_input 中。
        """
        while self.running:
            self.motion_input = self.joy.read()
            # 添加短暂延时，避免CPU占用过高，并设定手柄的读取频率
            time.sleep(0.02) # 约50Hz

    def control_robot(self):
        """
        这个方法也将在一个独立的线程中运行。
        它的任务是持续地将最新的手柄输入传递给机器人控制器，以驱动机器人运动。
        """
        try:
            while self.running:
                self.ur5e.move_robot(self.motion_input)
                # 添加短暂延时，与机器人的高频控制（如500Hz）解耦
                time.sleep(0.01)
        except KeyboardInterrupt:
            # 如果在终端中按下Ctrl+C，会触发此异常，调用stop方法来安全退出
            self.stop()

    def data_collection_manager(self):
        """
        独立的管理线程，专门负责控制数据采集的整个流程：开始、过程和结束。
        """
        while self.running:
            # 从共享变量 self.motion_input 中获取Start和Back按钮的当前状态
            # 索引12和13是在xbox_control.py中定义的
            current_start_button_state = self.motion_input[12]
            current_back_button_state = self.motion_input[13]
            
            # --- 检查是否按下Start键来开始录制 ---
            # 条件：当前按键为按下(1)，上次为弹起(0)，且当前未在录制状态
            if current_start_button_state == 1 and self.last_start_button_state == 0 and not self.is_recording:
                self.is_recording = True  # 更新录制状态
                self.recorder.start_recording_session() # 通知记录器开始一个新会话
            
            # --- 检查是否按下Back键来结束录制 ---
            # 条件：当前按键为按下(1)，上次为弹起(0)，且当前正在录制状态
            if current_back_button_state == 1 and self.last_back_button_state == 0 and self.is_recording:
                self.is_recording = False # 更新录制状态
                self.recorder.stop_and_save_session() # 通知记录器停止并保存数据
            
            # 更新上一帧的按钮状态，为下一次检测做准备
            self.last_start_button_state = current_start_button_state
            self.last_back_button_state = current_back_button_state

            # --- 如果正在录制，则按固定频率采集数据 ---
            if self.is_recording:
                self.recorder.record_step() # 调用记录器的单步采集方法

            # --- 动态控制此管理线程的循环频率 ---
            # 如果正在录制，延时时间由RECORDING_FREQUENCY决定
            # 如果没有录制，则以较低频率（10Hz）检查按键即可，以降低CPU占用
            sleep_interval = 1.0 / RECORDING_FREQUENCY if self.is_recording else 0.1
            time.sleep(sleep_interval)

    def start(self):
        """
        启动整个应用程序。
        负责创建并启动所有后台线程。
        """
        # 创建线程对象，将类的方法作为目标(target)，并设置为守护线程(daemon=True)
        # 守护线程意味着当主线程退出时，这些子线程会自动被销毁
        joystick_thread = threading.Thread(target=self.read_joystick, daemon=True)
        robot_thread = threading.Thread(target=self.control_robot, daemon=True)
        collection_manager_thread = threading.Thread(target=self.data_collection_manager, daemon=True)
        
        print("正在启动所有控制线程...")
        # 启动所有线程，它们会开始并行运行
        joystick_thread.start()
        robot_thread.start()
        collection_manager_thread.start()
        
        # 打印操作指南给用户
        print("\n系统已启动！现在您可以开始操作了。")
        print("  - 按下手柄 'Start' 键开始录制。")
        print("  - 在录制过程中自由操作机械臂。")
        print("  - 按下手柄 'Back' 键结束录制并保存数据。")
        print("  - 在终端中按 Ctrl+C 来安全退出程序。")
        
        try:
            # 这个循环让主线程保持活动状态，否则程序会立即退出
            # 主线程本身不需要做太多事，只需等待退出信号
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            # 当用户在终端按下 Ctrl+C 时，捕获此异常
            self.stop()
            
    def stop(self):
        """
        安全地停止所有硬件和线程。
        """
        print("\n程序中断！正在安全停止...")
        if self.running:
            self.running = False # 将运行标志位设为False，通知所有子线程退出循环
            
            # 安全措施：如果退出时仍在录制，自动保存已采集的数据
            if self.is_recording:
                print("程序退出时仍在录制，将自动保存数据...")
                self.recorder.stop_and_save_session()

            # --- 依次关闭和释放硬件资源 ---
            self.ur5e.rtde_c.servoStop() # 立即停止机器人伺服运动
            self.ur5e.rtde_c.stopScript() # 停止在机器人控制器上运行的RTDE脚本
            self.d435_cam.release()       # 释放D435相机
            self.zed_cam.release()        # 释放ZED相机
            
            print("所有硬件已安全停止。程序退出。")

# --- Python程序的标准入口点 ---
if __name__ == "__main__":
    # 当直接运行此脚本时（而不是作为模块导入时），以下代码将被执行
    controller = UR5JoyControl() # 创建主控制类的实例
    controller.start()           # 调用start方法，启动整个应用