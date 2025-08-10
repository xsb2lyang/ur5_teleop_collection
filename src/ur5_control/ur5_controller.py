# ur5_control/ur5_controller.py

# 从ur_rtde库中导入RTDE的控制、接收和IO接口
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
# 导入用于IO控制的专门接口
from rtde_io import RTDEIOInterface
# 导入os, psutil, sys库，用于处理操作系统相关的功能，如设置进程优先级
import os
import psutil
import sys
# 导入math库，用于数学计算
import math
# 导入threading库，用于将耗时操作（如夹爪IO）放入子线程，防止阻塞主运动控制循环
import threading

# 定义机器人末端执行器（TCP）的运动空间限制 (单位: 米, 弧度)
# 用于防止机器人移动到危险或不期望的位置
MOTION_LIMITS = {
                "xmin": -0.3, "xmax": 0.8,      # X轴的最小/最大值
                "ymin": -0.3, "ymax": 0.8,      # Y轴的最小/最大值
                "zmin": -0.5, "zmax": 1.0,      # Z轴的最小/最大值
                "rzmin": -2 * math.pi, "rzmax": 2 * math.pi  # Z轴旋转的最小/最大值 (通常只限制Z轴旋转)
                }

# 定义固定的运动速度和遥感死区
FIXED_TRANSLATION_SPEED = 4.8  # 平移速度 (米/秒的一个比例系数)
FIXED_ROTATION_SPEED = 18    # 旋转速度 (弧度/秒的一个比例系数)
DEADZONE = 0.2                 # 摇杆死区，小于此阈值的摇杆输入将被忽略，以防止漂移

class UR5Controller:
    """
    一个使用RTDE接口控制UR5机器人的类。
    该控制器建立RTDE通信，应用实时优先级，并允许运动和IO控制。
    """
    def __init__(self, robot_ip="localhost", rtde_frequency=500.0, ur_cap_port=50002):
        """
        初始化UR5控制器，建立运动和IO两个独立的RTDE连接。

        :param robot_ip: 机器人控制器（“示教器”）的IP地址
        :param rtde_frequency: RTDE通信频率 (e系列机械臂推荐500Hz, CB3系列为125Hz)
        :param ur_cap_port: URCap通信端口，默认为50002
        """
        # --- 连接参数 ---
        self.robot_ip = robot_ip
        self.rtde_frequency = rtde_frequency
        self.ur_cap_port = ur_cap_port
        self.dt = 1.0 / rtde_frequency  # 计算每个控制周期的时长（秒），例如500Hz下dt=0.002秒

        # --- 运动控制参数 ---
        self.vel = 0.01  # moveL和servoL指令中的速度参数
        self.acc = 0.01  # moveL和servoL指令中的加速度参数
        self.lookahead_time = 0.1   # servoL的前瞻时间，用于平滑轨迹
        self.gain = 600             # servoL的比例增益，用于修正轨迹
        self.time_counter = 0.0     # 一个简单的时间计数器
        # RTDE控制标志位，FLAG_UPLOAD_SCRIPT表示会自动上传并运行一个控制脚本到机器人控制器
        self.flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT

        # --- 实时优先级设置 ---
        self.rt_receive_priority = 90  # 接收线程的优先级
        self.rt_control_priority = 85  # 控制线程的优先级

        # --- 初始化RTDE接口 ---
        # 初始化数据接收接口，用于获取机器人状态（如TCP位姿）
        self.rtde_r = RTDEReceive(self.robot_ip, self.rtde_frequency, [], True, False, self.rt_receive_priority)
        # 初始化数据控制接口，用于发送运动指令
        self.rtde_c = RTDEControl(self.robot_ip, self.rtde_frequency, self.flags, self.ur_cap_port, self.rt_control_priority)
        # 初始化独立的IO控制接口，专门用于控制数字/模拟IO（例如夹爪）
        self.rtde_io = RTDEIOInterface(self.robot_ip)
        print("RTDE运动和IO接口均已成功初始化。")
        
        # --- 状态变量 ---
        self.paused = False         # 暂停标志位
        self.last_a_state = 0       # 上一帧A键的状态，用于检测按钮“按下”的瞬间
        self.last_b_state = 0       # 上一帧B键的状态
        self.last_x_state = 0       # 上一帧X键的状态
        self.last_y_state = 0       # 上一帧Y键的状态
        self.gripper_is_open = True # 夹爪状态的标志位，True为张开，False为闭合

        # --- 初始化操作 ---
        # 根据操作系统设置当前进程的优先级为实时，以保证控制稳定性
        self.set_real_time_priority()
        # 移动机器人到一个初始的安全位置
        self.init_position()
        
    def set_real_time_priority(self):
        """
        根据操作系统设置进程为实时优先级。
        """
        os_used = sys.platform  # 获取当前操作系统平台 ('linux', 'win32', 'darwin')
        process = psutil.Process(os.getpid())  # 获取当前进程对象

        if os_used == "win32":  # 如果是Windows系统
            process.nice(psutil.REALTIME_PRIORITY_CLASS)  # 设置为实时优先级
        elif os_used == "linux":  # 如果是Linux系统
            rt_app_priority = 80  # 定义一个实时优先级数值（1-99，越高优先级越高）
            param = os.sched_param(rt_app_priority)  # 创建一个调度参数对象
            try:
                # 尝试设置调度策略为SCHED_FIFO（先入先出），这是一种实时策略
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
                print("进程实时优先级已设置为: %u" % rt_app_priority)
            except OSError:  # 如果设置失败（通常是因为权限不足）
                print("警告: 设置实时进程调度失败。请尝试使用 'sudo' 运行。")

    def init_position(self):
        """
        移动机器人到一个初始的安全位置。
        此处的逻辑是让机器人以0速度moveL到它当前所在的位置，
        目的是为了确保控制开始时机器人处于一个稳定和已知的状态。
        """
        self.actual_tcp_pose = self.rtde_r.getActualTCPPose()  # 获取机器人当前的TCP位姿
        motion_input = [0]*12  # 创建一个全零的输入，表示没有运动指令
        # 计算目标位姿（因为输入为0，所以目标位姿等于当前位姿）
        init_pose = self.getTarget(self.actual_tcp_pose, self.time_counter, motion_input)
        # 使用moveL指令移动到该位置，这是一个阻塞指令，完成后才会继续
        self.rtde_c.moveL(init_pose, self.vel, self.acc)

    # ur5_control/ur5_controller.py

    # ... (文件其他部分不变) ...
    def control_gripper(self, open_gripper):
        """
        使用专门的RTDE IO接口来控制Robotiq 2F-85夹爪。
        这个调用是异步的，不会中断运动控制。
        """
        try:
            if open_gripper:  # 如果指令是“打开夹爪”
                print("指令: 打开夹爪")
                self.rtde_io.setToolDigitalOut(1, False)
                self.rtde_io.setToolDigitalOut(0, True)
            else:  # 如果指令是“闭合夹爪”
                print("指令: 闭合夹爪")
                self.rtde_io.setToolDigitalOut(1, False)
                self.rtde_io.setToolDigitalOut(0, False)
            
            # --- 【核心修正】将状态更新移到if/else之外 ---
            # 这样无论 open_gripper 是 True 还是 False，状态都会被正确更新
            self.gripper_is_open = open_gripper
            
        except Exception as e:
            print(f"错误: 控制夹爪时发生错误: {e}")
   

    def getTarget(self, pose, timestep, motion_input):
        """
        根据手柄输入计算下一个目标位姿。
        """
        target = pose[:]  # 复制当前位姿作为计算基础

        # --- 计算平移增量 ---
        dx, dy, dz = 0, 0, 0
        # 左摇杆上下 -> X轴平移 (前后)
        if abs(motion_input[1]) > DEADZONE:
            dx = -math.copysign(FIXED_TRANSLATION_SPEED, motion_input[1]) * timestep
        # 左摇杆左右 -> Y轴平移 (左右)
        if abs(motion_input[0]) > DEADZONE:
            dy = -math.copysign(FIXED_TRANSLATION_SPEED, motion_input[0]) * timestep
        # 右扳机 -> Z轴正向平移 (上)
        if motion_input[2] > DEADZONE:
            dz = FIXED_TRANSLATION_SPEED * timestep
        # 左扳机 -> Z轴负向平移 (下)
        elif motion_input[3] > DEADZONE:
            dz = -FIXED_TRANSLATION_SPEED * timestep

        # --- 计算旋转增量 (轴角式) ---
        drx, dry, drz = 0, 0, 0
        # 左/右肩键 -> 绕X轴旋转
        if motion_input[6] > 0: # Left Bumper
            drx = -FIXED_ROTATION_SPEED * timestep
        elif motion_input[7] > 0: # Right Bumper
            drx = FIXED_ROTATION_SPEED * timestep
        # 右摇杆上下 -> 绕Y轴旋转
        if abs(motion_input[4]) > DEADZONE:
            dry = math.copysign(FIXED_ROTATION_SPEED, motion_input[4]) * timestep
        # 右摇杆左右 -> 绕Z轴旋转
        if abs(motion_input[5]) > DEADZONE:
            drz = math.copysign(FIXED_ROTATION_SPEED, motion_input[5]) * timestep
        
        # 将计算出的增量应用到目标位姿上
        target[0] += dx; target[1] += dy; target[2] += dz
        target[3] += drx; target[4] += dry; target[5] += drz

        # --- 应用运动范围限制，防止机器人超出安全区域 ---
        target[0] = max(MOTION_LIMITS["xmin"], min(target[0], MOTION_LIMITS["xmax"]))
        target[1] = max(MOTION_LIMITS["ymin"], min(target[1], MOTION_LIMITS["ymax"]))
        target[2] = max(MOTION_LIMITS["zmin"], min(target[2], MOTION_LIMITS["zmax"]))
        target[5] = max(MOTION_LIMITS["rzmin"], min(target[5], MOTION_LIMITS["rzmax"])) # 通常只限制工具Z轴旋转

        return target
    
    def move_robot(self, xbox_input):
        """
        根据Xbox手柄输入移动机器人，并处理暂停/恢复和夹爪控制。
        这个方法会在一个高频循环中被持续调用。
        """
        #  --- 【新增】初始化一个变量来存储本轮的动作指令 ---
        action_command = None
        # --- 处理功能按钮 ---
        # A键暂停逻辑: 检测A键是否从“未按下”变为“按下”
        current_a_state = xbox_input[8]
        if current_a_state == 1 and self.last_a_state == 0:
            self.paused = True
            print("机器人已暂停。")
        self.last_a_state = current_a_state # 更新上一帧状态

        # B键恢复逻辑: 检测B键是否从“未按下”变为“按下”
        current_b_state = xbox_input[9]
        if current_b_state == 1 and self.last_b_state == 0:
            self.paused = False
            print("机器人已恢复。")
        self.last_b_state = current_b_state # 更新上一帧状态
        
        # X/Y键夹爪控制
        current_x_state = xbox_input[10]
        current_y_state = xbox_input[11]

        # Y键打开夹爪: 检测Y键是否从“未按下”变为“按下”
        if current_y_state == 1 and self.last_y_state == 0:
            # 将夹爪控制放入一个独立的守护线程中执行
            # 这样可以避免IO操作的潜在延迟阻塞主运动控制循环
            gripper_thread = threading.Thread(target=self.control_gripper, args=(True,))
            gripper_thread.daemon = True
            gripper_thread.start()

        # X键闭合夹爪: 检测X键是否从“未按下”变为“按下”
        if current_x_state == 1 and self.last_x_state == 0:
            gripper_thread = threading.Thread(target=self.control_gripper, args=(False,))
            gripper_thread.daemon = True
            gripper_thread.start()

        self.last_x_state = current_x_state # 更新上一帧状态
        self.last_y_state = current_y_state # 更新上一帧状态

        # --- 主运动循环 ---
        if not self.paused:
            if self.rtde_c.isProgramRunning():
                self.actual_tcp_pose = self.rtde_r.getActualTCPPose()
                t_start = self.rtde_c.initPeriod()
                
                # --- 【核心修改点 1】---
                # getTarget 方法现在计算的是目标位姿，我们需要的是相对变化。
                # 我们可以通过计算 servo_target 和 actual_tcp_pose 的差值来得到动作。
                servo_target = self.getTarget(self.actual_tcp_pose, self.dt, xbox_input)
                
                # 【新增】计算相对位姿变化 (delta_pose)
                delta_pose = [servo_target[i] - self.actual_tcp_pose[i] for i in range(6)]
                
                # 【新增】获取当前夹爪的“动作”意图 (我们想让它开还是关)
                # 因为夹爪控制是事件驱动的，我们需要一个状态来表示“意图”
                # 一个简单的方法是直接使用 self.gripper_is_open 的当前状态
                # 更好的方法是在 control_gripper 中设置一个“目标状态”变量
                gripper_action = 1.0 if self.gripper_is_open else 0.0 # 保持和您现有格式一致 (1=开, 0=闭)
                
                # 【新增】将计算出的指令拼接成一个7维向量
                action_command = delta_pose + [gripper_action]
                
                # 发送servoL指令 (这部分逻辑不变)
                self.rtde_c.servoL(servo_target, self.vel, self.acc, self.dt, self.lookahead_time, self.gain)
                
                self.rtde_c.waitPeriod(t_start)
                self.time_counter += self.dt
            else:
                print("错误: RTDE控制程序已停止运行！无法发送servoL指令。")
        else:
            self.rtde_c.servoStop()

        # --- 【新增】返回计算出的动作指令 ---
        return action_command