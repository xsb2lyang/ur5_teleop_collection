import sys
import time
import keyboard
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_io import RTDEIOInterface
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import socket # 用于检查连接

def control_gripper_with_keyboard(robot_ip="localhost"):
    """
    使用键盘的 'x' 和 'y' 键来控制 Robotiq 2F-85 夹爪的开合。
    
    参数:
        robot_ip (str): UR 机器人的 IP 地址。
    """
    # 假设 Robotiq 夹爪通过工具端的数字输出引脚 D_OUT0 (用于控制) 和 D_OUT1 (用于状态指示或辅助控制) 连接。
    # 具体的引脚可能因集成商而异，这里使用D_OUT0和D_OUT1作为示例。
    
    rtde_c = None
    rtde_io = None
    
    try:
        # 1. 尝试连接到 UR 机器人 (使用 RTDEControlInterface 进行基础连接)
        rtde_c = RTDEControl(robot_ip)
        print(f"成功连接到 UR 机器人，IP: {robot_ip}")

        # 2. 初始化 RTDE IO 接口 (专门用于I/O控制)
        rtde_io = RTDEIOInterface(robot_ip)
        print("RTDE IO 接口已初始化。")

        # 3. 激活夹爪 (通常只需要在程序启动时激活一次)
        # 根据Robotiq 2F-85的URScript命令，激活通常是通过设置数字输出来完成的
        print("正在激活夹爪...")
        rtde_io.setToolDigitalOut(1, False) # 设置引脚1为False
        rtde_io.setToolDigitalOut(0, True)  # 设置引脚0为True
        time.sleep(1) # 等待夹爪响应激活指令
        print("夹爪激活指令已发送。")
        
        # 4. 主循环，监听键盘输入
        print("\n--- 键盘控制夹爪 ---")
        print("按下 'x' 键来打开夹爪。")
        print("按下 'y' 键来关闭夹爪。")
        print("按下 'Esc' 键来退出程序。")
        
        # 用于防止按键长按的标志
        last_x_state = False
        last_y_state = False

        while True:
            current_x_state = keyboard.is_pressed('x')
            current_y_state = keyboard.is_pressed('y')
            
            # 检测 'x' 键按下 (边缘触发)
            if current_x_state and not last_x_state:
                # 打开夹爪
                print("指令: 打开夹爪")
                rtde_io.setToolDigitalOut(1, False)
                rtde_io.setToolDigitalOut(0, True)
                
            # 检测 'y' 键按下 (边缘触发)
            if current_y_state and not last_y_state:
                # 关闭夹爪
                print("指令: 关闭夹爪")
                rtde_io.setToolDigitalOut(1, False)
                rtde_io.setToolDigitalOut(0, False)
                
            last_x_state = current_x_state
            last_y_state = current_y_state

            # 监听 'Esc' 键，按下则退出程序
            if keyboard.is_pressed('esc'):
                print("正在退出程序...")
                break
                
            time.sleep(0.01) # 降低 CPU 占用

    except socket.gaierror:
        print(f"错误: 无法解析 IP 地址 '{robot_ip}'。请检查IP地址是否正确。")
    except ConnectionRefusedError:
        print(f"错误: 连接被拒绝。请确保 UR 机器人控制柜已打开且 IP '{robot_ip}' 正确。")
    except Exception as e:
        print(f"发生错误: {e}")

    finally:
        # 确保在程序退出时断开连接
        if rtde_c:
            rtde_c.disconnect()
        if rtde_io:
            rtde_io.disconnect()
        print("已断开与 UR 机器人的连接。")

if __name__ == "__main__":
    # 在这里填入你的 UR 机器人 IP 地址
    ur_robot_ip = "192.168.192.7"  # 替换为您的UR机器人实际IP
    control_gripper_with_keyboard(robot_ip=ur_robot_ip)