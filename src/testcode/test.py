from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time

# 设置机械臂 IP 地址
ROBOT_IP = "192.168.192.7"

# 建立控制连接
rtde_c = RTDEControlInterface(ROBOT_IP)
# 建立接收连接
rtde_r = RTDEReceiveInterface(ROBOT_IP)

# 打印当前位置的 TCP 位姿（基于 Base 坐标系）
tcp_pose = rtde_r.getActualTCPPose()
print("当前 TCP 位姿 [x, y, z, rx, ry, rz]:")
print(tcp_pose)

# 移动到指定位置（示例为 joint space 运动）
print("开始移动机械臂...")
rtde_c.moveJ([0, -1.57, 0, -1.57, 0, 0])

# 等待运动完成一会
time.sleep(2)

# 再次读取 TCP 位姿
tcp_pose_after = rtde_r.getActualTCPPose()
print("移动后的 TCP 位姿:")
print(tcp_pose_after)

# 断开连接
rtde_c.disconnect()
