import numpy as np

# 加载 .npy 文件
data = np.load('robot_dataset/session_20250807_153113/robot_state.npy')

# 显示数据内容
print(data)

# 如果需要查看数据的形状，可以使用 .shape
print("Shape of the data:", data.shape)
