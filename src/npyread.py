import numpy as np

# 加载 .npy 文件
data = np.load('robot_dataset/pick_up_the_apple/session_20250811_092406/robot_action.npy')

# 显示数据内容
print(data)

# 如果需要查看数据的形状，可以使用 .shape
print("Shape of the data:", data.shape)
