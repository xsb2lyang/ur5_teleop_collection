# UR5 机器人遥操作与多模态数据采集平台

本项目提供了一套完整的解决方案，用于通过Xbox手柄实时遥操作UR5/UR5e机械臂，并同步采集来自机械臂本体、Robotiq二指夹爪以及两个异构相机（Intel RealSense D435, Stereolabs ZED 2i）的多模态数据。

该平台专为机器人学习领域的数据集采集而设计，支持“开始-停止”式的连续数据录制，并将数据以对机器学习友好的格式进行保存。

## ✨ 主要功能

- **实时遥操作**：通过Xbox手柄对UR5机械臂进行低延迟、直观的6自由度运动控制。
- **多模态数据同步**：在遥操作的同时，同步记录以下数据流：
    - **机械臂状态**：TCP（工具中心点）的6D位姿（X, Y, Z, Rx, Ry, Rz）。
    - **夹爪状态**：Robotiq二指夹爪的开合状态（1 for open, 0 for closed）。
    - **D435相机**：RGB彩色图像和16位原始深度图像。
    - **ZED 2i相机**：RGB彩色图像。
- **连续数据采集**：
    - 使用手柄 `Start` 键开始一个采集会话。
    - 在会话期间，以预设频率（默认为10Hz）连续记录数据。
    - 使用手柄 `Back` 键结束会话，并将所有数据一次性保存。
- **友好的数据格式**：
    - 机械臂和夹爪状态被整合为一个 `N x 7` 的NumPy数组，并保存为 `robot_state.npy` 文件。
    - 所有图像均以时间戳命名，保存为 `.png` 格式，易于和 `.npy` 文件中的数据对齐。
- **模块化与多线程架构**：代码结构清晰，采用多线程分离IO、运动控制和数据采集，确保系统性能和稳定性。

## 🛠️ 环境依赖

### 硬件 (Hardware)
- UR5 或 UR5e 机械臂
- Robotiq 2F-85 二指夹爪
- 标准 Xbox 控制器
- Intel RealSense D435 深度相机
- Stereolabs ZED 2i 深度相机
- 一台连接以上所有设备的Linux工作站（本人采用Ubuntu 22.04）

### 软件 (Software)
- Python (>= 3.8)
- **ZED SDK**: 请从 [Stereolabs官网](https://www.stereolabs.com/developers/release/) 下载并安装。
- **Python 依赖库**:
  创建一个名为 `requirements.txt` 的文件，内容如下，然后通过 `pip install -r requirements.txt` 安装。

  ```txt
  ur-rtde
  inputs
  pyrealsense2
  pyzed
  numpy
  opencv-python
  psutil
  ```

## 🚀 安装与配置

1.  **克隆项目**
    ```bash
    git clone [您的项目仓库地址]
    cd [项目文件夹]
    ```

2.  **安装ZED SDK**
    请务必先安装与您CUDA版本匹配的ZED SDK，并根据指引完成Python API的安装。

3.  **配置机器人IP地址 (重要!)**
    打开 `src/ur5_control/ur5_controller.py` 文件，修改 `UR5Controller` 初始化时的 `robot_ip` 参数为您机械臂的实际IP地址。
    ```python
    # line 27 in ur5_controller.py
    def __init__(self, robot_ip="192.168.192.7", ...): # <-- 修改此处的IP地址
    ```

4.  **安装Python依赖**
    ```bash
    pip install -r requirements.txt
    ```

## ▶️ 运行程序

**启动程序**: 确保所有硬件（机械臂、相机、手柄）已连接并上电。进入 `src` 目录，在终端中运行主程序，由于程序需要设置实时系统优先级并访问底层硬件，推荐使用 `sudo` 来运行：

```bash
cd src
# 这个命令会使用sudo来获取权限，同时确保运行的是当前激活的Python虚拟环境
sudo $(which python) main.py
```

程序启动后，会初始化所有设备，并等待您的操作指令。

## 📋 采集流程
---
按照以下步骤来完成一个完整的任务样本采集：

1. **任务准备**: 程序启动后，可以通过手柄将机械臂移动到一个合适的起始位置，准备开始执行操作任务。
    
2. **开始录制**: 当一切准备就绪，**按一下手柄上的 `Start` 键**。终端会提示“录制开始...”，程序会以约10Hz的频率在后台记录所有数据流。
    
3. **执行任务**: 在录制状态下，使用手柄遥操作机械臂和夹爪，完成一个完整的操作任务（例如：从A点抓取一个物体并移动到B点）。
    
4. **结束并保存**: 任务完成后，**按一下手柄上的 `Back` 键**。录制会立即停止，程序会自动处理并保存这段时间采集到的所有数据。终端会显示“数据保存完毕！”的消息。
    
5. **重复或退出**: 您可以重复第2步至第5步来采集多个不同的任务样本。全部任务结束后，在终端按 `Ctrl+C` 即可安全退出程序。

## 🎮 操作指南
![[xbox.jpg]]

| 控制功能 | 按键 / 摇杆 |
| :--- | :--- |
| **运动控制** | |
| 前后 / 左右移动 | `左摇杆` |
| 上下移动 | `右扳机 (RT)` / `左扳机 (LT)` |
| 工具姿态旋转 (Roll/Pitch) | `右摇杆` |
| 工具姿态旋转 (Yaw) | `右肩键 (RB)` / `左肩键 (LB)` |
| **夹爪控制** | |
| 张开夹爪 | `Y` 键 |
| 闭合夹爪 | `X` 键 |
| **程序控制** | |
| 暂停/恢复机器人运动 | `A` 键 / `B` 键 |
| **开始** 数据录制 | `Start` 键 |
| **结束** 并 **保存** 数据 | `Back` 键 |
| 安全退出程序 | 在终端按 `Ctrl + C` |

## 📁 项目结构

```
.
├── src/
│   ├── camera_control/
│   │   ├── realsense_handler.py
│   │   └── zed_handler.py
│   ├── data_collection/
│   │   └── recorder.py
│   ├── main.py
│   ├── ur5_control/
│   │   └── ur5_controller.py
│   └── xbox/
│       └── xbox_control.py
├── robot_dataset/
│   └── session_YYYYMMDD_HHMMSS/
│       ├── robot_state.npy
│       ├── d435_color/
│       ├── d435_depth/
│       └── zed_color/
└── README.md
```

## 📊 输出数据格式

每次成功录制后，数据会保存在 `robot_dataset` 文件夹下的一个以会话开始时间命名的文件夹中。

- **`robot_state.npy`**
  - 这是一个NumPy数组，维度为 `(N, 7)`，其中 `N` 是采集的总帧数。
  - 每一行代表一个时间戳下的机器人状态，7个维度的含义分别是：
    1.  `tcp_x`: TCP位置X (米)
    2.  `tcp_y`: TCP位置Y (米)
    3.  `tcp_z`: TCP位置Z (米)
    4.  `tcp_rx`: TCP姿态Rx (轴角式, 弧度)
    5.  `tcp_ry`: TCP姿态Ry (轴角式, 弧度)
    6.  `tcp_rz`: TCP姿态Rz (轴角式, 弧度)
    7.  `gripper_state`: 夹爪状态 (1代表张开, 0代表闭合)

- **图像文件夹**
  - `d435_color/`, `d435_depth/`, `zed_color/`
  - 文件夹内存放着对应数据流的图像，均为 `.png` 格式。
  - 文件名以Unix时间戳命名（如 `1722984590.1234.png`），可用于和 `robot_state.npy` 中的数据进行精确的时间对齐。

## 📝 注意事项
- **深度图查看**：保存的 `d435_depth` 图像是16位的原始深度数据，用普通看图软件打开会显示为**全黑**。这是正常现象，因为它存储的是距离（单位：毫米）而非颜色。请在Python脚本中使用 `cv2.imread(path, cv2.IMREAD_UNCHANGED)` 来读取，以获取真实的深度值。
- **程序退出**：如果在录制过程中使用 `Ctrl+C` 强行退出程序，程序会尝试自动保存当前已采集的数据，防止数据丢失。