<p align="center">
  <a href="./README.md">
    <img alt="English" src="https://img.shields.io/badge/Language-English-blue" />
  </a>
  <a href="./README.zh-CN.md">
    <img alt="Chinese" src="https://img.shields.io/badge/语言-中文-red" />
  </a>
</p>

# UR5 遥操作数据采集项目

这是一个面向机器人学习数据集构建的 **UR5/UR5e + Xbox 手柄遥操作多模态采集工具**。

## 功能特性

- UR5/UR5e 实时遥操作（6DoF + 夹爪开合）
- 同步记录 `action/state`
- 多视角图像采集 `image0`（ZED 2i 左目 RGB）
- 多视角图像采集 `image1`（RealSense D435 RGB）
- 多视角图像采集 `image3`（RealSense D435 深度图）
- 按任务指令自动组织数据目录
- 手柄 `Start` / `Back` 控制开始和结束录制
- 统一日志系统（控制台 + 文件滚动日志）
- 配置外置化（`config/defaults.ini`、`.env`、CLI 覆盖）

## 项目结构

```text
.
├── src/
│   ├── main.py                    # 主入口
│   ├── ur5_control/               # UR5 控制逻辑
│   ├── xbox/                      # 手柄输入模块
│   ├── camera_control/            # ZED / D435 封装
│   ├── data_collection/           # 数据录制与存储
│   └── common/                    # 通用模块（配置/日志/文本）
├── examples/                      # 硬件示例与历史脚本
│   └── README.md
├── config/defaults.ini            # 默认配置
├── .env.example                   # 环境变量模板
├── requirements.txt               # 最小运行依赖
├── requirements-dev.txt           # 开发依赖
└── requirements-legacy-full.txt   # 历史完整导出（仅参考）
```

## 环境准备

硬件：
- UR5 / UR5e
- Robotiq 2F-85
- Xbox 手柄
- Intel RealSense D435
- Stereolabs ZED 2i
- Ubuntu 22.04（推荐）

软件：
1. 安装 ZED SDK
2. 安装 librealsense
3. 安装 Python 依赖

```bash
pip install -r requirements.txt
```

4. 安装本地 `pyzed` wheel

```bash
pip install ./pyzed-5.0-cp310-cp310-linux_x86_64.whl
```

相机安装文档：
- `documents/ZED 2i 相机的配置和使用.md`
- `documents/RealSense D435 相机的配置和使用.md`

## 配置优先级

配置解析顺序（从高到低）：

1. CLI 参数
2. 系统环境变量
3. `.env` 文件
4. `config/defaults.ini`
5. 代码内置默认值

## 运行方式

交互式运行：

```bash
cd src
sudo $(which python) main.py
```

推荐（显式配置）：

```bash
sudo $(which python) src/main.py \
  --config-file config/defaults.ini \
  --env-file .env \
  --robot-ip 192.168.192.7 \
  --task "pick up the red block" \
  --frequency 10 \
  --dataset-dir ./robot_dataset \
  --log-level INFO
```

仅控制台日志（不写文件）：

```bash
python src/main.py --no-file-log
```

## 输出数据格式

默认输出目录为 `./robot_dataset`：

```text
robot_dataset/
└── pick_up_the_red_block_demo/
    ├── demo_0/
    │   ├── image0/        # ZED RGB (*.jpg)
    │   ├── image1/        # D435 RGB (*.jpg)
    │   ├── image3/        # D435 深度 (*.png, 16-bit)
    │   ├── action.npy     # [N, 7] => [dX,dY,dZ,dRx,dRy,dRz,gripper]
    │   ├── state.npy      # [N, 7] => [X,Y,Z,Rx,Ry,Rz,gripper]
    │   ├── lang.txt
    │   └── metadata.json
    └── demo_1/
```

## 控制映射（与代码一致）

| 功能 | 输入 |
|---|---|
| X/Y 平移 | 左摇杆 |
| Z 平移 | RT / LT |
| 绕 X 轴旋转（Roll） | RB / LB |
| 绕 Y 轴旋转（Pitch） | 右摇杆上下 |
| 绕 Z 轴旋转（Yaw） | 右摇杆左右 |
| 夹爪开 / 合 | `Y` / `X` |
| 暂停 / 恢复运动 | `A` / `B` |
| 开始 / 结束录制 | `Start` / `Back` |

## 日志系统

项目已接入统一日志：
- 控制台输出
- 文件滚动日志（默认：`./logs/ur5_teleop.log`）

配置项：
- INI：`[logging] level`、`[logging] file`
- ENV：`UR5_LOG_LEVEL`、`UR5_LOG_FILE`
- CLI：`--log-level`、`--log-file`、`--no-file-log`

## 开发

```bash
pip install -r requirements-dev.txt
python -m compileall -q src
pytest
```

## 许可证

MIT
