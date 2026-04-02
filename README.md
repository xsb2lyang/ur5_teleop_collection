<p align="center">
  <a href="./README.md">
    <img alt="English" src="https://img.shields.io/badge/Language-English-blue" />
  </a>
  <a href="./README.zh-CN.md">
    <img alt="Chinese" src="https://img.shields.io/badge/语言-中文-red" />
  </a>
</p>

# UR5 Teleoperation Data Collection

A multimodal data collection toolkit for **UR5/UR5e teleoperation with Xbox controller**, designed for robot learning datasets.

## Features

- Real-time UR5/UR5e teleoperation (6DoF + gripper open/close)
- Synchronized `action/state` recording
- Multi-view capture `image0` (ZED 2i left RGB)
- Multi-view capture `image1` (RealSense D435 RGB)
- Multi-view capture `image3` (RealSense D435 depth)
- Task-oriented dataset organization
- Start/stop recording with gamepad buttons (`Start` / `Back`)
- Unified logging (console + rotating file logs)
- Externalized configuration (`config/defaults.ini`, `.env`, CLI overrides)

## Project Layout

```text
.
├── src/
│   ├── main.py                    # main entry
│   ├── ur5_control/               # UR5 control logic
│   ├── xbox/                      # Xbox input handling
│   ├── camera_control/            # ZED / D435 wrappers
│   ├── data_collection/           # recording and persistence
│   └── common/                    # shared utils (config/logging/text)
├── examples/                      # hardware examples and legacy scripts
│   └── README.md
├── config/defaults.ini            # default runtime config
├── .env.example                   # optional env overrides template
├── requirements.txt               # minimal runtime dependencies
├── requirements-dev.txt           # development dependencies
└── requirements-legacy-full.txt   # historical full export (reference only)
```

## Prerequisites

Hardware:
- UR5 or UR5e
- Robotiq 2F-85 gripper
- Xbox controller
- Intel RealSense D435
- Stereolabs ZED 2i
- Ubuntu 22.04 (recommended)

Software:
1. Install ZED SDK
2. Install librealsense
3. Install Python deps

```bash
pip install -r requirements.txt
```

4. Install local `pyzed` wheel

```bash
pip install ./pyzed-5.0-cp310-cp310-linux_x86_64.whl
```

Camera setup references:
- `documents/ZED 2i 相机的配置和使用.md`
- `documents/RealSense D435 相机的配置和使用.md`

## Configuration Priority

Config values are resolved in this order (high to low):

1. CLI arguments
2. OS environment variables
3. `.env` file
4. `config/defaults.ini`
5. built-in defaults

## Run

Interactive mode:

```bash
cd src
sudo $(which python) main.py
```

Configured mode (recommended):

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

Disable file logging:

```bash
python src/main.py --no-file-log
```

## Data Format

Default output root: `./robot_dataset`

```text
robot_dataset/
└── pick_up_the_red_block_demo/
    ├── demo_0/
    │   ├── image0/        # ZED RGB (*.jpg)
    │   ├── image1/        # D435 RGB (*.jpg)
    │   ├── image3/        # D435 depth (*.png, 16-bit)
    │   ├── action.npy     # [N, 7] => [dX,dY,dZ,dRx,dRy,dRz,gripper]
    │   ├── state.npy      # [N, 7] => [X,Y,Z,Rx,Ry,Rz,gripper]
    │   ├── lang.txt
    │   └── metadata.json
    └── demo_1/
```

## Controls (mapped to current code)

| Function | Input |
|---|---|
| X/Y translation | Left stick |
| Z translation | RT / LT |
| X-axis rotation (Roll) | RB / LB |
| Y-axis rotation (Pitch) | Right stick up/down |
| Z-axis rotation (Yaw) | Right stick left/right |
| Gripper open / close | `Y` / `X` |
| Pause / resume robot motion | `A` / `B` |
| Start / stop recording | `Start` / `Back` |

## Logging

The app uses a unified logger:
- console output
- rotating file logs (default: `./logs/ur5_teleop.log`)

Config keys:
- INI: `[logging] level`, `[logging] file`
- ENV: `UR5_LOG_LEVEL`, `UR5_LOG_FILE`
- CLI: `--log-level`, `--log-file`, `--no-file-log`

## Development

```bash
pip install -r requirements-dev.txt
python -m compileall -q src
pytest
```

## License

MIT
