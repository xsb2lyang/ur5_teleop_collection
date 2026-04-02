import argparse
import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from common.config_loader import get_ini_value, load_env_file, load_ini_config
from common.logging_utils import normalize_log_level, setup_logging

DEFAULT_CONFIG_FILE = "config/defaults.ini"
DEFAULT_ENV_FILE = ".env"
DEFAULT_ROBOT_IP = "192.168.192.7"
DEFAULT_RECORDING_FREQUENCY = 10
DEFAULT_DATASET_DIR = "./robot_dataset"
DEFAULT_D435_WIDTH = 1280
DEFAULT_D435_HEIGHT = 720
DEFAULT_CAMERA_FPS = 30
DEFAULT_ZED_RESOLUTION = "HD720"
DEFAULT_LOG_LEVEL = "INFO"
DEFAULT_LOG_FILE = "./logs/ur5_teleop.log"

MIN_RECORDING_FREQUENCY = 1
MAX_RECORDING_FREQUENCY = 120
VALID_ZED_RESOLUTIONS = {"HD2K", "HD1080", "HD720", "VGA"}
LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class RuntimeConfig:
    robot_ip: str
    task_instruction: str
    recording_frequency: int
    dataset_dir: str
    d435_width: int
    d435_height: int
    camera_fps: int
    zed_resolution: str
    log_level: str
    log_file: Optional[str]
    config_file: str
    env_file: str


def _validate_frequency(freq: int) -> int:
    if not (MIN_RECORDING_FREQUENCY <= freq <= MAX_RECORDING_FREQUENCY):
        raise ValueError(
            f"采集频率必须在 {MIN_RECORDING_FREQUENCY}~{MAX_RECORDING_FREQUENCY} Hz 之间，当前值: {freq}"
        )
    return freq


def _validate_positive_int(name: str, value: int) -> int:
    if value <= 0:
        raise ValueError(f"{name} 必须为正整数，当前值: {value}")
    return value


def _first_nonempty(*values: Optional[str]) -> Optional[str]:
    for value in values:
        if value is None:
            continue
        candidate = value.strip()
        if candidate:
            return candidate
    return None


def _resolve_task_instruction(task_candidate: Optional[str]) -> str:
    if task_candidate:
        return task_candidate

    while True:
        task_instruction = input(">>> (1/2) 请输入当前任务的语言指令，然后按 Enter: ").strip()
        if task_instruction:
            return task_instruction
        LOGGER.error("任务指令不能为空。")


def _resolve_frequency(freq_candidate: Optional[str | int]) -> int:
    if freq_candidate is not None:
        try:
            return _validate_frequency(int(freq_candidate))
        except ValueError as exc:
            raise ValueError(f"频率配置无效: {exc}") from exc

    while True:
        freq_str = input(">>> (2/2) 请输入数据采集频率 (Hz, 推荐 10-20)，然后按 Enter: ").strip()
        try:
            return _validate_frequency(int(freq_str))
        except ValueError as exc:
            LOGGER.error("频率输入错误: %s", exc)


def _resolve_int_config(
    *,
    name: str,
    cli_value: Optional[int],
    env_value: Optional[str],
    ini_value: Optional[str],
    default: int,
) -> int:
    raw = cli_value if cli_value is not None else _first_nonempty(env_value, ini_value)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} 配置无效，无法解析为整数: {raw}") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UR5 遥操作与多模态数据采集")
    parser.add_argument(
        "--config-file",
        default=DEFAULT_CONFIG_FILE,
        help=f"INI 配置文件路径（默认: {DEFAULT_CONFIG_FILE}）",
    )
    parser.add_argument(
        "--env-file",
        default=DEFAULT_ENV_FILE,
        help=f".env 文件路径（默认: {DEFAULT_ENV_FILE}）",
    )
    parser.add_argument("--robot-ip", help="UR 机器人 IP 地址")
    parser.add_argument("--task", help="任务自然语言指令。不提供时将交互式输入。")
    parser.add_argument(
        "--frequency",
        type=int,
        help=f"采集频率（Hz，范围 {MIN_RECORDING_FREQUENCY}-{MAX_RECORDING_FREQUENCY}）",
    )
    parser.add_argument("--dataset-dir", help="数据集根目录")
    parser.add_argument("--d435-width", type=int, help="D435 图像宽度")
    parser.add_argument("--d435-height", type=int, help="D435 图像高度")
    parser.add_argument("--camera-fps", type=int, help="相机采集帧率")
    parser.add_argument(
        "--zed-resolution",
        choices=sorted(VALID_ZED_RESOLUTIONS),
        help="ZED 分辨率",
    )
    parser.add_argument(
        "--log-level",
        help=f"日志级别（{', '.join(sorted(['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG']))}）",
    )
    parser.add_argument("--log-file", help="日志文件路径（启用文件日志时生效）")
    parser.add_argument("--no-file-log", action="store_true", help="仅输出控制台日志，不写文件")
    return parser.parse_args()


def build_runtime_config(args: argparse.Namespace) -> RuntimeConfig:
    ini_parser = load_ini_config(args.config_file)
    env_from_file = load_env_file(args.env_file)

    def env_value(name: str) -> Optional[str]:
        return _first_nonempty(os.getenv(name), env_from_file.get(name))

    robot_ip = _first_nonempty(
        args.robot_ip,
        env_value("UR5_ROBOT_IP"),
        get_ini_value(ini_parser, "robot", "ip"),
        DEFAULT_ROBOT_IP,
    )
    dataset_dir = _first_nonempty(
        args.dataset_dir,
        env_value("UR5_DATASET_DIR"),
        get_ini_value(ini_parser, "data", "dataset_dir"),
        DEFAULT_DATASET_DIR,
    )
    zed_resolution = _first_nonempty(
        args.zed_resolution,
        env_value("UR5_ZED_RESOLUTION"),
        get_ini_value(ini_parser, "camera", "zed_resolution"),
        DEFAULT_ZED_RESOLUTION,
    )
    if zed_resolution not in VALID_ZED_RESOLUTIONS:
        raise ValueError(
            f"ZED 分辨率无效: {zed_resolution}，可选值: {', '.join(sorted(VALID_ZED_RESOLUTIONS))}"
        )

    d435_width = _resolve_int_config(
        name="d435_width",
        cli_value=args.d435_width,
        env_value=env_value("UR5_D435_WIDTH"),
        ini_value=get_ini_value(ini_parser, "camera", "d435_width"),
        default=DEFAULT_D435_WIDTH,
    )
    d435_height = _resolve_int_config(
        name="d435_height",
        cli_value=args.d435_height,
        env_value=env_value("UR5_D435_HEIGHT"),
        ini_value=get_ini_value(ini_parser, "camera", "d435_height"),
        default=DEFAULT_D435_HEIGHT,
    )
    camera_fps = _resolve_int_config(
        name="camera_fps",
        cli_value=args.camera_fps,
        env_value=env_value("UR5_CAMERA_FPS"),
        ini_value=get_ini_value(ini_parser, "camera", "camera_fps"),
        default=DEFAULT_CAMERA_FPS,
    )
    _validate_positive_int("d435_width", d435_width)
    _validate_positive_int("d435_height", d435_height)
    _validate_positive_int("camera_fps", camera_fps)

    task_candidate = _first_nonempty(
        args.task,
        env_value("UR5_TASK"),
        get_ini_value(ini_parser, "runtime", "task_instruction"),
    )
    frequency_candidate = (
        args.frequency
        if args.frequency is not None
        else _first_nonempty(
            env_value("UR5_RECORDING_FREQUENCY"),
            get_ini_value(ini_parser, "runtime", "recording_frequency"),
            str(DEFAULT_RECORDING_FREQUENCY),
        )
    )

    task_instruction = _resolve_task_instruction(task_candidate)
    recording_frequency = _resolve_frequency(frequency_candidate)

    log_level_raw = _first_nonempty(
        args.log_level,
        env_value("UR5_LOG_LEVEL"),
        get_ini_value(ini_parser, "logging", "level"),
        DEFAULT_LOG_LEVEL,
    )
    log_level = normalize_log_level(log_level_raw)

    configured_log_file = _first_nonempty(
        args.log_file,
        env_value("UR5_LOG_FILE"),
        get_ini_value(ini_parser, "logging", "file"),
        DEFAULT_LOG_FILE,
    )
    log_file = None if args.no_file_log else configured_log_file

    return RuntimeConfig(
        robot_ip=robot_ip,
        task_instruction=task_instruction,
        recording_frequency=recording_frequency,
        dataset_dir=dataset_dir,
        d435_width=d435_width,
        d435_height=d435_height,
        camera_fps=camera_fps,
        zed_resolution=zed_resolution,
        log_level=log_level,
        log_file=log_file,
        config_file=args.config_file,
        env_file=args.env_file,
    )


class UR5JoyControl:
    """
    主控制类，负责初始化所有硬件模块，并编排各个线程的运行。
    """

    def __init__(self, config: RuntimeConfig):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.config = config
        self.ur5e = None
        self.joy = None
        self.d435_cam = None
        self.zed_cam = None
        self.recorder = None

        self.logger.info("正在初始化系统...")
        try:
            # 延迟导入硬件相关模块，保证 `python src/main.py --help` 在无硬件依赖环境下也可用
            from camera_control.realsense_handler import RealSenseCamera
            from camera_control.zed_handler import ZEDCamera
            from data_collection.recorder import DataRecorder
            from ur5_control.ur5_controller import UR5Controller
            from xbox.xbox_control import XboxController

            self.ur5e = UR5Controller(robot_ip=self.config.robot_ip)
            self.joy = XboxController()
            self.d435_cam = RealSenseCamera(
                width=self.config.d435_width,
                height=self.config.d435_height,
                fps=self.config.camera_fps,
            )
            self.zed_cam = ZEDCamera(
                resolution=self.config.zed_resolution,
                fps=self.config.camera_fps,
            )
            self.recorder = DataRecorder(
                self.ur5e,
                self.d435_cam,
                self.zed_cam,
                base_path=self.config.dataset_dir,
            )
            self.recorder.set_task_instruction(self.config.task_instruction)
        except Exception:
            self.logger.exception("系统初始化失败。")
            self._cleanup_hardware()
            raise

        self.task_instruction = self.config.task_instruction
        self.recording_frequency = self.config.recording_frequency
        self.control_loop_interval = 1.0 / self.recording_frequency
        self.logger.info("任务指令已设置: %s", self.task_instruction)
        self.logger.info(
            "采集频率已设置为: %s Hz (每帧间隔约 %.3f 秒)",
            self.recording_frequency,
            self.control_loop_interval,
        )

        # --- 初始化状态变量 ---
        self.motion_input = (0,) * 14
        self.is_recording = False
        self.last_start_button_state = 0
        self.last_back_button_state = 0
        self.running = True

        self.logger.info("系统初始化完成。")

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
        while self.running:
            try:
                # 1. 从控制器获取计算出的动作指令
                action_command = self.ur5e.move_robot(self.motion_input)

                # 2. 如果正在录制，则将该指令传递给记录器
                if self.is_recording:
                    self.recorder.record_step(action_command)

                # 3. 使用计算出的延时间隔来控制循环频率
                time.sleep(self.control_loop_interval)
            except Exception:
                self.logger.exception("机器人控制线程发生异常，正在触发停止流程。")
                self.stop()
                break

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
                self.logger.info("检测到 Start 按键，开始录制。")
                self.recorder.start_recording_session(sample_rate_hz=self.recording_frequency)

            # 检查是否按下Back键来结束录制
            if current_back_button_state == 1 and self.last_back_button_state == 0 and self.is_recording:
                self.is_recording = False
                self.logger.info("检测到 Back 按键，停止录制并保存。")
                self.recorder.stop_and_save_session()

            self.last_start_button_state = current_start_button_state
            self.last_back_button_state = current_back_button_state

            # 以较低频率检查按键即可
            time.sleep(0.1)

    def start(self):
        joystick_thread = threading.Thread(target=self.read_joystick, daemon=True, name="JoystickThread")
        robot_thread = threading.Thread(target=self.control_robot, daemon=True, name="RobotControlThread")
        collection_manager_thread = threading.Thread(
            target=self.data_collection_manager,
            daemon=True,
            name="DataCollectionManagerThread",
        )

        self.logger.info("正在启动所有控制线程...")
        joystick_thread.start()
        robot_thread.start()
        collection_manager_thread.start()

        self.logger.info("系统已启动，可开始操作。")
        self.logger.info("当前任务指令: %s", self.task_instruction)
        self.logger.info("机器人IP: %s", self.config.robot_ip)
        self.logger.info("数据保存目录: %s", self.config.dataset_dir)
        self.logger.info("采集频率: %s Hz", self.recording_frequency)
        self.logger.info("操作提示: Start 开始录制，Back 结束保存，Ctrl+C 安全退出。")

        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        if not self.running:
            return

        self.logger.info("程序中断，正在安全停止...")
        self.running = False

        if self.is_recording and self.recorder is not None:
            self.logger.info("程序退出时仍在录制，将自动保存数据...")
            self.recorder.stop_and_save_session()
            self.is_recording = False

        self._cleanup_hardware()
        self.logger.info("所有硬件已安全停止，程序退出。")

    def _cleanup_hardware(self):
        if self.ur5e is not None:
            try:
                self.ur5e.shutdown()
            except Exception:
                self.logger.exception("关闭 UR5 控制器时发生异常。")

        if self.d435_cam is not None:
            try:
                self.d435_cam.release()
            except Exception:
                self.logger.exception("释放 D435 相机时发生异常。")

        if self.zed_cam is not None:
            try:
                self.zed_cam.release()
            except Exception:
                self.logger.exception("释放 ZED 相机时发生异常。")

        if self.joy is not None and hasattr(self.joy, "stop"):
            try:
                self.joy.stop()
            except Exception:
                self.logger.exception("停止手柄监听线程时发生异常。")


def main():
    args = parse_args()
    setup_logging(log_level=DEFAULT_LOG_LEVEL, log_file=None)

    try:
        config = build_runtime_config(args)
        setup_logging(log_level=config.log_level, log_file=config.log_file)
    except ValueError as exc:
        LOGGER.error("配置错误: %s", exc)
        return

    LOGGER.info("配置加载完成。config_file=%s env_file=%s", config.config_file, config.env_file)
    LOGGER.info(
        "运行参数: robot_ip=%s dataset_dir=%s frequency=%sHz zed=%s d435=%sx%s@%sfps log_level=%s",
        config.robot_ip,
        config.dataset_dir,
        config.recording_frequency,
        config.zed_resolution,
        config.d435_width,
        config.d435_height,
        config.camera_fps,
        config.log_level,
    )
    if config.log_file:
        LOGGER.info("文件日志路径: %s", Path(config.log_file).resolve())
    else:
        LOGGER.info("文件日志已禁用。")

    try:
        controller = UR5JoyControl(config)
    except Exception:
        LOGGER.exception("初始化失败，程序退出。")
        return

    try:
        controller.start()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
