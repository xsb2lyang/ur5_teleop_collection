# Changelog

## 0.1.0 - 2026-04-02

- 主程序支持命令行参数配置（机器人 IP、任务、采样频率、数据目录、相机参数）
- 增强主程序退出流程与硬件资源清理
- `UR5Controller` 新增 `shutdown()`，并修复暂停时重复 `servoStop` 的控制逻辑
- `DataRecorder` 增加任务名清洗逻辑
- `DataRecorder` 增加图像落盘成功校验
- `DataRecorder` 增加 `metadata.json` 会话元数据
- 补充包结构 `__init__.py`
- 新增 `common/text_utils.py` 与对应单元测试
- 依赖拆分为 `requirements.txt` 与 `requirements-dev.txt`
- 保留历史完整依赖导出为 `requirements-legacy-full.txt`
- 重构 README，补充 CONTRIBUTING、pyproject、GitHub CI

## 0.2.0 - 2026-04-02

- 引入统一日志系统（控制台 + RotatingFileHandler 文件日志）
- 核心运行模块将 `print` 迁移为 `logging`（`main/ur5_controller/recorder/camera`）
- 配置外置化：新增 `config/defaults.ini` 与 `.env.example`
- 主程序支持 `配置文件 + .env + 环境变量 + CLI` 分层覆盖
- 主程序新增日志相关参数：`--log-level`、`--log-file`、`--no-file-log`
- README 改为英文默认，并新增中文版 `README.zh-CN.md`（按钮切换）
- 新增配置加载单元测试 `tests/test_config_loader.py`

## 0.3.0 - 2026-04-02

- 将 `src/testcode` 迁移为 `examples/`，与生产代码解耦
- 示例脚本按场景分组：`examples/camera`、`examples/robot`、`examples/legacy`
- 新增 `examples/README.md`，明确示例用途、风险与运行边界
- README / README.zh-CN 的项目结构同步更新为 `examples/`
