from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

VALID_LOG_LEVELS = {"CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"}


def normalize_log_level(level: str) -> str:
    normalized = level.upper().strip()
    if normalized not in VALID_LOG_LEVELS:
        raise ValueError(
            f"不支持的日志级别: {level}，可选值: {', '.join(sorted(VALID_LOG_LEVELS))}"
        )
    return normalized


def setup_logging(
    *,
    log_level: str = "INFO",
    log_file: str | None = None,
    max_bytes: int = 10 * 1024 * 1024,
    backup_count: int = 5,
) -> None:
    """
    配置全局日志：
    - 控制台输出
    - 可选滚动文件输出
    """
    level = normalize_log_level(log_level)
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, level))

    # 避免重复初始化时叠加 handler
    for handler in list(root_logger.handlers):
        root_logger.removeHandler(handler)

    formatter = logging.Formatter(
        fmt="%(asctime)s | %(levelname)s | %(name)s | %(threadName)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, level))
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = RotatingFileHandler(
            filename=log_path,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding="utf-8",
        )
        file_handler.setLevel(getattr(logging, level))
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)
