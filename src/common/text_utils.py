import re

INVALID_FILENAME_CHARS = r'[<>:"/\\|?*\x00-\x1F]'


def sanitize_task_name(instruction: str) -> str:
    """
    将任务指令转换为文件系统安全的目录名。
    保留中文等 Unicode 字符，仅移除非法字符并规整空白。
    """
    cleaned = re.sub(INVALID_FILENAME_CHARS, "", instruction.strip())
    cleaned = re.sub(r"\s+", "_", cleaned)
    cleaned = cleaned.strip("._")
    return cleaned
