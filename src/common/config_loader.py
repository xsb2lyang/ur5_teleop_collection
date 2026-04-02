from __future__ import annotations

import configparser
from pathlib import Path


def load_ini_config(path: str | Path) -> configparser.ConfigParser:
    """
    读取 INI 配置文件。文件不存在时返回空配置对象。
    """
    parser = configparser.ConfigParser()
    config_path = Path(path)
    if config_path.exists():
        parser.read(config_path, encoding="utf-8")
    return parser


def get_ini_value(parser: configparser.ConfigParser, section: str, key: str) -> str | None:
    if parser.has_option(section, key):
        value = parser.get(section, key).strip()
        return value if value else None
    return None


def load_env_file(path: str | Path) -> dict[str, str]:
    """
    读取 .env 文件内容，不写入全局环境变量，返回键值字典。
    支持：
    - KEY=value
    - export KEY=value
    - 引号包裹值
    - 注释行
    """
    result: dict[str, str] = {}
    env_path = Path(path)
    if not env_path.exists():
        return result

    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if line.startswith("export "):
            line = line[len("export ") :].strip()

        if "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            continue

        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        elif " #" in value:
            value = value.split(" #", 1)[0].strip()

        result[key] = value

    return result
