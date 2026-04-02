import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from common.config_loader import get_ini_value, load_env_file, load_ini_config


def test_load_env_file_parse_key_values(tmp_path):
    env_file = tmp_path / ".env"
    env_file.write_text(
        "\n".join(
            [
                "# comment",
                "export A=1",
                "B='two words'",
                'C="three words"',
                "D=four # trailing comment",
                "INVALID_LINE",
            ]
        ),
        encoding="utf-8",
    )
    values = load_env_file(env_file)
    assert values["A"] == "1"
    assert values["B"] == "two words"
    assert values["C"] == "three words"
    assert values["D"] == "four"
    assert "INVALID_LINE" not in values


def test_load_ini_config_and_get_value(tmp_path):
    ini_file = tmp_path / "config.ini"
    ini_file.write_text("[robot]\nip=192.168.0.10\n", encoding="utf-8")
    parser = load_ini_config(ini_file)
    assert get_ini_value(parser, "robot", "ip") == "192.168.0.10"
    assert get_ini_value(parser, "robot", "missing") is None
