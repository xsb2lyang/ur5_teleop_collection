import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from common.text_utils import sanitize_task_name


def test_sanitize_task_name_keep_chinese_and_replace_spaces():
    assert sanitize_task_name("  抓起 红色 方块  ") == "抓起_红色_方块"


def test_sanitize_task_name_remove_invalid_chars():
    assert sanitize_task_name('pick/up:<red>|block?*') == "pickupredblock"


def test_sanitize_task_name_strip_dot_and_underscore():
    assert sanitize_task_name("..__task_name__..") == "task_name"
