import argparse

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="读取并预览 .npy 文件")
    parser.add_argument("npy_path", help="要读取的 .npy 文件路径")
    parser.add_argument(
        "--max-rows",
        type=int,
        default=5,
        help="最多打印前几行数据（默认: 5）",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    data = np.load(args.npy_path)
    print(f"Loaded: {args.npy_path}")
    print("Shape:", data.shape)
    if data.ndim == 0:
        print("Value:", data.item())
        return
    print("Preview:")
    print(data[: args.max_rows])


if __name__ == "__main__":
    main()
