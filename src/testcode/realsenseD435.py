import pyrealsense2 as rs
import cv2
import numpy as np

def main():
    # 1. 创建一个pipeline
    pipeline = rs.pipeline()

    # 2. 配置要使用的流 (彩色图)
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 3. 启动pipeline
    try:
        pipeline.start(config)
        print("RealSense D435 相机已成功打开!")
    except Exception as e:
        print(f"打开 RealSense 相机失败: {e}")
        exit(1)

    # 循环捕捉和显示图像
    try:
        while True:
            # 4. 等待一组成对的帧: 深度和彩色
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # 5. 将图像转换为NumPy数组
            color_image = np.asanyarray(color_frame.get_data())

            # 6. 显示图像
            cv2.imshow('RealSense D435 - Color', color_image)

            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 7. 停止pipeline
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()