# camera_control/realsense_handler.py

import pyrealsense2 as rs
import numpy as np
import cv2
import time

class RealSenseCamera:
    """
    一个用于控制 Intel RealSense D435 相机的类。
    """
    def __init__(self, width=1280, height=720, fps=30):
        """
        初始化相机和数据流。
        :param width: 图像宽度
        :param height: 图像高度
        :param fps: 帧率
        """
        print("正在初始化RealSense D435相机...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 配置彩色图和深度图的数据流
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # 启动数据流
        self.profile = self.pipeline.start(self.config)
        
        # 等待几帧，让自动曝光稳定
        for _ in range(15):
            self.pipeline.wait_for_frames()

        print("RealSense D435 相机初始化完成。")

    def get_frames(self):
        """
        捕获一对对齐的彩色图和深度图。
        :return: (color_image, depth_image) 元组。如果失败则返回 (None, None)。
        """
        try:
            # 等待一对连贯的帧：深度和彩色
            frames = self.pipeline.wait_for_frames()
            
            # 对齐深度帧到彩色帧
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                print("警告: 未能捕获到有效的帧。")
                return None, None

            # 将图像转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
        except Exception as e:
            print(f"从RealSense捕获帧时发生错误: {e}")
            return None, None
            
    def release(self):
        """
        停止数据流并释放相机资源。
        """
        print("正在释放RealSense D435相机资源...")
        self.pipeline.stop()

# --- 测试代码 ---
if __name__ == '__main__':
    # 初始化相机
    d435_cam = RealSenseCamera()
    
    # 获取一帧图像
    print("正在捕获一帧图像...")
    color_img, depth_img = d435_cam.get_frames()

    if color_img is not None and depth_img is not None:
        # 保存彩色图像
        color_filename = "d435_color_test.png"
        cv2.imwrite(color_filename, color_img)
        print(f"彩色图像已保存为 {color_filename}")

        # 将深度图可视化并保存
        # 深度图是16位的，直接显示是黑的，需要转换成8位图
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_img, alpha=0.03), # alpha参数可调整对比度
            cv2.COLORMAP_JET
        )
        depth_filename = "d435_depth_visualized.png"
        cv2.imwrite(depth_filename, depth_colormap)
        print(f"深度图的可视化版本已保存为 {depth_filename}")
        
        # 显示图像（可选）
        cv2.imshow("D435 Color Image", color_img)
        cv2.imshow("D435 Depth Image", depth_colormap)
        print("按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # 释放相机
    d435_cam.release()