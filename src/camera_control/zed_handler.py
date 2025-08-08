# camera_control/zed_handler.py

import pyzed.sl as sl
import cv2
import numpy as np

class ZEDCamera:
    """
    一个用于控制 Stereolabs ZED2i 相机的类。
    """
    def __init__(self, resolution='HD720', fps=30):
        """
        初始化ZED相机。
        :param resolution: 分辨率 ('HD2K', 'HD1080', 'HD720', 'VGA')
        :param fps: 帧率
        """
        print("正在初始化ZED2i相机...")
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()

        # 设置初始化参数
        res_map = {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA
        }
        self.init_params.camera_resolution = res_map.get(resolution, sl.RESOLUTION.HD720)
        self.init_params.camera_fps = fps
        self.init_params.coordinate_units = sl.UNIT.METER # 使用米作为单位
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # 可以选择其他模式

        # 打开相机
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"打开ZED相机失败，错误码: {err}")
            raise ConnectionError("无法连接到ZED相机。")

        # 准备用于图像捕获的变量
        self.runtime_params = sl.RuntimeParameters()
        self.image_mat = sl.Mat()
        print("ZED2i相机初始化完成。")

    def get_frames(self):
        """
        从ZED相机捕获左眼的彩色图像。
        :return: color_image (numpy array)。如果失败则返回 None。
        """
        # 从相机抓取一帧新图像
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            # 获取左眼图像
            self.zed.retrieve_image(self.image_mat, sl.VIEW.LEFT)
            # 转换为Numpy数组 (默认是BGRA格式，4通道)
            color_image = self.image_mat.get_data()
            # 移除alpha通道，变为BGR
            return color_image[:, :, :3]
        
        print("警告: 未能从ZED相机抓取到图像。")
        return None

    def release(self):
        """
        关闭相机并释放资源。
        """
        print("正在释放ZED2i相机资源...")
        self.zed.close()

# --- 测试代码 ---
if __name__ == '__main__':
    try:
        # 初始化相机
        zed_cam = ZEDCamera(resolution='HD720')

        # 捕获一帧图像
        print("正在捕获一帧图像...")
        color_img = zed_cam.get_frames()

        if color_img is not None:
            # 保存图像
            filename = "zed2i_color_test.png"
            cv2.imwrite(filename, color_img)
            print(f"ZED2i的彩色图像已保存为 {filename}")
            
            # 显示图像（可选）
            cv2.imshow("ZED2i Color Image", color_img)
            print("按任意键关闭窗口...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    except ConnectionError as e:
        print(e)
    except Exception as e:
        print(f"发生未知错误: {e}")
    finally:
        # 确保即使出错也能尝试释放相机
        if 'zed_cam' in locals() and zed_cam.zed.is_opened():
            zed_cam.release()