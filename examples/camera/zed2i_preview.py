import pyzed.sl as sl
import cv2
import numpy as np

def main():
    # 1. 创建相机对象
    zed = sl.Camera()

    # 2. 设置初始化参数
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 设置分辨率
    init_params.camera_fps = 30  # 设置帧率

    # 3. 打开相机
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"打开 ZED 相机失败: {err}")
        exit(1)

    print("ZED 相机已成功打开!")

    # 创建一个Mat对象来存储图像
    image_zed = sl.Mat()

    # 循环捕捉和显示图像
    while True:
        # 4. 从相机抓取一帧
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # 5. 将左目图像数据检索到Mat对象
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            
            # 6. 将图像转换为OpenCV格式 (NumPy array)
            frame = image_zed.get_data()
            
            # 显示图像
            cv2.imshow("ZED 2i - Left Eye", frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 7. 关闭相机和窗口
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()