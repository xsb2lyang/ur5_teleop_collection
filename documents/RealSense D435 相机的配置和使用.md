

***Intel RealSense 相机依赖于开源的 librealsense SDK。***

#### 步骤 A: 安装 librealsense SDK 和驱动

Intel 官方为 Ubuntu 提供了非常方便的 `apt` 仓库，可以轻松安装。

1. **注册公钥**:
    
    Bash
    
    ```
    sudo mkdir -p /etc/apt/keyrings
    sudo curl -sSf "https://librealsense.intel.com/Debian/librealsense.pgp" | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    ```
    
2. **添加仓库源**:
    
    Bash
    
    ```
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
    ```
    
3. **安装 SDK**:
    
    Bash
    
    ```
    sudo apt-get update
    sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev
    ```
    
    - `librealsense2-utils` 包含一个非常有用的工具 `realsense-viewer`，可以用它来测试相机是否正常工作。
        
4. 安装 Python 依赖 (pyrealsense2):
    
    使用 pip 安装官方的 Python 封装库。
    
    Bash
    
    ```
    # 建议在虚拟环境(venv 或 conda)中执行
    pip install pyrealsense2
    # 如果之前没装过，同样需要OpenCV和NumPy
    pip install numpy opencv-python
    ```
    

#### 步骤 B: 在 Python 代码中打开 RealSense D435

以下是打开 D435 并显示彩色图像的基础代码。

Python

```
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
```

---


问题：
Installing collected packages: numpy, cython, pyzed
  Attempting uninstall: numpy
    Found existing installation: numpy 2.1.3
    Uninstalling numpy-2.1.3:
      Successfully uninstalled numpy-2.1.3
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
numba 0.61.0 requires numpy<2.2,>=1.24, but you have numpy 2.3.2 which is incompatible.
Successfully installed cython-3.1.2 numpy-2.3.2 pyzed-5.0
