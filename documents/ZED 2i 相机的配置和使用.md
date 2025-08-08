
## ZED 2i 相机在 Ubuntu 22.04 和 Conda 环境下的完整配置教程

本教程旨在为在 Ubuntu 22.04 系统上，特别是在 Conda 虚拟环境中配置 Stereolabs ZED 2i 相机的用户提供一份详尽的、端到端的指南。教程整合了官方安装步骤以及在实践中可能遇到的常见问题及其解决方案。

### **前期准备 (Prerequisites)**

在开始之前，请确保您已具备以下条件：

1. **硬件**:
    
    - 一台 ZED 2i 相机。
        
    - 一台装有 NVIDIA 显卡的电脑。
        
2. **软件**:
    
    - 已安装 **Ubuntu 22.04** 操作系统。
        
    - 已正确安装 **NVIDIA 显卡驱动**。
        
    - 已安装 **Anaconda** 或 **Miniconda**。
        

### **步骤一：创建并激活 Conda 环境**

为了保持项目依赖的纯净，避免与系统或其他项目冲突，我们首先创建一个全新的 Conda 环境。

1. 打开您的终端（Terminal）。
    
2. 运行以下命令创建名为 `zed_env` 的环境（您也可以自定义名称或者就是后面要用的虚拟环境也可以），并指定 Python 版本（例如 3.10）：
    
    Bash
    
    ```
    conda create --name zed_env python=3.10
    ```
    
3. 激活刚刚创建的环境。在后续所有操作中，请确保您始终处于此激活的环境中。
    
    Bash
    
    ```
    conda activate zed_env
    ```
    
    成功激活后，您的终端提示符前会显示 `(zed_env)`。
    

### **步骤二：安装 ZED SDK 核心程序**

这是最基础的一步，我们需要从官方网站下载并安装 ZED SDK，它包含了驱动、库文件和工具。

1. **访问 Stereolabs 官网下载页面**: [https://www.stereolabs.com/developers/release/](https://www.stereolabs.com/developers/release/)
    
2. 在页面上，根据您的系统选择对应的选项：
    
    - **Product**: ZED SDK
        
    - **Host**: Ubuntu 22
        
    - **CUDA Version**: 选择与您系统匹配的 CUDA 版本。
        
3. 点击 "Download" 按钮，下载一个 `.run` 格式的安装文件。
    ![[Pasted image 20250807130333.png]]
4. 下载完成后，进入文件所在的目录，首先赋予该文件可执行权限：
    
    Bash
    
    ```
    # 将 "ZED_SDK_Ubuntu22_..." 替换为您下载的实际文件名
    chmod +x ZED_SDK_Ubuntu22_*.run
    ```
    
5. 运行安装程序：
    
    Bash
    
    ```
    ./ZED_SDK_Ubuntu22_*.run
    ```
    
6. 按照屏幕上的提示完成安装。请仔细阅读并同意许可协议，通常保持默认的安装选项即可。
    

### **步骤三：安装 Python 依赖库 (pyzed) 及问题排查**

这是最容易出错的环节。我们将主动解决在实践中遇到的所有问题。

#### **3.1 准备工作：预先安装 `pyzed` 的依赖项**

官方的 `pyzed` 安装脚本自身也需要一些库来运行。为了避免中途出错，我们提前把它们装好。

1. **安装 `requests`**:
    
    Bash
    
    ```
    pip install requests
    ```
    
2. **安装 `cython`**: 在某些网络环境下，直接安装可能会因 SSL 问题失败。我们直接使用“信任主机”的方法来确保成功安装。
    
    Bash
    
    ```
    pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org cython
    ```
    

#### **3.2 运行官方脚本安装 `pyzed`**

现在，在所有准备工作就绪后，我们运行 ZED SDK 自带的脚本来为当前的 Conda 环境安装 `pyzed`。

Bash

```
python /usr/local/zed/get_python_api.py
```

这个脚本会自动检测您 `(zed_env)` 环境中的 Python 版本，并下载、安装与之匹配的 `pyzed` 库。

#### **3.3 验证 `pyzed` 安装**

运行以下命令，检查 `pyzed` 是否已存在于您的环境包列表中：

Bash

```
pip list | grep pyzed
```

如果看到 `pyzed` 及其版本号，说明此步骤已成功。

### **步骤四：解决 GLIBCXX 库版本冲突**

这是 Conda 环境下特有的一个关键问题。即使 `pyzed` 安装成功，运行时也可能因为C++库版本冲突而失败。

1. **问题原因**: Conda 环境自带的 C++ 标准库 (`libstdc++.so.6`) 版本，可能比 ZED SDK 编译时所用的版本要旧。
    
2. **解决方法**: 使用 `conda-forge` 渠道，为当前环境安装一个更新的C++库。
    
    Bash
    
    ```
    conda install -c conda-forge libstdcxx-ng
    ```
    

### **步骤五：编写并运行测试代码**

所有环境问题都已解决，现在我们可以通过一段简单的 Python 代码来验证最终成果。

1. 创建一个名为 `test_zed.py` 的文件，并将以下代码粘贴进去：
    
    Python
    
    ```
    import pyzed.sl as sl
    import cv2
    import numpy as np
    
    def main():
        print("正在尝试打开 ZED 相机...")
        zed = sl.Camera()
    
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
    
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"打开 ZED 相机失败，错误码: {err}")
            exit(1)
    
        print("ZED 相机已成功打开!")
        print(f"序列号 (Serial Number): {zed.get_camera_information().serial_number}")
    
        image_zed = sl.Mat()
    
        while True:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                frame = image_zed.get_data()
                cv2.imshow("ZED 2i - Press 'q' to exit", frame)
    
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
        zed.close()
        cv2.destroyAllWindows()
        print("程序已退出。")
    
    if __name__ == "__main__":
        main()
    ```
    
2. 运行这个脚本：
    
    Bash
    
    ```
    python test_zed.py
    ```
    

### **步骤六：查看成功标志**

如果一切顺利，您将会看到：

1. 终端输出 ZED SDK 的初始化日志，其中包含关键的一行： `[ZED][INFO] [Init] Camera successfully opened.`
    
2. 终端打印出 "ZED 相机已成功打开!" 以及您的设备序列号。
    
3. 弹出一个标题为 "ZED 2i - Press 'q' to exit" 的窗口，其中显示着相机捕捉到的实时视频画面。
    

---
