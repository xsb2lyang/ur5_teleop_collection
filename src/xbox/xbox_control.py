# xbox/xbox_control.py

# --- 文件头部信息 ---
# 核心改动：read() 方法现在返回 A, B, X, Y, Start, Back 按钮的状态。

# MIT 许可证
# 
# 版权所有 (c) 2017 Kevin Hughes
#
# 此处为开源软件的MIT许可证文本，允许用户自由地使用、修改和分发代码，
# 但要求在所有副本或重要部分中包含版权声明和此许可声明。
# 作者不对软件的性能或可能产生的问题提供任何保证。
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
这是对 Kevin Hughes (2017) 原始 XboxController 类的修改版本。
- 主要修改了 `read()` 函数，使其返回不同的手柄输入值。
"""
# --- 导入库 ---
# 从 inputs 库导入 get_gamepad 函数，这是捕获手柄底层事件的核心
from inputs import get_gamepad
# 导入 math 库，用于数学运算，如此处的幂运算
import math
# 导入 threading 库，用于在后台线程中持续监听手柄事件，避免阻塞主程序
import threading

class XboxController(object):
    """
    一个用于读取Xbox手柄输入的类。
    它在后台启动一个监听线程，持续更新手柄各个组件的状态，
    并提供一个read()方法来获取这些状态。
    """
    # --- 类常量 ---
    # 定义扳机（Trigger）的最大值。Xbox手柄的扳机是8位的，所以最大值是 2^8 = 256。
    MAX_TRIG_VAL = math.pow(2, 8)
    # 定义摇杆（Joystick）的最大值。摇杆的每个轴是15位的，所以最大值是 2^15。
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        """
        构造函数，在创建类的实例时被调用。
        负责初始化所有手柄组件的状态变量，并启动后台监听线程。
        """
        # --- 初始化所有状态变量为0 ---
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0      # 左摇杆按下
        self.RightThumb = 0     # 右摇杆按下
        self.Back = 0           # "Back" 或 "Select" 键
        self.Start = 0          # "Start" 键
        self.LeftDPad = 0       # D-Pad (方向键) 左
        self.RightDPad = 0      # D-Pad 右
        self.UpDPad = 0         # D-Pad 上
        self.DownDPad = 0       # D-Pad 下

        # --- 创建并启动后台监听线程 ---
        # target=self._monitor_controller 指定了线程要执行的方法
        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        # 设置为守护线程（daemon=True），意味着当主程序退出时，这个线程也会被自动销毁
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        """
        获取并返回所有关心的手柄输入值的当前状态。
        这个方法是给外部程序调用的，它不直接读取硬件，而是返回由后台线程更新的变量值。
        
        :return: 一个包含14个元素的状态列表。
        """
        # --- 平移控制相关的输入 ---
        x = self.LeftJoystickX      # 左摇杆左右
        y = self.LeftJoystickY      # 左摇杆上下
        z_up = self.RightTrigger    # 右扳机
        z_down = self.LeftTrigger   # 左扳机
        
        # --- 旋转控制相关的输入 ---
        rx = self.RightJoystickY    # 右摇杆上下
        ry = self.RightJoystickX    # 右摇杆左右
        rz_neg = self.LeftBumper    # 左肩键
        rz_pos = self.RightBumper   # 右肩键
        
        # --- 功能按钮 ---
        button_A = self.A
        button_B = self.B
        button_X = self.X
        button_Y = self.Y
        
        # --- 其他功能按钮 ---
        start_button = self.Start
        back_button = self.Back
        
        # 将所有状态值打包成一个列表并返回
        return [x, y, z_up, z_down, rx, ry, rz_neg, rz_pos, button_A, button_B, button_X, button_Y, start_button, back_button]

    def _monitor_controller(self):
        """
        这个方法在后台线程中无限循环运行，
        负责捕获手柄的原始事件，并更新类的状态变量。
        """
        while True:
            # get_gamepad() 是一个阻塞函数，它会等待直到有手柄事件发生
            events = get_gamepad()
            # 遍历所有捕获到的事件
            for event in events:
                # --- 摇杆事件 ---
                # event.code 是事件的唯一标识符
                if event.code == 'ABS_Y':
                    # 将摇杆的原始值（-32768 到 32767）归一化到 -1.0 到 1.0 之间
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL
                # --- 扳机事件 ---
                elif event.code == 'ABS_Z': # 左扳机
                    # 将扳机的原始值（0 到 255）归一化到 0.0 到 1.0 之间
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL
                elif event.code == 'ABS_RZ': # 右扳机
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL
                # --- 肩键事件 ---
                # 数字按钮的状态通常是 1（按下）或 0（松开）
                elif event.code == 'BTN_TL': # 左肩键
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR': # 右肩键
                    self.RightBumper = event.state
                # --- A,B,X,Y 按钮事件 ---
                elif event.code == 'BTN_SOUTH': # A键
                    self.A = event.state
                elif event.code == 'BTN_NORTH': # Y键
                    self.Y = event.state
                elif event.code == 'BTN_WEST': # X键
                    self.X = event.state
                elif event.code == 'BTN_EAST': # B键
                    self.B = event.state
                # --- 摇杆按下事件 ---
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                # --- Start/Back 按钮事件 ---
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                # --- D-Pad (方向键) 事件 ---
                # 'BTN_TRIGGER_HAPPY' 是 'inputs' 库对某些手柄D-Pad的命名
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

# --- 模块独立测试代码 ---
# 当直接运行 `python xbox_control.py` 时，以下代码会被执行
if __name__ == '__main__':
    # 创建一个XboxController实例
    joy = XboxController()
    # 无限循环，持续打印手柄状态，用于测试
    while True:
        print(joy.read())