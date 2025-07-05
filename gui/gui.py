# car_control_gui.py
import tkinter as tk
from mecanum_wheel_car import MecanumCar
import time
import threading

class CarControlGUI:
    def __init__(self, root, car):
        self.root = root
        self.car = car
        self.root.title("麦轮小车控制")
        
        # 设置窗口大小和位置
        self.root.geometry("600x400")
        self.root.resizable(False, False)
        
        # 创建标题
        title_label = tk.Label(root, text="麦轮小车控制面板", font=("Arial", 20))
        title_label.pack(pady=20)
        
        # 创建按键说明
        key_frame = tk.Frame(root)
        key_frame.pack(pady=10)
        
        instructions = [
            "W: 向前移动", 
            "A: 向左移动",
            "S: 向后移动", 
            "D: 向右移动",
            "Q: 逆时针旋转", 
            "E: 顺时针旋转",
            "ESC: 退出程序"
        ]
        
        for instr in instructions:
            label = tk.Label(key_frame, text=instr, font=("Arial", 12))
            label.pack(anchor="w", padx=20)
        
        # 创建状态显示框
        self.status_frame = tk.Frame(root, relief=tk.SUNKEN, borderwidth=2)
        self.status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.status_label = tk.Label(self.status_frame, text="就绪 - 等待按键输入", 
                                     font=("Arial", 12), fg="blue")
        self.status_label.pack(pady=10)
        
        # 电机状态显示
        self.motor_status = tk.Label(self.status_frame, text="电机速度: 所有电机停止", 
                                    font=("Arial", 10))
        self.motor_status.pack(pady=5)
        
        # 记录键盘状态
        self.key_pressed = set()
        
        # 绑定键盘事件
        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)
        
        # 焦点设置到窗口，这样键盘事件才能被捕获
        self.root.focus_set()
        
        # 创建控制线程，持续处理按键状态
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
    
    def update_status(self, action):
        """更新状态显示"""
        self.status_label.config(text=action)
        
        # 更新电机状态显示
        motor_speeds = self.car.get_motor_speeds()
        speed_text = "电机速度: "
        for motor, speed in motor_speeds.items():
            speed_text += f"{motor}: {speed}% "
        self.motor_status.config(text=speed_text)
    
    def key_press(self, event):
        """处理按键按下事件"""
        key = event.keysym.lower()
        
        # 将按键添加到按下集合中
        if key in ['w', 'a', 's', 'd', 'q', 'e']:
            self.key_pressed.add(key)
        
        # 处理ESC键 - 退出程序
        if key == 'escape':
            self.running = False
            self.car.stop()
            self.root.destroy()
    
    def key_release(self, event):
        """处理按键释放事件"""
        key = event.keysym.lower()
        
        # 从按下集合中移除按键
        if key in self.key_pressed:
            self.key_pressed.remove(key)
    
    def control_loop(self):
        """控制循环，持续检查按键状态并控制小车"""
        while self.running:
            if not self.key_pressed:
                # 没有按键被按下，停止小车
                self.car.stop()
                self.update_status("就绪 - 等待按键输入")
            elif 'w' in self.key_pressed and not {'a', 's', 'd'} & self.key_pressed:
                # 前进
                self.car.move(90, 100)
                self.update_status("前进")
            elif 's' in self.key_pressed and not {'w', 'a', 'd'} & self.key_pressed:
                # 后退
                self.car.move(270, 100)
                self.update_status("后退")
            elif 'a' in self.key_pressed and not {'w', 's', 'd'} & self.key_pressed:
                # 左移
                self.car.move(180, 100)
                self.update_status("左移")
            elif 'd' in self.key_pressed and not {'w', 's', 'a'} & self.key_pressed:
                # 右移
                self.car.move(0, 100)
                self.update_status("右移")
            elif 'w' in self.key_pressed and 'd' in self.key_pressed:
                # 右前
                self.car.move(45, 100)
                self.update_status("右前")
            elif 'w' in self.key_pressed and 'a' in self.key_pressed:
                # 左前
                self.car.move(135, 100)
                self.update_status("左前")
            elif 's' in self.key_pressed and 'd' in self.key_pressed:
                # 右后
                self.car.move(315, 100)
                self.update_status("右后")
            elif 's' in self.key_pressed and 'a' in self.key_pressed:
                # 左后
                self.car.move(225, 100)
                self.update_status("左后")
            elif 'q' in self.key_pressed and 'e' not in self.key_pressed:
                # 逆时针旋转
                self.car.rotate(90, 100)
                self.update_status("逆时针旋转")
            elif 'e' in self.key_pressed and 'q' not in self.key_pressed:
                # 顺时针旋转
                self.car.rotate(-90, 100)
                self.update_status("顺时针旋转")
            
            # 短暂睡眠，减少CPU使用率
            time.sleep(0.05)

if __name__ == "__main__":
    # 定义电机引脚
    pins = {
        'front_left': {'in1': 17, 'in2': 18, 'encoder_a': 27},
        'front_right': {'in1': 22, 'in2': 23, 'encoder_a': 24},
        'rear_left': {'in1': 5, 'in2': 6, 'encoder_a': 13},
        'rear_right': {'in1': 19, 'in2': 26, 'encoder_a': 16}
    }

    # 创建麦轮小车对象
    car = MecanumCar(pins)
    
    try:
        # 创建GUI
        root = tk.Tk()
        app = CarControlGUI(root, car)
        root.mainloop()
    finally:
        # 确保在退出时清理GPIO
        car.stop()
        del car