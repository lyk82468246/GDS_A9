import tkinter as tk
import threading
import time
from adafruit_servokit import ServoKit
from mecanum_wheel_car import MecanumCar
from peripherals import Peripherals

class ServoController:
    """舵机云台控制器，负责控制水平和垂直方向的舵机"""
    
    def __init__(self):
        # 创建舵机控制套件，使用16通道PWM控制器
        self.servo_kit = ServoKit(channels=16)
        
        # 设置默认参数
        self.pan_channel = 0       # 水平舵机通道
        self.tilt_channel = 1      # 垂直舵机通道
        self.pan_angle = 75        # 水平初始角度
        self.tilt_angle = 90       # 垂直初始角度
        self.angle_min = 0         # 最小角度
        self.angle_max = 180       # 最大角度
        self.servo_increment = 0.2   # 每次调整的角度增量
        
        # 初始化舵机位置
        self.set_pan_angle(self.pan_angle)
        self.set_tilt_angle(self.tilt_angle)
    
    def set_pan_angle(self, angle):
        """设置水平舵机角度"""
        # 确保角度在有效范围内
        angle = max(self.angle_min, min(self.angle_max, angle))
        self.pan_angle = angle
        self.servo_kit.servo[self.pan_channel].angle = angle
        return self.pan_angle
    
    def set_tilt_angle(self, angle):
        """设置垂直舵机角度"""
        # 确保角度在有效范围内
        angle = max(self.angle_min, min(self.angle_max, angle))
        self.tilt_angle = angle
        self.servo_kit.servo[self.tilt_channel].angle = angle
        return self.tilt_angle
    
    def adjust_pan_angle(self, increment):
        """水平角度调整"""
        return self.set_pan_angle(self.pan_angle + increment)
    
    def adjust_tilt_angle(self, increment):
        """垂直角度调整"""
        return self.set_tilt_angle(self.tilt_angle + increment)
    
    def cleanup(self):
        """清理资源"""
        # 将舵机恢复到初始位置
        self.set_pan_angle(90)
        self.set_tilt_angle(90)
        time.sleep(0.5)  # 等待舵机移动到位

class MecanumCarController:
    def __init__(self, root):
        self.root = root
        self.root.title("麦轮小车控制系统")
        self.root.geometry("700x550")
        self.root.configure(bg="#f0f0f0")
        
        # 定义电机引脚
        pins = {
            'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
            'front_right': {'in1': 13, 'in2': 19, 'encoder_a': 26},
            'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
            'rear_right': {'in1': 21, 'in2': 20, 'encoder_a': 16}
        }
        
        # 创建麦轮小车对象
        self.car = MecanumCar(pins)
        
        # 创建外设控制器（蜂鸣器和LED）
        self.peripherals = Peripherals()
        
        # 创建舵机控制器
        self.servo_controller = ServoController()
        
        # 标记是否获胜
        self.is_winner = False
        
        # 标记是否进行了初次校准
        self.is_calibrated = False
        
        # 舵机控制参数
        self.servo_increment = 1.5  # 舵机每次调整的角度
        
        # 创建UI组件
        self.create_widgets()
        
        # 设置键盘事件监听
        self.key_pressed = set()
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        
        # 启动控制线程
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # 关闭窗口时执行清理
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_widgets(self):
        # 标题
        title_label = tk.Label(self.root, text="麦轮小车与外设控制系统", font=("Arial", 18, "bold"), bg="#f0f0f0")
        title_label.pack(pady=10)
        
        # 创建三个框架，分别显示小车控制、伺服控制和外设控制
        control_frame = tk.Frame(self.root, bg="#f0f0f0")
        control_frame.pack(fill=tk.BOTH, expand=True, padx=10)
        
        # 小车控制说明框架
        car_frame = tk.LabelFrame(control_frame, text="小车控制", font=("Arial", 12), bg="#f0f0f0")
        car_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=10)
        
        car_instructions = [
            "W: 向前移动", 
            "S: 向后移动", 
            "A: 向左移动", 
            "D: 向右移动",
            "Q: 逆时针旋转", 
            "E: 顺时针旋转"
        ]
        
        for instr in car_instructions:
            label = tk.Label(car_frame, text=instr, font=("Arial", 11), bg="#f0f0f0")
            label.pack(anchor="w", padx=5, pady=2)
        
        # 伺服控制说明框架
        servo_frame = tk.LabelFrame(control_frame, text="伺服控制", font=("Arial", 12), bg="#f0f0f0")
        servo_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=10)
        
        servo_instructions = [
            "U: 向上调整角度", 
            "J: 向下调整角度",
            "H: 向左调整角度",
            "K: 向右调整角度"
        ]
        
        for instr in servo_instructions:
            label = tk.Label(servo_frame, text=instr, font=("Arial", 11), bg="#f0f0f0")
            label.pack(anchor="w", padx=5, pady=2)
        
        # 外设控制说明框架
        peripheral_frame = tk.LabelFrame(control_frame, text="外设控制", font=("Arial", 12), bg="#f0f0f0")
        peripheral_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=10)
        
        peripheral_instructions = [
            "B: 蜂鸣器短鸣", 
            "N: 蜂鸣器长鸣(1秒)", 
            "M: 蜂鸣器停止", 
            "L: LED开/关切换",
            "P: LED常亮",
            "C: 初次校准"
        ]
        
        for instr in peripheral_instructions:
            label = tk.Label(peripheral_frame, text=instr, font=("Arial", 11), bg="#f0f0f0")
            label.pack(anchor="w", padx=5, pady=2)
        
        # 当前状态显示框
        status_frame = tk.LabelFrame(self.root, text="当前状态", font=("Arial", 12), bg="#f0f0f0")
        status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        # 小车状态
        car_status_frame = tk.Frame(status_frame, bg="#f0f0f0")
        car_status_frame.pack(fill=tk.X, pady=3)
        
        tk.Label(car_status_frame, text="小车状态:", font=("Arial", 11), bg="#f0f0f0").pack(side=tk.LEFT, padx=5)
        self.car_status_label = tk.Label(car_status_frame, text="停止", font=("Arial", 11), fg="blue", bg="#f0f0f0")
        self.car_status_label.pack(side=tk.LEFT, padx=5)
        
        # 伺服状态
        servo_status_frame = tk.Frame(status_frame, bg="#f0f0f0")
        servo_status_frame.pack(fill=tk.X, pady=3)
        
        tk.Label(servo_status_frame, text="伺服状态:", font=("Arial", 11), bg="#f0f0f0").pack(side=tk.LEFT, padx=5)
        self.servo_status_label = tk.Label(servo_status_frame, text="水平: 90°, 垂直: 90°", 
                                         font=("Arial", 11), fg="blue", bg="#f0f0f0")
        self.servo_status_label.pack(side=tk.LEFT, padx=5)
        
        # 外设状态
        peripheral_status_frame = tk.Frame(status_frame, bg="#f0f0f0")
        peripheral_status_frame.pack(fill=tk.X, pady=3)
        
        tk.Label(peripheral_status_frame, text="外设状态:", font=("Arial", 11), bg="#f0f0f0").pack(side=tk.LEFT, padx=5)
        self.peripheral_status_label = tk.Label(peripheral_status_frame, text="蜂鸣器: 关闭, LED: 关闭", 
                                         font=("Arial", 11), fg="blue", bg="#f0f0f0")
        self.peripheral_status_label.pack(side=tk.LEFT, padx=5)
        
        # 提示信息
        note_label = tk.Label(self.root, text="注意: 窗口必须保持焦点才能接收键盘输入", 
                              font=("Arial", 10, "italic"), fg="red", bg="#f0f0f0")
        note_label.pack(side=tk.BOTTOM, pady=5)
    
    def on_key_press(self, event):
        key = event.keysym.lower()
        # 车辆控制键
        if key in ['w', 'a', 's', 'd', 'q', 'e']:
            self.key_pressed.add(key)
        # 伺服控制键
        elif key == 'k':  # 向上调整角度
            self.servo_controller.adjust_tilt_angle(-self.servo_increment)  # 注意方向调整
            self.update_servo_status()
        elif key == 'h':  # 向下调整角度
            self.servo_controller.adjust_tilt_angle(self.servo_increment)   # 注意方向调整
            self.update_servo_status()
        elif key == 'u':  # 向左调整角度
            self.servo_controller.adjust_pan_angle(self.servo_increment)
            self.update_servo_status()
        elif key == 'j':  # 向右调整角度
            self.servo_controller.adjust_pan_angle(-self.servo_increment)
            self.update_servo_status()
        # 外设控制键
        elif key == 'b':  # 蜂鸣器短鸣
            self.peripherals.buzzer_on(440, 0.2)  # 440Hz，持续0.2秒
            self.update_peripheral_status("蜂鸣器: 短鸣")
        elif key == 'n':  # 蜂鸣器长鸣
            self.peripherals.buzzer_on(440, 1.0)  # 440Hz，持续1秒
            self.update_peripheral_status("蜂鸣器: 长鸣")
        elif key == 'm':  # 蜂鸣器停止
            self.peripherals.buzzer_off()
            self.update_peripheral_status("蜂鸣器: 关闭")
        elif key == 'l':  # LED切换
            self.peripherals.led_toggle()
            led_status = "开启" if self.peripherals.led_state else "关闭"
            self.update_peripheral_status(f"LED: {led_status}")
        elif key == 'p':  # LED常亮
            self.peripherals.led_on()
            self.update_peripheral_status("LED: 开启")
        elif key == 'v':  # 胜利信号
            self.is_winner = True
            self.peripherals.buzzer_on(440, 2.0)  # 2秒鸣叫
            self.peripherals.led_on()
            self.update_peripheral_status("蜂鸣器: 长鸣, LED: 开启")
        elif key == 'c':  # 初次校准
            self.calibrate()
            self.is_calibrated = True
            self.update_peripheral_status("正在校准...")
        elif key == 'escape':
            self.on_closing()
    
    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.key_pressed:
            self.key_pressed.remove(key)
        elif key == 'v' and self.is_winner:
            # 胜利信号结束
            self.peripherals.buzzer_off()
            self.peripherals.led_off()
            self.update_peripheral_status("蜂鸣器: 关闭, LED: 关闭")
            self.is_winner = False
        elif key == 'c' and self.is_calibrated:
            # 校准结束
            self.is_calibrated = False
            self.update_peripheral_status("校准完成")
    
    def calibrate(self):
        """执行初次校准流程"""
        for _ in range(3):
            self.peripherals.buzzer_on(440, 0.2)
            self.peripherals.led_on()
            time.sleep(0.2)
            self.peripherals.buzzer_off()
            self.peripherals.led_off()
            time.sleep(0.2)
    
    def update_servo_status(self):
        """更新伺服状态显示"""
        self.servo_status_label.config(text=f"水平: {self.servo_controller.pan_angle}°, 垂直: {self.servo_controller.tilt_angle}°")
    
    def update_car_status(self, status):
        """更新小车状态显示"""
        self.car_status_label.config(text=status)
    
    def update_peripheral_status(self, status):
        """更新外设状态显示"""
        self.peripheral_status_label.config(text=status)
    
    def control_loop(self):
        """处理按键和控制小车的循环"""
        while self.running:
            if not self.key_pressed:
                # 没有按键按下，停止小车
                self.car.stop()
                self.update_car_status("停止")
            elif 'q' in self.key_pressed and 'e' not in self.key_pressed:
                # 逆时针旋转
                self.car.rotate(-90, 100)  # 逆时针旋转90度，速度100%
                self.update_car_status("逆时针旋转")
            elif 'e' in self.key_pressed and 'q' not in self.key_pressed:
                # 顺时针旋转
                self.car.rotate(90, 100)  # 顺时针旋转90度，速度100%
                self.update_car_status("顺时针旋转")
            elif 'w' in self.key_pressed and 's' not in self.key_pressed:
                if 'a' in self.key_pressed and 'd' not in self.key_pressed:
                    # 左前
                    self.car.move(135, 50)
                    self.update_car_status("左前")
                elif 'd' in self.key_pressed and 'a' not in self.key_pressed:
                    # 右前
                    self.car.move(45, 50)
                    self.update_car_status("右前")
                else:
                    # 前进
                    self.car.move(90, 100)  # 前进，速度100%
                    self.update_car_status("前进")
            elif 's' in self.key_pressed and 'w' not in self.key_pressed:
                if 'a' in self.key_pressed and 'd' not in self.key_pressed:
                    # 左后
                    self.car.move(225, 50)
                    self.update_car_status("左后")
                elif 'd' in self.key_pressed and 'a' not in self.key_pressed:
                    # 右后
                    self.car.move(315, 50)
                    self.update_car_status("右后")
                else:
                    # 后退
                    self.car.move(270, 100)  # 后退，速度100%
                    self.update_car_status("后退")
            elif 'a' in self.key_pressed and 'd' not in self.key_pressed:
                # 左移
                self.car.move(0, 100)  # 左移，速度100%
                self.update_car_status("左移")
            elif 'd' in self.key_pressed and 'a' not in self.key_pressed:
                # 右移
                self.car.move(180, 100)  # 右移，速度100%
                self.update_car_status("右移")
            
            # 短暂延时，减少CPU使用
            time.sleep(0.05)
    
    def on_closing(self):
        """窗口关闭时的清理"""
        self.running = False
        self.car.stop()
        time.sleep(0.1)  # 等待控制线程停止
        
        # 清理外设资源
        self.peripherals.cleanup()
        
        # 清理舵机资源
        self.servo_controller.cleanup()
        
        # 将所有引脚输出设置为 0
        GPIO.cleanup()
        
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MecanumCarController(root)
    root.mainloop()