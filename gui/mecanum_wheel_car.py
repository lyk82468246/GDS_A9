# mecanum_wheel_car.py
import RPi.GPIO as GPIO
import time
import math

class MecanumCar:
    def __init__(self, pins):
        """
        初始化麦轮小车
        
        参数:
        pins: 字典，包含四个电机的引脚配置
              格式: {
                'front_left': {'in1': pin1, 'in2': pin2, 'encoder_a': pinA},
                'front_right': {'in1': pin3, 'in2': pin4, 'encoder_a': pinB},
                'rear_left': {'in1': pin5, 'in2': pin6, 'encoder_a': pinC},
                'rear_right': {'in1': pin7, 'in2': pin8, 'encoder_a': pinD}
              }
        """
        self.pins = pins
        self.motors = {}
        self.encoder_counts = {'front_left': 0, 'front_right': 0, 'rear_left': 0, 'rear_right': 0}
        self.motor_speeds = {'front_left': 0, 'front_right': 0, 'rear_left': 0, 'rear_right': 0}
        
        # 设置GPIO模式
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 初始化各个电机
        for motor_name, motor_pins in self.pins.items():
            # 设置控制引脚为输出
            GPIO.setup(motor_pins['in1'], GPIO.OUT)
            GPIO.setup(motor_pins['in2'], GPIO.OUT)
            
            # 创建PWM对象，频率设为1000Hz
            pwm1 = GPIO.PWM(motor_pins['in1'], 1000)
            pwm2 = GPIO.PWM(motor_pins['in2'], 1000)
            pwm1.start(0)
            pwm2.start(0)
            
            # 设置编码器引脚为输入
            GPIO.setup(motor_pins['encoder_a'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # 配置编码器中断
            GPIO.add_event_detect(motor_pins['encoder_a'], GPIO.RISING, 
                                 callback=lambda channel, name=motor_name: self._encoder_callback(channel, name))
            
            self.motors[motor_name] = {
                'pwm1': pwm1,  # 正转PWM
                'pwm2': pwm2,  # 反转PWM
                'current_speed': 0
            }
    
    def __del__(self):
        """析构函数，清理GPIO"""
        self.stop()
        # 移除事件检测
        for motor_name, motor_pins in self.pins.items():
            GPIO.remove_event_detect(motor_pins['encoder_a'])
        GPIO.cleanup()
    
    def _encoder_callback(self, channel, motor_name):
        """编码器回调函数，用于累计脉冲计数"""
        self.encoder_counts[motor_name] += 1
    
    def get_motor_speeds(self):
        """获取当前电机速度"""
        return self.motor_speeds
    
    def _set_motor(self, motor_name, speed):
        """
        设置单个电机的转速和方向
        
        参数:
        motor_name: 电机名称
        speed: 速度值，-100到100，负值表示反转
        """
        motor = self.motors[motor_name]
        abs_speed = abs(speed)
        
        if speed > 0:  # 正转
            motor['pwm1'].ChangeDutyCycle(abs_speed)  # 给in1送PWM波
            motor['pwm2'].ChangeDutyCycle(0)
        elif speed < 0:  # 反转
            motor['pwm1'].ChangeDutyCycle(0)
            motor['pwm2'].ChangeDutyCycle(abs_speed)  # 给in2送PWM波
        else:  # 停止
            motor['pwm1'].ChangeDutyCycle(0)
            motor['pwm2'].ChangeDutyCycle(0)
        
        # 更新电机当前速度
        motor['current_speed'] = speed
        self.motor_speeds[motor_name] = speed
    
    def stop(self):
        """停止所有电机"""
        for motor_name in self.motors:
            self._set_motor(motor_name, 0)
        print("动作: 停止")
        print(f"电机速度: {self.get_motor_speeds()}")
    
    def move(self, angle, speed_percent):
        """
        控制小车按指定角度方向平动
        
        参数:
        angle: 移动方向角度（度），0表示向右，90表示向前，以此类推
        speed_percent: 速度百分比，0-100
        """
        # 将角度转换为弧度
        angle_rad = math.radians(angle)
        
        # 计算x和y分量
        x_component = math.cos(angle_rad)
        y_component = math.sin(angle_rad)
        
        # 根据X型麦轮运动学计算各个轮子的速度
        front_left_speed = y_component + x_component
        front_right_speed = y_component - x_component
        rear_left_speed = y_component - x_component
        rear_right_speed = y_component + x_component
        
        # 归一化速度，确保所有速度在-1到1之间，并且保持方向
        max_speed = max(abs(front_left_speed), abs(front_right_speed), 
                        abs(rear_left_speed), abs(rear_right_speed))
        
        if max_speed > 1:
            front_left_speed /= max_speed
            front_right_speed /= max_speed
            rear_left_speed /= max_speed
            rear_right_speed /= max_speed
        
        # 应用速度百分比并设置电机
        front_left_final = front_left_speed * speed_percent
        front_right_final = front_right_speed * speed_percent
        rear_left_final = rear_left_speed * speed_percent
        rear_right_final = rear_right_speed * speed_percent
        
        self._set_motor('front_left', front_left_final)
        self._set_motor('front_right', front_right_final)
        self._set_motor('rear_left', rear_left_final)
        self._set_motor('rear_right', rear_right_final)
        
        # 打印动作和电机速度
        direction_names = {
            0: "向右", 45: "向右前", 90: "向前", 135: "向左前",
            180: "向左", 225: "向左后", 270: "向后", 315: "向右后"
        }
        
        # 找到最接近的方向名称
        closest_angle = min(direction_names.keys(), key=lambda x: abs(x - (angle % 360)))
        direction = direction_names.get(closest_angle, f"{angle}度方向")
        
        print(f"动作: 平动 - {direction}，速度 {speed_percent}%")
        print(f"电机速度: {self.get_motor_speeds()}")
    
    def rotate(self, angle, speed_percent):
        """
        控制小车原地旋转指定角度
        
        参数:
        angle: 旋转角度（度），正值为逆时针，负值为顺时针
        speed_percent: 速度百分比，0-100
        """
        if angle == 0 or speed_percent == 0:
            self.stop()
            return
        
        # 确定旋转方向
        direction = 1 if angle > 0 else -1
        rotation_type = "逆时针" if direction > 0 else "顺时针"
        
        # 设置所有轮子以相同的速度向相同的方向旋转
        motor_speed = direction * speed_percent
        
        # 根据X型麦轮运动学调整旋转速度
        self._set_motor('front_left', -motor_speed)
        self._set_motor('rear_left', motor_speed)
        self._set_motor('front_right', motor_speed)
        self._set_motor('rear_right', -motor_speed)
        
        # 打印动作和电机速度
        print(f"动作: 旋转 - {rotation_type} {abs(angle)}度，速度 {speed_percent}%")
        print(f"电机速度: {self.get_motor_speeds()}")
    
    def read_speeds(self):
        """读取电机实际速度（从编码器）"""
        # 在实际应用中，应该定期调用此函数来计算速度
        # 这里简单演示计算脉冲频率的方法
        last_counts = self.encoder_counts.copy()
        time.sleep(0.1)  # 采样时间
        
        speeds = {}
        for motor_name in self.motors:
            pulse_diff = self.encoder_counts[motor_name] - last_counts[motor_name]
            # 速度单位: 脉冲/秒
            speeds[motor_name] = pulse_diff / 0.1
        
        return speeds