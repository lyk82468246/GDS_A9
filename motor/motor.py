# mecanum_car.py
import RPi.GPIO as GPIO
import time
import math
from enum import Enum

class MecanumCar:
    """麦轮小车控制类 - 支持任意角度平移和精确旋转"""
    
    def __init__(self, 
                 left_front_pins=(17, 18), 
                 right_front_pins=(22, 23), 
                 left_rear_pins=(24, 25), 
                 right_rear_pins=(27, 4),
                 pwm_pins=(12, 13, 19, 26),
                 encoder_pins=(5, 6, 16, 20),
                 pwm_freq=100):
        """
        初始化麦轮小车
        
        参数:
            left_front_pins: 左前轮电机控制引脚 (IN1, IN2)
            right_front_pins: 右前轮电机控制引脚 (IN1, IN2)
            left_rear_pins: 左后轮电机控制引脚 (IN1, IN2)
            right_rear_pins: 右后轮电机控制引脚 (IN1, IN2)
            pwm_pins: 四个轮子的PWM控制引脚 (左前, 右前, 左后, 右后)
            encoder_pins: 四个轮子的编码器引脚 (左前, 右前, 左后, 右后)
            pwm_freq: PWM频率
        """
        # 设置GPIO模式
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 电机控制引脚
        self.left_front_pins = left_front_pins
        self.right_front_pins = right_front_pins
        self.left_rear_pins = left_rear_pins
        self.right_rear_pins = right_rear_pins
        
        # PWM控制引脚和对象
        self.pwm_pins = pwm_pins
        self.pwm_freq = pwm_freq
        self.pwm_objects = []
        
        # 编码器引脚
        self.encoder_pins = encoder_pins
        self.pulse_counts = [0, 0, 0, 0]  # 四个轮子的脉冲计数
        self.previous_pulse_counts = [0, 0, 0, 0]  # 上一次的脉冲计数
        self.wheel_speeds = [0, 0, 0, 0]  # 四个轮子的速度 (每秒脉冲数)
        
        # PID控制参数
        self.kp = 0.5  # 比例系数
        self.ki = 0.2  # 积分系数
        self.kd = 0.1  # 微分系数
        self.pid_errors = [[0, 0, 0] for _ in range(4)]  # 每个轮子的[当前误差, 累积误差, 误差变化]
        self.target_speeds = [0, 0, 0, 0]  # 目标速度
        
        # 旋转控制相关参数
        self.current_angle = 0.0    # 当前车身角度(相对模式下始终为0)
        self.target_angle = 0.0     # 目标旋转角度(相对于当前角度)
        self.absolute_angle = 0.0   # 绝对角度(仅供内部计算使用)
        self.is_rotating = False  # 是否正在旋转
        self.rotation_speed = 30  # 默认旋转速度百分比
        self.rotation_pid = [0.8, 0.3, 0.2]  # 旋转PID参数 [kp, ki, kd]
        self.rotation_error = [0, 0, 0]  # 旋转PID误差 [当前误差, 累积误差, 误差变化]
        self._target_absolute_angle = 0.0  # 目标绝对角度
        
        # 车轮物理参数
        self.wheel_circumference = 20.0  # 车轮周长(cm)
        self.pulses_per_revolution = 20  # 每转一圈的脉冲数
        self.wheel_base = 15.0    # 轮距(cm)
        self.wheel_track = 15.0   # 轴距(cm)
        
        # 初始化所有GPIO引脚
        self._setup_gpio()
        
        # 启动速度监测线程
        self.speed_monitoring_active = True
        import threading
        self.speed_monitor_thread = threading.Thread(target=self._monitor_speed)
        self.speed_monitor_thread.daemon = True
        self.speed_monitor_thread.start()
    
    def _setup_gpio(self):
        """设置所有GPIO引脚"""
        # 设置电机控制引脚为输出
        for pins in [self.left_front_pins, self.right_front_pins, 
                    self.left_rear_pins, self.right_rear_pins]:
            GPIO.setup(pins[0], GPIO.OUT)
            GPIO.setup(pins[1], GPIO.OUT)
        
        # 设置PWM引脚并创建PWM对象
        for pin in self.pwm_pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, self.pwm_freq)
            pwm.start(0)
            self.pwm_objects.append(pwm)
        
        # 设置编码器引脚为输入，上拉
        for pin in self.encoder_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            # 添加上升沿检测的事件回调
            index = self.encoder_pins.index(pin)
            GPIO.add_event_detect(pin, GPIO.RISING, 
                                 callback=lambda channel, idx=index: self._pulse_callback(idx))
    
    def _pulse_callback(self, wheel_index):
        """编码器脉冲回调函数"""
        self.pulse_counts[wheel_index] += 1
    
    def _monitor_speed(self):
        """监测轮子速度的线程函数"""
        last_time = time.time()
        
        while self.speed_monitoring_active:
            current_time = time.time()
            elapsed_time = current_time - last_time
            
            # 记录当前脉冲计数
            current_counts = self.pulse_counts.copy()
            
            # 计算速度 (脉冲/秒)
            for i in range(4):
                pulses = current_counts[i] - self.previous_pulse_counts[i]
                self.wheel_speeds[i] = pulses / elapsed_time  # 脉冲/秒
                self.previous_pulse_counts[i] = current_counts[i]
            
            # 更新车身角度 (基于车轮转速差计算)
            if self.is_rotating:
                self._update_angle(elapsed_time)
            
            # 应用PID控制
            if self.is_rotating:
                self._apply_rotation_pid_control()
            else:
                self._apply_pid_control()
            
            last_time = current_time
            # 每100ms更新一次
            time.sleep(0.1)
    
    def _update_angle(self, elapsed_time):
        """根据轮子速度更新当前角度"""
        # 计算左右轮平均速度
        left_speed = (self.wheel_speeds[0] + self.wheel_speeds[2]) / 2  # 左前+左后
        right_speed = (self.wheel_speeds[1] + self.wheel_speeds[3]) / 2  # 右前+右后
        
        # 计算角速度 (弧度/秒)
        # 轮子转速差转换为车身角速度
        # 假设左轮正转为正方向，右轮正转为负方向
        wheel_diff = left_speed - (-right_speed)  # 左右轮速度差
        
        # 计算车身角速度 (弧度/秒)
        angular_velocity = (wheel_diff * self.wheel_circumference) / (self.pulses_per_revolution * (self.wheel_track + self.wheel_base))
        
        # 更新绝对角度
        angle_change = angular_velocity * elapsed_time
        self.absolute_angle = (self.absolute_angle + math.degrees(angle_change)) % 360
        
        # 不更新相对角度，因为在相对模式下，当前角度始终为0
    
    def _apply_pid_control(self):
        """应用PID控制调整轮子速度"""
        for i in range(4):
            # 如果目标速度为0，直接设置为0
            if self.target_speeds[i] == 0:
                self._set_motor_speed(i, 0)
                self.pid_errors[i] = [0, 0, 0]  # 重置PID误差
                continue
                
            # 计算当前误差
            error = self.target_speeds[i] - self.wheel_speeds[i]
            
            # 更新积分误差 (限制积分误差大小防止积分饱和)
            self.pid_errors[i][1] += error
            self.pid_errors[i][1] = max(-100, min(100, self.pid_errors[i][1]))
            
            # 计算微分误差
            diff_error = error - self.pid_errors[i][0]
            
            # 计算PID输出
            pid_output = (self.kp * error + 
                          self.ki * self.pid_errors[i][1] + 
                          self.kd * diff_error)
            
            # 更新当前误差
            self.pid_errors[i][0] = error
            
            # 获取当前速度百分比
            current_speed = self.pwm_objects[i].GetDutyCycle()
            
            # 调整速度 (限制在0-100范围内)
            new_speed = max(0, min(100, current_speed + pid_output))
            
            # 应用新速度
            if current_speed != new_speed:
                self._set_motor_speed(i, new_speed)
    
    def _apply_rotation_pid_control(self):
        """应用PID控制调整旋转速度以达到目标角度"""
        # 计算角度误差 (考虑最短路径)
        error = self._get_angle_error(self.absolute_angle, self._target_absolute_angle)
        
        # 如果误差很小，停止旋转并更新绝对角度
        if abs(error) < 2.0:  # 允许2度的误差
            self.stop()
            self.is_rotating = False
            self.absolute_angle = self._target_absolute_angle  # 更新绝对角度
            self.current_angle = 0.0  # 重置相对角度为0
            self.rotation_error = [0, 0, 0]  # 重置旋转PID误差
            return
        
        # 更新积分误差 (限制积分误差大小防止积分饱和)
        self.rotation_error[1] += error
        self.rotation_error[1] = max(-90, min(90, self.rotation_error[1]))
        
        # 计算微分误差
        diff_error = error - self.rotation_error[0]
        
        # 计算PID输出
        pid_output = (self.rotation_pid[0] * error + 
                      self.rotation_pid[1] * self.rotation_error[1] + 
                      self.rotation_pid[2] * diff_error)
        
        # 更新当前误差
        self.rotation_error[0] = error
        
        # 根据PID输出确定旋转速度和方向
        # 负误差表示需要顺时针旋转，正误差表示需要逆时针旋转
        speed = min(self.rotation_speed, abs(pid_output))
        
        # 设置所有电机旋转方向
        if error > 0:  # 需要逆时针旋转
            self._set_motor_direction(self.left_front_pins, False)
            self._set_motor_direction(self.right_front_pins, True)
            self._set_motor_direction(self.left_rear_pins, False)
            self._set_motor_direction(self.right_rear_pins, True)
        else:  # 需要顺时针旋转
            self._set_motor_direction(self.left_front_pins, True)
            self._set_motor_direction(self.right_front_pins, False)
            self._set_motor_direction(self.left_rear_pins, True)
            self._set_motor_direction(self.right_rear_pins, False)
        
        # 设置所有电机速度
        for i in range(4):
            self._set_motor_speed(i, speed)
    
    def _get_angle_error(self, current, target):
        """计算角度误差，考虑最短路径"""
        error = target - current
        
        # 处理跨越0/360度边界的情况
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        return error
    
    def _set_motor_direction(self, pins, forward=True):
        """
        设置电机方向
        
        参数:
            pins: 电机控制引脚 (IN1, IN2)
            forward: 是否正向旋转
        """
        if forward:
            GPIO.output(pins[0], GPIO.HIGH)
            GPIO.output(pins[1], GPIO.LOW)
        else:
            GPIO.output(pins[0], GPIO.LOW)
            GPIO.output(pins[1], GPIO.HIGH)
    
    def _set_motor_speed(self, motor_index, speed_percent):
        """
        设置电机速度
        
        参数:
            motor_index: 电机索引 (0-3)
            speed_percent: 速度百分比 (0-100)
        """
        # 限制速度百分比在0-100之间
        speed_percent = max(0, min(100, speed_percent))
        self.pwm_objects[motor_index].ChangeDutyCycle(speed_percent)
    
    def calibrate_angle(self, angle=0):
        """
        校准当前角度
        
        参数:
            angle: 设置当前绝对角度值
        """
        self.absolute_angle = angle % 360
        self.current_angle = 0.0  # 相对角度始终为0
    
    def move(self, angle, speed_percent):
        """
        控制小车从当前位置按指定角度平移
        
        参数:
            angle: 平移角度 (0-359度，基于当前朝向，0为向右，90为向前，180为向左，270为向后)
            speed_percent: 速度百分比 (0-100)
        """
        # 标准化角度到0-359范围
        angle = angle % 360
        
        # 停止旋转模式
        self.is_rotating = False
        
        # 将相对角度转换为绝对角度（考虑当前车身朝向）
        absolute_move_angle = (angle + self.absolute_angle) % 360
        
        # 将角度转换为弧度
        angle_rad = math.radians(absolute_move_angle)
        
        # 计算x和y分量 (极坐标转直角坐标)
        # x分量: 负表示左，正表示右
        # y分量: 负表示后，正表示前
        x_component = math.cos(angle_rad)
        y_component = math.sin(angle_rad)
        
        # 计算每个轮子的速度和方向
        # 麦轮运动学方程:
        # 左前 = +y +x
        # 右前 = +y -x
        # 左后 = +y -x
        # 右后 = +y +x
        
        # 计算每个轮子的速度比率 (-1.0 到 1.0)
        left_front_speed = y_component + x_component
        right_front_speed = y_component - x_component
        left_rear_speed = y_component - x_component
        right_rear_speed = y_component + x_component
        
        # 归一化速度，确保最大值不超过1.0
        max_speed = max(abs(left_front_speed), abs(right_front_speed), 
                        abs(left_rear_speed), abs(right_rear_speed), 1.0)
        
        left_front_speed /= max_speed
        right_front_speed /= max_speed
        left_rear_speed /= max_speed
        right_rear_speed /= max_speed
        
        # 转换为实际速度值
        speeds = [
            left_front_speed * speed_percent,
            right_front_speed * speed_percent,
            left_rear_speed * speed_percent,
            right_rear_speed * speed_percent
        ]
        
        # 设置电机方向和速度
        self._set_motor_direction(self.left_front_pins, left_front_speed >= 0)
        self._set_motor_direction(self.right_front_pins, right_front_speed >= 0)
        self._set_motor_direction(self.left_rear_pins, left_rear_speed >= 0)
        self._set_motor_direction(self.right_rear_pins, right_rear_speed >= 0)
        
        # 设置速度
        self._set_motor_speed(0, abs(speeds[0]))
        self._set_motor_speed(1, abs(speeds[1]))
        self._set_motor_speed(2, abs(speeds[2]))
        self._set_motor_speed(3, abs(speeds[3]))
        
        # 设置目标速度 (用于PID控制)
        target_pulse_rate = 10  # 假设最大速度大约为每100ms 10个脉冲
        for i in range(4):
            self.target_speeds[i] = abs(speeds[i]) * target_pulse_rate
    
    def rotate(self, angle, speed_percent=None):
        """
        控制小车从当前角度旋转指定角度
        
        参数:
            angle: 旋转角度 (正值逆时针，负值顺时针)
            speed_percent: 旋转速度百分比 (0-100)，默认使用self.rotation_speed
        """
        # 计算目标绝对角度
        self.target_angle = angle  # 相对旋转角度
        target_absolute_angle = (self.absolute_angle + angle) % 360
        
        # 设置旋转速度
        if speed_percent is not None:
            self.rotation_speed = max(1, min(100, speed_percent))
        
        # 进入旋转模式
        self.is_rotating = True
        
        # 初始旋转方向设置
        error = self._get_angle_error(self.absolute_angle, target_absolute_angle)
        
        # 如果误差很小，不需要旋转
        if abs(error) < 2.0:
            self.stop()
            self.is_rotating = False
            return
        
        # 设置旋转方向
        if error > 0:  # 需要逆时针旋转
            self._set_motor_direction(self.left_front_pins, False)
            self._set_motor_direction(self.right_front_pins, True)
            self._set_motor_direction(self.left_rear_pins, False)
            self._set_motor_direction(self.right_rear_pins, True)
        else:  # 需要顺时针旋转
            self._set_motor_direction(self.left_front_pins, True)
            self._set_motor_direction(self.right_front_pins, False)
            self._set_motor_direction(self.left_rear_pins, True)
            self._set_motor_direction(self.right_rear_pins, False)
        
        # 设置初始旋转速度
        initial_speed = min(self.rotation_speed, abs(error) * 0.5)  # 误差越大，初始速度越大
        for i in range(4):
            self._set_motor_speed(i, initial_speed)
            
        # 储存目标绝对角度，用于PID控制
        self._target_absolute_angle = target_absolute_angle
    
    def wait_for_rotation_complete(self, timeout=None):
        """
        等待旋转完成
        
        参数:
            timeout: 超时时间(秒)，超过该时间将停止等待
        
        返回:
            bool: 旋转是否成功完成
        """
        # 如果旋转速度设为100%，不等待直接返回
        if self.rotation_speed == 100:
            return True
            
        start_time = time.time()
        
        while self.is_rotating:
            if timeout is not None and (time.time() - start_time) > timeout:
                self.stop()  # 超时停止
                return False
            
            time.sleep(0.1)
        
        return True
    
    def stop(self):
        """停止小车"""
        # 将所有电机速度设为0
        for i in range(4):
            self._set_motor_speed(i, 0)
            self.target_speeds[i] = 0
            self.pid_errors[i] = [0, 0, 0]  # 重置PID误差
        
        # 退出旋转模式
        self.is_rotating = False
        self.rotation_error = [0, 0, 0]  # 重置旋转PID误差
    
    def get_wheel_speeds(self):
        """获取四个轮子的当前速度"""
        return self.wheel_speeds.copy()
    
    def get_current_angle(self):
        """获取当前相对角度（始终为0）"""
        return 0.0
    
    def get_absolute_angle(self):
        """获取绝对角度（内部跟踪用）"""
        return self.absolute_angle
    
    def tune_pid(self, kp=None, ki=None, kd=None):
        """调整PID参数"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        
        return {'kp': self.kp, 'ki': self.ki, 'kd': self.kd}
    
    def tune_rotation_pid(self, kp=None, ki=None, kd=None):
        """调整旋转PID参数"""
        if kp is not None:
            self.rotation_pid[0] = kp
        if ki is not None:
            self.rotation_pid[1] = ki
        if kd is not None:
            self.rotation_pid[2] = kd
        
        return {'kp': self.rotation_pid[0], 'ki': self.rotation_pid[1], 'kd': self.rotation_pid[2]}
    
    def set_wheel_parameters(self, circumference=None, pulses_per_rev=None, wheel_base=None, wheel_track=None):
        """
        设置车轮物理参数
        
        参数:
            circumference: 车轮周长(cm)
            pulses_per_rev: 每转一圈的脉冲数
            wheel_base: 轮距(cm)
            wheel_track: 轴距(cm)
        """
        if circumference is not None:
            self.wheel_circumference = circumference
        if pulses_per_rev is not None:
            self.pulses_per_revolution = pulses_per_rev
        if wheel_base is not None:
            self.wheel_base = wheel_base
        if wheel_track is not None:
            self.wheel_track = wheel_track
    
    def cleanup(self):
        """清理GPIO资源"""
        # 停止速度监测线程
        self.speed_monitoring_active = False
        time.sleep(0.2)  # 等待线程结束
        
        # 停止所有PWM
        for pwm in self.pwm_objects:
            pwm.stop()
        
        # 清理GPIO
        GPIO.cleanup()