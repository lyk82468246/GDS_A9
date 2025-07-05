#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import serial
import threading
import bisect
import time
import math
import RPi.GPIO as GPIO
from collections import defaultdict

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
        self._set_motor('rear_left', -motor_speed)
        self._set_motor('front_right', motor_speed)
        self._set_motor('rear_right', motor_speed)
        
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

class StableLidarProcessor:
    def __init__(self, port='/dev/ttyAMA2', baudrate=230400):
        self.ser = serial.Serial(port, baudrate, timeout=5)
        self._scan_dict = dict.fromkeys(range(360), (0, 0))
        self._raw_points = []
        self._last_angle = 0
        self._scan_started = False
        self._lock = threading.Lock()
        self._running = True

        # 调试计数器
        self._valid_point_count = 0 
        self._total_point_count = 0

        self._thread = threading.Thread(target=self._process_data)
        self._thread.daemon = True
        self._thread.start()

    def _parse_frame(self, data):
        try:
            # 调试输出原始数据头
            # print(f"Raw header: {data[0:4].hex()}")

            # 修正解析逻辑
            start = (int.from_bytes(data[2:4], byteorder='little')) / 100.0
            end = (int.from_bytes(data[40:42], byteorder='little')) / 100.0

            points = []
            for i in range(12):
                offset = 4 + i*3
                # 确保不会越界
                if offset+2 >= len(data):
                    break
                
                # 修正距离解析方式
                dist_low = data[offset]
                dist_high = data[offset+1]
                distance = (dist_high << 8) | dist_low  # 手动组合高低字节
                intensity = data[offset+2]

                # 调试输出原始距离值
                # print(f"Point {i}: dist_bytes={[dist_low, dist_high]} -> {distance}")

                if distance > 0:
                    angle_diff = end - start if start <= end else (360 - start) + end
                    angle = (start + (angle_diff / 11) * i) % 360
                    points.append((round(angle, 2), distance, intensity))
                    self._valid_point_count += 1
                self._total_point_count += 1

            return {
                'start': start,
                'end': end,
                'points': points
            }
        except Exception as e:
            print(f"解析异常: {str(e)}")
            return {'start':0, 'end':0, 'points':[]}

    def _process_data(self):
        """使用滑动窗口方式处理串口数据，提高帧同步可靠性"""
        buffer = bytearray()
        while self._running:
            try:
                # 读取可用数据到缓冲区
                new_data = self.ser.read(max(1, self.ser.in_waiting))
                if not new_data:
                    time.sleep(0.001)  # 短暂休眠避免CPU占用过高
                    continue
                
                buffer.extend(new_data)
            
                # 使用滑动窗口查找帧头并处理
                while len(buffer) >= 47:  # 帧头(2字节) + 数据(45字节)
                    if buffer[0:2] == b'\x54\x2C':
                        # 找到有效帧头
                        frame_data = buffer[0:47]
                        del buffer[0:47]  # 移除已处理的数据
                    
                        # 处理帧数据
                        data = frame_data[2:]  # 跳过帧头
                        frame = self._parse_frame(data)

                        # 原有的处理逻辑
                        if frame['start'] < 5 and not self._scan_started:
                            self._scan_started = True
                            self._raw_points = []
                            print("--- 开始新扫描 ---")

                        if self._scan_started:
                            self._raw_points.extend(frame['points'])

                            if self._last_angle > 355 and frame['start'] < 5:
                                self._scan_started = False
                                self._generate_scan_dict()
                                print("--- 完成扫描 ---")
                                # 重置调试计数器
                                self._valid_point_count = 0
                                self._total_point_count = 0

                        self._last_angle = frame['end']
                    else:
                        # 无效帧头，只移除一个字节，继续查找
                        del buffer[0]
                    
                # 缓冲区管理：如果缓冲区过大但没找到帧头，清理一部分旧数据
                if len(buffer) > 1024:
                    del buffer[0:512]  # 删除前半部分，保留后半部分继续查找
                
            except Exception as e:
                print(f"处理异常: {str(e)}")
                # 出现异常时清空缓冲区，重新开始
                buffer.clear()
                time.sleep(0.1)

    def _generate_scan_dict(self):
        if not self._raw_points:
            print("警告: 无有效数据点")
            return

        sorted_points = sorted(self._raw_points, key=lambda x: x[0])
        angles = [p[0] for p in sorted_points]
        
        final_dict = {}
        
        for target_angle in range(360):
            idx = bisect.bisect_left(angles, target_angle)
            candidates = []
            
            # 向前后各扩展2度的范围搜索
            search_range = 2
            start_idx = max(0, idx - search_range)
            end_idx = min(len(sorted_points), idx + search_range)
            
            for p in sorted_points[start_idx:end_idx]:
                angle_diff = abs(p[0] - target_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                if angle_diff <= search_range:
                    candidates.append(p)

            if candidates:
                # 选择最近的三个点取中位数
                distances = sorted([p[1] for p in candidates])
                median_dist = distances[len(distances)//2]
                final_dict[target_angle] = (median_dist, 50)  # 示例强度值
            else:
                final_dict[target_angle] = (0, 0)

        with self._lock:
            self._scan_dict = final_dict
            # 调试输出统计
            non_zero = sum(1 for v in final_dict.values() if v[0] > 0)
            print(f"更新字典: 总点数 {len(final_dict)} 有效点 {non_zero}")

    @property
    def scan_data(self):
        with self._lock:
            return self._scan_dict.copy()

    def shutdown(self):
        self._running = False
        self.ser.close()

    def print_full_scan(self, max_lines=10):
        """打印完整扫描数据（默认显示首尾各5个点）"""
        data = self.scan_data
        if not data:
            print("无有效扫描数据")
            return
    
        # 按角度排序
        sorted_angles = sorted(data.keys())
        total_points = len(sorted_angles)
    
        print(f"\n=== 完整扫描数据（共{total_points}个点） ===")
    
        # 打印前5个点
        for angle in sorted_angles[:max_lines]:
            dist, intens = data[angle]
            print(f"{angle:03d}° | 距离: {dist:4}mm | 强度: {intens:3}")

        # 统计信息
        valid_points = sum(1 for v in data.values() if v[0] > 0)
        print(f"有效点比例: {valid_points/360:.1%}")

    def save_scan_to_csv(self, filename="scan_data.csv"):
        """保存扫描数据到CSV文件"""
        import csv
        data = self.scan_data
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['角度', '距离(mm)', '强度'])
            for angle in sorted(data.keys()):
                writer.writerow([angle, data[angle][0], data[angle][1]])
        print(f"数据已保存至 {filename}")

class LidarControlledCar:
    def __init__(self, lidar_port='/dev/ttyAMA2', lidar_baudrate=230400, motor_pins=None):
        """
        初始化带激光雷达的麦轮小车
        
        参数:
        lidar_port: 激光雷达串口端口
        lidar_baudrate: 激光雷达波特率
        motor_pins: 电机引脚配置
        """
        # 默认电机引脚配置
        if motor_pins is None:
            motor_pins = {
                'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
                'front_right': {'in1': 5, 'in2': 6, 'encoder_a': 26},
                'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
                'rear_right': {'in1': 21, 'in2': 20, 'encoder_a': 16}
            }
        
        # 初始化LiDAR处理器
        self.lidar = StableLidarProcessor(port=lidar_port, baudrate=lidar_baudrate)
        
        # 初始化麦轮小车
        self.car = MecanumCar(motor_pins)
        
        # 避障参数
        self.min_safe_distance = 300  # 最小安全距离(mm)
        self.is_obstacle_avoiding = False
        self.current_action = None
        
        # 启动避障线程
        self.obstacle_thread = threading.Thread(target=self._obstacle_avoidance_loop)
        self.obstacle_thread.daemon = True
        self.obstacle_thread.start()
    
    def move_forward(self, speed=50):
        """前进"""
        self.current_action = "forward"
        self.car.move(90, speed)
    
    def move_backward(self, speed=50):
        """后退"""
        self.current_action = "backward"
        self.car.move(270, speed)
    
    def turn_left(self, speed=50):
        """左转"""
        self.current_action = "left"
        self.car.rotate(90, speed)
    
    def turn_right(self, speed=50):
        """右转"""
        self.current_action = "right"
        self.car.rotate(-90, speed)
    
    def stop(self):
        """停止"""
        self.current_action = "stop"
        self.car.stop()
    
    def _obstacle_avoidance_loop(self):
        """障碍物检测和避障线程"""
        while True:
            try:
                # 获取雷达扫描数据
                scan_data = self.lidar.scan_data
                
                if not scan_data:
                    time.sleep(0.1)
                    continue
                
                # 如果当前是前进动作，检查前方障碍物
                if self.current_action == "forward":
                    # 检查前方60度范围内的障碍物
                    front_obstacles = False
                    for angle in range(60, 121):  # 60°到120°（前方区域）
                        dist, _ = scan_data.get(angle, (0, 0))
                        if dist > 0 and dist < self.min_safe_distance:
                            front_obstacles = True
                            break
                    
                    # 有障碍物时执行避障
                    if front_obstacles and not self.is_obstacle_avoiding:
                        self.is_obstacle_avoiding = True
                        print(f"检测到前方障碍物，距离 {dist}mm，开始避障")
                        
                        # 停止
                        self.car.stop()
                        time.sleep(0.5)
                        
                        # 检查左右侧空间
                        left_space = min([scan_data.get(a, (10000, 0))[0] for a in range(135, 180) if scan_data.get(a, (0, 0))[0] > 0], default=10000)
                        right_space = min([scan_data.get(a, (10000, 0))[0] for a in range(0, 45) if scan_data.get(a, (0, 0))[0] > 0], default=10000)
                        
                        # 选择空间较大的方向转向
                        if left_space > right_space:
                            print(f"左侧空间较大 ({left_space}mm)，向左转")
                            self.car.rotate(90, 40)  # 左转
                        else:
                            print(f"右侧空间较大 ({right_space}mm)，向右转")
                            self.car.rotate(-90, 40)  # 右转
                        
                        time.sleep(1.0)  # 转向时间
                        self.car.stop()
                        self.is_obstacle_avoiding = False
                
                time.sleep(0.1)
            except Exception as e:
                print(f"避障线程异常: {str(e)}")
                time.sleep(0.5)
    
    def navigate_to_clear_path(self):
        """寻找并导航到最开阔的路径"""
        scan_data = self.lidar.scan_data
        if not scan_data:
            print("无有效雷达数据，无法寻找开阔路径")
            return
        
        # 将360度分成12个扇区，每个扇区30度
        sectors = []
        for i in range(12):
            start_angle = i * 30
            end_angle = (i + 1) * 30
            sector_distances = [scan_data.get(a, (0, 0))[0] for a in range(start_angle, end_angle) 
                              if scan_data.get(a, (0, 0))[0] > 0]
            
            if sector_distances:
                avg_distance = sum(sector_distances) / len(sector_distances)
                sectors.append((start_angle, avg_distance))
        
        # 找出平均距离最大的扇区
        if sectors:
            best_sector = max(sectors, key=lambda x: x[1])
            best_angle = best_sector[0] + 15  # 扇区中间角度
            
            print(f"寻找到最开阔路径: {best_angle}度方向，平均距离 {best_sector[1]:.0f}mm")
            
            # 先旋转到目标方向
            # 计算需要旋转的角度（考虑最短路径）
            current_angle = 90  # 假设车头朝前为90度
            angle_diff = best_angle - current_angle
            
            # 标准化到-180到180度范围
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            
            # 旋转到目标方向
            print(f"旋转 {angle_diff}度 到目标方向")
            self.car.rotate(angle_diff, 40)
            time.sleep(abs(angle_diff) / 90)  # 旋转时间根据角度调整
            
            # 前进
            self.move_forward(40)
        else:
            print("无法找到开阔路径")
    
    def analyze_environment(self):
        """分析环境并打印详细信息"""
        scan_data = self.lidar.scan_data
        if not scan_data:
            print("无有效雷达数据，无法分析环境")
            return
        
        # 将360度分成4个主要方向
        directions = {
            "前侧": (135, 225),
            "后方": (0, 45),
            "左方": (45, 135),
            "右方": (225, 315)
            
        }

        results = {}
        for name, (start, end) in directions.items():
            # 收集该方向上的有效距离
            distances = []
            if start < end:
                angle_range = range(start, end)
            else:
                angle_range = list(range(start, 360)) + list(range(0, end))
            
            for angle in angle_range:
                dist, _ = scan_data.get(angle, (0, 0))
                if dist > 0:
                    distances.append(dist)
            
            if distances:
                min_dist = min(distances)
                avg_dist = sum(distances) / len(distances)
                results[name] = {
                    "最小距离": min_dist,
                    "平均距离": avg_dist,
                    "有效点数": len(distances)
                }
        
        # 打印分析结果
        print("\n===== 环境分析 =====")
        for direction, data in results.items():
            status = "警告！太近" if data["最小距离"] < self.min_safe_distance else "安全"
            print(f"{direction}: 最小距离 {data['最小距离']:.0f}mm ({status}), "
                  f"平均距离 {data['平均距离']:.0f}mm, 有效点数 {data['有效点数']}")
    
    def shutdown(self):
        """关闭资源"""
        self.car.stop()
        self.lidar.shutdown()
        print("系统已关闭")

# 引脚配置示例（根据实际硬件调整）
motor_pins = {
    'front_left': {'in1': 12, 'in2': 16, 'encoder_a': 5},
    'front_right': {'in1': 20, 'in2': 21, 'encoder_a': 6},
    'rear_left': {'in1': 13, 'in2': 19, 'encoder_a': 26},
    'rear_right': {'in1': 17, 'in2': 18, 'encoder_a': 23}
}

# 全局实例
lidar_car = LidarControlledCar(motor_pins=motor_pins)

if __name__ == '__main__':
    try:
        print("启动激光雷达小车系统...")
        print("等待雷达数据稳定...")
        time.sleep(3)  # 等待雷达数据稳定
        
        # 演示控制流程
        print("\n===== 开始演示 =====")
        
        # 分析当前环境
        lidar_car.analyze_environment()
        time.sleep(1)
        
        # 导航到开阔路径
        lidar_car.navigate_to_clear_path()
        time.sleep(3)
        lidar_car.stop()
        time.sleep(1)
        
        # 前进测试（带自动避障）
        print("\n测试前进（带避障）")
        lidar_car.move_forward(40)
        time.sleep(5)  # 让避障逻辑有机会运行
        lidar_car.stop()
        
        # 旋转测试
        print("\n测试原地旋转")
        lidar_car.turn_left(40)
        time.sleep(2)
        lidar_car.stop()
        time.sleep(0.5)
        lidar_car.turn_right(40)
        time.sleep(2)
        lidar_car.stop()
        
        print("\n演示完成")
        
        # 主循环保持程序运行
        while True:
            last_update = id(lidar_car.lidar.scan_data)
            
            # 等待新的扫描数据
            while True:
                current = lidar_car.lidar.scan_data
                if current and id(current) != last_update:
                    # 打印环境分析
                    lidar_car.analyze_environment()
                    last_update = id(current)
                    break
                time.sleep(0.1)
            
            time.sleep(2)  # 每两秒分析一次环境
            
    except KeyboardInterrupt:
        print("\n用户中断，关闭系统")
        lidar_car.shutdown()