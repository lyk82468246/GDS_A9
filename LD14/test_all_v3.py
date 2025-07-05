import bisect
import serial
import re
import time
import threading
from adafruit_servokit import ServoKit
from peripherals import Peripherals
import RPi.GPIO as GPIO
from mecanum_wheel_car import MecanumCar

# 初始化舵机控制器
kit = ServoKit(channels=16)

# 初始化外设控制器
peripherals = Peripherals()

# 配置串口
# 注意：可能需要根据实际情况修改串口设备名和波特率
ser = serial.Serial('/dev/ttyCH343USB0', 115200, timeout=1)

# 正则表达式匹配数据格式: $+123-456$
pattern = r'\$([+-]\d{3})([+-]\d{3})\$'

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
        """使用滑动窗口方式处理串口数据,提高帧同步可靠性"""
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
                        # 无效帧头,只移除一个字节,继续查找
                        del buffer[0]
                    
                # 缓冲区管理:如果缓冲区过大但没找到帧头,清理一部分旧数据
                if len(buffer) > 1024:
                    del buffer[0:512]  # 删除前半部分,保留后半部分继续查找
                
            except Exception as e:
                print(f"处理异常: {str(e)}")
                # 出现异常时清空缓冲区,重新开始
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
            end_idx = min(len(sorted_points),idx + search_range)
            
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
        """打印完整扫描数据(默认显示首尾各5个点)"""
        data = self.scan_data
        if not data:
            print("无有效扫描数据")
            return
    
        # 按角度排序
        sorted_angles = sorted(data.keys())
        total_points = len(sorted_angles)
    
        print(f"\n=== 完整扫描数据(共{total_points}个点) ===")
    
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
    def __init__(self, lidar_port='/dev/ttyAMA2', lidar_baudrate=230400, motor_pins=None, base_speed=60, start_button_pin=17, min_distance_to_midline=2000):
        # 默认电机引脚配置
        if motor_pins is None:
            motor_pins = {
                'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
                'front_right': {'in1': 19, 'in2': 13, 'encoder_a': 26},
                'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
                'rear_right': {'in1': 20, 'in2': 21, 'encoder_a': 16}
            }
        
        # 初始化LiDAR处理器
        self.lidar = StableLidarProcessor(port=lidar_port, baudrate=lidar_baudrate)
        
        # 初始化麦轮小车
        self.car = MecanumCar(motor_pins)
        
        # 控制参数
        self.reference_right_distance = None  # 参考右侧距离(mm)，将在初始化时设置
        self.enemy_distance_threshold = 1500  # 敌方小车最小检测距离(mm)
        self.enemy_angle_threshold = 10  # 敌方小车角度突变阈值(度)
        self.enemy_angle = None  # 敌方小车方位角度
        self.enemy_distance = 0  # 敌方小车距离
        self.enemy_tracking_range = 30  # 敌方小车追踪范围(度)
        self.base_speed = base_speed  # 基础速度百分比
        self.min_speed = 40  # 最小速度限制
        self.max_speed = 100  # 最大速度限制
        self.forward_speed = 60  # 前进速度
        self.min_distance_to_midline = min_distance_to_midline  # 检测到中线的最小距离(mm)
        
        # 启动按键相关
        self.start_button_pin = start_button_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.start_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.is_started = False
        self.is_initialized = False
        self.is_crossing_midline = False
        self.is_enemy_detected = False
        self.is_enemy_tracked = False

        # 舵机控制相关变量
        self.servo_angles = {0: 75, 1: 90}  # 初始化舵机角度变量 - 使用0号和1号舵机分别作为x和y轴
        self.servo_moving = {0: False, 1: False}  # 标记舵机是否正在运动
        self.servo_stop_flags = {0: False, 1: False}  # 停止标志
        self.servo_threads = {0: None, 1: None}  # 线程引用
        self.servo_lock = threading.Lock()  # 线程锁

        # 胜利条件相关变量
        self.target_angles = {0: 90, 1: 90}  # 目标角度，可以根据需要修改
        self.angle_threshold = 5  # 角度阈值
        self.stable_duration = 2.0  # 稳定持续时间（秒）
        self.stable_start_time = None  # 开始稳定的时间
        self.victory_achieved = False  # 是否已经达成胜利
        self.victory_lock = threading.Lock()  # 胜利条件检查锁

        # 启动主控制线程
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

    def _map_speed(self, value, input_min, input_max, output_min=40, output_max=100):
        """将输入值从输入范围映射到输出范围(40-100)"""
        value = max(input_min, min(input_max, value))
        if input_max == input_min:
            return output_min
        input_proportion = (value - input_min) / (input_max - input_min)
        output = output_min + input_proportion * (output_max - output_min)
        return output

    def _control_loop(self):
        """小车主控制循环"""
        while True:
            try:
                # 检查启动按键状态
                if not self.is_started and GPIO.input(self.start_button_pin) == GPIO.LOW:
                    print("启动按键已按下,记录初始右侧距离...")
                    self.reference_right_distance = self._get_right_distance()
                    print(f"初始右侧距离: {self.reference_right_distance}mm")
                    time.sleep(2)
                    self.is_started = True
                    forward_speed = self._map_speed(self.forward_speed, 0, 100, self.min_speed, self.max_speed)
                    self.car.move(90, forward_speed)
                    print(f"小车开始向前行驶,速度: {forward_speed}")

                if self.is_started:
                    # 获取雷达扫描数据
                    scan_data = self.lidar.scan_data

                    if not scan_data:
                        time.sleep(0.1)
                        continue

                    # 获取右侧距离(0-30度范围内的最小值)
                    right_distance = self._get_right_distance(scan_data)

                    # 初始化阶段: 记录初始右侧距离作为参考值
                    if not self.is_initialized and right_distance < 10000:
                        self.reference_right_distance = right_distance
                        self.is_initialized = True
                        print(f"初始化完成，参考右侧距离: {self.reference_right_distance}mm")
                        forward_speed = self._map_speed(self.forward_speed, 0, 100, self.min_speed, self.max_speed)
                        self.car.move(90, forward_speed)

                    # 如果已初始化，开始正常控制流程
                    if self.is_initialized:
                        # 检查是否已越过中线
                        if not self.is_crossing_midline and self._is_crossing_midline(right_distance):
                            self.is_crossing_midline = True
                            print("已越过中线,开始检测敌方小车")

                        # 如果已越过中线,开始扫描并识别敌方小车
                        if self.is_crossing_midline and not self.is_enemy_detected:
                            enemy_angle, enemy_distance = self._detect_enemy(scan_data)
                            if enemy_angle is not None:
                                self.is_enemy_detected = True
                                self.enemy_angle = enemy_angle
                                self.enemy_distance = enemy_distance
                                print(f"检测到敌方小车方位角度: {self.enemy_angle}度, 距离: {self.enemy_distance}mm")

                                # 调整角度和速度
                                angle_to_rotate = self.enemy_angle - 90
                                if angle_to_rotate > 180:
                                    angle_to_rotate -= 360
                                elif angle_to_rotate < -180:
                                    angle_to_rotate += 360
                                rotation_speed = self._map_speed(self.base_speed, 0, 100, self.min_speed, self.max_speed)
                                self.car.rotate(angle_to_rotate, rotation_speed)
                                rotation_time = min(3.0, abs(angle_to_rotate) / 90)
                                time.sleep(rotation_time)
                                self.car.stop()
                                self.is_enemy_tracked = True

                        # 如果已检测到敌方小车,实时跟踪
                        if self.is_enemy_tracked:
                            new_angle, new_distance = self._track_enemy(scan_data)
                            if new_angle is not None:
                                angle_diff = new_angle - self.enemy_angle
                                if angle_diff > 180:
                                    angle_diff -= 360
                                elif angle_diff < -180:
                                    angle_diff += 360
                            
                                if abs(angle_diff) > self.enemy_tracking_range:
                                    tracking_speed = self._map_speed(self.base_speed, 0, 100, self.min_speed, self.max_speed)
                                    self.car.rotate(angle_diff, tracking_speed)
                                    tracking_time = min(2.0, abs(angle_diff) / 90)
                                    time.sleep(tracking_time)
                                    self.car.stop()
                                    self.enemy_angle = new_angle
                                    self.enemy_distance = new_distance
                                    print(f"跟踪到敌方小车新位置: {self.enemy_angle}度, 距离: {self.enemy_distance}mm")

                        # 检查是否需要控制舵机
                        if self.is_enemy_detected:
                            self._control_servos()

                        # 检查胜利条件
                        self.check_victory_condition()

                        # 如果已经达成胜利条件,退出循环
                        if self.victory_achieved:
                            break

                time.sleep(0.1)
            except Exception as e:
                print(f"控制线程异常: {str(e)}")
                time.sleep(0.5)

    def _get_right_distance(self, scan_data=None):
        """获取右侧距离(0-30度范围内的最小值)"""
        if scan_data is None:
            scan_data = self.lidar.scan_data

        return min([scan_data.get(a, (10000, 0))[0] for a in range(0, 30) if scan_data.get(a, (0, 0))[0] > 0], default=10000)

    def _is_crossing_midline(self, right_distance):
        """检查是否已越过中线"""
        return right_distance > self.reference_right_distance + 1000  # 右侧距离增加1米认为已越过中线

    def _detect_enemy(self, scan_data):
        """识别敌方小车位置"""
        best_angle = None
        best_distance = 0
        
        for angle in range(0, 360, 10):
            distances = []
            for i in range(angle, angle+10):
                dist, _ = scan_data.get(i % 360, (0, 0))
                if dist > 0:
                    distances.append(dist)
            
            if distances:
                min_dist = min(distances)
                avg_dist = sum(distances) / len(distances)
                
                if min_dist < self.enemy_distance_threshold and avg_dist - min_dist > self.enemy_angle_threshold:
                    best_angle = angle + 5
                    best_distance = min_dist
                    break
        
        return best_angle, best_distance

    def _track_enemy(self, scan_data):
        """实时跟踪敌方小车位置"""
        best_angle = None
        best_distance = 0
        
        for angle in range(int(self.enemy_angle - self.enemy_tracking_range/2), int(self.enemy_angle + self.enemy_tracking_range/2)):
            dist, _ = scan_data.get(angle % 360, (0, 0))
            if dist > 0 and dist < self.enemy_distance:
                best_angle = angle % 360
                best_distance = dist
                break
        
        if best_angle is None:
            best_angle = self.enemy_angle
            best_distance = self.enemy_distance
        
        return best_angle, best_distance

    def _control_servos(self):
        """控制舵机"""
        try:
            print("开始监听串口数据...")
            print(f"初始角度 - 舵机0(X轴): {self.servo_angles[0]}°, 舵机1(Y轴): {self.servo_angles[1]}°")
            print(f"目标角度 - 舵机0(X轴): {self.target_angles[0]}°±{self.angle_threshold}°, 舵机1(Y轴): {self.target_angles[1]}°±{self.angle_threshold}°")
            print(f"胜利条件：在目标范围内保持稳定 {self.stable_duration} 秒")
            buffer = ""
            
            while not self.victory_achieved:
                # 读取串口数据
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting).decode('utf-8')
                    buffer += data
                    
                    # 实时输出串口接收到的内容
                    print(f"接收到串口数据: {data}")
                    
                    # 处理完整数据
                    while '$' in buffer and '$' in buffer[1:]:
                        start = buffer.find('$')
                        end = buffer.find('$', start + 1)
                        
                        if end > start and end - start == 9:  # 确保长度正确（应为10个字符，包括两个$符号）
                            command = buffer[start:end+1]
                            buffer = buffer[end+1:]  # 移除已处理的数据
                            
                            # 解析命令
                            match = re.match(pattern, command)
                            if match:
                                x_change = int(match.group(1))
                                y_change = int(match.group(2))
                                print(f"解析指令: X轴变化{x_change:+d}°, Y轴变化{y_change:+d}°")
                                
                                # 更新舵机位置 - 0号为x轴，1号为y轴
                                if x_change != 0:
                                    self.update_servo_angle(0, x_change)
                                if y_change != 0:
                                    self.update_servo_angle(1, y_change)
                            else:
                                print(f"无效数据格式: {command}")
                        else:
                            # 数据不完整或格式错误，丢弃开头的$
                            buffer = buffer[start+1:]
                
                # 检查胜利条件
                self.check_victory_condition()
                
                time.sleep(0.01)  # 小延迟，避免CPU占用过高
        except KeyboardInterrupt:
            print("程序已停止")
            self.cleanup_servos()

    def move_servo_gradually(self, servo_id, target_angle):
        """逐度移动舵机到目标角度"""
        current_angle = self.servo_angles[servo_id]
        
        if current_angle == target_angle:
            return
        
        step = 1 if target_angle > current_angle else -1
        
        print(f"舵机 {servo_id} 开始从 {current_angle}° 移动到 {target_angle}°")
        
        while current_angle != target_angle:
            if self.servo_stop_flags[servo_id]:
                print(f"舵机 {servo_id} 运动被新指令中断，当前位置: {current_angle}°")
                break
                
            current_angle += step
            current_angle = max(0, min(180, current_angle))
            kit.servo[servo_id].angle = current_angle
            
            with self.servo_lock:
                self.servo_angles[servo_id] = current_angle
            
            print(f"舵机 {servo_id} 当前角度: {current_angle}°")
            time.sleep(0.05)
        
        with self.servo_lock:
            self.servo_moving[servo_id] = False
            self.servo_stop_flags[servo_id] = False
        
        print(f"舵机 {servo_id} 运动完成，最终角度: {self.servo_angles[servo_id]}°")

    def update_servo_angle(self, servo_id, change):
        """更新舵机角度并控制舵机"""
        if self.victory_achieved:
            print("已达成胜利条件，忽略新指令")
            return
        
        current_angle = self.servo_angles[servo_id]
        new_angle = current_angle + change
        new_angle = max(0, min(180, new_angle))
        
        if self.servo_moving[servo_id]:
            print(f"舵机 {servo_id} 收到新指令，中断当前运动")
            self.servo_stop_flags[servo_id] = True
            
            if self.servo_threads[servo_id] and self.servo_threads[servo_id].is_alive():
                self.servo_threads[servo_id].join(timeout=1.0)
        
        self.servo_stop_flags[servo_id] = False
        
        if new_angle == self.servo_angles[servo_id]:
            print(f"舵机 {servo_id} 已在目标位置 {new_angle}°")
            return
        
        self.servo_moving[servo_id] = True
        
        self.servo_threads[servo_id] = threading.Thread(
            target=self.move_servo_gradually, 
            args=(servo_id, new_angle)
        )
        self.servo_threads[servo_id].daemon = True
        self.servo_threads[servo_id].start()

    def check_victory_condition(self):
        """检查胜利条件"""
        if self.victory_achieved:
            return
        
        with self.victory_lock:
            servo0_in_range = abs(self.servo_angles[0] - self.target_angles[0]) <= self.angle_threshold
            servo1_in_range = abs(self.servo_angles[1] - self.target_angles[1]) <= self.angle_threshold
            
            current_time = time.time()
            
            if servo0_in_range and servo1_in_range:
                if self.stable_start_time is None:
                    self.stable_start_time = current_time
                    print(f"舵机位置进入目标范围 (X: {self.servo_angles[0]}°±{self.angle_threshold}°, Y: {self.servo_angles[1]}°±{self.angle_threshold}°)，开始计时...")
                else:
                    stable_time = current_time - self.stable_start_time
                    if stable_time >= self.stable_duration:
                        self.victory_achieved = True
                        print("🎉 胜利！达成目标位置并保持稳定!")
                        print(f"最终位置 - 舵机0(X轴): {self.servo_angles[0]}°, 舵机1(Y轴): {self.servo_angles[1]}°")
                        self.trigger_victory_signal()
                    else:
                        print(f"目标范围内稳定时间: {stable_time:.1f}s / {self.stable_duration}s")
            else:
                if self.stable_start_time is not None:
                    print(f"舵机离开目标范围 (X: {self.servo_angles[0]}°, Y: {self.servo_angles[1]}°)，重置计时")
                    self.stable_start_time = None

    def trigger_victory_signal(self):
        """触发胜利信号：蜂鸣器长鸣2秒，LED常亮"""
        print("触发胜利信号：蜂鸣器长鸣2秒，LED常亮")
        peripherals.led_on()
        peripherals.buzzer_on(frequency=800, duration=2.0)

    def cleanup_servos(self):
        """停止所有舵机运动并清理资源"""
        for servo_id in [0, 1]:
            self.servo_stop_flags[servo_id] = True
        
        # 等待所有线程结束
        for servo_id in [0, 1]:
            if self.servo_threads[servo_id] and self.servo_threads[servo_id].is_alive():
                self.servo_threads[servo_id].join(timeout=2.0)
        
        # 清理外设
        peripherals.cleanup()
        ser.close()
        print("串口已关闭，外设已清理")

    def shutdown(self):
        """关闭资源"""
        self.car.stop()
        self.lidar.shutdown()
        self.cleanup_servos()
        GPIO.cleanup()
        print("系统已关闭")


if __name__ == '__main__':
    try:
        # 引脚配置示例
        motor_pins = {
            'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
            'front_right': {'in1': 19, 'in2': 13, 'encoder_a': 26},
            'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
            'rear_right': {'in1': 20, 'in2': 21, 'encoder_a': 16}
        }

        # 全局实例
        lidar_car = LidarControlledCar(motor_pins=motor_pins, base_speed=40, start_button_pin=17, min_distance_to_midline=2000)

        print("启动激光雷达小车系统,等待启动按键按下...")

        # 等待启动按键被按下
        while not lidar_car.is_started:
            time.sleep(0.5)

        print("小车开始前进,保持参考右侧距离...")

        # 等待检测到中线并开始控制舵机
        while not lidar_car.is_enemy_detected:
            time.sleep(1)

        print("已检测到敌方小车,开始跟踪和控制舵机...")

        while not lidar_car.victory_achieved:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断,关闭系统")
        lidar_car.shutdown()