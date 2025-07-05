import bisect
import serial
import re
import time
import threading
import random
from adafruit_servokit import ServoKit
from peripherals import Peripherals
import RPi.GPIO as GPIO
from mecanum_wheel_car import MecanumCar

# 初始化舵机控制器
kit = ServoKit(channels=16)

# 初始化外设控制器
peripherals = Peripherals()

# 配置串口
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
            start = (int.from_bytes(data[2:4], byteorder='little')) / 100.0
            end = (int.from_bytes(data[40:42], byteorder='little')) / 100.0

            points = []
            for i in range(12):
                offset = 4 + i * 3
                if offset + 2 >= len(data):
                    break

                dist_low = data[offset]
                dist_high = data[offset + 1]
                distance = (dist_high << 8) | dist_low
                intensity = data[offset + 2]

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
            return {'start': 0, 'end': 0, 'points': []}

    def _process_data(self):
        buffer = bytearray()
        while self._running:
            try:
                new_data = self.ser.read(max(1, self.ser.in_waiting))
                if not new_data:
                    time.sleep(0.001)
                    continue

                buffer.extend(new_data)

                while len(buffer) >= 47:
                    if buffer[0:2] == b'\x54\x2C':
                        frame_data = buffer[0:47]
                        del buffer[0:47]

                        data = frame_data[2:]
                        frame = self._parse_frame(data)

                        if frame['start'] < 5 and not self._scan_started:
                            self._scan_started = True
                            self._raw_points = []

                        if self._scan_started:
                            self._raw_points.extend(frame['points'])

                            if self._last_angle > 355 and frame['start'] < 5:
                                self._scan_started = False
                                self._generate_scan_dict()
                                self._valid_point_count = 0
                                self._total_point_count = 0

                        self._last_angle = frame['end']
                    else:
                        del buffer[0]

                if len(buffer) > 1024:
                    del buffer[0:512]

            except Exception as e:
                print(f"处理异常: {str(e)}")
                buffer.clear()
                time.sleep(0.1)

    def _generate_scan_dict(self):
        if not self._raw_points:
            return

        sorted_points = sorted(self._raw_points, key=lambda x: x[0])
        angles = [p[0] for p in sorted_points]

        final_dict = {}

        for target_angle in range(360):
            idx = bisect.bisect_left(angles, target_angle)
            candidates = []

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
                distances = sorted([p[1] for p in candidates])
                median_dist = distances[len(distances) // 2]
                final_dict[target_angle] = (median_dist, 50)
            else:
                final_dict[target_angle] = (0, 0)

        with self._lock:
            self._scan_dict = final_dict

    @property
    def scan_data(self):
        with self._lock:
            return self._scan_dict.copy()

    def shutdown(self):
        self._running = False
        self.ser.close()


class ModifiedLidarControlledCar:
    def __init__(self, lidar_port='/dev/ttyAMA2', lidar_baudrate=230400, motor_pins=None, start_button_pin=17):
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

        # 启动按键相关
        self.start_button_pin = start_button_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.start_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.is_started = False
        self.is_forward_completed = False

        # 舵机控制相关变量
        self.servo_angles = {0: 90, 1: 90}  # 0号为水平舵机(由雷达控制)，1号为垂直舵机(由摄像头控制)
        self.servo_moving = {0: False, 1: False}
        self.servo_stop_flags = {0: False, 1: False}
        self.servo_threads = {0: None, 1: None}
        self.servo_lock = threading.Lock()

        # 控制标志
        self.running = True

        # 胜利信号相关
        self.next_victory_time = None

        # 启动各个控制线程
        self.start_control_threads()

    def start_control_threads(self):
        """启动所有控制线程"""
        # 主控制线程
        self.main_thread = threading.Thread(target=self._main_control_loop)
        self.main_thread.daemon = True
        self.main_thread.start()

        # 雷达控制水平舵机线程
        self.lidar_servo_thread = threading.Thread(target=self._lidar_servo_control)
        self.lidar_servo_thread.daemon = True
        self.lidar_servo_thread.start()

        # 摄像头控制垂直舵机线程
        self.camera_servo_thread = threading.Thread(target=self._camera_servo_control)
        self.camera_servo_thread.daemon = True
        self.camera_servo_thread.start()

        # 随机运动控制线程
        self.random_movement_thread = threading.Thread(target=self._random_movement_control)
        self.random_movement_thread.daemon = True
        self.random_movement_thread.start()

        # 胜利信号线程
        self.victory_signal_thread = threading.Thread(target=self._victory_signal_control)
        self.victory_signal_thread.daemon = True
        self.victory_signal_thread.start()

    def _main_control_loop(self):
        """主控制循环"""
        while self.running:
            try:
                # 检查启动按键状态
                if not self.is_started and GPIO.input(self.start_button_pin) == GPIO.LOW:
                    print("启动按键已按下，开始向前直线运动...")
                    self.is_started = True

                    # 向前直线运动5秒
                    self.car.move(90, 60)  # 向前，速度60%
                    time.sleep(5)
                    self.car.stop()

                    self.is_forward_completed = True
                    print("直线运动完成，开始执行四项并行任务...")

                    # 设置第一次胜利信号时间
                    self.next_victory_time = time.time() + random.randint(5000, 10000) / 1000.0

                time.sleep(0.1)
            except Exception as e:
                print(f"主控制线程异常: {str(e)}")
                time.sleep(0.5)

    def _lidar_servo_control(self):
        """雷达控制水平舵机"""
        print("雷达控制水平舵机线程已启动")
        while self.running:
            try:
                if not self.is_forward_completed:
                    time.sleep(0.1)
                    continue

                # 获取雷达扫描数据
                scan_data = self.lidar.scan_data
                if not scan_data:
                    time.sleep(0.1)
                    continue

                # 检测前方最近的障碍物
                front_distances = []
                for angle in range(85, 96):  # 前方±5度范围
                    dist, _ = scan_data.get(angle, (0, 0))
                    if dist > 0:
                        front_distances.append((angle, dist))

                if front_distances:
                    # 找到最近的障碍物
                    closest_obstacle = min(front_distances, key=lambda x: x[1])
                    obstacle_angle = closest_obstacle[0]
                    obstacle_distance = closest_obstacle[1]

                    # 根据障碍物角度调整水平舵机
                    # 将雷达角度(0-360)映射到舵机角度(0-180)
                    target_servo_angle = int((obstacle_angle / 360.0) * 180)
                    target_servo_angle = max(0, min(180, target_servo_angle))

                    # 缓慢移动舵机
                    current_angle = self.servo_angles[0]
                    angle_diff = target_servo_angle - current_angle

                    if abs(angle_diff) > 2:  # 只有角度差大于2度才移动
                        step = 1 if angle_diff > 0 else -1
                        new_angle = current_angle + step
                        self.move_servo_slowly(0, new_angle)

                time.sleep(0.1)  # 控制频率
            except Exception as e:
                print(f"雷达舵机控制异常: {str(e)}")
                time.sleep(0.5)

    def _camera_servo_control(self):
        """摄像头控制垂直舵机"""
        print("摄像头控制垂直舵机线程已启动")
        buffer = ""

        while self.running:
            try:
                if not self.is_forward_completed:
                    time.sleep(0.1)
                    continue

                # 读取串口数据
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting).decode('utf-8')
                    buffer += data

                    # 处理完整数据
                    while '$' in buffer and '$' in buffer[1:]:
                        start = buffer.find('$')
                        end = buffer.find('$', start + 1)

                        if end > start and end - start == 9:
                            command = buffer[start:end + 1]
                            buffer = buffer[end + 1:]

                            # 解析命令
                            match = re.match(pattern, command)
                            if match:
                                x_change = int(match.group(1))  # 不再使用x轴变化
                                y_change = int(match.group(2))  # 只控制垂直舵机

                                print(f"摄像头指令: Y轴变化{y_change:+d}°")

                                # 只更新垂直舵机(1号舵机)
                                if y_change != 0:
                                    current_angle = self.servo_angles[1]
                                    new_angle = current_angle + y_change
                                    new_angle = max(0, min(180, new_angle))
                                    self.move_servo_slowly(1, new_angle)
                        else:
                            buffer = buffer[start + 1:]

                time.sleep(0.01)
            except Exception as e:
                print(f"摄像头舵机控制异常: {str(e)}")
                time.sleep(0.5)

    def _random_movement_control(self):
        """随机运动控制"""
        print("随机运动控制线程已启动")
        while self.running:
            try:
                if not self.is_forward_completed:
                    time.sleep(0.1)
                    continue

                # 随机选择运动类型
                movement_type = random.choice(['move', 'rotate', 'stop'])

                if movement_type == 'move':
                    # 随机平动
                    angle = random.randint(0, 359)
                    speed = random.randint(30, 70)
                    duration = random.uniform(1.0, 3.0)

                    print(f"随机平动: 角度{angle}°, 速度{speed}%, 持续{duration:.1f}s")
                    self.car.move(angle, speed)
                    time.sleep(duration)

                elif movement_type == 'rotate':
                    # 随机转动
                    rotation_angle = random.randint(-180, 180)
                    speed = random.randint(30, 60)
                    duration = random.uniform(0.5, 2.0)

                    print(f"随机转动: {rotation_angle}°, 速度{speed}%, 持续{duration:.1f}s")
                    self.car.rotate(rotation_angle, speed)
                    time.sleep(duration)

                else:  # stop
                    # 停止一段时间
                    duration = random.uniform(0.5, 2.0)
                    print(f"停止运动: 持续{duration:.1f}s")
                    self.car.stop()
                    time.sleep(duration)

                # 运动间隔
                time.sleep(random.uniform(0.5, 2.0))

            except Exception as e:
                print(f"随机运动控制异常: {str(e)}")
                time.sleep(1.0)

    def _victory_signal_control(self):
        """胜利信号控制"""
        print("胜利信号控制线程已启动")
        while self.running:
            try:
                if not self.is_forward_completed or self.next_victory_time is None:
                    time.sleep(0.1)
                    continue

                current_time = time.time()
                if current_time >= self.next_victory_time:
                    # 发出胜利信号
                    self.trigger_victory_signal()

                    # 生成下一次胜利信号的时间
                    next_interval = random.randint(5000, 10000)  # 5-10秒
                    self.next_victory_time = current_time + next_interval / 1000.0
                    print(f"下一次胜利信号将在{next_interval / 1000.0:.1f}秒后发出")

                time.sleep(0.1)
            except Exception as e:
                print(f"胜利信号控制异常: {str(e)}")
                time.sleep(0.5)

    def move_servo_slowly(self, servo_id, target_angle):
        """缓慢移动舵机到目标角度（使用slow.py中的逐度移动逻辑）"""
        if self.servo_moving[servo_id]:
            self.servo_stop_flags[servo_id] = True
            if self.servo_threads[servo_id] and self.servo_threads[servo_id].is_alive():
                self.servo_threads[servo_id].join(timeout=0.5)

        self.servo_stop_flags[servo_id] = False

        if target_angle == self.servo_angles[servo_id]:
            return

        self.servo_moving[servo_id] = True

        self.servo_threads[servo_id] = threading.Thread(
            target=self._move_servo_gradually,
            args=(servo_id, target_angle)
        )
        self.servo_threads[servo_id].daemon = True
        self.servo_threads[servo_id].start()

    def _move_servo_gradually(self, servo_id, target_angle):
        """逐度移动舵机到目标角度"""
        current_angle = self.servo_angles[servo_id]

        if current_angle == target_angle:
            return

        step = 1 if target_angle > current_angle else -1

        while current_angle != target_angle:
            if self.servo_stop_flags[servo_id]:
                break

            current_angle += step
            current_angle = max(0, min(180, current_angle))

            try:
                kit.servo[servo_id].angle = current_angle
            except Exception as e:
                print(f"舵机{servo_id}控制异常: {str(e)}")
                break

            with self.servo_lock:
                self.servo_angles[servo_id] = current_angle

            time.sleep(0.05)  # 50ms延迟，防止转动过快

        with self.servo_lock:
            self.servo_moving[servo_id] = False
            self.servo_stop_flags[servo_id] = False

    def trigger_victory_signal(self):
        """触发胜利信号：蜂鸣器长鸣1秒，LED闪烁"""
        print("🎉 发出胜利信号！")
        try:
            peripherals.led_on()
            peripherals.buzzer_on(frequency=800, duration=1.0)
            time.sleep(1.0)
            peripherals.led_off()
        except Exception as e:
            print(f"胜利信号异常: {str(e)}")

    def cleanup(self):
        """清理资源"""
        print("正在清理资源...")
        self.running = False

        # 停止所有舵机运动
        for servo_id in [0, 1]:
            self.servo_stop_flags[servo_id] = True

        # 等待所有线程结束
        for servo_id in [0, 1]:
            if self.servo_threads[servo_id] and self.servo_threads[servo_id].is_alive():
                self.servo_threads[servo_id].join(timeout=2.0)

        # 停止小车
        self.car.stop()

        # 关闭雷达
        self.lidar.shutdown()

        # 清理外设
        peripherals.cleanup()
        ser.close()
        GPIO.cleanup()
        print("资源清理完成")


if __name__ == '__main__':
    try:
        # 引脚配置
        motor_pins = {
            'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
            'front_right': {'in1': 19, 'in2': 13, 'encoder_a': 26},
            'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
            'rear_right': {'in1': 20, 'in2': 21, 'encoder_a': 16}
        }

        # 创建修改后的激光雷达小车系统
        modified_car = ModifiedLidarControlledCar(motor_pins=motor_pins, start_button_pin=17)

        print("修改后的激光雷达小车系统已启动，等待启动按键按下...")
        print("系统功能:")
        print("1. 按下启动按键后，小车向前直线运动5秒")
        print("2. 然后同时执行四项任务:")
        print("   - 雷达控制水平舵机")
        print("   - 摄像头控制垂直舵机")
        print("   - 小车随机运动")
        print("   - 定时发出胜利信号")

        # 等待启动按键被按下
        while not modified_car.is_started:
            time.sleep(0.5)

        print("小车已启动，正在执行任务...")

        # 主循环，等待用户中断
        while modified_car.running:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断，关闭系统")
    finally:
        if 'modified_car' in locals():
            modified_car.cleanup()