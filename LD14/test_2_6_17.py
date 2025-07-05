#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import serial
import threading
import bisect
import time
import math
from mecanum_wheel_car import MecanumCar

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
    def __init__(self, lidar_port='/dev/ttyAMA2', lidar_baudrate=230400, motor_pins=None, base_speed=60):
        """
        初始化带激光雷达的麦轮小车
        
        参数:
        lidar_port: 激光雷达串口端口
        lidar_baudrate: 激光雷达波特率
        motor_pins: 电机引脚配置
        base_speed: 基础速度百分比(0-100)
        """
        # 默认电机引脚配置
        if motor_pins is None:
            motor_pins = {
                'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
                'front_right': {'in1': 6, 'in2': 5, 'encoder_a': 26},
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
        
        # 状态标志
        self.is_initialized = False  # 初始化标志
        self.is_crossing_midline = False
        self.is_enemy_detected = False
        self.is_enemy_tracked = False
        
        # 启动主控制线程
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
    
    def _map_speed(self, value, input_min, input_max, output_min=40, output_max=100):
        """
        将输入值从输入范围映射到输出范围(40-100)
        
        例如, 如果输入值是输入范围的50%, 则输出值是输出范围的50%
        如果输入值小于输入最小值,则输出最小值
        如果输入值大于输入最大值,则输出最大值
        """
        # 确保输入值在输入范围内
        value = max(input_min, min(input_max, value))
        
        # 计算映射比例
        if input_max == input_min:  # 防止除零错误
            return output_min
            
        input_proportion = (value - input_min) / (input_max - input_min)
        
        # 应用映射比例到输出范围
        output = output_min + input_proportion * (output_max - output_min)
        
        return output
    
    def _control_loop(self):
        """小车主控制循环"""
        while True:
            try:
                # 获取雷达扫描数据
                scan_data = self.lidar.scan_data
                
                if not scan_data:
                    time.sleep(0.1)
                    continue
                
                # 获取右侧距离(0-30度范围内的最小值)
                right_distance = min([scan_data.get(a, (10000, 0))[0] for a in range(0, 30) if scan_data.get(a, (0, 0))[0] > 0], default=10000)
                
                # 初始化阶段: 记录初始右侧距离作为参考值
                if not self.is_initialized and right_distance < 10000:
                    self.reference_right_distance = right_distance
                    self.is_initialized = True
                    print(f"初始化完成，参考右侧距离: {self.reference_right_distance}mm")
                    
                    # 开始向前行驶
                    forward_speed = self._map_speed(self.forward_speed, 0, 100, self.min_speed, self.max_speed)
                    self.car.move(90, forward_speed)  # 90度为前进方向
                    print(f"开始向前行驶，速度: {forward_speed}")
                
                # 如果已初始化，开始正常控制流程
                if self.is_initialized:
                    # 获取前方距离(80-100度范围内的最小值)
                    front_distance = min([scan_data.get(a, (10000, 0))[0] for a in range(80, 100) if scan_data.get(a, (0, 0))[0] > 0], default=10000)
                    
                    # 检测是否越过中线
                    if front_distance < self.enemy_distance_threshold and not self.is_crossing_midline:
                        self.is_crossing_midline = True
                        print("检测到已越过中线")
                    
                    # 根据右侧距离调整行驶方向，保持与右侧障碍物的距离接近参考距离
                    right_distance_error = self.reference_right_distance - right_distance
                    
                    # 计算偏航角度，根据误差大小调整
                    # 如果右侧距离小于参考距离，向左偏航(>90度)
                    # 如果右侧距离大于参考距离，向右偏航(<90度)
                    yaw_angle = 90
                    if abs(right_distance_error) > 50:  # 设置一个阈值，避免小误差造成频繁调整
                        # 根据误差大小计算偏航角度，最大偏航±30度
                        max_yaw_deviation = 30
                        yaw_deviation = min(max_yaw_deviation, abs(right_distance_error) / 20)  # 每20mm误差调整1度，最大30度
                        
                        if right_distance_error > 0:  # 右侧距离小于参考值，需要向左偏
                            yaw_angle = 90 + yaw_deviation
                        else:  # 右侧距离大于参考值，需要向右偏
                            yaw_angle = 90 - yaw_deviation
                    
                    # 如果已越过中线，开始扫描并识别敌方小车
                    if self.is_crossing_midline and not self.is_enemy_detected:
                        enemy_angle, enemy_distance = self._detect_enemy(scan_data)
                        if enemy_angle is not None:
                            self.is_enemy_detected = True
                            self.enemy_angle = enemy_angle
                            self.enemy_distance = enemy_distance
                            print(f"检测到敌方小车方位角度: {self.enemy_angle}度, 距离: {self.enemy_distance}mm")
                            
                            # 停止前进
                            self.car.stop()
                            time.sleep(0.5)  # 短暂停顿
                            
                            # 旋转以面向敌方小车
                            angle_to_rotate = self.enemy_angle - 90
                            # 归一化角度到-180到180度
                            if angle_to_rotate > 180:
                                angle_to_rotate -= 360
                            elif angle_to_rotate < -180:
                                angle_to_rotate += 360
                            
                            # 确保旋转速度在40-100范围内
                            rotation_speed = self._map_speed(self.base_speed, 0, 100, self.min_speed, self.max_speed)
                            self.car.rotate(angle_to_rotate, rotation_speed)
                            
                            # 根据角度计算旋转时间,但设置上限
                            rotation_time = min(3.0, abs(angle_to_rotate) / 90)
                            time.sleep(rotation_time)
                            self.car.stop()
                            self.is_enemy_tracked = True
                            continue  # 跳过本次循环剩余部分，直接进入跟踪模式
                    
                    # 如果已检测到敌方小车，实时跟踪
                    if self.is_enemy_tracked:
                        new_angle, new_distance = self._track_enemy(scan_data)
                        if new_angle is not None:
                            angle_diff = new_angle - self.enemy_angle
                            # 归一化角度差到-180到180度
                            if angle_diff > 180:
                                angle_diff -= 360
                            elif angle_diff < -180:
                                angle_diff += 360
                                
                            if abs(angle_diff) > self.enemy_tracking_range:
                                # 确保速度在40-100范围内
                                tracking_speed = self._map_speed(self.base_speed, 0, 100, self.min_speed, self.max_speed)
                                self.car.rotate(angle_diff, tracking_speed)
                                
                                # 根据角度计算旋转时间,但设置上限
                                tracking_time = min(2.0, abs(angle_diff) / 90)
                                time.sleep(tracking_time)
                                self.car.stop()
                                self.enemy_angle = new_angle
                                self.enemy_distance = new_distance
                                print(f"跟踪到敌方小车新位置: {self.enemy_angle}度, 距离: {self.enemy_distance}mm")
                    else:
                        # 正常前进，根据计算的偏航角调整方向
                        forward_speed = self._map_speed(self.forward_speed, 0, 100, self.min_speed, self.max_speed)
                        self.car.move(yaw_angle, forward_speed)

                time.sleep(0.1)
            except Exception as e:
                print(f"控制线程异常: {str(e)}")
                time.sleep(0.5)

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
                
                # 检查是否有距离突变,判断为可能的敌方小车位置
                if min_dist < self.enemy_distance_threshold and avg_dist - min_dist > self.enemy_angle_threshold:
                    best_angle = angle + 5  # 取扇区中间角度
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

    def shutdown(self):
        """关闭资源"""
        self.car.stop()
        self.lidar.shutdown()
        print("系统已关闭")

# 引脚配置示例(与之前一致)
motor_pins = {
    'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
    'front_right': {'in1': 6, 'in2': 5, 'encoder_a': 26},
    'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
    'rear_right': {'in1': 20, 'in2': 21, 'encoder_a': 16}
}

# 全局实例
lidar_car = LidarControlledCar(motor_pins=motor_pins, base_speed=40)

if __name__ == '__main__':
    try:
        print("启动激光雷达小车系统...")
        print("等待雷达数据稳定和初始化...")
        
        # 等待初始化完成
        while not lidar_car.is_initialized:
            time.sleep(0.5)
            
        print(f"初始化完成，参考右侧距离: {lidar_car.reference_right_distance}mm")
        print("小车开始前进，保持参考距离...")

        # 等待检测到敌方小车
        while not lidar_car.is_enemy_detected:
            time.sleep(1)

        print("已检测到敌方小车,开始跟踪...")

        # 在这里可以添加攻击等进一步的控制逻辑
        while lidar_car.is_enemy_tracked:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断,关闭系统")
        lidar_car.shutdown()