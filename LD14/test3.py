#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import serial
import threading
import bisect
from collections import defaultdict

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
        while self._running:
            try:
                header = self.ser.read(2)
                if header == b'\x54\x2C':
                    data = self.ser.read(45)
                    if len(data) == 45:
                        # 调试输出帧统计
                        # print(f"\n收到有效帧 | 总点数: {self._total_point_count} 有效点: {self._valid_point_count}")
                        
                        frame = self._parse_frame(data)
                        
                        # 调试输出帧信息
                        # print(f"帧数据: 起始角度={frame['start']:.2f} 结束角度={frame['end']:.2f} 点数={len(frame['points'])}")

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
                    print(f"无效帧头: {header.hex()}")
            except Exception as e:
                print(f"处理异常: {str(e)}")

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
        for angle in sorted_angles[:360]:
            dist, intens = data[angle]
            print(f"{angle:03d}° | 距离: {dist:4}mm | 强度: {intens:3}")

        # 统计信息
        valid_points = sum(1 for v in data.values() if v[0] > 0)
        print(f"有效点比例: {valid_points/360:.1%}")

    def save_scan_to_csv(self, filename="scan_data.csv"):
        """保存扫描数据到CS文件"""
        import csv
        data = self.scan_data
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['角度', '距离(mm)', '强度'])
            for angle in sorted(data.keys()):
                writer.writerow([angle, data[angle][0], data[angle][1]])
        print(f"数据已保存至 {filename}")

lidar = StableLidarProcessor()

# 全局实例
if __name__ == '__main__':
    try:
        import time
        last_update = 0
        
        while True:
            current = lidar.scan_data
            if current and id(current) != last_update:
                # 打印精简版数据
                lidar.print_full_scan()
                
                # 完整保存数据（首次扫描时）
                if last_update == 0:
                    lidar.save_scan_to_csv()
                
                last_update = id(current)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        lidar.shutdown()
