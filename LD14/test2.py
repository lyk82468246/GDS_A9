#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import serial
import math
from collections import defaultdict

class LidarProcessor:
    def __init__(self, port='/dev/ttyCH343USB0', baudrate=230400):
        self.ser = serial.Serial(port, baudrate, timeout=5)
        self.current_scan = defaultdict(dict)
        self.temp_points = []
        self.last_angle = 0
        self.scan_started = False

    def _parse_frame(self, data):
        frame = {}
        # 解析基础参数
        frame['speed'] = int.from_bytes(data[0:2], byteorder='little')  # 转速
        start_angle = int.from_bytes(data[2:4], 'little') / 100.0  # 起始角度
        end_angle = int.from_bytes(data[40:42], 'little') / 100.0  # 结束角度
        timestamp = int.from_bytes(data[42:44], 'little')  # 时间戳
        
        # 处理角度环
        if start_angle > end_angle:
            angle_diff = (360 - start_angle) + end_angle
        else:
            angle_diff = end_angle - start_angle
        
        # 解析12个点
        points = []
        for i in range(12):
            offset = 4 + i*3
            distance = int.from_bytes(data[offset:offset+2], 'little')
            intensity = data[offset+2]
            
            # 计算当前点角度（线性插值）
            angle = start_angle + (angle_diff / 11) * i
            if angle >= 360:
                angle -= 360
                
            points.append((round(angle, 2), distance, intensity))
        
        return {
            'start': start_angle,
            'end': end_angle,
            'points': points,
            'timestamp': timestamp
        }

    def _check_full_scan(self, new_start):
        # 通过角度变化检测完整扫描
        if not self.scan_started:
            if new_start < 5:  # 检测到新扫描开始
                self.scan_started = True
                return False
            return False
        
        # 当角度重新接近0度时认为完成扫描
        if self.last_angle > 355 and new_start < 5:
            self.scan_started = False
            return True
        return False

    def process_scans(self):
        while True:
            # 读取帧头
            header = self.ser.read(2)
            if len(header) != 2:
                continue
            
            if header[0] == 0x54 and header[1] == 0x2C:
                # 读取完整数据帧
                data = self.ser.read(45)
                if len(data) != 45:
                    continue
                
                # 解析数据
                frame = self._parse_frame(data)
                
                # 检测是否完成完整扫描
                if self._check_full_scan(frame['start']):
                    # 生成最终字典并重置
                    result = {angle: (dist, intens) for angle, dist, intens in self.temp_points}
                    self.temp_points = []  # 清空缓存
                    yield result
                
                # 缓存当前帧数据
                self.temp_points.extend(frame['points'])
                self.last_angle = frame['end']

    def run(self):
        try:
            for scan in self.process_scans():
                # 这里可以添加数据处理或输出逻辑
                print("完整扫描数据:")
                for angle in sorted(scan.keys()):
                    print(f"角度: {angle}°, 距离: {scan[angle][0]}mm, 强度: {scan[angle][1]}")
                print("\n===== 扫描完成 =====")
        except KeyboardInterrupt:
            self.ser.close()
            print("串口已关闭")

if __name__ == '__main__':
    processor = LidarProcessor()
    processor.run()