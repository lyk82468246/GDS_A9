#!/usr/bin/env python3
import serial
import math
import sys
from collections import defaultdict

class UbuntuLidarProcessor:
    def __init__(self, port='/dev/ttyS0'):
        """
        参数说明：
        port: 串口设备路径，默认为常用雷达别名
             常见实际路径可能是 /dev/ttyUSB0 或 /dev/ttyACM0
        """
        self.port = port
        self._setup_serial()
        self.current_scan = defaultdict(list)
        self.angle_cache = []
        self.last_angle = 0
        self.full_scan = {}

    def _setup_serial(self):
        """Ubuntu系统专用串口配置"""
        try:
            # 配置LD14P雷达专用参数
            self.ser = serial.Serial(
                port=self.port,
                baudrate=230400,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # 较短的超时时间保证实时性
            )
            # 清空缓冲区防止积压旧数据
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
        except serial.SerialException as e:
            print(f"串口初始化失败: {str(e)}")
            print("\n故障排除建议：")
            print("1. 确认设备路径：使用 ls /dev/tty* 查看可用端口")
            print("2. 检查用户权限：执行 sudo usermod -aG dialout $USER")
            print("3. 关闭冲突进程：sudo killall gpsd cat")
            sys.exit(1)

    def _angle_normalize(self, angle):
        """角度归一化处理（0-360范围）"""
        return angle % 360

    def _detect_new_scan(self, current_start_angle):
        """使用角度突变检测新扫描周期"""
        # 处理角度环绕情况（359° -> 0°）
        if current_start_angle < 100 and self.last_angle > 260:
            return True
        return abs(current_start_angle - self.last_angle) > 100

    def parse_frame(self, data):
        """改进的帧解析方法，包含数据校验"""
        if len(data) != 45:
            return []
            
        try:
            # 使用更精确的浮点处理
            start_angle = (data[3] * 256 + data[2]) / 100.0
            end_angle = (data[41] * 256 + data[40]) / 100.0
            
            # 角度归一化
            start_angle = self._angle_normalize(start_angle)
            end_angle = self._angle_normalize(end_angle)
            
            points = []
            for i in range(12):
                # 精确线性插值（保留3位小数）
                ratio = i / 11.0
                angle = start_angle + (end_angle - start_angle) * ratio
                angle = round(self._angle_normalize(angle), 3)
                
                # 读取距离数据并验证有效性
                offset = 4 + i * 3
                distance = (data[offset+1] << 8) | data[offset]
                
                if 50 <= distance <= 15000:  # 有效距离范围检测
                    points.append((angle, distance))
            
            self.last_angle = start_angle
            return points
            
        except IndexError:
            return []

    def get_full_scan(self):
        """持续获取数据直到完成完整360度扫描"""
        while True:
            # 使用更可靠的头字节检测
            header = self.ser.read(1)
            if not header:
                continue
                
            if header[0] == 0x54:
                second_byte = self.ser.read(1)
                if not second_byte or second_byte[0] != 0x2C:
                    continue
                    
                data = self.ser.read(45)
                points = self.parse_frame(data)
                
                if not points:
                    continue
                
                # 检测新扫描周期
                if self._detect_new_scan(points[0][0]):
                    # 生成高精度扫描字典（角度为键，保留3位小数）
                    self.full_scan = {ang: dist for ang, dist in self.angle_cache}
                    self.angle_cache = []
                    return self.full_scan
                else:
                    self.angle_cache.extend(points)

    def continuous_scan(self):
        """持续扫描模式生成器"""
        try:
            while True:
                yield self.get_full_scan()
        except KeyboardInterrupt:
            self.ser.close()
            print("\n串口已安全关闭")

def print_scan_data(scan_dict, decimal_places=3, show_all=False):
    """
    打印雷达扫描字典的增强版函数
    
    参数：
    scan_dict: 扫描数据字典 {角度: 距离}
    decimal_places: 角度显示小数位数（默认3位）
    show_all: 是否显示全部数据点（默认只显示摘要）
    """
    if not scan_dict:
        print("无有效扫描数据")
        return
    
    # 数据预处理
    sorted_angles = sorted(scan_dict.keys())
    total_points = len(sorted_angles)
    max_dist = max(scan_dict.values())
    min_dist = min(scan_dict.values())
    avg_dist = sum(scan_dict.values()) / total_points
    
    # 打印统计信息
    print("\n" + "="*60)
    print(f"扫描数据摘要（共{total_points}个数据点）")
    print("-"*60)
    print(f"最远距离: {max_dist} mm @ {[k for k,v in scan_dict.items() if v == max_dist][0]:.{decimal_places}f}°")
    print(f"最近距离: {min_dist} mm @ {[k for k,v in scan_dict.items() if v == min_dist][0]:.{decimal_places}f}°")
    print(f"平均距离: {avg_dist:.2f} mm")
    
    if show_all:
        # 打印完整数据表
        print("\n详细数据（角度 → 距离）：")
        print("-"*35)
        for i, angle in enumerate(sorted_angles, 1):
            # 按列格式化输出
            print(f"{angle:{decimal_places+4}.{decimal_places}f}° : {scan_dict[angle]:>6} mm", end=' | ')
            if i % 3 == 0:  # 每行显示3个数据点
                print()
        print("\n" + "="*60)
        
    # 添加极坐标示意图
    print("\n极坐标示意图（简化版）：")
    degrees = [0, 90, 180, 270]
    for d in degrees:
        nearest = min(sorted_angles, key=lambda x: abs(x-d))
        print(f"{d:>3}°方向: {scan_dict[nearest]} mm")

if __name__ == '__main__':
    # 使用示例
    lidar = UbuntuLidarProcessor('/dev/ttyS0')  # 根据实际设备修改
    
    try:
        for scan_count, scan_data in enumerate(lidar.continuous_scan()):
            print(f"\n=== 第 {scan_count+1} 次扫描 ===")
            
           # 获取扫描数据
            full_scan = lidar.get_full_scan()

            # 打印详细数据（显示3位小数）
            print_scan_data(full_scan, decimal_places=2, show_all=True)

            # 快速查看摘要
            print_scan_data(full_scan) 
            
    except serial.SerialException as e:
        print(f"串口通信错误: {str(e)}")