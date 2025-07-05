import serial
import re
import time
from adafruit_servokit import ServoKit

# 初始化舵机控制器
kit = ServoKit(channels=16)

# 初始化舵机角度变量 - 使用0号和1号舵机分别作为x和y轴
servo_angles = {0: 75, 1: 90}  # 假设初始角度

# 配置串口
# 注意：可能需要根据实际情况修改串口设备名和波特率
ser = serial.Serial('/dev/ttyCH343USB0', 115200, timeout=1)

# 正则表达式匹配数据格式: $+123-456$
pattern = r'\$([+-]\d{3})([+-]\d{3})\$'

def update_servo_angle(servo_id, change):
    """更新舵机角度并控制舵机"""
    # 计算新角度
    new_angle = servo_angles[servo_id] + change
    
    # 限制角度范围在0-180之间
    new_angle = max(0, min(180, new_angle))
    
    # 更新保存的角度值
    servo_angles[servo_id] = new_angle
    
    # 控制舵机
    kit.servo[servo_id].angle = new_angle
    print(f"舵机 {servo_id} 角度更新为: {new_angle}")

try:
    print("开始监听串口数据...")
    buffer = ""
    
    while True:
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
                        #print("")
                        
                        # 更新舵机位置 - 0号为x轴，1号为y轴
                        update_servo_angle(0, x_change)
                        update_servo_angle(1, y_change)
                    else:
                        print(f"无效数据格式: {command}")
                else:
                    # 数据不完整或格式错误，丢弃开头的$
                    buffer = buffer[start+1:]
                    
        time.sleep(0.01)  # 小延迟，避免CPU占用过高

except KeyboardInterrupt:
    print("程序已停止")
finally:
    ser.close()
    print("串口已关闭")