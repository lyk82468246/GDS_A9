import serial
import re
import time
import threading
from adafruit_servokit import ServoKit
from peripherals import Peripherals

# 初始化舵机控制器
kit = ServoKit(channels=16)

# 初始化外设控制器
peripherals = Peripherals()

# 初始化舵机角度变量 - 使用0号和1号舵机分别作为x和y轴
servo_angles = {0: 75, 1: 90}  # 假设初始角度

# 配置串口
# 注意：可能需要根据实际情况修改串口设备名和波特率
ser = serial.Serial('/dev/ttyCH343USB0', 115200, timeout=1)

# 正则表达式匹配数据格式: $+123-456$
pattern = r'\$([+-]\d{3})([+-]\d{3})\$'

# 舵机运动控制相关变量
servo_moving = {0: False, 1: False}  # 标记舵机是否正在运动
servo_stop_flags = {0: False, 1: False}  # 停止标志
servo_threads = {0: None, 1: None}  # 线程引用
servo_lock = threading.Lock()  # 线程锁

# 胜利条件相关变量
target_angles = {0: 90, 1: 90}  # 目标角度，可以根据需要修改
angle_threshold = 5  # 角度阈值
stable_duration = 2.0  # 稳定持续时间（秒）
stable_start_time = None  # 开始稳定的时间
victory_achieved = False  # 是否已经达成胜利
victory_lock = threading.Lock()  # 胜利条件检查锁

def move_servo_gradually(servo_id, target_angle):
    """逐度移动舵机到目标角度"""
    global servo_angles, servo_moving, servo_stop_flags
    
    current_angle = servo_angles[servo_id]
    
    if current_angle == target_angle:
        return
    
    # 确定移动方向
    step = 1 if target_angle > current_angle else -1
    
    print(f"舵机 {servo_id} 开始从 {current_angle}° 移动到 {target_angle}°")
    
    while current_angle != target_angle:
        # 检查是否需要停止（收到新指令）
        if servo_stop_flags[servo_id]:
            print(f"舵机 {servo_id} 运动被新指令中断，当前位置: {current_angle}°")
            break
            
        # 移动一度
        current_angle += step
        
        # 限制角度范围
        current_angle = max(0, min(180, current_angle))
        
        # 更新舵机位置
        kit.servo[servo_id].angle = current_angle
        
        # 更新保存的角度值
        with servo_lock:
            servo_angles[servo_id] = current_angle
        
        print(f"舵机 {servo_id} 当前角度: {current_angle}°")
        
        # 每度之间的延迟，可以根据需要调整速度
        time.sleep(0.05)  # 50ms延迟，相当于20度/秒的速度
    
    # 运动完成，重置状态
    with servo_lock:
        servo_moving[servo_id] = False
        servo_stop_flags[servo_id] = False
    
    print(f"舵机 {servo_id} 运动完成，最终角度: {servo_angles[servo_id]}°")

def update_servo_angle(servo_id, change):
    """更新舵机角度并控制舵机"""
    global servo_moving, servo_stop_flags, servo_threads
    
    # 如果已经胜利，忽略新的指令
    if victory_achieved:
        print("已达成胜利条件，忽略新指令")
        return
    
    # 计算新角度
    current_angle = servo_angles[servo_id]
    new_angle = current_angle + change
    
    # 限制角度范围在0-180之间
    new_angle = max(0, min(180, new_angle))
    
    # 如果舵机正在运动，停止当前运动
    if servo_moving[servo_id]:
        print(f"舵机 {servo_id} 收到新指令，中断当前运动")
        servo_stop_flags[servo_id] = True
        
        # 等待当前线程结束
        if servo_threads[servo_id] and servo_threads[servo_id].is_alive():
            servo_threads[servo_id].join(timeout=1.0)
    
    # 重置停止标志
    servo_stop_flags[servo_id] = False
    
    # 如果目标角度与当前角度相同，不需要移动
    if new_angle == servo_angles[servo_id]:
        print(f"舵机 {servo_id} 已在目标位置 {new_angle}°")
        return
    
    # 标记为运动状态
    servo_moving[servo_id] = True
    
    # 创建新线程执行逐度移动
    servo_threads[servo_id] = threading.Thread(
        target=move_servo_gradually, 
        args=(servo_id, new_angle)
    )
    servo_threads[servo_id].daemon = True
    servo_threads[servo_id].start()

def check_victory_condition():
    """检查胜利条件"""
    global stable_start_time, victory_achieved
    
    if victory_achieved:
        return
    
    with victory_lock:
        # 检查两个舵机是否都在目标角度阈值范围内
        servo0_in_range = abs(servo_angles[0] - target_angles[0]) <= angle_threshold
        servo1_in_range = abs(servo_angles[1] - target_angles[1]) <= angle_threshold
        
        current_time = time.time()
        
        if servo0_in_range and servo1_in_range:
            # 两个舵机都在阈值范围内
            if stable_start_time is None:
                stable_start_time = current_time
                print(f"舵机位置进入目标范围 (X: {servo_angles[0]}°±{angle_threshold}°, Y: {servo_angles[1]}°±{angle_threshold}°)，开始计时...")
            else:
                # 检查是否已经稳定足够长的时间
                stable_time = current_time - stable_start_time
                if stable_time >= stable_duration:
                    # 达成胜利条件
                    victory_achieved = True
                    print("🎉 胜利！达成目标位置并保持稳定!")
                    print(f"最终位置 - 舵机0(X轴): {servo_angles[0]}°, 舵机1(Y轴): {servo_angles[1]}°")
                    
                    # 触发胜利信号
                    trigger_victory_signal()
                else:
                    print(f"目标范围内稳定时间: {stable_time:.1f}s / {stable_duration}s")
        else:
            # 有舵机离开了阈值范围，重置计时
            if stable_start_time is not None:
                print(f"舵机离开目标范围 (X: {servo_angles[0]}°, Y: {servo_angles[1]}°)，重置计时")
                stable_start_time = None

def trigger_victory_signal():
    """触发胜利信号：蜂鸣器长鸣2秒，LED常亮"""
    print("触发胜利信号：蜂鸣器长鸣2秒，LED常亮")
    
    # LED常亮
    peripherals.led_on()
    
    # 蜂鸣器长鸣2秒
    peripherals.buzzer_on(frequency=800, duration=2.0)

try:
    print("开始监听串口数据...")
    print(f"初始角度 - 舵机0(X轴): {servo_angles[0]}°, 舵机1(Y轴): {servo_angles[1]}°")
    print(f"目标角度 - 舵机0(X轴): {target_angles[0]}°±{angle_threshold}°, 舵机1(Y轴): {target_angles[1]}°±{angle_threshold}°")
    print(f"胜利条件：在目标范围内保持稳定 {stable_duration} 秒")
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
                        print(f"解析指令: X轴变化{x_change:+d}°, Y轴变化{y_change:+d}°")
                        
                        # 更新舵机位置 - 0号为x轴，1号为y轴
                        if x_change != 0:
                            update_servo_angle(0, x_change)
                        if y_change != 0:
                            update_servo_angle(1, y_change)
                    else:
                        print(f"无效数据格式: {command}")
                else:
                    # 数据不完整或格式错误，丢弃开头的$
                    buffer = buffer[start+1:]
        
        # 检查胜利条件
        if not victory_achieved:
            check_victory_condition()
        
        time.sleep(0.01)  # 小延迟，避免CPU占用过高

except KeyboardInterrupt:
    print("程序已停止")
    
    # 停止所有舵机运动
    for servo_id in [0, 1]:
        servo_stop_flags[servo_id] = True
    
    # 等待所有线程结束
    for servo_id in [0, 1]:
        if servo_threads[servo_id] and servo_threads[servo_id].is_alive():
            servo_threads[servo_id].join(timeout=2.0)
    
finally:
    # 清理外设
    peripherals.cleanup()
    ser.close()
    print("串口已关闭，外设已清理")