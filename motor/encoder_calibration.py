# encoder_calibration.py
import RPi.GPIO as GPIO
import time
import signal
import sys

# 设置GPIO模式
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 配置参数
ENCODER_PIN = 5  # 根据您的接线修改为实际使用的编码器引脚
MOTOR_CONTROL_PINS = (17, 18)  # 电机控制引脚 (IN1, IN2)
MOTOR_PWM_PIN = 12  # 电机PWM控制引脚

# 全局变量
pulse_count = 0
start_time = 0
running = True

# 设置GPIO引脚
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_CONTROL_PINS[0], GPIO.OUT)
GPIO.setup(MOTOR_CONTROL_PINS[1], GPIO.OUT)
GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)

# 创建PWM对象
motor_pwm = GPIO.PWM(MOTOR_PWM_PIN, 100)  # 100Hz频率

def pulse_callback(channel):
    """编码器脉冲回调函数"""
    global pulse_count
    pulse_count += 1
    
def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global running
    running = False
    print("\n停止测量")
    GPIO.cleanup()
    sys.exit(0)

def set_motor_direction(forward=True):
    """设置电机方向"""
    if forward:
        GPIO.output(MOTOR_CONTROL_PINS[0], GPIO.HIGH)
        GPIO.output(MOTOR_CONTROL_PINS[1], GPIO.LOW)
    else:
        GPIO.output(MOTOR_CONTROL_PINS[0], GPIO.LOW)
        GPIO.output(MOTOR_CONTROL_PINS[1], GPIO.HIGH)

def main():
    global pulse_count, start_time, running
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    print("电机编码器校准程序")
    print("------------------")
    print(f"使用的编码器引脚: GPIO {ENCODER_PIN}")
    print("此程序将测量电机旋转时产生的脉冲数")
    print("请确保电机可以自由旋转，且编码器已正确连接")
    
    # 添加脉冲计数事件
    GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING, callback=pulse_callback)
    
    while running:
        print("\n选择操作模式:")
        print("1. 手动模式 - 手动旋转电机并计数")
        print("2. 自动模式 - 电机低速运行固定时间")
        print("3. 标记模式 - 标记起始点并计数完整的一圈")
        print("4. 退出程序")
        
        choice = input("请选择 (1-4): ")
        
        if choice == '1':
            # 手动模式
            pulse_count = 0
            input("请手动旋转电机，完成后按回车键...")
            print(f"检测到 {pulse_count} 个脉冲")
            
        elif choice == '2':
            # 自动模式
            speed = float(input("请输入电机速度百分比 (1-100): "))
            speed = max(1, min(100, speed))
            duration = float(input("请输入运行时间 (秒): "))
            
            pulse_count = 0
            print(f"电机将以 {speed}% 的速度运行 {duration} 秒...")
            
            # 设置电机方向和速度
            set_motor_direction(True)
            motor_pwm.start(speed)
            
            start_time = time.time()
            try:
                while (time.time() - start_time) < duration and running:
                    time.sleep(0.1)
                    sys.stdout.write(f"\r已运行: {time.time() - start_time:.1f}秒, 脉冲数: {pulse_count}   ")
                    sys.stdout.flush()
            finally:
                motor_pwm.ChangeDutyCycle(0)
                print(f"\n检测到 {pulse_count} 个脉冲，运行时间: {time.time() - start_time:.2f} 秒")
                
        elif choice == '3':
            # 标记模式
            print("标记模式 - 请在电机的某个位置做一个明显标记")
            print("程序将计算电机从标记点旋转回到标记点的脉冲数")
            
            speed = float(input("请输入电机速度百分比 (1-15建议): "))
            speed = max(1, min(15, speed))  # 限制在低速范围内，方便观察
            
            input("准备好后按回车键开始...")
            
            # 启动电机
            set_motor_direction(True)
            motor_pwm.start(speed)
            
            # 开始计数
            pulse_count = 0
            print("电机开始旋转，当看到标记点再次出现时，请按回车键...")
            input()
            
            # 停止电机
            motor_pwm.ChangeDutyCycle(0)
            
            print(f"一圈的脉冲数为: {pulse_count}")
            print(f"建议在MecanumCar类的set_wheel_parameters方法中设置pulses_per_rev={pulse_count}")
            
        elif choice == '4':
            running = False
            break
            
        else:
            print("无效选择，请重试")
    
    # 清理GPIO
    GPIO.cleanup()
    print("程序已退出")

if __name__ == "__main__":
    main()