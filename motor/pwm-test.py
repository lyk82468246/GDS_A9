import RPi.GPIO as GPIO
import time
import threading
import numpy
import matplotlib.pyplot as plt

# 引脚重新定义（移除EA、EB）
I1, I2, I3, I4, LS, RS = (23, 24, 20, 21, 18, 16)
FREQUENCY = 50
GPIO.setmode(GPIO.BCM)
GPIO.setup([I1, I2, I3, I4], GPIO.OUT)
GPIO.setup([LS, RS], GPIO.IN)

# 初始化方向控制（I2和I3保持低电平）
GPIO.output([I2, I3], GPIO.LOW)

# 创建PWM对象（I1控制左电机，I4控制右电机）
pwma = GPIO.PWM(I1, FREQUENCY)
pwmb = GPIO.PWM(I4, FREQUENCY)
pwma.start(0)
pwmb.start(0)

lspeed = 0
rspeed = 0
lcounter = 0
rcounter = 0

class PID:
    """PID控制器（保持原有实现不变）"""
    def __init__(self, P=80, I=0, D=0, speed=0.4, duty=26):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.err_pre = 0
        self.err_last = 0
        self.u = 0
        self.integral = 0
        self.ideal_speed = speed
        self.last_duty = duty
        self.pre_duty = duty

    def update(self, feedback_value):
        self.err_pre = self.ideal_speed - feedback_value
        self.integral += self.err_pre
        self.u = self.Kp*self.err_pre + self.Ki*self.integral + self.Kd*(self.err_pre-self.err_last)
        self.err_last = self.err_pre
        self.pre_duty = self.last_duty + self.u
        self.pre_duty = max(0, min(100, self.pre_duty))  # 限制占空比范围
        self.last_duty = self.pre_duty
        return self.pre_duty

    # 以下setter方法保持不变...
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

def my_callback(channel):
    global lcounter, rcounter
    if channel == LS:
        lcounter += 1
    elif channel == RS:
        rcounter += 1

def getspeed():
    global rspeed, lspeed, lcounter, rcounter
    GPIO.add_event_detect(LS, GPIO.RISING, callback=my_callback)
    GPIO.add_event_detect(RS, GPIO.RISING, callback=my_callback)
    while True:
        rspeed = rcounter / 585.0
        lspeed = lcounter / 585.0
        rcounter = 0
        lcounter = 0
        time.sleep(0.1)

thread1 = threading.Thread(target=getspeed)
thread1.start()

# 初始化PID控制器
i = 0
x = []
y1 = []
y2 = []
speed = 0.2
l_origin_duty = 5
r_origin_duty = 5

L_control = PID(45, 3.20, 200, speed, l_origin_duty)
R_control = PID(45, 3.20, 200, speed, r_origin_duty)

try:
    while True:
        # 更新PWM占空比到I1和I4引脚
        pwma.ChangeDutyCycle(L_control.update(lspeed))
        pwmb.ChangeDutyCycle(R_control.update(rspeed))
        
        # 数据记录
        x.append(i)
        y1.append(lspeed)
        y2.append(rspeed)
        time.sleep(0.1)
        i += 0.1
        print(f'Left: {lspeed:.2f}  Right: {rspeed:.2f}  L_Duty: {L_control.pre_duty:.1f}  R_Duty: {R_control.pre_duty:.1f}')

except KeyboardInterrupt:
    pass

pwma.stop()
pwmb.stop()
GPIO.cleanup()

# 绘制图表
plt.plot(x, y1, '-o', label='Left Speed')
plt.plot(x, y2, '-*', label='Right Speed')
plt.legend()
plt.show()