import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16) #我们用的是16路舵机

kit.servo[0].angle = 180 #设置0号转舵机为180度
time.sleep(1) #等待1秒
kit.continuous_servo[1].throttle = 1 #  设置1号舵机为正转
time.sleep(1)
kit.continuous_servo[1].throttle = -1 # 设置1号舵机为反转
time.sleep(1)
kit.servo[0].angle = 0
kit.continuous_servo[1].throttle = 0 #停止转动

kit.servo[1].angle = 180 #设置1号转舵机为180度
time.sleep(1) #等待1秒
kit.continuous_servo[1].throttle = 1 #  设置1号舵机为正转
time.sleep(1)
kit.continuous_servo[1].throttle = -1 # 设置1号舵机为反转
time.sleep(1)
kit.servo[1].angle = 0
kit.continuous_servo[1].throttle = 0 #停止转动