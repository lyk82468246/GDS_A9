# example.py
from motor import MecanumCar
import time

# 创建麦轮小车对象
car = MecanumCar()

try:
    # 设置车轮物理参数（根据实际硬件调整）
    car.set_wheel_parameters(
        circumference=20.0,     # 车轮周长(cm)
        pulses_per_rev=20,      # 每转一圈的脉冲数
        wheel_base=15.0,        # 轮距(cm)
        wheel_track=15.0        # 轴距(cm)
    )
    
    # 调整PID参数
    car.tune_pid(kp=0.6, ki=0.3, kd=0.1)
    car.tune_rotation_pid(kp=0.8, ki=0.3, kd=0.2)
    
    # 1. 前进 (90度)，速度50%
    print("前进 (90度)")
    car.move(90, 50)
    time.sleep(2)
    car.stop()
    time.sleep(1)
    
    # 2. 相对于当前位置，向右旋转90度
    print("向右旋转90度")
    car.rotate(-90, 40)  # 负值表示顺时针
    car.wait_for_rotation_complete()
    time.sleep(1)
    
    # 3. 相对于当前位置，前进 (90度是当前朝向的正前方)
    print("相对于当前位置前进")
    car.move(90, 50)
    time.sleep(2)
    car.stop()
    time.sleep(1)
    
    # 4. 相对于当前位置，向左旋转180度
    print("向左旋转180度")
    car.rotate(180, 30)
    car.wait_for_rotation_complete()
    time.sleep(1)
    
    # 5. 相对于当前位置，向左平移 (180度现在是当前朝向的正左方)
    print("向左平移")
    car.move(180, 40)
    time.sleep(2)
    car.stop()
    
except KeyboardInterrupt:
    print("程序被用户中断")
finally:
    car.cleanup()