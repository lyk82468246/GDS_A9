# 使用示例
from mecanum_wheel_car import MecanumCar
import time

# 定义电机引脚
pins = {
    'front_left': {'in1': 27, 'in2': 17, 'encoder_a': 22},
    'front_right': {'in1': 6, 'in2': 5, 'encoder_a': 26},
    'rear_left': {'in1': 23, 'in2': 24, 'encoder_a': 18},
    'rear_right': {'in1': 20, 'in2': 21, 'encoder_a': 16}
}

# 创建麦轮小车对象
car = MecanumCar(pins)

try:
    # 向前移动 (90度) 速度为50%
    car.move(90, 50)
    time.sleep(2)
    
    # 向右前方移动 (45度) 速度为70%
    car.move(45, 70)
    time.sleep(2)
    
    # 原地逆时针旋转 (正90度) 速度为40%
    car.rotate(90, 40)
    time.sleep(2)
    
    # 原地顺时针旋转 (负90度) 速度为40%
    car.rotate(-90, 40)
    time.sleep(2)
    
    # 停止
    car.stop()
    
    # 读取编码器速度
    actual_speeds = car.read_speeds()
    print(f"编码器测量速度: {actual_speeds}")
    
finally:
    # 确保在退出时清理GPIO
    car.stop()
    del car