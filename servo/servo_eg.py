from servo import move_servo

# 设置 X 轴舵机为 45 度
move_servo(0, 45)

# 设置 Y 轴舵机为 -30 度 
move_servo(1, -30)

print("test")

import servo_control

# 进行软复位
servo_control.soft_reset()

# 设置舵机通道 1 旋转到 45 度
servo_control.set_servo_angle(1, 45)