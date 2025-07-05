import smbus
import time

# I2C 总线和设备地址
BUS_NUMBER = 1
DEVICE_ADDRESS = 0x80

# 软复位指令
SOFT_RESET_COMMAND = [0xFB, 0xFB]

# 初始化 I2C 总线
bus = smbus.SMBus(BUS_NUMBER)


def soft_reset():
    """
    对 LU9685 - 20CU 芯片进行软复位
    """
    try:
        bus.write_i2c_block_data(DEVICE_ADDRESS, 0, SOFT_RESET_COMMAND)
        time.sleep(0.1)  # 等待复位完成
    except Exception as e:
        print(f"软复位出错: {e}")


def set_servo_angle(servo_channel, angle):
    """
    设置指定舵机通道的角度

    :param servo_channel: 舵机通道，范围 0 - 19
    :param angle: 舵机角度，范围 0 - 180，大于 200 为不输出 PWM 信号
    """
    if not (0 <= servo_channel <= 19):
        print("舵机通道号超出范围 (0 - 19)")
        return
    if not (0 <= angle <= 180) and angle <= 200:
        print("舵机角度超出范围 (0 - 180) 或大于 200")
        return
    try:
        command = [servo_channel, angle]
        bus.write_i2c_block_data(DEVICE_ADDRESS, 0, command)
    except Exception as e:
        print(f"设置舵机角度出错: {e}")


if __name__ == "__main__":
    # 进行软复位
    soft_reset()
    # 设置舵机通道 0 旋转到 90 度
    set_servo_angle(0, 90)    