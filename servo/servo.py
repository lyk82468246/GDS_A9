import smbus
import time

# 舵机管脚定义
SERVO_X_CHANNEL = 0
SERVO_Y_CHANNEL = 1

# LU9685-20CU 寄存器地址
REG_MODE1 = 0x00
REG_MODE2 = 0x01
REG_SUBADR1 = 0x02
REG_SUBADR2 = 0x03
REG_SUBADR3 = 0x04
REG_ALLCALLADR = 0x05
REG_CHANNEL_START = 0x06

# 舵机脉宽范围 (单位: 微秒)
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500

class DualAxisServoController:
    def __init__(self, i2c_bus=1, address=0x40):
        self.bus = smbus.SMBus(i2c_bus)
        self.address = address
        self.init_device()

    def init_device(self):
        # 设置 MODE1 和 MODE2 寄存器
        self.bus.write_byte_data(self.address, REG_MODE1, 0x00)
        self.bus.write_byte_data(self.address, REG_MODE2, 0x04)

    def set_servo_angle(self, channel, angle):
        """
        设置舵机角度
        :param channel: 舵机通道 (0 或 1)
        :param angle: 舵机角度 (-90 到 90 度)
        """
        pulse_width = self.angle_to_pulse_width(angle)
        reg_start = REG_CHANNEL_START + 4 * channel
        self.bus.write_word_data(self.address, reg_start, int(pulse_width))

    def angle_to_pulse_width(self, angle):
        """
        角度转换为脉宽
        :param angle: 舵机角度 (-90 到 90 度)
        :return: 脉宽值 (微秒)
        """
        pulse_width = (angle + 90) / 180 * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) + SERVO_MIN_PULSE
        return pulse_width

def move_servo(servo_num, angle):
    """
    移动指定的舵机到指定角度
    :param servo_num: 舵机编号 (0 或 1)
    :param angle: 舵机角度 (-90 到 90 度)
    """
    controller = DualAxisServoController()
    if servo_num == 0:
        controller.set_servo_angle(SERVO_X_CHANNEL, angle)
    elif servo_num == 1:
        controller.set_servo_angle(SERVO_Y_CHANNEL, angle)
    else:
        raise ValueError("无效的舵机编号,应为 0 或 1")

#Here comes my code
move_servo(0, 45)
print("test")