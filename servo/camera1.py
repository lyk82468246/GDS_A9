# 基于Adafruit ServoKit的二维云台控制系统

# 导入必要库
import time
import busio
from board import SCL, SDA
from adafruit_servokit import ServoKit
import sensor
import image

class PID:
    def __init__(self, p=0.0, i=0.0, d=0.0, imax=0.0):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.Imax = imax

        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0

    def get_pid(self, error, dt):
        if dt <= 0:
            return 0.0

        # 计算微分项(带噪声抑制)
        derivative = (error - self._last_error) / dt
        derivative = self._last_derivative + 0.5*(derivative - self._last_derivative)

        # 计算积分项(带限幅)
        self._integral += error * dt
        if self.Imax != 0:
            self._integral = min(max(self._integral, -self.Imax), self.Imax)

        # 计算输出
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)

        # 保存状态
        self._last_error = error
        self._last_derivative = derivative

        return output

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0



# ========== 硬件配置 ========== #
# I2C初始化
i2c_bus = busio.I2C(SCL, SDA)

# 云台舵机配置
SERVO_CHANNELS = 16      # PCA9685通道数
PAN_CHANNEL = 0          # 水平通道
TILT_CHANNEL = 1         # 垂直通道

# 初始化ServoKit
kit = ServoKit(channels=SERVO_CHANNELS, i2c=i2c_bus)

# ========== 参数配置 ========== #
# 云台参数
PAN_RANGE = (0, 180)    # 水平角度范围
TILT_RANGE = (60, 120)  # 垂直角度范围
SERVO_FREQ = 50         # PWM频率(Hz)

# PID参数
PAN_PID = PID(p=0.5, i=0.01, d=0.05)
TILT_PID = PID(p=0.5, i=0.01, d=0.05)

class GimbalController:
    def __init__(self):
        # 配置舵机参数
        self._configure_servo(PAN_CHANNEL, PAN_RANGE)
        self._configure_servo(TILT_CHANNEL, TILT_RANGE)

        # 初始化位置
        self.pan_angle = 90
        self.tilt_angle = 90
        self._update_position()

    def _configure_servo(self, channel, angle_range):
        """配置舵机参数"""
        kit.servo[channel].set_pulse_width_range(500, 2500)
        kit.servo[channel].actuation_range = angle_range[1] - angle_range[0]
        kit.servo[channel].angle = 0  # 初始归零

    def _update_position(self):
        """更新云台位置"""
        kit.servo[PAN_CHANNEL].angle = self.pan_angle
        kit.servo[TILT_CHANNEL].angle = self.tilt_angle

    def move(self, pan_delta, tilt_delta):
        """相对移动云台"""
        self.pan_angle = max(PAN_RANGE[0], min(PAN_RANGE[1], self.pan_angle + pan_delta))
        self.tilt_angle = max(TILT_RANGE[0], min(TILT_RANGE[1], self.tilt_angle + tilt_delta))
        self._update_position()

    def track_target(self, target_x, target_y, frame_center=(160, 120)):
        """PID目标追踪"""
        # 计算误差
        pan_error = target_x - frame_center[0]
        tilt_error = target_y - frame_center[1]

        # 计算PID输出
        pan_output = PAN_PID.update(pan_error)
        tilt_output = TILT_PID.update(tilt_error)

        # 更新位置
        self.pan_angle -= pan_output
        self.tilt_angle += tilt_output
        self._update_position()

# ========== 视觉系统集成 ========== #
class VisionSystem:
    def __init__(self):
        # 摄像头初始化
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(30)

        # 云台控制器
        self.gimbal = GimbalController()

    def run_tracking(self):
        while True:
            img = sensor.snapshot()

            # 目标检测逻辑（示例：颜色追踪）
            target = self._detect_target(img)

            if target:
                self.gimbal.track_target(target.cx(), target.cy())
            else:
                self.gimbal.move(5, 0)  # 扫描模式

    def _detect_target(self, img):
        # 示例：红色物体检测
        thresholds = [(30, 60, 15, 50, 15, 50)]  # 根据实际调整
        blobs = img.find_blobs(thresholds, pixels_threshold=200)
        return max(blobs, key=lambda b: b.pixels()) if blobs else None

# ========== 主程序 ========== #
if __name__ == "__main__":
    system = VisionSystem()
    try:
        system.run_tracking()
    except KeyboardInterrupt:
        # 安全复位云台
        system.gimbal.pan_angle = 90
        system.gimbal.tilt_angle = 90
        system.gimbal._update_position()
        kit.servo[PAN_CHANNEL].angle = None
        kit.servo[TILT_CHANNEL].angle = None
