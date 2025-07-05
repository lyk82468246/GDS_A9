# 带垂直控制的CanMV K210虚拟云台系统
import sensor, image, time, math

# ===== 视觉参数配置 =====
WHITE_THRESHOLD = (70, 100, -10, 10, -10, 10)  # LAB白区阈值
H_FOV = 60    # 水平视场角（度）
V_FOV = 40    # 垂直视场角（度）
CENTER_X = 160  # 水平中心（QVGA 320x240）
CENTER_Y = 120  # 垂直中心

# ===== 云台限制参数 =====
PAN_RANGE = (0, 180)    # 水平转动范围
TILT_RANGE = (60, 120)  # 垂直转动范围（防止俯角过大）

# ===== 初始化摄像头 =====
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=1000)

# ===== 虚拟舵机控制系统 =====
class VirtualGimbal:
    def __init__(self):
        # 初始化虚拟舵机状态
        self.pan_angle = 90    # 水平初始角度（居中）
        self.tilt_angle = 90   # 垂直初始角度

    def move_servos(self, pan_delta, tilt_delta):
        """相对角度移动"""
        # 水平控制
        new_pan = self.pan_angle + pan_delta
        self.pan_angle = max(PAN_RANGE[0], min(PAN_RANGE[1], new_pan))

        # 垂直控制
        new_tilt = self.tilt_angle + tilt_delta
        self.tilt_angle = max(TILT_RANGE[0], min(TILT_RANGE[1], new_tilt))

        # 打印状态
        print("云台状态 >> 水平：{0:.1f}° 垂直：{1:.1f}°".format(self.pan_angle, self.tilt_angle))

    def absolute_move(self, pan_angle, tilt_angle):
        """绝对角度控制"""
        self.pan_angle = max(PAN_RANGE[0], min(PAN_RANGE[1], pan_angle))
        self.tilt_angle = max(TILT_RANGE[0], min(TILT_RANGE[1], tilt_angle))
        print("云台定位 >> 水平：{0}° 垂直：{1}°".format(pan_angle, tilt_angle))

# ===== 主程序逻辑 =====
def main():
    gimbal = VirtualGimbal()

    while True:
        img = sensor.snapshot()

        # 白块检测
        blobs = img.find_blobs([WHITE_THRESHOLD],
                              pixels_threshold=500,
                              area_threshold=500,
                              merge=True)

        if blobs:
            max_blob = max(blobs, key=lambda b: b.area())
            cx, cy = max_blob.cx(), max_blob.cy()

            # 计算角度偏差
            h_pixel_per_deg = 320 / H_FOV  # 水平每度像素数
            v_pixel_per_deg = 240 / V_FOV  # 垂直每度像素数

            pan_offset = (CENTER_X - cx) / h_pixel_per_deg
            tilt_offset = (CENTER_Y - cy) / v_pixel_per_deg

            # 生成控制指令（比例因子避免过调）
            pan_delta = pan_offset * 0.8
            tilt_delta = tilt_offset * 0.6

            # 执行虚拟舵机控制
            gimbal.move_servos(pan_delta, -tilt_delta)  # 垂直方向取反

            # 绘制调试图形
            img.draw_rectangle(max_blob.rect(), color=(255,0,0))
            img.draw_cross(cx, cy, color=(0,255,0))
            img.draw_line(CENTER_X, 0, CENTER_X, 240, color=(255,255,0))  # 垂直中线
            img.draw_line(0, CENTER_Y, 320, CENTER_Y, color=(255,255,0))  # 水平中线
            print("检测到目标偏移：水平 {0:.1f}° 垂直 {1:.1f}°".format(pan_offset, tilt_offset))
        else:
            # 扫描模式
            gimbal.move_servos(2, 0)  # 水平自动扫描
            print("未检测到目标，进入扫描模式...".format())

        time.sleep_ms(100)

if __name__ == "__main__":
    main()
