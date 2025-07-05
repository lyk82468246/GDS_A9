from motor import MecanumCar
import time
import serial
import threading
import bisect
from collections import defaultdict

class StableLidarProcessor:
    def __init__(self, port='/dev/ttyS0', baudrate=230400):
        self.ser = serial.Serial(port, baudrate, timeout=5)
        self._scan_dict = dict.fromkeys(range(360), (0, 0))
        self._raw_points = []
        self._last_angle = 0
        self._scan_started = False
        self._lock = threading.Lock()
        self._running = True

        self._thread = threading.Thread(target=self._process_data)
        self._thread.daemon = True
        self._thread.start()

    def _parse_frame(self, data):
        try:
            start = (int.from_bytes(data[2:4], byteorder='little')) / 100.0
            end = (int.from_bytes(data[40:42], byteorder='little')) / 100.0

            points = []
            for i in range(12):
                offset = 4 + i*3
                if offset+2 >= len(data):
                    break
                
                dist_low = data[offset]
                dist_high = data[offset+1]
                distance = (dist_high << 8) | dist_low  # 毫米单位
                intensity = data[offset+2]

                if distance > 0:
                    angle_diff = end - start if start <= end else (360 - start) + end
                    angle = (start + (angle_diff / 11) * i) % 360
                    points.append((round(angle, 2), distance, intensity))

            return {'start': start, 'end': end, 'points': points}
        except Exception as e:
            print(f"解析异常: {str(e)}")
            return {'start':0, 'end':0, 'points':[]}

    def _process_data(self):
        while self._running:
            try:
                header = self.ser.read(2)
                if header == b'\x54\x2C':
                    data = self.ser.read(45)
                    if len(data) == 45:
                        frame = self._parse_frame(data)
                        
                        if frame['start'] < 5 and not self._scan_started:
                            self._scan_started = True
                            self._raw_points = []

                        if self._scan_started:
                            self._raw_points.extend(frame['points'])
                            
                            if self._last_angle > 355 and frame['start'] < 5:
                                self._scan_started = False
                                self._generate_scan_dict()

                        self._last_angle = frame['end']
            except Exception as e:
                print(f"处理异常: {str(e)}")

    def _generate_scan_dict(self):
        if not self._raw_points:
            return

        sorted_points = sorted(self._raw_points, key=lambda x: x[0])
        angles = [p[0] for p in sorted_points]
        
        final_dict = {}
        
        for target_angle in range(360):
            idx = bisect.bisect_left(angles, target_angle)
            candidates = []
            
            search_range = 2
            start_idx = max(0, idx - search_range)
            end_idx = min(len(sorted_points), idx + search_range)
            
            for p in sorted_points[start_idx:end_idx]:
                angle_diff = abs(p[0] - target_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                if angle_diff <= search_range:
                    candidates.append(p)

            if candidates:
                distances = sorted([p[1] for p in candidates])
                median_dist = distances[len(distances)//2]
                final_dict[target_angle] = (median_dist, 50)
            else:
                final_dict[target_angle] = (0, 0)

        with self._lock:
            self._scan_dict = final_dict

    @property
    def scan_data(self):
        with self._lock:
            return self._scan_dict.copy()

    def shutdown(self):
        self._running = False
        self.ser.close()

def find_opponent_car(around):
    MIN_CAR_WIDTH = 12
    MAX_CAR_WIDTH = 30
    FIELD_SIZE = 400

    potential_cars = []
    current_region = None
    
    sorted_angles = sorted(around.keys())
    sorted_angles = sorted_angles + [sorted_angles[0] + 360]
    
    for i in range(len(sorted_angles) - 1):
        angle = sorted_angles[i] % 360
        distance = around[angle]
        
        if distance < FIELD_SIZE:
            if current_region is None:
                current_region = {
                    'start_angle': angle,
                    'end_angle': angle,
                    'min_distance': distance,
                    'angles': [angle]
                }
            else:
                if (sorted_angles[i] - sorted_angles[i-1]) <= 2:
                    current_region['end_angle'] = angle
                    current_region['min_distance'] = min(current_region['min_distance'], distance)
                    current_region['angles'].append(angle)
                else:
                    finish_region(current_region, around)
                    potential_cars.append(current_region)
                    current_region = {
                        'start_angle': angle,
                        'end_angle': angle,
                        'min_distance': distance,
                        'angles': [angle]
                    }
        elif current_region is not None:
            finish_region(current_region, around)
            potential_cars.append(current_region)
            current_region = None

    if current_region is not None:
        finish_region(current_region, around)
        potential_cars.append(current_region)
    
    car_regions = []
    for region in potential_cars:
        if region['start_angle'] > region['end_angle']:
            region['angle_width'] = (region['end_angle'] + 360) - region['start_angle']
        else:
            region['angle_width'] = region['end_angle'] - region['start_angle']
            
        arc_length = (region['angle_width'] / 360) * 2 * 3.14159 * region['min_distance']
        
        if MIN_CAR_WIDTH <= arc_length <= MAX_CAR_WIDTH:
            car_regions.append(region)
    
    if car_regions:
        closest_region = min(car_regions, key=lambda r: r['min_distance'])
        center_angle = calculate_center_angle(closest_region['start_angle'], closest_region['end_angle'])
        print(f"[雷达] 发现目标 | 角度范围: {closest_region['start_angle']}-{closest_region['end_angle']}° | 距离: {closest_region['min_distance']}cm")
        return center_angle
    
    return None

def finish_region(region, around):
    region['avg_distance'] = sum(around[a] for a in region['angles']) / len(region['angles'])

def calculate_center_angle(start_angle, end_angle):
    if start_angle > end_angle:
        center = (start_angle + (end_angle + 360)) / 2
        if center >= 360:
            center -= 360
    else:
        center = (start_angle + end_angle) / 2
    return center

def face_opponent(car, around):
    opponent_angle = find_opponent_car(around)
    
    if opponent_angle is not None:
        rotation_angle = opponent_angle
        
        if rotation_angle > 180:
            rotation_angle -= 360
        
        print(f"[控制] 旋转角度: {rotation_angle:.1f}° | 目标角度: {opponent_angle:.1f}°")
        car.rotate(rotation_angle, 30)
        car.wait_for_rotation_complete()
        print("[控制] 旋转完成")
        return opponent_angle
    else:
        print("[雷达] 扫描中...")
        return None

def main():
    car = MecanumCar()
    lidar = StableLidarProcessor()

    try:
        car.set_wheel_parameters(
            circumference=20.0,
            pulses_per_rev=20,
            wheel_base=15.0,
            wheel_track=15.0
        )
        car.tune_pid(kp=0.6, ki=0.3, kd=0.1)
        car.tune_rotation_pid(kp=0.8, ki=0.3, kd=0.2)

        while True:
            scan_data = lidar.scan_data
            around = {}
            for angle in range(360):
                dist_mm, _ = scan_data.get(angle, (0, 0))
                around[angle] = 401 if dist_mm == 0 else dist_mm / 10  # 毫米转厘米，无效数据设为401cm

            opponent_angle = face_opponent(car, around)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("程序终止")
    finally:
        car.cleanup()
        lidar.shutdown()

if __name__ == "__main__":
    main()