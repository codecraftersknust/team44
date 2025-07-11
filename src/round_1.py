import pyrealsense2 as rs
import numpy as np
import cv2
import time
import serial
from rplidar import RPLidar

# ---------- Configuration ----------
# Devices
REALSENSE_RES = (640, 480)
LIDAR_PORT = '/dev/ttyUSB1'
ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# HSV range for orange line detection
ORANGE_LOW = np.array([10, 100, 100])
ORANGE_HIGH = np.array([25, 255, 255])
THRESHOLD = 30000  # Pixel count threshold for orange

# PID Constants
Kp = 0.5
Ki = 0.0
Kd = 0.01
dt = 0.1  # seconds

# Steering limits
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135
BASE_THROTTLE = 1650
STOP_THROTTLE = 1500

# ---------- Setup ----------
# RealSense camera setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# LIDAR
lidar = RPLidar(LIDAR_PORT)

# Serial to Arduino
ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# ---------- State ----------
prev_error = 0
integral = 0
lap_count = 0
orange_count = 0
last_detect_time = 0
orange_detected = False

# ---------- Helper Functions ----------
def send_to_arduino(throttle, steer):
    cmd = f"<THROTTLE:{int(throttle)}><STEER:{int(steer)}>"
    ser.write(cmd.encode())

def detect_orange_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv[frame.shape[0] - 80:frame.shape[0], :]  # Bottom portion of frame
    mask = cv2.inRange(roi, ORANGE_LOW, ORANGE_HIGH)
    return np.sum(mask) > THRESHOLD

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        if detect_orange_line(frame):
            if not orange_detected and time.time() - last_detect_time > 2:
                orange_count += 1
                last_detect_time = time.time()
                orange_detected = True
                print(f"Orange line detected! Count: {orange_count}")

                if orange_count == 4:
                    lap_count += 1
                    orange_count = 0
                    print(f"Lap completed! Total laps: {lap_count}")
        else:
            orange_detected = False

def average_distance(scan, angle_range):
    readings = [
        dist for (_, angle, dist) in scan
        if angle_range[0] <= angle <= angle_range[1] and dist > 50
    ]
    return sum(readings) / len(readings) if readings else None

def pid_control(error, prev_error, integral, Kp, Ki, Kd, dt):
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral

# ---------- Main Loop ----------
try:
    lidar.start_motor()
    print("Starting navigation loop...")

    for scan in lidar.iter_scans():
        # Read RealSense frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_img = np.asanyarray(color_frame.get_data())
        detect_orange_line(color_img)

        if lap_count >= 3:
            print("Lap goal reached. Stopping robot.")
            send_to_arduino(STOP_THROTTLE, STEERING_CENTER)
            break

        # Wall following with LIDAR
        left = average_distance(scan, (265, 310))

        if left:
            error = left - 150  # Desired distance from the wall

            control, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt)
            prev_error = error

            control = max(min(control, 500), -500)
            steer = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
            steer = max(min(steer, STEERING_MAX), STEERING_MIN)

            send_to_arduino(BASE_THROTTLE, steer)

            # Debug info
            print(f"Left: {left:.1f}mm | Err: {error:.1f} | Steer: {steer:.1f} | Lap: {lap_count}")

        time.sleep(dt)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    print("Shutting down...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    ser.close()
    cv2.destroyAllWindows()
