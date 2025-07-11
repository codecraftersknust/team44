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
Kp = 0.6
Ki = 0.0
Kd = 0.15
dt = 0.1  # seconds

# Steering limits
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135
BASE_THROTTLE = 1655
STOP_THROTTLE = 1000

# ---------- Setup ----------
# RealSense camera setup
rs_pipeline = rs.pipeline()
rs_config = rs.config()
rs_config.enable_stream(rs.stream.color, *REALSENSE_RES, rs.format.bgr8, 30)
rs_pipeline.start(rs_config)

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
    global orange_count, lap_count, last_detect_time, orange_detected
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv[-80:, :]  # bottom of the frame
    mask = cv2.inRange(roi, ORANGE_LOW, ORANGE_HIGH)
    pixel_count = np.sum(mask > 0)

    if pixel_count > THRESHOLD:
        if not orange_detected and time.time() - last_detect_time > 2:
            orange_count += 1
            orange_detected = True
            last_detect_time = time.time()
            print(f"Orange line detected: {orange_count}")
            if orange_count == 4:
                lap_count += 1
                orange_count = 0
                print(f"Lap completed! Total laps: {lap_count}")
    else:
        orange_detected = False

    return mask

def average_distance(scan, angle_range):
    readings = [
        dist for (_, angle, dist) in scan
        if angle_range[0] <= angle <= angle_range[1] and dist > 50
    ]
    return sum(readings) / len(readings) if readings else None

def pid_control(error):
    global prev_error, integral
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    prev_error = error
    return output

# ---------- Main Loop ----------
try:
    lidar.start_motor()
    print("Starting navigation loop...")

    for scan in lidar.iter_scans():
        # Read RealSense frame
        frames = rs_pipeline.wait_for_frames()
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
        left = average_distance(scan, (225, 315))

        if left:
            error = left - 250  # Desired distance from the wall
            control = pid_control(error)
            control = max(min(control, 500), -500)
            steer = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
            steer = max(min(steer, STEERING_MAX), STEERING_MIN)
            send_to_arduino(BASE_THROTTLE, steer)

            # Debug info
            print(f"Left: {left:.1f}mm | Err: {error:.1f} | Steer: {steer:.1f}")

        # Optional visual debug
        cv2.putText(color_img, f"Laps: {lap_count}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        cv2.imshow("RealSense View", color_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(dt)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    print("Shutting down...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    rs_pipeline.stop()
    ser.close()
    cv2.destroyAllWindows()

"""
#include <Servo.h>

Servo esc;       // ESC for throttle (connected to D10)
Servo steering;  // Servo for steering (connected to D9)

String inputString = "";
bool readingCommand = false;

void setup() {
  Serial.begin(9600);
  esc.attach(10);       // ESC signal pin
  steering.attach(9);   // Servo signal pin

  // Initialize with safe defaults
  esc.writeMicroseconds(1000);     // Full stop throttle
  steering.write(90);              // Center steering
}

void loop() {
  // Read serial stream character by character
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '<') {
      inputString = "";             // Start a new command
      readingCommand = true;
    } else if (c == '>') {
      readingCommand = false;      // End of command
      parseCommand(inputString);
    } else if (readingCommand) {
      inputString += c;            // Build up command string
    }
  }
}

void parseCommand(String cmd) {
  if (cmd.startsWith("THROTTLE:")) {
    int throttle = cmd.substring(9).toInt();
    throttle = constrain(throttle, 1000, 2000);
    esc.writeMicroseconds(throttle);
  }

  if (cmd.startsWith("STEER:")) {
    int angle = cmd.substring(6).toInt();
    angle = constrain(angle, 45, 135);  // Safe turning range
    steering.write(angle);
  }
}
"""
