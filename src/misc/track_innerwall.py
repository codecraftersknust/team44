import math
import time
from rplidar import RPLidar
import serial

# LIDAR and Serial Port
PORT = '/dev/ttyUSB1'
SERIAL_PORT = '/dev/ttyUSB0'  # UART port for Arduino
BAUD_RATE = 9600 # baud rate for the serial communication

# PID Parameters
Kp = 0.5
Ki = 0.0
Kd = 0.01

# Control variables
prev_error = 0
integral = 0
desired_distance_diff = 0  # target: equal distance to both walls (centered)
dt = 0.1  # Loop delay in seconds

# Servo range
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135

BASE_THROTTLE = 1650

# Create lidar and serial objects
lidar = RPLidar(PORT)
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def send_to_arduino(throttle, steer):
    cmd = f"<THROTTLE:{int(throttle)}><STEER:{int(steer)}>"
    ser.write(cmd.encode())

def average_distance(scan, angle_range):
    readings = [
        dist for (_, angle, dist) in scan
        if angle_range[0] <= angle <= angle_range[1] and dist > 50  # ignore near-zero junk
    ]
    if len(readings) == 0:
        return None
    return sum(readings) / len(readings)

def pid_control(error, prev_error, integral, Kp, Ki, Kd, dt):
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral

try:
    lidar.start_motor()
    print("Starting PID wall-following...")

    for scan in lidar.iter_scans():
        # Get average distances to left and right walls
        left_dist = average_distance(scan, (265, 310))  # LEFT

        if left_dist is not None:
            # Error = how far off center the car is (+ = too close to right)
            error = left_dist - 150

            control, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt)
            prev_error = error

            # Map PID output to steering angle
            # Assume max error around ±500mm maps to full steering range
            control = max(min(control, 500), -500)  # Clamp control effort
            steering_angle = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
            steering_angle = max(min(steering_angle, STEERING_MAX), STEERING_MIN)

            # Send steering command to Arduino
            steer = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
            steer = max(min(steer, STEERING_MAX), STEERING_MIN)
            send_to_arduino(BASE_THROTTLE, steer)

            # Debug
            print(f"Left: {left_dist:.1f}mm, Error: {error:.1f}, Steering: {steering_angle:.1f}")

        time.sleep(dt)

except KeyboardInterrupt:
    print("Interrupted by user. Stopping...")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    ser.close()
    print("Shutdown complete.")