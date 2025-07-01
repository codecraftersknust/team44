import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Give Arduino time to reset

def send_angle(angle):
    angle = max(0, min(180, int(angle)))  # Clamp angle
    command = f"{angle}\n"
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")

# Example test: Move servo back and forth
while True:
    angle = input("Enter angle (0-180): ")
    if angle.lower() == 'q':
        break
    send_angle(angle)

ser.close()

