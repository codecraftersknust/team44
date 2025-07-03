
import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Setup RealSense RGB stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# HSV range for orange (adjust based on lighting)
ORANGE_LOW = np.array([10, 100, 100])
ORANGE_HIGH = np.array([25, 255, 255])
THRESHOLD = 30000  # Minimum pixel count to detect orange

lap_count = 0
orange_count = 0
last_detect_time = 0
orange_detected = False

def detect_orange_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv[frame.shape[0] - 80:frame.shape[0], :]  # Bottom portion of frame
    mask = cv2.inRange(roi, ORANGE_LOW, ORANGE_HIGH)
    return np.sum(mask) > THRESHOLD

try:
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
                print(f"🟧 Orange line detected! Count: {orange_count}")

                if orange_count == 4:
                    lap_count += 1
                    orange_count = 0
                    print(f"🏁 Lap completed! Total laps: {lap_count}")
        else:
            orange_detected = False

        # Visual feedback
        cv2.rectangle(frame, (0, frame.shape[0] - 80), (frame.shape[1], frame.shape[0]), (0, 140, 255), 2)
        cv2.putText(frame, f"Laps: {lap_count}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        cv2.imshow("Lap Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()