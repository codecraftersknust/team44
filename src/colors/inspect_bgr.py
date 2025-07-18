import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_img = np.asanyarray(color_frame.get_data())

        cv2.imshow("Color", color_img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
