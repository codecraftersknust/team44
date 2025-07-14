import cv2
import pyrealsense2 as rs
import numpy as np

REALSENSE_RES = (640, 480)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, *REALSENSE_RES, rs.format.bgr8, 30)
pipeline.start(config)

def nothing(x):
    pass

cv2.namedWindow("Trackbars")
cv2.createTrackbar("LH", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("LS", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("LV", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("UH", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("US", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("UV", "Trackbars", 255, 255, nothing)

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_img = np.asanyarray(color_frame.get_data())
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

    lh = cv2.getTrackbarPos("LH", "Trackbars")
    ls = cv2.getTrackbarPos("LS", "Trackbars")
    lv = cv2.getTrackbarPos("LV", "Trackbars")
    uh = cv2.getTrackbarPos("UH", "Trackbars")
    us = cv2.getTrackbarPos("US", "Trackbars")
    uv = cv2.getTrackbarPos("UV", "Trackbars")

    lower = np.array([lh, ls, lv])
    upper = np.array([uh, us, uv])

    mask = cv2.inRange(hsv, lower, upper)

    cv2.imshow("Color", color_img)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"Lower HSV: {lower}, Upper HSV: {upper}")
        break

pipeline.stop()
cv2.destroyAllWindows()
