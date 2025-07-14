import cv2
import numpy as np

bgr_color = np.uint8([[[160, 80, 57]]])  # Example BGR orange
hsv_color = cv2.cvtColor(bgr_color, cv2.COLOR_BGR2HSV)
print(hsv_color)
