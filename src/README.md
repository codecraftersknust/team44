# Control Software

This directory contains the control software for an autonomous vehicle platform using RealSense camera, RPLidar, and Arduino-based motor control. The software is organized into main control scripts, supporting modules, and utility tools for color and sensor calibration.

## Main Scripts

- **round_1.py**: Implements basic wall-following and lap-counting using LIDAR and RealSense. The vehicle follows the left wall and counts laps by detecting orange lines on the track. PID control is used for steering.
- **round_2.py**: Extends `round_1.py` by adding block detection and avoidance (red/green blocks) using the camera. The vehicle temporarily overrides steering to avoid detected blocks, then resumes wall-following.
- **block_detector.py**: Provides the `detect_block(frame)` function to identify red and green blocks in camera frames using HSV color segmentation.

## Supporting and Utility Scripts

- **misc/plot_lidar.py**: Visualizes LIDAR scans in real time using matplotlib.
- **misc/detect_orange_count.py**: Standalone tool for tuning and testing orange line detection and lap counting.
- **misc/track_innerwall.py**: Wall-following using only the left wall (PID control, similar to main scripts).
- **misc/lidar_center_following.py**: Wall-following using both left and right LIDAR distances to keep the vehicle centered.
- **misc/motor_control.py**: Simple tool to send servo angles to Arduino for manual testing.
- **colors/hsv.py**: Interactive HSV range tuner for color segmentation using RealSense camera.
- **colors/inspect_bgr.py**: Displays the camera feed for BGR color inspection.
- **colors/bgr_hsv.py**: Converts a sample BGR color to HSV (for calibration).

## Hardware Requirements
- Intel RealSense camera (RGB stream)
- RPLidar (A1/A2/A3)
- Arduino-compatible board for motor/servo control

## Software Dependencies
- Python 3.x
- `pyrealsense2` (Intel RealSense SDK)
- `numpy`
- `opencv-python`
- `serial` (pyserial)
- `rplidar`
- `matplotlib` (for LIDAR plotting)

Install dependencies with:
```bash
pip install pyrealsense2 numpy opencv-python pyserial rplidar matplotlib
```

## Usage

1. **Connect hardware**: Ensure RealSense, LIDAR, and Arduino are connected to the correct USB ports (see port names in scripts).
2. **Run main control**:
   - For basic wall-following and lap counting:
     ```bash
     python round_1.py
     ```
   - For wall-following with block avoidance:
     ```bash
     python round_2.py
     ```
3. **Utility scripts**: Use scripts in `misc/` and `colors/` for calibration, visualization, and testing as needed.

## Notes
- Adjust serial port names and HSV color ranges in scripts as needed for your setup and environment.
- Scripts are designed for real-time operation and require the appropriate hardware to be connected.
