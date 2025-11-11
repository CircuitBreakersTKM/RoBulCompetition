#!/usr/bin/env python3
"""
FRC QR Code Detector for Raspberry Pi Coprocessor

Reads from a PhotonVision camera stream, detects QR codes,
and publishes the largest one (by bounding box area) to NetworkTables.

Dependencies:
    pip install opencv-python pyzbar pynetworktables
"""

import cv2
from pyzbar.pyzbar import decode
from networktables import NetworkTables
import time

# === CONFIGURATION ===
PHOTONVISION_STREAM_URL = "http://10.0.3.11:1182/stream.mjpg"
TEAM_NUMBER = 3
TABLE_NAME = "CircuitBreakers"
PREFIX = "Camera Tower/QR Detector/"
QR_SIZE_CENTIMETERS = 16.6 # Physical size of the QR codes side in centimeters

# CAMERA_FOCAL_LENGTH_PIXELS: The focal length in pixel units for distance estimation.
# This combines the physical lens focal length with the sensor's pixel density.
# 
# CALIBRATION METHOD:
# 1. Place a QR code at a known distance (e.g., 100 cm) from the camera
# 2. Run this script and note the pixel width (w) of the detected QR code
# 3. Calculate: CAMERA_FOCAL_LENGTH_PIXELS = (pixel_width × distance_cm) / QR_SIZE_CENTIMETERS
#    Example: If w=166 pixels at 100cm distance: (166 × 100) / 16.6 = 1000 pixels
# 4. Update the value below and restart the script
# 5. Verify accuracy by testing at multiple known distances
CAMERA_FOCAL_LENGTH_PIXELS = 632.53  # Default estimate - calibrate for your specific camera

# === INITIALIZE NETWORKTABLES ===
# Use the default FRC team IP scheme: 10.TE.AM.2
robot_ip = f"10.{TEAM_NUMBER//100}.{TEAM_NUMBER%100}.2"
print("[INFO] Connecting to NetworkTables server at", robot_ip)

NetworkTables.initialize(server=robot_ip)
qr_table = NetworkTables.getTable(TABLE_NAME)

# === INITIALIZE CAMERA ===
print("[INFO] Opening camera stream:", PHOTONVISION_STREAM_URL)
cap = cv2.VideoCapture(PHOTONVISION_STREAM_URL)

print("[INFO] Setting up NetworkTables entries...")

qr_table.putBoolean(PREFIX + "hasTarget", False)
qr_table.putString(PREFIX + "data", "initializing")
qr_table.putNumber(PREFIX + "ticker", 0)
qr_table.putBoolean(PREFIX + "resetTicker", False)
qr_table.putNumber(PREFIX + "distance", 0.0)
qr_table.putNumber(PREFIX + "width", 0.0)

print ("[INFO] Waiting for camera stream to open...")

if not cap.isOpened():
    print("[ERROR] Could not open PhotonVision stream!")
    exit(1)

print("[INFO] QR detector started successfully.")

# === MAIN LOOP ===
last_detect_time = 0
DETECTION_TIMEOUT = 0.3  # seconds before clearing stale data
ticker = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARN] Frame grab failed, retrying...")
        time.sleep(0.1)
        continue

    # Rotate frame 90 degrees counter-clockwise (top is on right, need to rotate left)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Use pyzbar - more reliable, no false positives
    qrs = decode(frame)
    
    if qrs:
        # Find the largest QR code (by bounding box area)
        largest_qr = max(qrs, key=lambda q: q.rect.width * q.rect.height)
        qr_data = largest_qr.data.decode("utf-8")
        x, y, w, h = largest_qr.rect
        area = w * h
        center_x = x + w / 2
        center_y = y + h / 2

        if qr_table.getBoolean(PREFIX + "resetTicker", False):
            ticker = 0
            qr_table.putBoolean(PREFIX + "resetTicker", False)
        
        # Calculate distance using similar triangles
        # Distance = (RealSize * FocalLength) / PixelSize
        qr_pixel_width = w
        distance_cm = (QR_SIZE_CENTIMETERS * CAMERA_FOCAL_LENGTH_PIXELS) / qr_pixel_width
        
        # Publish to NetworkTables
        qr_table.putString(PREFIX + "data", qr_data)
        qr_table.putBoolean(PREFIX + "hasTarget", True)
        qr_table.putNumber(PREFIX + "ticker", ticker)
        qr_table.putNumber(PREFIX + "distance", distance_cm)
        qr_table.putNumber(PREFIX + "width", w)
        ticker += 1

        last_detect_time = time.time()
        # print(f"[QR] {qr_data} (area={area:.0f}, distance={distance_cm:.1f}cm)")
    
    # Always check timeout, even if we just detected
    if time.time() - last_detect_time > DETECTION_TIMEOUT:
        qr_table.putBoolean(PREFIX + "hasTarget", False)
        qr_table.putString(PREFIX + "data", "")
        qr_table.putNumber(PREFIX + "distance", 0.0)
        qr_table.putNumber(PREFIX + "width", 0.0)

cap.release()
cv2.destroyAllWindows()
