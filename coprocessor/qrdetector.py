#!/usr/bin/env python3
"""
FRC QR Code Detector for Raspberry Pi Coprocessor
Optimized for LOW LATENCY.
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

QR_SIZE_CENTIMETERS = 16.6

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
CAMERA_FOCAL_LENGTH_PIXELS = 1289.15  # Default estimate - calibrate for your specific camera

# === INITIALIZE NETWORKTABLES ===
robot_ip = f"10.{TEAM_NUMBER//100}.{TEAM_NUMBER%100}.2"
print("[INFO] Connecting to NetworkTables server at", robot_ip)

NetworkTables.initialize(server=robot_ip)
qr_table = NetworkTables.getTable(TABLE_NAME)

# === INITIALIZE CAMERA ===
print("[INFO] Opening camera stream:", PHOTONVISION_STREAM_URL)

# Use FFMPEG backend + small buffer
cap = cv2.VideoCapture(PHOTONVISION_STREAM_URL, cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

print("[INFO] Setting up NetworkTables entries...")
qr_table.putBoolean(PREFIX + "hasTarget", False)
qr_table.putString(PREFIX + "data", "initializing")
qr_table.putNumber(PREFIX + "ticker", 0)
qr_table.putBoolean(PREFIX + "resetTicker", False)
qr_table.putNumber(PREFIX + "distance", 0.0)
qr_table.putNumber(PREFIX + "width", 0.0)

print("[INFO] Waiting for camera stream to open...")

if not cap.isOpened():
    print("[ERROR] Could not open PhotonVision stream!")
    exit(1)

print("[INFO] QR detector started successfully.")

# === MAIN LOOP ===
last_detect_time = 0
DETECTION_TIMEOUT = 0.3
ticker = 0

while True:
    NetworkTables.flush() # Ensure timely updates

    if not qr_table.getBoolean(PREFIX + "enabled", True):
        time.sleep(0.1)
        continue
    # ———————— CRITICAL LOW-LATENCY FIX ————————
    # Flush buffer: grab 2–3 frames but only decode the newest
    for _ in range(3):
        cap.grab()
    ret, frame = cap.retrieve()
    # —————————————————————————————————————————————

    if not ret:
        print("[WARN] Frame grab failed.")
        time.sleep(0.05)
        continue

    # Optional: grayscale improves decode speed
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    qrs = decode(gray)

    if qrs:
        largest_qr = max(qrs, key=lambda q: q.rect.width * q.rect.height)
        qr_data = largest_qr.data.decode("utf-8")
        x, y, w, h = largest_qr.rect

        if qr_table.getBoolean(PREFIX + "resetTicker", False):
            ticker = 0
            qr_table.putBoolean(PREFIX + "resetTicker", False)

        # Distance estimation
        distance_cm = (QR_SIZE_CENTIMETERS * CAMERA_FOCAL_LENGTH_PIXELS) / w

        # Publish
        qr_table.putString(PREFIX + "data", qr_data)
        qr_table.putBoolean(PREFIX + "hasTarget", True)
        qr_table.putNumber(PREFIX + "ticker", ticker)
        qr_table.putNumber(PREFIX + "distance", distance_cm)
        qr_table.putNumber(PREFIX + "width", w)

        ticker += 1
        last_detect_time = time.time()

    # Clear old data if QR disappears
    if time.time() - last_detect_time > DETECTION_TIMEOUT:
        qr_table.putBoolean(PREFIX + "hasTarget", False)
        qr_table.putString(PREFIX + "data", "")
        qr_table.putNumber(PREFIX + "distance", 0.0)
        qr_table.putNumber(PREFIX + "width", 0.0)

cap.release()
cv2.destroyAllWindows()
