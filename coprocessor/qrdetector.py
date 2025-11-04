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
TABLE_NAME = "QRDetector"

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

qr_table.putBoolean("hasTarget", False)
qr_table.putString("data", "initializing")
qr_table.putNumber("ticker", 0)

print ("[INFO] Waiting for camera stream to open...")

if not cap.isOpened():
    print("[ERROR] Could not open PhotonVision stream!")
    exit(1)

print("[INFO] QR detector started successfully.")

# === MAIN LOOP ===
last_detect_time = 0
DETECTION_TIMEOUT = 1.0  # seconds before clearing stale data
ticker = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARN] Frame grab failed, retrying...")
        time.sleep(0.1)
        continue

    # Decode all QR codes in the frame
    qrs = decode(frame)

    if qrs:
        # Find the largest QR code (by bounding box area)
        largest_qr = max(qrs, key=lambda q: q.rect.width * q.rect.height)
        qr_data = largest_qr.data.decode("utf-8")
        x, y, w, h = largest_qr.rect
        area = w * h
        center_x = x + w / 2
        center_y = y + h / 2

        if qr_table.getBoolean("resetTicker", False):
            ticker = 0
            qr_table.putBoolean("resetTicker", False)
        
        # Publish to NetworkTables
        qr_table.putString("data", qr_data)
        qr_table.putBoolean("hasTarget", True)
        qr_table.putNumber("ticker", ticker)
        ticker += 1

        last_detect_time = time.time()
        print(f"[QR] {qr_data} (area={area:.0f})")
    else:
        # If no QR detected for a while, clear entries
        if time.time() - last_detect_time > DETECTION_TIMEOUT:
            qr_table.putBoolean("hasTarget", False)
            qr_table.putString("data", "")

cap.release()
cv2.destroyAllWindows()
