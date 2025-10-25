#!/usr/bin/env python3
import time
from djitellopy import Tello
import cv2

def main():
    tello = Tello()
    tello.connect()
    print(f"Conectado al Tello. Batería: {tello.get_battery()}%")

    # Reinicia stream
    tello.streamoff()
    time.sleep(0.5)
    tello.streamon()
    frame_reader = tello.get_frame_read()
    time.sleep(1.0)  # deja estabilizar

    for i in range(10):
        frame = frame_reader.frame
        if frame is not None:
            print(f"Frame {i}: shape = {frame.shape}")  # (alto, ancho, canales)
        else:
            print(f"Frame {i}: None")
        time.sleep(0.2)

    tello.streamoff()
    tello.end()
    print("Stream cerrado.")

if __name__ == '__main__':
    main()
