#/***********************************************************************
# Project: ESP32 IMU + ToF Fusion Visualizer
# File: imu_tof_visualizer.py
#
# Copyright (c) 2026 Jesrin Celas
#
# Author: Jesrin Celas A S
# Description:
# Real-time 3D aircraft visualization using roll, pitch, and height
# data streamed from ESP32 over UART.
#************************************************************************/

import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
from math import radians, sin, cos
import sys

# ================= SERIAL =================
SERIAL_PORT = "COM4"
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
except:
    print("❌ Cannot open serial port")
    sys.exit()

# ============== VISUAL SCALE ==============
HEIGHT_SCALE = 5

# -------- Rotation Function --------
def rotate(points, roll, pitch):
    r = radians(-roll)
    p = radians(-pitch)

    Rx = np.array([[1,0,0],
                   [0,cos(r),-sin(r)],
                   [0,sin(r),cos(r)]])

    Ry = np.array([[cos(p),0,sin(p)],
                   [0,1,0],
                   [-sin(p),0,cos(p)]])

    return points @ (Ry @ Rx).T

# -------- Aircraft Geometry --------
fuselage = np.array([
    [0,0,0],[2,0,0],[2,0.2,0],[0,0.2,0],
    [0,0,-0.2],[2,0,-0.2],[2,0.2,-0.2],[0,0,-0.2]
])

wings = np.array([[1,-2,0],[1,2,0],[1.2,2,0],[1.2,-2,0]])
tail  = np.array([[0.2,-0.8,0],[0.2,0.8,0],[0.4,0.8,0],[0.4,-0.8,0]])

# -------- Plot Setup --------
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-3,3)
ax.set_ylim(-3,3)
ax.set_zlim(0,5)
ax.view_init(elev=20, azim=-60)

body_poly = Poly3DCollection([], color='silver', alpha=0.9)
wing_poly = Poly3DCollection([], color='red', alpha=0.8)
tail_poly = Poly3DCollection([], color='green', alpha=0.8)

ax.add_collection3d(body_poly)
ax.add_collection3d(wing_poly)
ax.add_collection3d(tail_poly)

roll = pitch = height = 0

# -------- Update Loop --------
def update(frame):
    global roll, pitch, height

    try:
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            parts = line.split(',')

            if len(parts) == 3:
                roll = float(parts[0])
                pitch = float(parts[1])
                height = float(parts[2])
    except:
        pass

    fus = rotate(fuselage, roll, pitch)
    wng = rotate(wings, roll, pitch)
    tl  = rotate(tail, roll, pitch)

    z_disp = height * HEIGHT_SCALE

    fus[:,2] += z_disp
    wng[:,2] += z_disp
    tl[:,2]  += z_disp

    body_poly.set_verts([
        fus[[0,1,2,3]],
        fus[[4,5,6,7]],
        fus[[0,1,5,4]],
        fus[[2,3,7,6]]
    ])

    wing_poly.set_verts([wng])
    tail_poly.set_verts([tl])

    ax.set_title(f"Roll {roll:.1f}°  Pitch {pitch:.1f}°  Height {height:.2f} m  (x{HEIGHT_SCALE})")
    return body_poly, wing_poly, tail_poly

def on_close(event):
    ser.close()
    print("Serial closed")

fig.canvas.mpl_connect('close_event', on_close)

ani = FuncAnimation(fig, update, interval=20, blit=False)
plt.show()
