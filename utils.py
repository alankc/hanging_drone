import cv2
import numpy as np
import time

from simple_pid import PID
from tellopy import Tello
from easydrone import EasyDrone
from vision import Vision
from stereo import Stereo

def draw_text(image, text, row):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_size = 50
    font_color = (255,255,255)
    bg_color = (0,0,0)
    height, width = image.shape[:2]
    left_mergin = 10
    if row < 0:
        pos =  (left_mergin, height + font_size * row + 1)
    else:
        pos =  (left_mergin, font_size * (row + 1))
    
    cv2.putText(image, text, pos, font, font_scale, bg_color, 6)
    cv2.putText(image, text, pos, font, font_scale, font_color, 1)

def draw_line(image, start_point, end_point):
    color = (0, 255, 0)
    thickness = 1
    cv2.line(image, start_point, end_point, color, thickness)

def draw_dot(image, point):
    cv2.circle(image, point, radius=5, color=(0, 0, 255), thickness=-1)

def draw_polylines(image, points):
    color = (255, 0, 0)
    thickness = 3
    cv2.polylines(image, points, True, color, thickness)

def draw_rectangle(image, start_point, end_point):
    color = (255, 0, 0)
    thickness = 2
    cv2.putText(image, 'YOLO', (start_point[0], start_point[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    cv2.rectangle(image, start_point, end_point, color, thickness)

def rc_control(key, ed:EasyDrone):
    
    if key == ord("w"): #up
        ed.rc_control(throttle=0.25)
        return

    if key == ord("s"): #down
        ed.rc_control(throttle=-0.25)
        return

    if key == ord("d"): #cwr
        ed.rc_control(yaw=0.25)
        return

    if key == ord("a"): #ccwr
        ed.rc_control(yaw=-0.5)
        return

    if key == ord("i"): #forward
        ed.rc_control(pitch=0.5)
        return

    if key == ord("k"): #backward
        ed.rc_control(pitch=-0.25)
        return

    if key == ord("j"): #left
        ed.rc_control(roll=-0.25)
        return

    if key == ord("l"): #right
        ed.rc_control(roll=0.25)
        return

    ed.rc_control()
