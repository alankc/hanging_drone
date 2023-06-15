import cv2
import pickle
import numpy as np
import time
import utils as ut
import argparse
import os
from threading import Event
from easydrone import EasyDrone
from vision import Vision
from stereo import Stereo
from landing_pipeline import LandingPipeline
from yolo_detector import YOLODetector

select_rect = []

def mouse_click(event, x, y, flags, param):
    
    global select_rect

    if event == cv2.EVENT_LBUTTONDOWN:
        select_rect.append([x,y])


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('-f', '--folder_path',
                        required=False,
                        default=None,
                        help='Folder path to save the images'
                        )
    
    parser.add_argument('-t', '--takeoff',
                        action="store_true",
                        required=False,
                        default=False,
                        help='Define if the drone takeoff'
                        )
    
    parser.add_argument('-v', '--video_stream',
                        required=False,
                        default=None,
                        help='Video stream'
                        )
    
    args = parser.parse_args()

    out_file = None
    if not (args.folder_path is None):
        if os.path.exists(args.folder_path):
            print("======================================================")
            print("================ File already exists =================")
            print("======================================================")
            exit(0)
        else:
            if not os.path.exists(os.path.dirname(args.folder_path)):
                os.makedirs(os.path.dirname(args.folder_path))
            out_file = open(args.folder_path, "w")
            
    
    ed = EasyDrone(True, args.video_stream)
    ed.connect()
    ed.start()
    
    v = Vision()
    v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=10)
    #v.set_sift_detector()
    #v.set_orb_detector()
    v.set_bf_matcher()

    intrinsics_path = "drone/intrinsics.pkl"
    dist_path = "drone/dist.pkl"
    camera_pitch = -13

    #loading camera parammters
    intrinsics_file = open(intrinsics_path, "rb")
    intrinsics = pickle.load(intrinsics_file)
    [[fx,_,cx],[_,fy,cy],[_,_,_]] = intrinsics
    cy = 175

    s = Stereo()
    s.set_camera_params(fx, fy, -13, cx, cy)

    path_yolo = "yolo_dataset/runs/detect/train6/weights/best.pt"
    yd = YOLODetector(path_yolo)

    k_ref = None
    d_ref = None

    if args.takeoff:
        ed.takeoff()
    else:
        #ed.cooler_on()
        pass
    #ed.manual_takeoff()

    cv2.namedWindow("Camera")
    cv2.setMouseCallback("Camera", mouse_click)

    manual_control = True
    lp = None

    time_start = time.time()
    alpha = 0.1
    fps = 0

    while True:
        frame = ed.get_curr_frame()
        
        if frame is None:
                image = np.zeros((720, 960, 3), dtype = np.uint8)
                select_rect = []
                key_delay = 30
        
        elif (len(select_rect) < 1):
                image = s.rotateImage(frame)
                key_delay = 1
                
        image_s = image.copy()

        if not (frame is None) and (len(select_rect) > 2) and manual_control:
            ut.draw_polylines(image_s, [np.array(select_rect)])

        if not manual_control:
            #if not (lp is None):
            result = lp.run(image, image_s)
            if (result == lp.SUCESS) or (result == lp.FAIL):
                manual_control = True
                lp = None
                ed.rc_control() #STOPING all controllers
            #else:
                #print("Press 1 to run landing pipeline with Yolo or")
                #print("Press 2 to run landing pipeline after selecting landing site")
                #manual_control = True
            
        if time.time() - time_start > 0:
            fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
            time_start = time.time()
        
        if manual_control:
            ut.draw_text(image_s, f"FPS={fps:.1f}         MANUAL CONTROL", -1)
        else:
            ut.draw_text(image_s, f"FPS={fps:.1f}", -1)

        cv2.imshow('Camera', image_s)

        key = cv2.waitKey(key_delay) & 0xFF
        if key == ord("q"):
            ed.land()
            time.sleep(5)
            ed.quit()
            select_rect = []
            exit(0)

        elif key == ord("1"): # Use YOLO
            manual_control = False
            lp = LandingPipeline(ed, s, v, yd, None, None, int(round(cx, 0)), int(round(cy, 0)), out_file)
            ed.PID_reset() #reseting all PIDs

        elif key == ord("2") and len(select_rect) > 2: # USe selected rectangle
            manual_control = False
            k_ref, d_ref = v.detect_features_in_polygon(image, np.array(select_rect))
            select_rect = []
            lp = LandingPipeline(ed, s, v, yd, k_ref, d_ref, int(round(cx, 0)), int(round(cy, 0)), out_file)
            ed.PID_reset() #reseting all PIDs

        elif key == ord("3"):# Clear selected rectangle
            select_rect = []

        elif key == ord(" "):# Change to manual control
            if not (lp is None):
                manual_control = not manual_control
                lp.reset()  
            else:
                manual_control = True

            ed.rc_control() #STOPING all controllers
            ed.PID_reset() #reseting all PIDs

        elif manual_control: 
            ut.rc_control(key, ed)
