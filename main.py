import cv2
import pickle
import tellopy
import numpy as np
import time
import utils as ut
import argparse
import os
from threading import Event
from easydrone import EasyDrone
from vision import Vision
from stereo import Stereo


select_rect = []

def mouse_click(event, x, y, flags, param):
    
    global select_rect

    if event == cv2.EVENT_LBUTTONDOWN:
        select_rect.append([x,y])


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('-f', '--folder_path',
                        required=True,
                        help='Folder path to save the images'
                        )
    
    args = parser.parse_args()

    if os.path.exists(args.folder_path):
        print("======================================================")
        print("================ File already exists =================")
        print("======================================================")
        exit(0)
    else:
        if not os.path.exists(os.path.dirname(args.folder_path)):
            os.makedirs(os.path.dirname(args.folder_path))
        out_file = open(args.folder_path, "w")
            
    
    ed = EasyDrone(True)
    ed.start()
    
    v = Vision()
    v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=15)
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

    kp_ref = None
    dsc_ref = None

    ed.takeoff()
    #td.manual_takeoff()

    cv2.namedWindow("Camera")
    cv2.setMouseCallback("Camera", mouse_click)

    while True:

        if (len(select_rect) < 1):
            image = ed.get_curr_frame()
            if image is None:
                frame = np.zeros((10, 10, 3), dtype = np.uint8)
                time.sleep(0.1)
                cv2.imshow('Camera', frame)
                key = cv2.waitKey(1) & 0xFF
                continue

            image = s.rotateImage(image)

        image_s = image.copy()
        if (len(select_rect) > 2):
            ut.draw_polylines(image_s, [np.array(select_rect)])

        cv2.imshow('Camera', image_s)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            ed.stop()
            select_rect = []
            break

        if key == ord("1"):
            kp_ref, dsc_ref = v.detect_features_in_polygon(image, np.array(select_rect))
            break

        if key == ord("2"):
            select_rect = []

        #print(ed.get_curr_speed_corrected())

    #(cx, cy) = np.mean([x.pt for x in kp_ref], axis=0)
    cx = int(round(cx, 0))
    cy = int(round(cy, 0))
    #print((cx, cy))
    ut.stereo_landing_pipeline(cx, cy, kp_ref, dsc_ref, ed, v, s, out_file)

    ed.land()
    ed.quit()
