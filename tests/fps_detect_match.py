import argparse
import pickle
import cv2
import time
import numpy as np
import sys
import os

sys.path.insert(0, '../')
from vision import Vision
from stereo import Stereo
import utils as ut

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

    if not os.path.exists(args.folder_path):
        print("======================================================")
        print("=============== Folder does not exists ===============")
        print("======================================================")
        exit(0)
    else:
        tst1 = not os.path.exists(f"{args.folder_path}/video.mp4")
        
        if tst1:
            print("======================================================")
            print("========= Folder does not contains video.mp4 =========")
            print("======================================================")
            exit(0)
    
    v = Vision()
    v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=25)
    v.set_bf_matcher()

    intrinsics_path = "../drone/intrinsics.pkl"
    dist_path = "../drone/dist.pkl"
    camera_pitch = -13

    #loading camera parammters
    intrinsics_file = open(intrinsics_path, "rb")
    intrinsics = pickle.load(intrinsics_file)
    [[fx,_,cx],[_,fy,cy],[_,_,_]] = intrinsics
    cy = 175

    s = Stereo()
    s.set_camera_params(fx, fy, camera_pitch, cx, cy)


   
    run = True
    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", mouse_click)
    while run:

        video = cv2.VideoCapture(f"{args.folder_path}/video.mp4")
        k_ref = None
        d_ref = None
        time_start = time.time()
        fps = 0

        while(True):

            ret, frame = video.read()   
            
            if(ret==True):
                frame = s.rotateImage(frame)
                frame_s = frame.copy()

                if not ((k_ref is None) and (d_ref is None)):
                    k, d = v.detect_features(frame)
                    k_curr, d_curr, error, good_matches = v.bf_matching_descriptors(d_ref, k, d, 0.6, (cx, cy))
                    cv2.drawKeypoints(frame_s, k_curr, frame_s, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

                if (len(select_rect) > 2):
                    ut.draw_polylines(frame_s, [np.array(select_rect)])

                fps = 0.3 * fps + 0.7 / (time.time()-time_start)  # exponential moving average
                time_start = time.time()
                ut.draw_text(frame_s, f"FPS={fps:.1f}", -1)
                cv2.imshow("Frame", frame_s)
                
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    run = False
                    break
                
                if key == ord("1"):
                    k_ref, d_ref = v.detect_features_in_polygon(frame, np.array(select_rect))
                    select_rect = []

                if key == ord(" "):
                    select_rect = []
                    k_ref = None
                    d_ref = None
            else:
                break