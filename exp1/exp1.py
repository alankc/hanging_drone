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
    
    parser.add_argument('-t', '--ty',
                        default=15.0,
                        help='Ty distance'
                        )
    
    args = parser.parse_args()

    if not os.path.exists(args.folder_path):
        print("======================================================")
        print("=============== Folder does not exists ===============")
        print("======================================================")
        exit(0)
    else:
        tst1 = not os.path.exists(f"{args.folder_path}/1.png")
        tst2 = not os.path.exists(f"{args.folder_path}/2.png")
        
        if tst1:
            print("======================================================")
            print("=========== Folder does not contains 1.png ===========")
            print("======================================================")
        
        if tst2:
            print("======================================================")
            print("=========== Folder does not contains 2.png ===========")
            print("======================================================")
        
        if tst1 or tst2:
            exit(0)

    v = Vision()
    v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=30)
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

    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", mouse_click)

    frame = np.zeros((10, 10, 3), dtype = np.uint8)
    frame_s = frame
    while True:

        frame_s = frame.copy()

        if (len(select_rect) > 2):
            ut.draw_polylines(frame_s, [np.array(select_rect)])
            k1, d1 = v.detect_features_in_polygon(frame, np.array(select_rect))
        
        cv2.imshow("Frame", frame_s)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        if key == ord(" "):
            select_rect = []

        if key == ord("1"):
            frame = s.rotateImage(cv2.imread(f"{args.folder_path}/1.png"))

        if key == ord("2"):
            select_rect = []
            frame = s.rotateImage(cv2.imread(f"{args.folder_path}/2.png"))
            k2, d2 = v.detect_features(frame)
            k_curr_match, d_curr_match, error, good_matches = v.bf_matching_descriptors(d1, k2, d2, 0.7, (cx, cy)) #0.7 maibe??
            x_out, y_out, depth_out, yaw_out, roll_out  = s.compute_relative_depth(15, k1, k2, good_matches)
            frame = cv2.drawKeypoints(frame, k_curr_match, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            
            data = open(f"{args.folder_path}/data.txt", "w+")
            data.write(f"Landing point: X Y Z YAW ROLL\n")
            data.write(f"{np.mean(x_out)} ")
            data.write(f"{np.mean(depth_out)} ")
            data.write(f"{np.mean(y_out)} ")
            data.write(f"{yaw_out} ")
            data.write(f"{roll_out}\n")
            
            out_data = []
            for i in range(len(x_out)):
                out_data.append((x_out[i], depth_out[i], y_out[i]))
            out_data = sorted(out_data, key=lambda x: x[0])

            data.write(f"Points: X Y Z\n")
            for i in range(len(x_out)):
                data.write(f"{out_data[i][0]} {out_data[i][1]} {out_data[i][2]}\n")
            data.flush()
            data.close()

        time.sleep(1/30)