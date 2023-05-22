import argparse
import tellopy
import pickle
import cv2
import time
import numpy as np
import sys
import os

sys.path.insert(0, '/home/x/Documents/Experiments-RMTT/landing_system/')
from easydrone import EasyDrone
import utils as ut

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
        print(f"Creating folder: {args.folder_path}")
        os.makedirs(args.folder_path, exist_ok=True)
    else:
        tst1 = os.path.exists(f"{args.folder_path}/1.png")
        tst2 = os.path.exists(f"{args.folder_path}/2.png")
        
        if tst1:
            print("=============================================")
            print("=========== Folder contains 1.png ===========")
            print("=============================================")
        
        if tst2:
            print("=============================================")
            print("=========== Folder contains 2.png ===========")
            print("=============================================")
        
        if tst1 and tst2:
            exit(0)

    td = tellopy.Tello()
    td.connect()
    td.wait_for_connection(60.0)

    ed = EasyDrone(td, True)
    ed.start()

    intrinsics_path = "/home/x/Documents/Experiments-RMTT/landing_system/drone/intrinsics.pkl"
    dist_path = "/home/x/Documents/Experiments-RMTT/landing_system/drone/dist.pkl"

    #loading camera parammters
    intrinsics_file = open(intrinsics_path, "rb")
    intrinsics = pickle.load(intrinsics_file)
    [[fx,_,cx],[_,fy,cy],[_,_,_]] = intrinsics
    cy = 175
    cx = int(np.round(cx, 0))

    while True:
        image = ed.get_curr_frame()
        if image is None:
            frame = np.zeros((10, 10, 3), dtype = np.uint8)
            time.sleep(0.1)
            cv2.imshow('Camera', frame)
            key = cv2.waitKey(1) & 0xFF
            continue

        image_s = image.copy()
        ut.draw_line(image_s, (cx,0), (cx, 720))
        ut.draw_line(image_s, (0,cy), (960, cy))
        cv2.imshow('Camera', image_s)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord("q"):
            break

        if key == ord("w"):
            td.manual_takeoff()
        
        if key == ord(" "):
            td.set_pitch(0)
            td.set_roll(0)
            td.set_yaw(0)
            td.set_throttle(0)

        if key == ord("1"):
            print("Saving picture 1")
            cv2.imwrite(f"{args.folder_path}/1.png", image)
            print("Picture 1 saved")

        if key == ord("2"):
            print("Saving picture 2")
            cv2.imwrite(f"{args.folder_path}/2.png", image)
            print("Picture 2 saved")  
    
    ed.stop()
    td.quit()