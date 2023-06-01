import argparse
import pickle
import sys
sys.path.insert(0, '../')
import os
import cv2
import numpy as np

from vision import Vision
from stereo import Stereo

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('-f', '--folder_path',
                        required=True,
                        help='Folder path to save the images'
                        )

    args = parser.parse_args()

    os.mkdir(args.folder_path + "/out")
                          
    files = os.listdir(args.folder_path)
    files = [f for f in files if os.path.isfile(args.folder_path + '/' + f)]
    
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

    count = 1
    kernel = np.array([[ 0,-1, 0], 
                       [-1, 5,-1],
                       [ 0,-1, 0]])
    for f in files:
        img = cv2.imread(args.folder_path + '/' + f)
        img = cv2.filter2D(img, -1, kernel)
        img = cv2.bilateralFilter(img, d=5, sigmaColor=80, sigmaSpace=80)
        img = s.rotateImage(img)

        cv2.imwrite(f"{args.folder_path}/out/{count}.png", img)
        count = count + 1

        cv2.imshow("Frame", img)

        key = cv2.waitKey(100) & 0xFF
        if key == ord("q"):
            break

        #144610 / 142839