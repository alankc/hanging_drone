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

import numpy as np
import matplotlib.pyplot as plt
#%matplotlib inline
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

select_rect = []

def mouse_click(event, x, y, flags, param):
    
    global select_rect

    if event == cv2.EVENT_LBUTTONDOWN:
        select_rect.append([x,y])

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('-r', '--run_cluster',
                        action="store_true",
                        required=False,
                        default=False,
                        help='Folder path to save the images'
                        )
    
    args = parser.parse_args()
    run_cluster = args.run_cluster
    print(args.run_cluster)

    v = Vision()
    v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=30)
    v.set_bf_matcher()

    intrinsics_path = "../drone/intrinsics.pkl"
    dist_path = "../drone/dist.pkl"
    folder_path = "../exp1/images/140cm_y45_r0/"

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
            frame = s.rotateImage(cv2.imread(f"{folder_path}/1.png"))

        if key == ord("2"):
            select_rect = []
            frame = s.rotateImage(cv2.imread(f"{folder_path}/2.png"))
            k2, d2 = v.detect_features(frame)
            k_curr_match, d_curr_match, error, good_matches = v.bf_matching_descriptors(d1, k2, d2, 0.7, (cx, cy)) #0.7 maibe??
            x_out, y_out, depth_out, yaw_out, roll_out  = s.compute_relative_depth(15, k1, k2, good_matches)
            frame = cv2.drawKeypoints(frame, k_curr_match, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

            fig = plt.figure()
            ax = fig.add_subplot()
            if run_cluster:
                data = []
                for i in range(len(x_out)):
                    data.append([x_out[i], depth_out[i]])
                
                data = np.array(data)

                max_n_cluster = 2
                max_derivate = 0
                from collections import deque
                sil_coeff = deque(maxlen=3)
                for n_cluster in range(2, 10):
                    kmeans = KMeans(n_init=1000, n_clusters=n_cluster, algorithm='elkan', max_iter=1000).fit(data)
                    label = kmeans.labels_

                    if len(sil_coeff) < 2:
                        sil_coeff.append(silhouette_score(data, label, metric='euclidean'))
                        continue
                    
                    sil_coeff.append(silhouette_score(data, label, metric='euclidean'))

                    n = n_cluster - 1
                    print(f"n={n}")
                    d = sil_coeff[2] + sil_coeff[0] - 2 * sil_coeff[1]
                    print(f"d={d}")
                    print(f"--------")
                    if d > max_derivate:
                        max_n_cluster = n
                        max_derivate = d


    
                model = KMeans(n_init=100, n_clusters=max_n_cluster, algorithm='elkan', max_iter=1000)
                model.fit_predict(data)
                pred = model.fit_predict(data)
                ax.scatter(x_out, depth_out, s=300, c=model.labels_)
                print("number of cluster found: {}".format(len(set(model.labels_))))
                print('cluster for each point: ', model.labels_)
            else:
                ax.scatter(x_out, depth_out, s=300)
                
            plt.show()


            
        time.sleep(1/30)

    cv2.destroyAllWindows()


