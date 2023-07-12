import numpy as np
import cv2
from scipy.optimize import curve_fit

from collections import deque
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

"""
class stereo, computes distance using features in diferent photos
""" 
class Stereo:
    def __init__(self) -> None:
        pass

    def set_camera_params(self, fx:np.float32, fy:np.float32, angle:np.float32, cx:np.float32, cy_aligned:np.float32):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy_aligned = cy_aligned
        self.angle = angle * np.pi / 180
    
    def compute_relative_depth_filtered(self, ty, kp1, kp2, matches, rect1, rect2):
        """
        Deprecated function

        Compute the depth considering 
        the keypoints kp1 inside the rectangle rect1 and 
        the keypoints kp2 inside the rectangle rect2 
        """
        # For each match compute the disparity, and 3D position
        depth = []

        rec1_x_i, rec1_y_i = rect1[0]
        rec1_x_e, rec1_y_e = rect1[1]
        rec2_x_i, rec2_y_i = rect2[0]
        rec2_x_e, rec2_y_e = rect2[1]

        #Ensure optimization
        FY_TY = self.fy * ty

        for m in matches:
            # Getting the matching keypoints row in their lists
            img1_id = m.queryIdx
            img2_id = m.trainIdx

            # x ares columns
            # y are rows
            # Get the coordinates
            (x1, y1) = kp1[img1_id].pt
            (x2, y2) = kp2[img2_id].pt

            if (rec1_x_i <= x1 <= rec1_x_e) and (rec1_y_i <= y1 <= rec1_y_e) and (rec2_x_i <= x2 <= rec2_x_e) and (rec2_y_i <= y2 <= rec2_y_e):
                # compute disparity
                disparity = np.abs(y1 - y2)
                depth_z = FY_TY / disparity #depth at ty/2
                #x1 and x2 must have an negligible difference
                pos_x = depth_z * x2 / self.fx
                pos_y = ty * (self.cy_aligned - (y1 + y2) * 0.5) / disparity - ty/2 #The distance is to the point betwen y1 and y2, then remove ty/2 

                #print(f"{pos_y} {pos_z} {depth_x}".replace('.',','))
                #drone's mvo.Y is pos_x
                #drone's mvo.X is depth_z
                #drone's mvo.Z is pos_y
                depth.append((pos_x, pos_y, depth_z))
        
        return depth
    
    def __k_means_filter(self, depth, x, y, cluster_range):
        """
        k_means filter to select the cluster nearest the drone

        used when detects several features in the background
        """
        data = [[x[i], depth[i], y[i]] for i in range(len(x))]
        data = np.array(data)

        max_n_cluster = 2
        max_derivate = -np.inf
        sil_coeff = deque(maxlen=3)
        sil_coeff.append(0)
        best_model = None
        curr_kmeans = None
        prev_kmeans = None
        #selecting the number of clusters
        for n_cluster in range(2, cluster_range):
            prev_kmeans = curr_kmeans
            curr_kmeans = KMeans(n_init=1000, n_clusters=n_cluster, max_iter=1000, algorithm='lloyd').fit(data)
            curr_label  = curr_kmeans.labels_

            if len(sil_coeff) < 2: #At least 3 elements in the deque to compute second derivate
                sil_coeff.append(silhouette_score(data, curr_label, metric='euclidean'))
                continue

            sil_coeff.append(silhouette_score(data, curr_label, metric='euclidean'))

            derivate = sil_coeff[2] + sil_coeff[0] - 2 * sil_coeff[1]
            if derivate > max_derivate:
                max_n_cluster = n_cluster - 1
                max_derivate = derivate
                best_model = prev_kmeans
        

        pred = best_model.labels_
        data_by_cluster = [[] for i in range(max_n_cluster)] 
        
        #selecting the cluster with the smallest depth (y coordinate of real world) distance
        for i in range(len(pred)):
            data_by_cluster[pred[i]].append(data[i][1])

        mean_by_cluster = [np.mean(i) for i in data_by_cluster]
        index = np.argmin(mean_by_cluster)

        depth_out = []
        x_out = []
        y_out = []
        for i in range(len(pred)):
            if index == pred[i]:
                depth_out.append(depth[i])
                x_out.append(x[i])
                y_out.append(y[i])
        
        return depth_out, x_out, y_out

    def compute_relative_depth(self, ty, kp1, kp2, matches, kfilter = True, kdist = 50):
        """
        Return the landing site's x, y, depth, yaw, and roll relative to the drone's position in the second picture

        In the landing pipeline:
            x: x in the landing pipeline
            y: height in the landing pipeline
            depth: y in the landing pipeline
            yaw: used rotate the drone and hit the landing site facing forward
            roll: used to define if the drone can or cannot land in the place
        """

        # For each match compute the disparity, and 3D position
        depth_out = []
        x_out = []
        y_out = []
        yaw_out = 0
        roll_out = 0
        #Ensure optimization
        FY_TY = self.fy * ty #* 1.073401022 #systematic error

        for m in matches:
            # Getting the matching keypoints row in their lists
            img1_id = m.queryIdx
            img2_id = m.trainIdx

            # x ares columns
            # y are rows
            # Get the coordinates
            (x1, y1) = kp1[img1_id].pt
            (x2, y2) = kp2[img2_id].pt

            # compute disparity
            disparity = np.abs(y1 - y2)
            depth_z = FY_TY / disparity #depth at ty/2
            #x1 and x2 must have an negligible difference
            pos_x = depth_z * (x2 - self.cx) / self.fx
            pos_y = ty * (self.cy_aligned - (y1 + y2) * 0.5) / disparity - ty/2 #The distance is to the point betwen y1 and y2, then remove ty/2 

            #print(f"{pos_y} {pos_z} {depth_x}".replace('.',','))
            #drone's mvo.Y is pos_x
            #drone's mvo.X is depth_z
            #drone's mvo.Z is pos_y
            depth_out.append(depth_z)
            x_out.append(pos_x)
            y_out.append(pos_y)

        #if the filter is required and the variation in depth is bigger than 50 cm       
        if kfilter and np.abs(max(depth_out) - min(depth_out)) > kdist and len(depth_out) > 3:

            if len(depth_out) > 10:
                depth_out, x_out, y_out = self.__k_means_filter(depth_out, x_out, y_out, 8)
            else:
                depth_out, x_out, y_out = self.__k_means_filter(depth_out, x_out, y_out, len(depth_out) - 1)

            #After applying k-means do not have enough data
            if len(depth_out) < 2:
                return  [],[],[], 0, 0
                        
        elif np.abs(max(depth_out) - min(depth_out)) > kdist: #variance too big and kfilter = false
            return  [],[],[], 0, 0
        
        #compute the yaw
        try:
            f_aux = lambda x, a, b: a * x + b
            popt, _ = curve_fit(f_aux, x_out, depth_out)
            yaw_out = np.arctan(popt[0]) * 180 / np.pi
        except:
            yaw_out = 0

        #compute the roll
        try:
            f_aux = lambda x, a, b: a * x + b
            popt, _ = curve_fit(f_aux, x_out, y_out)
            roll_out = np.arctan(popt[0]) * 180 / np.pi
        except:
            roll_out = 0

        return x_out, y_out, depth_out, yaw_out, roll_out
    
    def rotateImage(self, input):
        """
        Rotate the image considering the camera pitch
        """
        roll = 0
        pitch = self.angle
        yaw = 0
        dx = 0
        dy = 0
        dz = 1

        A2 = np.array([[self.fx,  0, self.cx, 0],
                       [ 0, self.fy, self.cy_aligned, 0],
                       [ 0,  0,  1, 0]], dtype=np.float64)
            
        # Inverted Camera Calibration Intrinsics Matrix
        A1 = np.array([[1/self.fx,    0, -self.cx/self.fx],
                       [   0, 1/self.fy, -self.cy_aligned/self.fy],
                       [   0,    0,      0],
                       [   0,    0,      1]], dtype=np.float64)
        
        # Rotation matrices around the X, Y, and Z axis
        c = np.cos(pitch)
        s = np.sin(pitch)
        RX = np.array([[1, 0,  0, 0],
                       [0, c, -s, 0],
                       [0, s,  c, 0],
                       [0, 0,  0, 1]], dtype=np.float64)
        
        c = np.cos(yaw)
        s = np.sin(yaw)
        RY = np.array([[ c, 0, s, 0],
                       [ 0, 1, 0, 0],
                       [-s, 0, c, 0],
                       [ 0, 0, 0, 1]], dtype=np.float64)

        c = np.cos(roll)
        s = np.sin(roll)
        RZ = np.array([[c, -s, 0, 0],
                       [s,  c, 0, 0],
                       [0,  0, 1, 0],
                       [0,  0, 0, 1]], dtype=np.float64)        

        R = RZ @ RY @ RX
        
        T = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0,  1]], dtype=np.float64) 
        
        # Compose rotation matrix with (RX, RY, RZ)
        R = RZ @ RY @ RX

        # Final transformation matrix
        H = A2 @ (T @ (R @ A1))

        # Apply matrix transformation
        shape = (input.shape[1], input.shape[0])
        #return cv2.warpPerspective(input, H, shape, None, cv2.INTER_LANCZOS4)
        #return cv2.warpPerspective(input, H, shape, None, cv2.INTER_LINEAR)
        return cv2.warpPerspective(input, H, shape, None, cv2.INTER_CUBIC)