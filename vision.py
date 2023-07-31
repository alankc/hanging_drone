import cv2
from concurrent.futures import ThreadPoolExecutor
import numpy as np


class Vision:
    """
    IMPORTANT: ALWAYS! use set_x_detector(...), then set_x_matching(...), then bf_matching(...)
    """
    __DETECTOR_TYPE_FAST = 0
    __DETECTOR_TYPE_ORB = 1
    __DETECTOR_TYPE_SIFT = 2

    def __init__(self) -> None:
        self.__detector = None
        self.__detec_and_compute = None
        self.__detector_type = None
        self.tracker = None

    def set_fast_detector(self, nonmaxSuppression, threshold, type):
        self.__detector_type = Vision.__DETECTOR_TYPE_FAST
        
        #Fast detector
        self.__detector = cv2.FastFeatureDetector_create()
        self.__detector.setNonmaxSuppression(nonmaxSuppression)
        self.__detector.setThreshold(threshold)
        self.__detector.setType(type)
        
        #BRISK descriptor extractor
        self.__desc_extractor= cv2.BRISK_create()

        # create a mask
        #m = np.zeros((720, 960), np.uint8)
        #m[100:175, 0:958] = 255

        #Combine the FAST detector with the brisk extractor
        self.__detec_and_compute = lambda img, m = None: self.__desc_extractor.compute(img, self.__detector.detect(img, mask = m))

    def set_orb_detector(self):
        self.__detector_type = Vision.__DETECTOR_TYPE_ORB
        #ORB detector
        self.__detector = cv2.ORB_create()

        #detect and compute using SIFT
        self.__detec_and_compute = lambda img, m = None: self.__detector.detectAndCompute(img, mask = m)

    def set_sift_detector(self):
        self.__detector_type = Vision.__DETECTOR_TYPE_SIFT

        #SIFT detector
        self.__detector = cv2.SIFT_create(nfeatures=0, nOctaveLayers=3, contrastThreshold=0.02, edgeThreshold=25, sigma=1)

        #detect and compute using SIFT
        self.__detec_and_compute = lambda img, m = None: self.__detector.detectAndCompute(img, mask = m)

    def set_bf_matcher(self):
        #if SIFT
        if self.__detector_type == self.__DETECTOR_TYPE_SIFT:
            self.__bfm = cv2.BFMatcher(normType=cv2.NORM_L2)
        else: #IF FAST or ORB
            self.__bfm = cv2.BFMatcher(normType=cv2.NORM_HAMMING)
    
    def detect_features(self, img, m = None):
        """
        Detect the feature in an image
        """
        return self.__detec_and_compute(img, m)
    
    def detect_features_in_rect(self, img, rect):
        """
        Detect the feature in an image that are inside the rectangle rect
        """
        rec_x_i, rec_y_i = rect[0]
        rec_x_e, rec_y_e = rect[1]

        keypoints, descriptors = self.__detec_and_compute(img)
        
        k_out = []
        d_out = []

        for i in range(len(keypoints)):
            (x, y) = keypoints[i].pt
            if (rec_x_i <= x <= rec_x_e) and (rec_y_i <= y <= rec_y_e):
                k_out.append(keypoints[i])
                d_out.append(descriptors[i])

        return np.array(k_out), np.array(d_out)
    
    def detect_features_in_polygon(self, img, polygon):
        """
        Detect the feature in an image that are inside the polygon
        """    
        keypoints, descriptors = self.__detec_and_compute(img)
        
        k_out = []
        d_out = []

        for i in range(len(keypoints)):
            if cv2.pointPolygonTest(polygon, keypoints[i].pt, False) >= 0:
                k_out.append(keypoints[i])
                d_out.append(descriptors[i])

        return np.array(k_out), np.array(d_out)
    

    def bf_matching_descriptors(self, d1, k2, d2, ratio, ref):
        """
        Matches the descriptors and returns the the variation in X and Y of the reference
        """
        matches = []
        if d1 is None:
            print("D1 NONE")
        elif d2 is None:
            print("D2 NONE")
        else:
            matches = self.__bfm.knnMatch(d1, d2, k=2)
            x_ref, y_ref =  ref
        
        error = []
        k_curr = []
        d_curr = []
        good_matches = []
        try:
            for m,n in matches:
                if m.distance < ratio * n.distance:
                    
                    # Getting the matching keypoints row in their lists
                    img2_id = m.trainIdx

                    # x ares columns
                    # y are rows
                    # Get the coordinates
                    (x2, y2) = k2[img2_id].pt
                    
                    dx = x2 - x_ref
                    dy = y2 - y_ref
                    
                    error.append((dx, dy))
                    k_curr.append(k2[img2_id])
                    d_curr.append(d2[img2_id])
                    good_matches.append(m)
        except:
            print("Failure in matching")

        
        return np.array(k_curr), np.array(d_curr), np.array(error), np.array(good_matches)

    def bf_matching(self, img1, img2, ratio):
        """
        make feature dection and matching using two images
        """
        #PARALLEL if FAST or ORB
        if self.__detector_type != self.__DETECTOR_TYPE_SIFT:
            executor = ThreadPoolExecutor(max_workers=2)
            f1 = executor.submit(self.__detec_and_compute, img1)
            f2 = executor.submit(self.__detec_and_compute, img2)
            keypoints1, descriptors1 = f1.result()
            keypoints2, descriptors2 = f2.result()
        else: #SEQUENTIAL if SIFT
            keypoints1, descriptors1 = self.__detec_and_compute(img1)
            keypoints2, descriptors2 = self.__detec_and_compute(img2)

        matches = self.__bfm.knnMatch(descriptors1, descriptors2, k=2)

        good_matches = []
        for m,n in matches:
            if m.distance < ratio * n.distance:
                good_matches.append(m)

        return keypoints1, keypoints2, good_matches
    
    T_BOOSTING = 'BOOSTING'
    T_MIL = 'MIL'
    T_KCF = 'KCF'
    T_TLD = 'TLD'
    T_MEDIANFLOW = 'MEDIANFLOW'
    T_MOSSE = 'MOSSE'
    T_CSRT = 'CSRT'

    def set_tracker(self, tracker_type):
        if tracker_type == self.T_BOOSTING:
            tracker = cv2.legacy.TrackerBoosting_create()
        if tracker_type == self.T_MIL:
            tracker = cv2.TrackerMIL_create() 
        if tracker_type == self.T_KCF:
            tracker = cv2.TrackerKCF_create() 
        if tracker_type == self.T_TLD:
            tracker = cv2.legacy.TrackerTLD_create() 
        if tracker_type == self.T_MEDIANFLOW:
            tracker = cv2.legacy.TrackerMedianFlow_create() 
        if tracker_type == self.T_MOSSE:
            tracker = cv2.legacy.TrackerMOSSE_create()
        if tracker_type == self.T_CSRT:
            tracker = cv2.TrackerCSRT_create()
        
        self.tracker = tracker
   
    def rotateImage(self, input, roll, pitch, yaw, dx, dy, dz, fx, fy, cx, cy):
        """
        Rotate an image
        """
        roll = roll * np.pi / 180
        pitch = pitch * np.pi / 180
        yaw = yaw * np.pi / 180

        A2 = np.array([[fx,  0, cx, 0],
                       [ 0, fy, cy, 0],
                       [ 0,  0,  1, 0]], dtype=np.float64)
            
        # Inverted Camera Calibration Intrinsics Matrix
        A1 = np.array([[1/fx,    0, -cx/fx],
                       [   0, 1/fy, -cy/fy],
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
        return cv2.warpPerspective(input, H, shape, None, cv2.INTER_LANCZOS4)