#Laplacian of Gaussian (LoG)
#Anisotropic diffusion

import cv2
import numpy as np
import time

filter_list = []
filter_list.append(("Gaussian blur", lambda frame: cv2.GaussianBlur(frame, (5,5), 0))) #Gaussian blur
filter_list.append(("Median", lambda frame: cv2.medianBlur(frame, 5))) #Median
filter_list.append(("Bilateral", lambda frame: cv2.bilateralFilter(frame, d=5, sigmaColor=75, sigmaSpace=75))) #Bilateral
filter_list.append(("Sobel", lambda frame: cv2.Sobel(src=frame, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5))) #Sobel filter
filter_list.append(("Canny", lambda frame: cv2.Canny(frame, 100, 300))) #Canny filter 3x100 as in the paper
filter_list.append(("Unsharp masking", lambda frame: frame - 0.7 * cv2.Laplacian(cv2.medianBlur(frame, 5), cv2.CV_64F))) #Unsharp masking
def contrast_stretching(frame):
    minmax_img = np.zeros((frame.shape[0], frame.shape[1]),dtype = 'uint8')
    fmin = np.min(frame)
    fmax = np.max(frame)
    for i in range(frame.shape[0]):
        for j in range(frame.shape[1]):
            minmax_img[i,j] = 255 * (frame[i,j] - fmin)/(fmax - fmin)
    return minmax_img
filter_list.append(("Contrast stretching", contrast_stretching)) #Contrast stretching
filter_list.append(("Histogram", lambda frame: cv2.equalizeHist(frame))) #Histogram equalization

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


import sys
sys.path.insert(0, '/home/x/Documents/Experiments-RMTT/landing_system/')
from vision import Vision

if __name__ == "__main__":

    v = Vision()
    #v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=30)
    #v.set_sift_detector()
    v.set_orb_detector()
    v.set_bf_matcher()

    frame = cv2.imread("galho1.png")
    k,d = v.detect_features(frame)
    image_s = cv2.drawKeypoints(frame.copy(), k, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    draw_text(image_s, f"{len(k)}", 1)
    cv2.imshow("Original", image_s)    

    s = time.time()
    #g = cv2.GaussianBlur(frame, (7,7), 0)
    #new_frame = cv2.addWeighted(frame, 2, g, -1, 0) #Unsharp masking
    kernel = np.array([[ 0,-1, 0], 
                       [-1, 5,-1],
                       [ 0,-1, 0]])
    new_frame = cv2.filter2D(frame, -1, kernel)
    new_frame = cv2.bilateralFilter(new_frame, d=5, sigmaColor=80, sigmaSpace=80)

    #g = cv2.GaussianBlur(frame, (7,7), 0)
    #new_frame = cv2.addWeighted(frame, 2, g, -1, 0) #Unsharp masking
    #new_frame = cv2.bilateralFilter(new_frame, d=5, sigmaColor=80, sigmaSpace=80)

    dt = np.round((time.time() - s) * 1000, 2)

    k,d = v.detect_features(new_frame)
    print([x.pt for x in k])

    new_frame = cv2.drawKeypoints(new_frame.copy(), k, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    draw_text(new_frame, f"Time: {dt} ms", 0)
    draw_text(new_frame, f"{len(k)}", 1)
    cv2.imshow("1", new_frame)

    """
    for i in range(len(filter_list)):
        s = time.time()
        new_frame = filter_list[i][1](frame.copy())
        dt = np.round((time.time() - s) * 1000, 2)
        draw_text(new_frame, f"Time: {dt} ms", 0)
        cv2.imshow(filter_list[i][0], new_frame)
    """

    cv2.waitKey(0)
    cv2.destroyAllWindows()
