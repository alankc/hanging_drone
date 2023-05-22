import cv2
import numpy as np
import time
import sys
sys.path.insert(0, '/home/alan/Documents/Experiments-RMTT/landing_system/')
from vision import Vision

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

#https://github.com/bnsreenu/python_for_microscopists/blob/master/Tips_Tricks_45_white-balance_using_python/Tips_Tricks_45_white-balance_using_python.py
def GW_white_balance(img):
    img_LAB = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(img_LAB[:, :, 1])
    avg_b = np.average(img_LAB[:, :, 2])
    img_LAB[:, :, 1] = img_LAB[:, :, 1] - ((avg_a - 128) * (img_LAB[:, :, 0] / 255.0) * 1.2)
    img_LAB[:, :, 2] = img_LAB[:, :, 2] - ((avg_b - 128) * (img_LAB[:, :, 0] / 255.0) * 1.2)
    balanced_image = cv2.cvtColor(img_LAB, cv2.COLOR_LAB2BGR)
    return balanced_image


if __name__ == "__main__":

    v = Vision()
    v.set_fast_detector(nonmaxSuppression=1, type=2, threshold=30)
    #v.set_sift_detector()
    #v.set_orb_detector()
    v.set_bf_matcher()

    nf1 = cv2.imread("galho1.png")
    nf2 = cv2.imread("galho2.png")
    kernel = np.array([[ 0,-1, 0], 
                       [-1, 5,-1],
                       [ 0,-1, 0]])
    
    nf1 = GW_white_balance(nf1)
    nf1 = cv2.filter2D(nf1, -1, kernel)
    nf1 = cv2.bilateralFilter(nf1, d=5, sigmaColor=80, sigmaSpace=80)

    nf2 = GW_white_balance(nf2)
    nf2 = cv2.filter2D(nf2, -1, kernel)
    nf2 = cv2.bilateralFilter(nf2, d=5, sigmaColor=80, sigmaSpace=80)

    k1, k2, gm = v.bf_matching(nf1, nf2, 0.7)

    out = cv2.drawMatches(nf1, k1, nf2, k2, gm, None, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    #out = cv2.drawMatches(nf1, k1, nf2, k2, gm, None)
    draw_text(out, f"{len(gm)} matches", 0)
    draw_text(out, f"{len(k1)}kpts                  {len(k2)}kpts", -1)
    cv2.imshow("Matches", out)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
