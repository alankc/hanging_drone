import cv2
import numpy as np
from ultralytics import YOLO

class YOLODetector:
    def __init__(self, path_to_weights) -> None:
        self.__model = YOLO(path_to_weights)

    def detect_best(self, image, confidence = 0.7, classes_to_detect=[0]):
        results = self.__model(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), conf=confidence, classes=classes_to_detect)
        best_func = 0
        best_pt1 = None
        best_pt2 = None
        for r in results:
            #getting the boxes from the class
            boxes = r.boxes
            for box in boxes:  
                b = box.xyxy[0]  #coordinates in (top, left, bottom, right)
                pt1 = np.int32((b[0].cpu().numpy(), b[1].cpu().numpy()))
                pt2 = np.int32((b[2].cpu().numpy(), b[3].cpu().numpy()))
                #func = box.conf.cpu().numpy()[0] * abs(pt2[0] - pt1[0]) / abs(pt2[1] - pt1[1]) #confidence * rectangle_width / rectangle_height
                func = abs(pt2[0] - pt1[0]) / abs(pt2[1] - pt1[1])
                if func > best_func:
                    best_func = func 
                    best_pt1 = pt1
                    best_pt2 = pt2

        return best_pt1, best_pt2

if __name__ == "__main__":
    import utils as ut

    yd = YOLODetector("yolov8x.pt")
    vid = cv2.VideoCapture(0)

    while True:
        ret, frame = vid.read()
        pt1, pt2 = yd.detect_best(frame, classes_to_detect=[0,41])
        ut.draw_rectangle(frame, pt1, pt2)
        
        cv2.imshow('frame', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break