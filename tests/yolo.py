from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

# Load a model
model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)

import sys
sys.path.insert(0, '../')
import cv2
import numpy as np
import time
import utils as ut
from easydrone import EasyDrone


if __name__ == "__main__":
           
    ed = EasyDrone(True)
    ed.connect()
    ed.start()
    ed.cooler_on()

    time_start = time.time()
    fps = 0
    alpha = 0.1
    while True:

        image = ed.get_curr_frame()
        if image is None:
            frame = np.zeros((10, 10, 3), dtype = np.uint8)
            time.sleep(0.1)
            cv2.imshow('Camera', frame)
            key = cv2.waitKey(1) & 0xFF
            continue

        if time.time() - time_start > 0:
            fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
            time_start = time.time()

        results = model(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))  # predict on an image

        for r in results:
            
            annotator = Annotator(image)
            
            boxes = r.boxes
            for box in boxes:
                
                b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                annotator.box_label(b, model.names[int(c)], (255,0,0))
            
        image = annotator.result() 

        ut.draw_text(image, f"FPS={fps}", -1)
        cv2.imshow("Camera", image)

        

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or not ed.is_alive():
            ed.quit()
            break


    ed.quit()