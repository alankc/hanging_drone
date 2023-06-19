import cv2
import time
import numpy as np
import utils as ut

from easydrone import EasyDrone
from vision import Vision
from stereo import Stereo
from yolo_detector import YOLODetector
from landing_pipeline import LandingPipeline
from communication import D2RS

class GlobalStateMachine:
    S_DISCONECTED = 0
    S_WAITING_RS = 1
    S_MANUAL = 2
    S_AUTONOMOUS = 3

    def __init__(self, parameters:dict) -> None:
        self.__parameters = parameters
        self.__select_rect = []
        self.__state = self.S_DISCONECTED

    def mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.__select_rect.append([x,y])

    def state_disconected(self):
        self.state_waiting_rs()
        if self.__state == self.S_DISCONECTED and self.__d2rs.wifi_conect(None):
            print("Ready to takeoff")
            self.__state = self.S_MANUAL
            self.__ed.connect()
            self.__ed.start()

    def state_waiting_rs(self):
        res_ssid = self.__d2rs.takeoff_request(0.5)
        if res_ssid:
            count = 5
            check = False
            while not check and count > 0:
                print(f"Trying to connect to Wifi: {res_ssid}")
                check = self.__d2rs.wifi_conect(res_ssid)
                count = count - 1

            if count == 0 and not check:
                print("Failed to connect wifi")
            else:
                print("Ready to takeoff")
                self.__state = self.S_MANUAL
                self.__ed.connect()
                self.__ed.start()

    def state_manual(self):
        pass

    def state_autonomous(self):
        pass

    def start(self):
        self.__ed = EasyDrone(True, self.__parameters['Camera']['stream'], self.__parameters['Control']['pid'])

        self.__v = Vision()
        if 'fast' in self.__parameters['Vision']:
            vp = self.__parameters['Vision']['fast']
            self.__v.set_fast_detector(nonmaxSuppression=vp['nonmaxSuppression'], type=vp['type'], threshold=vp['threshold'])
        elif 'sift' in self.__parameters['Vision']:
            self.__v.set_sift_detector()
        elif 'orb' in self.__parameters['Vision']: 
            self.__v.set_orb_detector()
        self.__v.set_bf_matcher()

        self.__s = Stereo()
        sp = self.__parameters['Camera']
        self.__s.set_camera_params(sp['fx'], sp['fy'], sp['pitch'], sp['cx'], sp['cy_aligned'])
        cx = sp['cx']
        cy = sp['cy_aligned']

        out_file = self.__parameters['Control']['folder_odom']

        self.__yd = YOLODetector(self.__parameters['YOLO']['path'])

        pwifi = self.__parameters['WiFi']
        prs = self.__parameters['RechargeStation']
        self.__d2rs = D2RS(prs['ip'], prs['port'], pwifi['interface'], pwifi['ssid'], pwifi['password'])

        cv2.namedWindow("Camera")
        cv2.setMouseCallback("Camera", self.mouse_click)

        manual_control = True
        lp = None

        time_start = time.time()
        alpha = 0.1
        fps = 0

        self.__ed.connect()
        self.__ed.start()

        while True:
            frame = self.__ed.get_curr_frame()
            
            if frame is None:
                    image = np.zeros((720, 960, 3), dtype = np.uint8)
                    self.__select_rect = []
                    key_delay = 30
            
            elif (len(self.__select_rect) < 1):
                    image = self.__s.rotateImage(frame)
                    key_delay = 1
                    
            image_s = image.copy()

            if not (frame is None) and (len(self.__select_rect) > 2) and manual_control:
                ut.draw_polylines(image_s, [np.array(self.__select_rect)])

            if not manual_control:
                #if not (lp is None):
                result = lp.run(image, image_s)
                if (result == lp.SUCESS) or (result == lp.FAIL):
                    manual_control = True
                    lp = None
                    self.__ed.rc_control() #STOPING all controllers
                #else:
                    #print("Press 1 to run landing pipeline with Yolo or")
                    #print("Press 2 to run landing pipeline after selecting landing site")
                    #manual_control = True
                
            if time.time() - time_start > 0:
                fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                time_start = time.time()
            
            if manual_control:
                ut.draw_text(image_s, f"FPS={fps:.1f}         MANUAL CONTROL", -1)
            else:
                ut.draw_text(image_s, f"FPS={fps:.1f}", -1)

            cv2.imshow('Camera', image_s)

            key = cv2.waitKey(key_delay) & 0xFF
            if key == 27:
                self.__ed.land()
                time.sleep(5)
                self.__ed.quit()
                self.__select_rect = []
                exit(0)

            elif key == ord("q"):
                self.__ed.land()
                print("*************************************************")
                print("******************** LAND ***********************")
                print("*************************************************", flush=True)

            elif key == ord("e"):
                self.__ed.takeoff()
                print("*************************************************")
                print("****************** TAKE OFF *********************")
                print("*************************************************", flush=True)

            elif key == ord("1"): # Use YOLO
                manual_control = False
                lp = LandingPipeline(self.__ed, self.__s, self.__v, self.__yd, None, None, int(round(cx, 0)), int(round(cy, 0)), out_file)
                self.__ed.PID_reset() #reseting all PIDs

            elif key == ord("2") and len(self.__select_rect) > 2: # USe selected rectangle
                manual_control = False
                k_ref, d_ref = self.__v.detect_features_in_polygon(image, np.array(self.__select_rect))
                self.__select_rect = []
                lp = LandingPipeline(self.__ed, self.__s, self.__v, self.__yd, k_ref, d_ref, int(round(cx, 0)), int(round(cy, 0)), out_file)
                self.__ed.PID_reset() #reseting all PIDs

            elif key == ord("3"):# Clear selected rectangle
                self.__select_rect = []

            elif key == ord(" "):# Change to manual control
                if not (lp is None):
                    manual_control = not manual_control
                    lp.reset()  
                else:
                    manual_control = True

                self.__ed.rc_control() #STOPING all controllers
                self.__ed.PID_reset() #reseting all PIDs

            elif manual_control: 
                ut.rc_control(key, self.__ed)
        