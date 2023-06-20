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
    S_WAITING_RS_TAKEOFF = 1
    S_WAITING_RS_LAND = 2
    S_MANUAL = 3
    S_MANUAL_LAND = 4
    S_AUTONOMOUS = 5
    S_GO_TO = 6

    def __init__(self, parameters:dict) -> None:
        self.__parameters = parameters
        self.__select_rect = []
        self.__state = self.S_DISCONECTED
        self.__lp = None

    def mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.__select_rect.append([x,y])

    def state_disconected(self):
        print("*************************************************")
        print("************* TRYING CONNECTION *****************")
        print("*************************************************", flush=True)
        self.state_waiting_rs_takeoff()
        if self.__state == self.S_DISCONECTED and self.__d2rs.wifi_conect(None):
            print("Ready to takeoff")
            self.__state = self.S_MANUAL
            self.__ed.connect()
            self.__ed.start()

    def state_waiting_rs_takeoff(self):
        
        frame = np.zeros((720, 960, 3), dtype = np.uint8)
        ut.draw_big_text(frame, "WAITING CONNECTION")
        ut.draw_text(frame, "Press ESC to exit", -1)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            exit(0)

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

    def state_waiting_rs_land(self):
        
        frame = np.zeros((720, 960, 3), dtype = np.uint8)
        ut.draw_big_text(frame, "WAITING LAND PERMISSION")
        ut.draw_text(frame, "Press ESC to exit", -1)
        ut.draw_text(frame, "Press SPACE return to MANUAL CONTROL", -2)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
            self.__select_rect = []
            exit(0)
        elif key  == ord(" "):
            self.__state = self.S_MANUAL
            return

        result = self.__d2rs.land_request(0.2)
        if result:
            self.__state = self.S_MANUAL_LAND

    def state_manual(self):
        
        frame = self.__ed.get_curr_frame()
            
        if frame is None:
            self.__image = np.zeros((720, 960, 3), dtype = np.uint8)
            ut.draw_big_text(self.__image, "WAITING FRAME")
            self.__select_rect = []
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                self.__ed.quit()
                exit(0)
            return
            
        elif (len(self.__select_rect) < 1):
            self.__image = self.__s.rotateImage(frame)
                    
        self.__image_s = self.__image.copy()

        if (len(self.__select_rect) > 2):
            ut.draw_polylines(self.__image_s, [np.array(self.__select_rect)])

        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f}         MANUAL CONTROL", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(1) & 0xFF
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
            ty = self.__parameters['Control']['ty']
            dhc = self.__parameters['Control']['drone_hook_center']
            self.__lp = LandingPipeline(self.__ed, self.__s, self.__v, self.__yd, None, None, int(round(self.__cx, 0)), int(round(self.__cy, 0)), self.__out_file, ty, dhc)
            self.__ed.PID_reset() #reseting all PIDs
            self.__state = self.S_AUTONOMOUS

        elif key == ord("2") and len(self.__select_rect) > 2: # USe selected rectangle
            k_ref, d_ref = self.__v.detect_features_in_polygon(self.__image, np.array(self.__select_rect))
            self.__select_rect = []
            ty = self.__parameters['Control']['ty']
            dhc = self.__parameters['Control']['drone_hook_center']
            self.__lp = LandingPipeline(self.__ed, self.__s, self.__v, self.__yd, k_ref, d_ref, int(round(self.__cx, 0)), int(round(self.__cy, 0)), self.__out_file, ty, dhc)
            self.__ed.PID_reset() #reseting all PIDs
            self.__state = self.S_AUTONOMOUS

        elif key == ord("3"):# Clear selected rectangle
            self.__select_rect = []

        elif key == ord("m"):# Manual land
            self.__state = self.S_WAITING_RS_LAND

        elif key == ord("g"):# Manual land
            self.__state = self.S_GO_TO
            self.__ed.set_destination(150, 150, 30, 0)

        elif key == ord(" "):# Clear selected rectangle
            if not (self.__lp is None):
                self.__ed.rc_control() #STOPING all controllers
                self.__ed.PID_reset() #reseting all PIDs
                self.__state = self.S_AUTONOMOUS
        else:
            ut.rc_control(key, self.__ed)

    def state_manual_land(self):
        
        frame = self.__ed.get_curr_frame()
            
        if frame is None:
            self.__image = np.zeros((720, 960, 3), dtype = np.uint8)
            ut.draw_big_text(self.__image, "WAITING FRAME")
            self.__select_rect = []
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                self.__ed.quit()
                exit(0)
            return
            
        self.__image = self.__s.rotateImage(frame)        
        self.__image_s = self.__image.copy()

        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f}         MANUAL LAND", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(1) & 0xFF
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
            self.__ed.quit()
            time.sleep(5)
            self.__ed = EasyDrone(True, self.__parameters['Camera']['stream'], self.__parameters['Control']['pid'])
            self.__state = self.S_WAITING_RS_TAKEOFF

        elif key == ord(" "):# Clear selected rectangle
                self.__state = self.S_MANUAL
        else:
            ut.rc_control(key, self.__ed)

    def state_autonomous(self):
        frame = self.__ed.get_curr_frame()
        self.__image = self.__s.rotateImage(frame)
        self.__image_s = self.__image.copy()

        result = self.__lp.run(self.__image, self.__image_s)
        if (result == self.__lp.SUCESS) or (result == self.__lp.FAIL):
            self.__state = self.S_MANUAL
            self.__lp = None
            self.__ed.rc_control() #STOPING all controllers

        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f}", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
            self.__select_rect = []
            exit(0)

        elif key == ord(" "):# Change to manual control
            self.__lp.reset()
            self.__ed.rc_control() #STOPING all controllers
            self.__ed.PID_reset() #reseting all PIDs
            self.__state = self.S_MANUAL
    
    def state_go_to(self):
        frame = self.__ed.get_curr_frame()
        self.__image = self.__s.rotateImage(frame)
        self.__image_s = self.__image.copy()

        if self.__ed.update_control():
            self.__state = self.S_MANUAL
            return
        
        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f}", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
            self.__select_rect = []
            exit(0)

        elif key == ord(" "):# Change to manual control
            self.__ed.rc_control() #STOPING all controllers
            self.__ed.PID_reset() #reseting all PIDs
            self.__state = self.S_MANUAL

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
        self.__cx = sp['cx']
        self.__cy = sp['cy_aligned']

        self.__out_file = self.__parameters['Control']['folder_odom']

        self.__yd = YOLODetector(self.__parameters['YOLO']['path'])

        pwifi = self.__parameters['WiFi']
        prs = self.__parameters['RechargeStation']
        self.__d2rs = D2RS(prs['ip'], prs['port'], pwifi['interface'], pwifi['ssid'], pwifi['password'])

        cv2.namedWindow("Camera")
        cv2.setMouseCallback("Camera", self.mouse_click)

        time_start = time.time()
        alpha = 0.1
        self.__fps = 0

        while True:
            curr_state = self.__state
            if curr_state == self.S_DISCONECTED:
                self.state_disconected()

            if curr_state == self.S_WAITING_RS_TAKEOFF:
                self.state_waiting_rs_takeoff()

            if curr_state == self.S_WAITING_RS_LAND:
                self.state_waiting_rs_land()

            if curr_state == self.S_MANUAL_LAND:
                self.state_manual_land()

            if curr_state == self.S_MANUAL:
                self.state_manual()

            if curr_state == self.S_AUTONOMOUS:
                self.state_autonomous()

            if curr_state == self.S_GO_TO:
                self.state_go_to()

            if time.time() - time_start > 0:
                self.__fps = (1 - alpha) * self.__fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                time_start = time.time()
            