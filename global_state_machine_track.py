import cv2
import time
import numpy as np
import utils as ut
import tkinter as tk

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
        self.__state = self.S_DISCONECTED
        self.__curr_state_method = self.state_disconected
        self.__lp = None

    def state_disconected(self):
        """
        First, try to receive SSID from the recharge station. 
        If it does not receive, try to connect to the ssid saved on the yaml file

        Options:
            ESC: Exit
        """
        print("*************************************************")
        print("************* TRYING CONNECTION *****************")
        print("*************************************************", flush=True)
        self.state_waiting_rs_takeoff()
        if self.__state == self.S_DISCONECTED and self.__d2rs.wifi_conect(None):
            print("Ready to takeoff")
            self.__state = self.S_MANUAL
            self.__curr_state_method = self.state_manual
            self.__ed.connect()
            self.__ed.start()

    def state_waiting_rs_takeoff(self):
        """
        First, try to receive SSID from the recharge station. 
        If it works, try to connect to it

        Options:
            ESC: Exit
        """
        frame = np.zeros((720, 960, 3), dtype = np.uint8)
        ut.draw_big_text(frame, "WAITING CONNECTION")
        ut.draw_text(frame, "Press ESC to exit", -1)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(10) & 0xFF
        if key == 27:
            exit(0)

        res_ssid = self.__d2rs.takeoff_request(0.5)
        if res_ssid:
            count = 1
            check = False
            while not check and count <= 30:
                
                attempt_str = f"Attempt {count} - Connecting to WiFi: {res_ssid}"
                
                #SCREEN
                frame_s = frame.copy()
                ut.draw_text(frame_s, attempt_str, -2)
                cv2.imshow('Camera', frame_s)
                key = cv2.waitKey(10) & 0xFF
                if key == 27:
                    exit(0)

                #TERMINAL
                print(attempt_str)

                #RUN
                check = self.__d2rs.wifi_conect(res_ssid)
                count = count + 1

            if not check:
                print("Failed to connect wifi")
            else:
                print("Ready to takeoff")
                self.__state = self.S_MANUAL
                self.__curr_state_method = self.state_manual
                self.__ed.connect()
                self.__ed.start()

    def state_waiting_rs_land(self):
        """
        Waiting land permission

        Options:
            ESC: Exit
            SPACE: return to MANUAL CONTROL
        """
        frame = np.zeros((720, 960, 3), dtype = np.uint8)
        ut.draw_big_text(frame, "WAITING LAND PERMISSION")
        ut.draw_text(frame, "Press ESC to exit", -1)
        ut.draw_text(frame, "Press SPACE return to MANUAL CONTROL", -2)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(10) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
            exit(0)
        elif key  == ord(" "):
            self.__state = self.S_MANUAL
            self.__curr_state_method = self.state_manual
            return

        result = self.__d2rs.land_request(0.2)
        if result:
            self.__state = self.S_MANUAL_LAND
            self.__curr_state_method = self.state_manual_land

    def state_manual(self):
        """
        Manual control

        General options:
            ESC: Exit
            1: Runs landing pipeline with Yolo (autonomous mode)
            2: Runs landing pipeline with the selected shape (autonomous mode)
            3: Reset the selected shape
            m: Send land request
            g: Open destination window
            SPACE: switch between autonomous and manual
            
        Movement OPTIONS:
            q: land
            e: takeoff

            w: up
            s: down
            a: rotate counterclockwise
            d: rotate clockwise

            i: forward
            k: backward
            j: left
            l: right
        """
        frame = self.__ed.get_curr_frame()
            
        if frame is None:
            self.__image = np.zeros((720, 960, 3), dtype = np.uint8)
            ut.draw_big_text(self.__image, "WAITING FRAME")
            key = cv2.waitKey(10) & 0xFF
            if key == 27:
                self.__ed.quit()
                exit(0)
            return
            
        self.__image = self.__s.rotateImage(frame)
                    
        self.__image_s = self.__image.copy()

        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f}         MANUAL CONTROL", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(20) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
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
            self.__ed.attitude_reset() #ensure proper positions during the landing
            self.__state = self.S_AUTONOMOUS
            self.__curr_state_method = self.state_autonomous

        elif key == ord("2"): # USe selected rectangle
            bbox = cv2.selectROI('Camera', self.__image, False)
            
            if bbox != (0,0,0,0):

                self.__v.tracker.init(self.__image, bbox)

                ty = self.__parameters['Control']['ty']
                dhc = self.__parameters['Control']['drone_hook_center']
                self.__lp = LandingPipeline(self.__ed, self.__s, self.__v, self.__yd, bbox, self.__image, int(round(self.__cx, 0)), int(round(self.__cy, 0)), self.__out_file, ty, dhc)
                self.__ed.PID_reset() #reseting all PIDs
                self.__ed.attitude_reset() #ensure proper positions during the landing
                self.__state = self.S_AUTONOMOUS
                self.__curr_state_method = self.state_autonomous

        elif key == ord("m"):# Manual land
            self.__state = self.S_WAITING_RS_LAND
            self.__curr_state_method = self.state_waiting_rs_land

        elif key == ord("g"):# Manual land
            gdw = GetDataWindow()
            res = gdw.get_values()
            if res:
                (x, y, z, yaw) = res
                self.__state = self.S_GO_TO
                self.__curr_state_method = self.state_go_to
                self.__ed.PID_reset()
                self.__ed.attitude_reset()
                self.__ed.save_quaternion()
                self.__ed.set_destination(x, y, z, yaw)

        elif key == ord(" "):# Clear selected rectangle
            if not (self.__lp is None):
                self.__ed.rc_control() #STOPING all controllers
                self.__ed.PID_reset() #reseting all PIDs
                self.__state = self.S_AUTONOMOUS
                self.__curr_state_method = self.state_autonomous
        else:
            ut.rc_control(key, self.__ed)

    def state_manual_land(self):
        """
        Manual Land

        Options:
            ESC: Exit
            q: land
            e: disconnect from drone
            SPACE: return to manual control

        Movement Options:
            w: up
            s: down
            a: rotate counterclockwise
            d: rotate clockwise

            i: forward
            k: backward
            j: left
            l: right
        """
        frame = self.__ed.get_curr_frame()
            
        if frame is None:
            self.__image = np.zeros((720, 960, 3), dtype = np.uint8)
            ut.draw_big_text(self.__image, "WAITING FRAME")
            key = cv2.waitKey(10) & 0xFF
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
            exit(0)

        elif key == ord("q"):
            self.__ed.land()
            print("*************************************************")
            print("******************** LAND ***********************")
            print("*************************************************", flush=True)

        elif key == ord("e"):
            self.__ed.quit()
            time.sleep(5)
            print("DELETING EASY DRONE")
            self.__ed = EasyDrone(True, self.__parameters['Camera']['stream'], self.__parameters['Control']['pid'], self.__parameters['WiFi'])
            self.__state = self.S_WAITING_RS_TAKEOFF
            self.__curr_state_method = self.state_waiting_rs_takeoff

        elif key == ord(" "):# Clear selected rectangle
                self.__state = self.S_MANUAL
                self.__curr_state_method = self.state_manual
        else:
            ut.rc_control(key, self.__ed)

    def state_autonomous(self):
        """
        Autonomous control

        Options:
            ESC: Exit
            SPACE: switch between autonomous and manual
        """

        frame = self.__ed.get_curr_frame()
        self.__image = self.__s.rotateImage(frame)
        self.__image_s = self.__image.copy()

        result = self.__lp.run(self.__image, self.__image_s)
        if result == self.__lp.SUCCESS:
            self.__state = self.S_MANUAL
            self.__curr_state_method = self.state_manual
            self.__lp = None
            self.__ed.rc_control() #STOPING all controllers
        
        elif result == self.__lp.FAIL:
            self.__state = self.S_GO_TO
            self.__curr_state_method = self.state_go_to
            self.__ed.rc_control()

            start_pos = self.__lp.get_p_start()
            (sy, sx, sz) = self.__ed.rotate_pos(start_pos)

            curr_pos = self.__ed.get_curr_pos_corrected()
            (cy, cx, cz) = curr_pos

            x = sx - cx
            y = sy - cy
            z = sz - cz

            self.__ed.PID_reset()
            self.__ed.set_destination(x, y, z, 0)

        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f}", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
            exit(0)

        elif key == ord(" "):# Change to manual control
            self.__lp.reset()
            self.__ed.rc_control() #STOPING all controllers
            self.__ed.PID_reset() #reseting all PIDs
            self.__state = self.S_MANUAL
            self.__curr_state_method = self.state_manual
    
    def state_go_to(self):
        """
        Go to desired position

        Options:
            ESC: Exit
            SPACE: return to manual mode
        """
        frame = self.__ed.get_curr_frame()
        self.__image = self.__s.rotateImage(frame)
        self.__image_s = self.__image.copy()

        if self.__ed.update_control():
            self.__state = self.S_MANUAL
            self.__curr_state_method = self.state_manual
            return
        
        (cy, cx, cz) = self.__ed.get_curr_pos_corrected()
        cyaw = self.__ed.get_curr_yaw()
        px = self.__ed.pid_roll.setpoint
        py = self.__ed.pid_pitch.setpoint
        pz = self.__ed.pid_throttle.setpoint
        pyaw = self.__ed.pid_yaw.setpoint

        ut.draw_text(self.__image_s, f"CURR: {cx:.1f} {cy:.1f} {cz:.1f} {cyaw:.1f}", 0)
        ut.draw_text(self.__image_s, f"DEST: {px:.1f} {py:.1f} {pz:.1f} {pyaw:.1f}", 1)

        ut.draw_text(self.__image_s, f"FPS={self.__fps:.1f} GOING TO", -1)
        cv2.imshow('Camera', self.__image_s)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.__ed.land()
            time.sleep(5)
            self.__ed.quit()
            exit(0)

        elif key == ord(" "):# Change to manual control
            self.__ed.rc_control() #STOPING all controllers
            self.__ed.PID_reset() #reseting all PIDs
            self.__state = self.S_MANUAL
            self.__curr_state_method = self.state_manual

    def setup(self):
        """
        Mandatory initial setup
        """
        self.__ed = EasyDrone(True, self.__parameters['Camera']['stream'], self.__parameters['Control']['pid'], self.__parameters['WiFi'])

        self.__v = Vision()
        if 'fast' in self.__parameters['Vision']:
            vp = self.__parameters['Vision']['fast']
            self.__v.set_fast_detector(nonmaxSuppression=vp['nonmaxSuppression'], type=vp['type'], threshold=vp['threshold'])
        elif 'sift' in self.__parameters['Vision']:
            self.__v.set_sift_detector()
        elif 'orb' in self.__parameters['Vision']: 
            self.__v.set_orb_detector()
        self.__v.set_bf_matcher()
        
        self.__v.set_tracker(self.__parameters['Vision']['tracker'])

        self.__s = Stereo()
        sp = self.__parameters['Camera']
        self.__s.set_camera_params(sp['fx'], sp['fy'], sp['pitch'], sp['cx'], sp['cy_aligned'])
        self.__cx = sp['cx']
        self.__cy = sp['cy_aligned']

        self.__out_file = None
        if self.__parameters['Control']['folder_odom']:
            self.__out_file = open(self.__parameters['Control']['folder_odom'], "a")

        self.__yd = YOLODetector(self.__parameters['YOLO']['path'])

        pwifi = self.__parameters['WiFi']
        prs = self.__parameters['RechargeStation']
        self.__d2rs = D2RS(prs['ip'], prs['port'], pwifi)

        self.__desired_fps = self.__parameters['Control']['desired_fps']

        cv2.namedWindow("Camera")
        cv2.setMouseCallback("Camera", self.mouse_click)

    def start(self):
        """
        State Machine loop
        """
        time_start = time.time()
        alpha = 0.1
        self.__fps = 0

        while True:
            
            curr_dt = time.time()-time_start
            if curr_dt < 1.0/self.__desired_fps:
                t_delay = 1.0/self.__desired_fps - curr_dt
                time.sleep(t_delay / 10)
                continue

            self.__fps = (1 - alpha) * self.__fps + alpha * 1 / curr_dt  # exponential moving average
            time_start = time.time()

            self.__curr_state_method()



class GetDataWindow:
    def __init__(self) -> None:
        self.__root = tk.Tk()
        self.__root.eval('tk::PlaceWindow . center')
        self.__root.title("Position")
        self.__root.resizable(0,0)

        self.__xtk = tk.StringVar(None, "0")
        self.__ytk = tk.StringVar(None, "0")
        self.__ztk = tk.StringVar(None, "0")
        self.__yawtk = tk.StringVar(None, "0")

        tk.Label(self.__root, text="  x").grid(row=0)
        tk.Label(self.__root, text="  y").grid(row=1)
        tk.Label(self.__root, text="  z").grid(row=2)
        tk.Label(self.__root, text="yaw").grid(row=3)
        tk.Button(self.__root,text ="Enter", command=self.__save_values, activebackground='green', justify='center').grid(row=4, column=1)

        e1 = tk.Entry(self.__root, textvariable=self.__xtk).grid(row=0, column=1)
        e2 = tk.Entry(self.__root, textvariable=self.__ytk).grid(row=1, column=1)
        e3 = tk.Entry(self.__root, textvariable=self.__ztk).grid(row=2, column=1)
        e4 = tk.Entry(self.__root, textvariable=self.__yawtk).grid(row=3, column=1)

    def __save_values(self):
        self.__x = self.__xtk.get().strip()
        self.__y = self.__ytk.get().strip()
        self.__z = self.__ztk.get().strip()
        self.__yaw = self.__yawtk.get().strip()
        self.__root.destroy()
          
    def get_values(self):
        self.__x = ''
        self.__y = ''
        self.__z = ''
        self.__yaw = ''

        self.__root.mainloop()
        self.__root.quit()

        if (self.__x == '') or (self.__y == '') or (self.__z == '') or (self.__yaw == ''):
            return None

        return (float(self.__x), float(self.__y), float(self.__z), float(self.__yaw))