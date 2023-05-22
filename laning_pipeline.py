import numpy as np
import time
import cv2
import utils as ut

from simple_pid import PID
from easydrone import EasyDrone
from stereo import Stereo
from vision import Vision

class LandingPipeline:
    def __init__(self, ed:EasyDrone, s:Stereo, v:Vision, k_ref, d_ref, cx, cy) -> None:
        self.__ed = ed
        self.__s = s
        self.__v = v
        self.__k_ref = k_ref
        self.__d_ref = d_ref
        self.__cx = cx
        self.__cy = cy

        self.__state = 0


    def PID_setup(self):
        """
        This function must be configured in the class
        """
        self.__pid_s_yaw = PID(Kp=-1/960, Ki=-0.2/960, Kd=-0.1/960)
        self.__pid_s_yaw.output_limits = (-0.3, 0.3) 
        self.__pid_s_yaw.setpoint = 0
        self.__pid_s_yaw.sample_time = None
        self.__pid_s_yaw.set_auto_mode(True, last_output=0)

        self.__pid_s_throttle = PID(Kp=1/240, Ki=0.25/240, Kd=0.3/240)
        self.__pid_s_throttle.output_limits = (-0.3, 0.3) 
        self.__pid_s_throttle.setpoint = 0
        self.__pid_s_throttle.sample_time = None
        self.__pid_s_throttle.set_auto_mode(True, last_output=0)

        self.__pid_throttle = PID(Kp=1/30, Ki=1/40, Kd=1/60)
        self.__pid_throttle.output_limits = (-0.3, 0.3) 
        self.__pid_throttle.setpoint = 0
        self.__pid_throttle.sample_time = None
        self.__pid_throttle.set_auto_mode(True, last_output=0)

        self.__pid_pitch = PID(Kp=1/30, Ki=0, Kd=0.25/25)
        self.__pid_pitch.output_limits = (-0.2, 0.2) 
        self.__pid_pitch.setpoint = 0
        self.__pid_pitch.sample_time = None
        self.__pid_pitch.set_auto_mode(True, last_output=0)

        self.__pid_roll = PID(Kp=1/100, Ki=1/800, Kd=1/1100) 
        self.__pid_roll.output_limits = (-0.3, 0.3) 
        self.__pid_roll.setpoint = 0
        self.__pid_roll.sample_time = None
        self.__pid_roll.set_auto_mode(True, last_output=0)

        self.__pid_yaw = PID(Kp=1/15, Ki=0, Kd=0.25/20)
        self.__pid_yaw.output_limits = (-0.5, 0.5) 
        self.__pid_yaw.setpoint = 0
        self.__pid_yaw.sample_time = None
        self.__pid_yaw.set_auto_mode(True, last_output=0)

    def run(self):

        time_start = time.time()
        alpha = 0.1
        fps = 0
        while self.__state < 6:

            image = self.__ed.get_curr_frame()
            if image is None:
                frame = np.zeros((10, 10, 3), dtype = np.uint8)
                time.sleep(0.1)
                cv2.imshow('Camera', frame)
                cv2.waitKey(1)
                continue
            
            #Image used in data processing
            self.__image = self.__s.rotateImage(image)
            
            #Image used only to show
            self.__image_s = image.copy()
            
            #loop state ensure that when the state is updated the loop will restart before run the next state
            loop_state = self.__state

            #state 0: the drone has to center a region of interest with cx and cy
            if loop_state == 0:
                self.state_0()

            if loop_state == 1:
                self.state_1()
            
            if loop_state == 2:
                self.state_2()
            
            if loop_state == 3:
                self.state_3()
            
            if loop_state == 4:
                self.state_4()
            
            if loop_state == 5:
                self.state_5()

            if loop_state == 6:
                self.state_6()
            
            if time.time() - time_start > 0:
                fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                time_start = time.time()

            ut.draw_text(self.__image_s, f"FPS={fps}", -1)
            ut.draw_text(self.__image_s, f"State={loop_state}", -2)
            height, width, _ = self.__image_s.shape
            ut.draw_line(self.__image_s, (self.__cx, 0), (self.__cx, height))
            ut.draw_line(self.__image_s, (0, self.__cy), (width, self.__cy))
            cv2.imshow('Camera', self.__image_s)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.__ed.land()
                time.sleep(5)
                self.__ed.quit()
                break

        print("STATE 7")
        self.__ed.land()
        time.sleep(5)
        self.__ed.quit()

    def state_0(self):
        #detect features in the current image
        k, d = self.__v.detect_features(self.__image)

        #get list of matched features
        k_curr, d_curr, error, _ = self.__v.bf_matching_descriptors(self.__d_ref, k, d, 0.65, (self.__cx, self.__cy))
        self.__image_s = cv2.drawKeypoints(self.__image_s, k_curr, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

        #at least 2 feature matched
        if len(error) > 1:
            #computing mean error
            mean_error = np.mean(error, axis=0)
            error_cx = mean_error[0]
            error_cy = mean_error[1]

            #computing control output based on features matched
            ctrl_s_yaw = np.round(self.__pid_s_yaw(error_cx), 2)
            ctrl_s_throttle = np.round(self.__pid_s_throttle(error_cy), 2)
            self.__ed.set_yaw(ctrl_s_yaw)
            self.__ed.set_throttle(ctrl_s_throttle)

            #drawing the point in image that must be in the center
            ut.draw_dot(self.__image_s, (int(self.__cx + error_cx), int(self.__cy + error_cy)))

            #error in y and x < 10 pixels and control output for yaw and throttle < 0.1
            if (abs(error_cy) < 10) and (abs(error_cx) < 10) and (abs(ctrl_s_yaw) < 0.1) and (abs(ctrl_s_throttle) < 0.1):
                #stop the drone
                self.__ed.rc_control()
                self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                self.__pid_s_throttle.set_auto_mode(enabled=True, last_output=0)

                #save keypoints and descriptors
                self.__k_start = k_curr
                self.__d_start = d_curr

                #save current pose
                self.__p_start = self.__ed.get_curr_pos()

                #setthe pid throttle to the current height + 15cm
                self.__pid_throttle.setpoint = self.__p_start[2] + 15

                print("-----------------------------------------------")
                print( " STATE 0 END")
                print(f" Start Position: {self.__p_start} ")
                print("-----------------------------------------------")
                self.__state = 1   

        else:
            self.__ed.set_yaw(0)
            self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                
            self.__ed.set_throttle(0)
            self.__pid_s_throttle.set_auto_mode(enabled=True, last_output=0)     
            

    def state_1(self):
        print("asd")

    def state_2(self):
        pass

    def state_3(self):
        pass

    def state_4(self):
        pass

    def state_5(self):
        pass

    def state_6(self):
        pass