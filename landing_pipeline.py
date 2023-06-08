import numpy as np
import time
import cv2
import utils as ut

from simple_pid import PID
from easydrone import EasyDrone
from stereo import Stereo
from vision import Vision
from yolo_detector import YOLODetector

class LandingPipeline:
    def __init__(self, ed:EasyDrone, s:Stereo, v:Vision, yd:YOLODetector, k_ref, d_ref, cx, cy, odom_file = None) -> None:
        self.__ed = ed
        self.__s = s
        self.__v = v
        self.__yd = yd
        self.__k_ref = k_ref
        self.__d_ref = d_ref
        self.__cx = cx
        self.__cy = cy
        self.__odom_file = odom_file
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
        
        KP = 1/150
        TI = 8
        TD = 1 
        self.__pid_pitch = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD)
        #self.__pid_pitch = PID(Kp=1/30, Ki=0, Kd=0.25/25)
        self.__pid_pitch.output_limits = (-0.2, 0.2) 
        self.__pid_pitch.setpoint = 0
        self.__pid_pitch.sample_time = None
        self.__pid_pitch.set_auto_mode(True, last_output=0)

        KP = 1/150
        TI = 5
        TD = 2
        self.__pid_roll = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD)
        #self.__pid_roll = PID(Kp=1/100, Ki=1/800, Kd=1/1100)
        #self.__pid_roll = PID(Kp=1/50, Ki=1/40, Kd=1/80) #good one
        self.__pid_roll.output_limits = (-0.2, 0.2) 
        self.__pid_roll.setpoint = 0
        self.__pid_roll.sample_time = None
        self.__pid_roll.set_auto_mode(True, last_output=0)
        
        KP = 1/10
        TI = 1
        TD = 0.1
        self.__pid_yaw = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD)
        #self.__pid_yaw = PID(Kp=1/15, Ki=0, Kd=0.25/20)
        self.__pid_yaw.output_limits = (-0.5, 0.5) 
        self.__pid_yaw.setpoint = 0
        self.__pid_yaw.sample_time = None
        self.__pid_yaw.set_auto_mode(True, last_output=0)

    def run(self):

        time_start = time.time()
        alpha = 0.1
        fps = 0
        while self.__state < 8:

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
            self.__image_s = self.__image.copy()
            
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
            
            if loop_state == 7:
                self.state_7()
           
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
                print("*************************************************")
                print("*************************************************")
                print("*************** LAND CANCELED *******************")
                print("*************************************************")
                print("*************************************************")
                break
            elif key == ord(" "):
                if self.__state == -1:
                    self.__state = 0 #OFF - RC CONTROL
                    self.__ed.rc_control()
                    self.PID_setup()
                else:
                    self.__state = -1 #ON - RC CONTROL
                    self.__ed.rc_control()
            elif loop_state == -1:
                ut.rc_control(key, self.__ed)
        
        cv2.destroyAllWindows()
        if self.__state == 9: #fail due roll
                self.__ed.land()
                time.sleep(5)
                self.__ed.quit()

    #state 0: using yolo to detect branch
    def state_0(self):

        if not ((self.__k_ref is None) and (self.__d_ref is None)):
            self.__state = self.__state + 1
            return

        pt1, pt2 = self.__yd.detect_best(self.__image)
        if not ((pt1 is None) and (pt2 is None)):
            #computing error
            error_cx = (pt1[0] + pt2[0]) * 0.5 - self.__cx
            error_cy = (pt1[1] + pt2[1]) * 0.5 - self.__cy

            #computing control output based on features matched
            ctrl_s_yaw = np.round(self.__pid_s_yaw(error_cx), 2)
            ctrl_s_throttle = np.round(self.__pid_s_throttle(error_cy), 2)
            self.__ed.set_yaw(ctrl_s_yaw)
            self.__ed.set_throttle(ctrl_s_throttle)

            #error in y and x < 10 pixels and control output for yaw and throttle < 0.1
            if (abs(error_cy) < 10) and (abs(error_cx) < 10) and (abs(ctrl_s_yaw) < 0.1) and (abs(ctrl_s_throttle) < 0.1):
                #stop the drone
                self.__ed.rc_control()
                self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                self.__pid_s_throttle.set_auto_mode(enabled=True, last_output=0)

                #save keypoints and descriptors
                k_ref, d_ref = self.__v.detect_features_in_rect(self.__image, [pt1, pt2])

                self.__k_ref = k_ref
                self.__d_ref = d_ref

                self.__state = self.__state + 1

            #drawing the point in image that must be in the center
            ut.draw_dot(self.__image_s, (int(self.__cx + error_cx), int(self.__cy + error_cy)))
            ut.draw_rectangle(self.__image_s, pt1, pt2)

        else:
            self.__ed.set_yaw(0)
            self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                
            self.__ed.set_throttle(0)
            self.__pid_s_throttle.set_auto_mode(enabled=True, last_output=0)   

    #state 1: the drone has to center a region of interest with cx and cy
    def state_1(self):
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
                self.__state = self.__state + 1   

        else:
            self.__ed.set_yaw(0)
            self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                
            self.__ed.set_throttle(0)
            self.__pid_s_throttle.set_auto_mode(enabled=True, last_output=0)     
            
    #state 2: center cx and move up
    def state_2(self):
        error_cz = self.__pid_throttle.setpoint - self.__ed.get_curr_pos()[2]
        #computing control output based on height
        ctrl_throttle = np.round(self.__pid_throttle(self.__ed.get_curr_pos()[2]), 2)
        self.__ed.set_throttle(ctrl_throttle)

        if abs(error_cz) < 2 and abs(ctrl_throttle) < 0.1:
            k, d = self.__v.detect_features(self.__image)
            
            #get list of matched features
            k_curr, d_curr, error, matches = self.__v.bf_matching_descriptors(self.__d_start, k, d, 0.65, (self.__cx, self.__cy))
            self.__image_s = cv2.drawKeypoints(self.__image_s, k_curr, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

            if len(error) > 1:
                #computing mean error
                mean_error = np.mean(error, axis=0)
                error_cx = mean_error[0]

                #computing control output based on features matched
                ctrl_s_yaw = np.round(self.__pid_s_yaw(error_cx), 2)
                self.__ed.set_yaw(ctrl_s_yaw)

                #drawing the point in image that must be in the center
                ut.draw_dot(self.__image_s, (int(self.__cx + error_cx), int(self.__cy)))

                #stop criteria of esate 1
                if (abs(error_cx) < 10) and (abs(ctrl_throttle) < 0.05) and (abs(ctrl_s_yaw) < 0.1):
                    #stop the drone
                    self.__ed.rc_control()
                    self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                    self.__pid_throttle.set_auto_mode(enabled=True, last_output=0)

                    #save keypoints, descriptors and good matches
                    self.__k_end = k
                    self.__d_end = d
                    self.__good_matches = matches

                    #save current pose
                    self.__p_end = self.__ed.get_curr_pos()
                    self.__ed.save_quaternion()
                        
                    print("-----------------------------------------------")
                    print( " STATE 1 END")
                    print(f" End Position: {self.__p_end} ")
                    print("-----------------------------------------------")
                    self.__state = self.__state + 1   

            else: # YES, it is necessary, do not remove!
                self.__ed.set_yaw(0)
                self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
            
        else: # YES, it is also necessary, do not remove!
            self.__ed.set_yaw(0)
            self.__pid_s_yaw.set_auto_mode(enabled=True, last_output=0)

    #state 3: compute landing site position
    def state_3(self):
        #compute relative position
        ty = self.__p_end[2] - self.__p_start[2]
        # x, y in the image and depth 
        x_out, y_out, depth_out, yaw_out, roll_out  = self.__s.compute_relative_depth(ty, self.__k_start, self.__k_end, self.__good_matches)

        if abs(roll_out) > 15: #cant land
            print("*************************************************")
            print("*************************************************")
            print("************** ROLL ANGLE TOO BIG ***************")
            print("*************************************************")
            print("*************************************************")
            self.__state = 9

        #choose as landing site the detect feature neares the mean
        # 1 - Compute mean
        depth_mean = np.mean(depth_out)
        # 2 - Calculate the difference array
        difference_array = np.absolute(depth_out-depth_mean)
        # 3 - Find the index of minimum element from the array
        index = difference_array.argmin()
        # 4 - Get the nearest position of the mean in world's coordinates
        y_pos = depth_mean
        x_pos = x_out[index]
        z_pos = y_out[index]

        #Getting drone pose adjusted
        (drone_y, drone_x, drone_z) = self.__ed.rotate_pos(self.__p_end)
        drone_yaw = self.__ed.get_curr_yaw()

        #Adjust constants in in world's coordinates
        y_adjust = 20 + np.abs(self.__p_start[0] - self.__p_end[0])
        t_adjust = -8

        #Setting PID's setpoints in world's coordinates
        self.__pid_pitch.setpoint     = drone_y + y_pos + y_adjust
        self.__pid_roll.setpoint      = drone_x + x_pos
        self.__pid_throttle.setpoint  = drone_z + z_pos + t_adjust
        self.__pid_yaw.setpoint       = drone_yaw - yaw_out

        ### ODOM DATA ###
        min_pos    = ("min", np.min(x_out) + drone_x, np.min(depth_out) + drone_y + y_adjust, np.min(y_out) + drone_z - t_adjust)
        max_pos    = ("max", np.max(x_out) + drone_x, np.max(depth_out) + drone_y + y_adjust, np.max(y_out) + drone_z - t_adjust)
        dlp        = ("dlp", self.__pid_roll.setpoint, self.__pid_pitch.setpoint, self.__pid_throttle.setpoint, self.__pid_yaw.setpoint)
        self.__dlp =  (self.__pid_roll.setpoint, self.__pid_pitch.setpoint, self.__pid_throttle.setpoint, self.__pid_yaw.setpoint)

        if not (self.__odom_file is None):
            self.__odom_start_time = time.time()
            self.__odometry = []
            self.__odometry.append(min_pos)
            self.__odometry.append(max_pos)
            self.__odometry.append(dlp)
            self.__odometry.append((0, drone_x, drone_y, drone_z, drone_yaw))
        #################

        print("-----------------------------------------------")
        print( " STATE 2 END")
        print(f"{min_pos}")
        print(f"{max_pos}")
        print(f"{dlp}")
        print("-----------------------------------------------")
        self.__state = self.__state + 1

    def state_4(self):
        (y, x, z) = self.__ed.get_curr_pos_corrected()

        ### ODOM DATA ###
        if not (self.__odom_file is None):
            curr_time = time.time() - self.__odom_start_time
            self.__odometry.append((curr_time, x, y, z, self.__ed.get_curr_yaw()))
        #################

        #computing errors in x and z
        error_cz = self.__pid_throttle.setpoint - z
        error_cx = self.__pid_roll.setpoint - x

        #computing throttle output
        ctrl_throttle = np.round(self.__pid_throttle(z), 2)
        self.__ed.set_throttle(ctrl_throttle)

        #if height is good enought, adjust the x position
        if (abs(error_cz) < 2) and (abs(ctrl_throttle) < 0.1):
            ctrl_roll = np.round(self.__pid_roll(x), 2)
            self.__ed.set_roll(ctrl_roll)

            #stop condition state 3
            if (abs(error_cz) < 1) and (abs(error_cx) < 1) and (abs(ctrl_roll) < 0.1):
                self.__ed.rc_control()
                self.__pid_throttle.set_auto_mode(enabled=True, last_output=0)
                self.__pid_roll.set_auto_mode(enabled=True, last_output=0)
                print("-----------------------------------------------")
                print( " STATE 3 END")
                print(f"error_x={np.round(error_cx, 1)}")
                print(f"error_z={np.round(error_cz, 1)}")
                print("-----------------------------------------------")
                self.__state = self.__state + 1

        else:
            self.__ed.set_roll(0)
            self.__pid_roll.set_auto_mode(enabled=True, last_output=0)

        ut.draw_text(self.__image_s, f"error_x={np.round(error_cx, 1)}", 0)
        ut.draw_text(self.__image_s, f"error_z={np.round(error_cz, 1)}", 1)

    def state_5(self):
        #ensure drone stoped
        time.sleep(0.5)
        (y, x, z) = self.__ed.get_curr_pos_corrected()

        ### ODOM DATA ###
        if not (self.__odom_file is None):
            curr_time = time.time() - self.__odom_start_time
            self.__odometry.append((curr_time, x, y, z, self.__ed.get_curr_yaw()))
        #################

        #computing controllers output
        ctrl_throttle = np.round(self.__pid_throttle(z), 2)
        ctrl_pitch    = np.round(self.__pid_pitch(y), 2)
        ctrl_roll     = np.round(self.__pid_roll(x), 2)

        self.__ed.rc_control(throttle=ctrl_throttle, pitch=ctrl_pitch, roll=ctrl_roll)

        #ensure initial movement
        time.sleep(0.5)

        self.__state = self.__state + 1
        self.__max_speed_y = self.__ed.get_curr_speed_corrected()[0]

    def state_6(self):
        (y, x, z) = self.__ed.get_curr_pos_corrected()
        yaw = self.__ed.get_curr_yaw()

        ### ODOM DATA ###
        if not (self.__odom_file is None):
            curr_time = time.time() - self.__odom_start_time
            self.__odometry.append((curr_time, x, y, z, yaw))
        #################

        ctrl_throttle = np.round(self.__pid_throttle(z), 2)
        ctrl_pitch    = np.round(self.__pid_pitch(y), 2)
        ctrl_roll     = np.round(self.__pid_roll(x), 2)
        ctrl_yaw      = np.round(self.__pid_yaw(yaw), 2)
            
        self.__ed.rc_control(ctrl_throttle, ctrl_pitch, ctrl_roll, ctrl_yaw)

        speed_y = self.__ed.get_curr_speed_corrected()[0]
        if speed_y > self.__max_speed_y :
            self.__max_speed_y  = speed_y

        pid_error_test = abs(y - self.__pid_pitch.setpoint) < 0.8 * abs(self.__pid_pitch.setpoint)
        #current speed < 1% of the maximum speed and the drone have moved at lest 20% forward
        #this 20% is just to ensure because sometimes the drone moves a little back
        if (speed_y < (0.01 * self.__max_speed_y))  and pid_error_test:
            self.__ed.rc_control(pitch=0.5)
            time.sleep(1)
            self.__ed.set_throttle(-1)
            time.sleep(0.5)
            print("-----------------------------------------------")
            print( " STATE 5 END")
            print("-----------------------------------------------")
            self.__state = self.__state + 1

        ut.draw_text(self.__image_s, f"landing_pos = {np.round(self.__dlp, 1)}", 0)
        ut.draw_text(self.__image_s, f"curr_pos    = {np.round((x, y, z, yaw), 1)}", 1)
        ut.draw_text(self.__image_s, f"max_speed   = {np.round(self.__max_speed_y, 1)}", 2)
        ut.draw_text(self.__image_s, f"curr_speed  = {np.round(speed_y, 1)}", 3)

    def state_7(self):
        print("*************************************************")
        print("*************************************************")
        print("*************** LAND COMPLETE *******************")
        print("*************************************************")
        print("*************************************************")
        self.__ed.land()
        self.__ed.quit()
        time.sleep(5)

        if not (self.__odom_file is None):
            f = self.__odom_file
            
            f.write("====================== HEADER ======================\n")
            f.write("Odometry data will folow format (time, x, y, z, yaw)\n")
            f.write("Landing site: (-1, x, y, z, yaw)\n")
            f.write("X = left and right\n")
            f.write("Y = foraward backward\n")
            f.write("Z = height\n")
            f.write("=====================================================\n")

            for i in range(len(self.__odometry)):
                f.write(f"{self.__odometry[i]}\n")
            f.close()

        self.__state = self.__state + 1