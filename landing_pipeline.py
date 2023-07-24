import numpy as np
import time
import cv2
import utils as ut

from easydrone import EasyDrone
from stereo import Stereo
from vision import Vision
from yolo_detector import YOLODetector
from collections import deque

class LandingPipeline:

    SUCCESS = 0
    RUNNING = 1
    FAIL = -1

    S_YOLO = 0
    S_FEATURES_1 = 1
    S_FEATURES_2 = 2
    S_ESTIMATION = 3
    S_HEIGHT_AND_X = 4
    S_INITIAL_MOVEMENT = 5
    S_NAVIGATION = 6
    S_LANDING_DETECTION = 7
    S_LANDING = 8
    S_SUCCESS = 9
    S_FAIL = 10

    def __init__(self, ed:EasyDrone, s:Stereo, v:Vision, yd:YOLODetector, k_ref, d_ref, cx, cy, odom_file, ty:float, drone_hook_center:float) -> None:
        self.__ed = ed
        self.__s = s
        self.__v = v
        self.__yd = yd
        self.__k_ref_i = k_ref
        self.__d_ref_i = d_ref
        self.__cx = cx
        self.__cy = cy
        self.__odom_file = odom_file
        self.__state = self.S_YOLO
        self.__curr_state_method = self.state_yolo
        self.__ret_status = self.RUNNING
        self.__ty = ty
        self.__drone_hook_center = drone_hook_center

        #NEW APPROACH FOR AVERAGES
        self.__prev_error_cx = deque(maxlen=10)
        self.__prev_error_cy = deque(maxlen=10)
    
    def reset(self):
        """
        Reset to state 0
        """
        self.__state = self.S_YOLO
        self.__curr_state_method = self.state_yolo
        self.__ret_status = self.RUNNING

    def get_p_start(self):
        return self.__p_start
    
    def state_yolo(self):
        """
        Uses YOLO to detect possible branches combined with picture-based PIDs 
        to centralize YOLO's rectangle in the midle of the screen

        When the rectangle is centralized, it captures the features inside it
        """

        #better here than in the reset, 
        #treats the case that the user selects the rectangle
        if not ((self.__k_ref_i is None) and (self.__d_ref_i is None)): 
            self.__k_ref = self.__k_ref_i
            self.__d_ref = self.__d_ref_i
            self.__state = self.S_FEATURES_1
            self.__curr_state_method = self.state_features_1
            return

        pt1, pt2, conf = self.__yd.detect_best(self.__image, confidence=0.4)
        if not ((pt1 is None) and (pt2 is None)):
            #computing error
            error_cx = (pt1[0] + pt2[0]) * 0.5 - self.__cx
            error_cy = (pt1[1] + pt2[1]) * 0.5 - self.__cy

            #computing control output based on features matched
            ctrl_s_yaw = np.round(self.__ed.pid_s_yaw(error_cx), 2)
            ctrl_s_throttle = np.round(self.__ed.pid_s_throttle(error_cy), 2)
            self.__ed.set_yaw(ctrl_s_yaw)
            self.__ed.set_throttle(ctrl_s_throttle)

            #error in y and x < 10 pixels and control output for yaw and throttle < 0.1
            if (abs(error_cy) < 10) and (abs(error_cx) < 10) and (abs(ctrl_s_yaw) < 0.1) and (abs(ctrl_s_throttle) < 0.1):
                #stop the drone
                self.__ed.rc_control()
                self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                self.__ed.pid_s_throttle.set_auto_mode(enabled=True, last_output=0)

                #save keypoints and descriptors
                k_ref, d_ref = self.__v.detect_features_in_rect(self.__image, [pt1, pt2])

                self.__k_ref = k_ref
                self.__d_ref = d_ref

                print("-----------------------------------------------")
                print(f" STATE {self.__state} END")
                print("-----------------------------------------------")

                self.__state = self.S_FEATURES_1
                self.__curr_state_method = self.state_features_1

            #drawing the point in image that must be in the center
            ut.draw_dot(self.__image_s, (int(self.__cx + error_cx), int(self.__cy + error_cy)))
            ut.draw_yolo_rectangle(self.__image_s, pt1, pt2, conf)

        else:
            self.__ed.set_yaw(0)
            self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                
            self.__ed.set_throttle(0)
            self.__ed.pid_s_throttle.set_auto_mode(enabled=True, last_output=0)   


    def state_features_1(self):
        """
        Centralizes the picture with the features using the features detected before


        Saves them and the current position when the features are centralized
        """

        #detect features in the current image
        k, d = self.__v.detect_features(self.__image)

        #get list of matched features
        k_curr, d_curr, error, _ = self.__v.bf_matching_descriptors(self.__d_ref, k, d, 0.65, (self.__cx, self.__cy))
        cv2.drawKeypoints(self.__image_s, k_curr, self.__image_s, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

        #at least 2 feature matched
        if len(error) > 1:
            #old computing mean error
            #mean_error = np.mean(error, axis=0)
            #error_cx = mean_error[0]
            #error_cy = mean_error[1]

            #computing mean error (window of 5 measurements)
            mean_error = np.mean(error, axis=0)
            self.__prev_error_cx.append(mean_error[0])
            self.__prev_error_cy.append(mean_error[1])
            error_cx = np.mean(self.__prev_error_cx)
            error_cy = np.mean(self.__prev_error_cy)

            #computing control output based on features matched
            ctrl_s_yaw = np.round(self.__ed.pid_s_yaw(error_cx), 2)
            ctrl_s_throttle = np.round(self.__ed.pid_s_throttle(error_cy), 2)
            self.__ed.set_yaw(ctrl_s_yaw)
            self.__ed.set_throttle(ctrl_s_throttle)

            #drawing the point in image that must be in the center
            ut.draw_dot(self.__image_s, (int(self.__cx + error_cx), int(self.__cy + error_cy)))

            #error in y and x < 10 pixels and control output for yaw and throttle < 0.1
            if (abs(error_cy) < 10) and (abs(error_cx) < 10) and (abs(ctrl_s_yaw) < 0.1) and (abs(ctrl_s_throttle) < 0.1):
                #stop the drone
                self.__ed.rc_control()
                self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                self.__ed.pid_s_throttle.set_auto_mode(enabled=True, last_output=0)

                #save keypoints and descriptors
                self.__k_start = k_curr
                self.__d_start = d_curr

                #NEW APPROACH FOR AVERAGES
                self.__prev_error_cx = deque(maxlen=5)

                #save current pose
                self.__p_start = self.__ed.get_curr_pos()

                #setthe pid throttle to the current height + 15cm
                self.__ed.pid_throttle.setpoint = self.__p_start[2] + self.__ty

                print("-----------------------------------------------")
                print(f" STATE {self.__state} END")
                print(f" Start Position: {self.__p_start} ")
                print("-----------------------------------------------")
                self.__state = self.S_FEATURES_2
                self.__curr_state_method = self.state_features_2  

        else:
            self.__ed.set_yaw(0)
            self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                
            self.__ed.set_throttle(0)
            self.__ed.pid_s_throttle.set_auto_mode(enabled=True, last_output=0)     
            
    #state 2: center cx and move up
    def state_features_2(self):
        """
        Moves the drone TY cm up while keeping the features centralized

        Saves them and the current position when the features are centralized and TY adjusted
        """

        error_cz = self.__ed.pid_throttle.setpoint - self.__ed.get_curr_pos()[2]
        #computing control output based on height
        ctrl_throttle = np.round(self.__ed.pid_throttle(self.__ed.get_curr_pos()[2]), 2)
        self.__ed.set_throttle(ctrl_throttle)

        if abs(error_cz) < 2 and abs(ctrl_throttle) < 0.1:
            k, d = self.__v.detect_features(self.__image)
            
            #get list of matched features
            k_curr, d_curr, error, matches = self.__v.bf_matching_descriptors(self.__d_start, k, d, 0.65, (self.__cx, self.__cy))
            cv2.drawKeypoints(self.__image_s, k_curr, self.__image_s, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

            if len(error) > 1:
                #computing mean error
                mean_error = np.mean(error, axis=0)
                self.__prev_error_cx.append(mean_error[0])
                error_cx = np.mean(self.__prev_error_cx)

                #computing control output based on features matched
                ctrl_s_yaw = np.round(self.__ed.pid_s_yaw(error_cx), 2)
                self.__ed.set_yaw(ctrl_s_yaw)

                #drawing the point in image that must be in the center
                ut.draw_dot(self.__image_s, (int(self.__cx + error_cx), int(self.__cy)))

                #stop criteria of esate 1
                if (abs(error_cx) < 10) and (abs(ctrl_throttle) < 0.05) and (abs(ctrl_s_yaw) < 0.1):
                    #stop the drone
                    self.__ed.rc_control()
                    self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
                    self.__ed.pid_throttle.set_auto_mode(enabled=True, last_output=0)

                    #save keypoints, descriptors and good matches
                    self.__k_end = k
                    self.__d_end = d
                    self.__good_matches = matches

                    #save current pose
                    self.__p_end = self.__ed.get_curr_pos()
                    self.__ed.save_quaternion()
                        
                    print("-----------------------------------------------")
                    print(f" STATE {self.__state} END")
                    print(f" End Position: {self.__p_end} ")
                    print("-----------------------------------------------")
                    self.__state = self.S_ESTIMATION
                    self.__curr_state_method = self.state_estimation

            else: # YES, it is necessary, do not remove!
                self.__ed.set_yaw(0)
                self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)
            
        else: # YES, it is also necessary, do not remove!
            self.__ed.set_yaw(0)
            self.__ed.pid_s_yaw.set_auto_mode(enabled=True, last_output=0)

    def state_estimation(self):
        """
        Compute the landing site position
        """
        if len(self.__good_matches) < 3:
            print("*************************************************")
            print("*************************************************")
            print("********* INSUFFICIENT NUMBER OF POINTS *********")
            print("*************************************************")
            print("*************************************************")
            self.__state = self.S_FAIL
            self.__curr_state_method = self.state_fail
            return
        
        #compute relative position
        ty = self.__p_end[2] - self.__p_start[2]
        # x, y in the image and depth 
        x_out, y_out, depth_out, yaw_out, roll_out  = self.__s.compute_relative_depth(ty, self.__k_start, self.__k_end, self.__good_matches)

        if (len(x_out) == 0) and (len(y_out) == 0) and (len(depth_out) == 0):
            print("*************************************************")
            print("*************************************************")
            print("*********** UNABLE TO FILTER VARIANCE  **********")
            print("*************************************************")
            print("*************************************************")
            self.__state = self.S_FAIL
            self.__curr_state_method = self.state_fail
            return

        if abs(roll_out) > 15: #cant land
            print("*************************************************")
            print("*************************************************")
            print("************** ROLL ANGLE TOO BIG ***************")
            print("*************************************************")
            print("*************************************************")
            self.__state = self.S_FAIL
            self.__curr_state_method = self.state_fail
            return
        
        """ V1
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
        """

        #choose as landing site the detect feature neares the mean
        # 1 - Compute mean
        depth_mean = np.mean(depth_out)
        x_mean = np.mean(x_out)
        # 2 - Calculate the difference array
        difference_array = np.absolute(x_out-x_mean)
        # 3 - Find the index of minimum element from the array
        index = difference_array.argmin()
        # 4 - Get the nearest position of the mean in world's coordinates
        y_pos = depth_mean
        x_pos = x_mean
        z_pos = y_out[index]

        #Getting drone pose adjusted
        (drone_y, drone_x, drone_z) = self.__ed.rotate_pos(self.__p_end)
        drone_yaw = self.__ed.get_curr_yaw()

        #Adjust constants in in world's coordinates
        y_adjust = 25 + np.abs(self.__p_start[0] - self.__p_end[0]) #originally 20, 25 for the new hook

        #Setting PID's setpoints in world's coordinates
        self.__ed.pid_pitch.setpoint     = drone_y + y_pos + y_adjust
        self.__ed.pid_roll.setpoint      = drone_x + x_pos
        self.__ed.pid_throttle.setpoint  = drone_z + z_pos + self.__drone_hook_center
        self.__ed.pid_yaw.setpoint       = drone_yaw - yaw_out

        ### ODOM DATA ###
        min_pos    = ("min", np.min(x_out) + drone_x, np.min(depth_out) + drone_y + y_adjust, np.min(y_out) + drone_z - self.__drone_hook_center)
        max_pos    = ("max", np.max(x_out) + drone_x, np.max(depth_out) + drone_y + y_adjust, np.max(y_out) + drone_z - self.__drone_hook_center)
        dlp        = ("dlp", self.__ed.pid_roll.setpoint, self.__ed.pid_pitch.setpoint, self.__ed.pid_throttle.setpoint, self.__ed.pid_yaw.setpoint)
        self.__dlp =  (self.__ed.pid_roll.setpoint, self.__ed.pid_pitch.setpoint, self.__ed.pid_throttle.setpoint, self.__ed.pid_yaw.setpoint)

        if not (self.__odom_file is None):
            self.__odom_start_time = time.time()
            self.__odometry = []
            self.__odometry.append(min_pos)
            self.__odometry.append(max_pos)
            self.__odometry.append(dlp)
            self.__odometry.append(y_adjust)
            self.__odometry.append((0, drone_x, drone_y, drone_z, drone_yaw))
        #################

        print("-----------------------------------------------")
        print(f" STATE {self.__state} END")
        print(f"{min_pos}")
        print(f"{max_pos}")
        print(f"{dlp}")
        print("-----------------------------------------------")
        self.__state = self.S_HEIGHT_AND_X
        self.__curr_state_method = self.state_height_and_x

    def state_height_and_x(self):
        """
        Adjust height and x position
        """
        (y, x, z) = self.__ed.get_curr_pos_corrected()

        ### ODOM DATA ###
        if not (self.__odom_file is None):
            curr_time = time.time() - self.__odom_start_time
            self.__odometry.append((curr_time, x, y, z, self.__ed.get_curr_yaw()))
        #################

        #computing errors in x and z
        error_cz = self.__ed.pid_throttle.setpoint - z
        error_cx = self.__ed.pid_roll.setpoint - x

        #computing throttle output
        ctrl_throttle = np.round(self.__ed.pid_throttle(z), 2)
        self.__ed.set_throttle(ctrl_throttle)

        #if height is good enought, adjust the x position
        if (abs(error_cz) < 3) and (abs(ctrl_throttle) < 0.15):
            ctrl_roll = np.round(self.__ed.pid_roll(x), 2)
            self.__ed.set_roll(ctrl_roll)

            #stop condition state 3
            if (abs(error_cz) < 1) and (abs(error_cx) < 1) and (abs(ctrl_roll) < 0.1):
                self.__ed.rc_control()
                self.__ed.pid_throttle.set_auto_mode(enabled=True, last_output=0)
                self.__ed.pid_roll.set_auto_mode(enabled=True, last_output=0)
                print("-----------------------------------------------")
                print(f" STATE {self.__state} END")
                print(f"error_x={np.round(error_cx, 1)}")
                print(f"error_z={np.round(error_cz, 1)}")
                print("-----------------------------------------------")
                self.__state = self.S_INITIAL_MOVEMENT
                self.__curr_state_method = self.state_initial_movement

        else:
            self.__ed.set_roll(0)
            self.__ed.pid_roll.set_auto_mode(enabled=True, last_output=0)

        ut.draw_text(self.__image_s, f"error_x={np.round(error_cx, 1)}", 0)
        ut.draw_text(self.__image_s, f"error_z={np.round(error_cz, 1)}", 1)

    def state_initial_movement(self):
        """
        Initial movement towards the landing site to get the first value of __max_speed_y
        """

        #ensure drone stoped
        time.sleep(0.5)
        (y, x, z) = self.__ed.get_curr_pos_corrected()

        ### ODOM DATA ###
        if not (self.__odom_file is None):
            curr_time = time.time() - self.__odom_start_time
            self.__odometry.append((curr_time, x, y, z, self.__ed.get_curr_yaw()))
        #################

        #computing controllers output
        ctrl_throttle = np.round(self.__ed.pid_throttle(z), 2)
        ctrl_pitch    = np.round(self.__ed.pid_pitch(y), 2)
        ctrl_roll     = np.round(self.__ed.pid_roll(x), 2)

        self.__ed.rc_control(throttle=ctrl_throttle, pitch=ctrl_pitch, roll=ctrl_roll)

        #ensure initial movement
        time.sleep(0.5)

        print("-----------------------------------------------")
        print(f" STATE {self.__state} END")
        print("-----------------------------------------------")

        self.__state = self.S_NAVIGATION
        self.__curr_state_method = self.state_navigation
        self.__max_speed_y = self.__ed.get_curr_velocity_corrected()[0]

    def state_navigation(self):
        """
        Navigation towards the landing site 
        """

        (y, x, z) = self.__ed.get_curr_pos_corrected()
        yaw = self.__ed.get_curr_yaw()

        ### ODOM DATA ###
        if not (self.__odom_file is None):
            curr_time = time.time() - self.__odom_start_time
            self.__odometry.append((curr_time, x, y, z, yaw))
        #################

        ctrl_throttle = np.round(self.__ed.pid_throttle(z), 2)
        ctrl_pitch    = np.round(self.__ed.pid_pitch(y), 2)
        ctrl_roll     = np.round(self.__ed.pid_roll(x), 2)
        ctrl_yaw      = np.round(self.__ed.pid_yaw(yaw), 2)
            
        self.__ed.rc_control(ctrl_throttle, ctrl_pitch, ctrl_roll, ctrl_yaw)

        speed_y = self.__ed.get_curr_velocity_corrected()[0]
        if speed_y > self.__max_speed_y :
            self.__max_speed_y  = speed_y

        #current error in Y posiion
        pos_error = (self.__ed.pid_pitch.setpoint - y) ** 2
        pos_error = pos_error + (self.__ed.pid_roll.setpoint - x) ** 2
        pos_error = (pos_error + (self.__ed.pid_throttle.setpoint - z) ** 2) ** 0.5

        #Reached a position around 3 cm the destination and speed below 0.1 max speed. 
        #Didn't hit the branch, so return fail       
        #(3*3 + 3*3 + 3*3)^0.5 = 5.2
        print(f"POS ERROR= {pos_error}", flush=True)
        if (pos_error < 5):
            print("*************************************************")
            print("*************************************************")
            print("******* LAND FAIL - DIDN'T HIT THE BRANCH *******")
            print(f"************ POSITION ERROR: {pos_error:.2f} *************")
            print("*************************************************")
            print("*************************************************")
            self.__state = self.S_FAIL
            self.__curr_state_method = self.state_fail
            return
        

        # abs(setpoint - curr) < 0.8 (setpoint - start)
        pid_error_test1 = abs(self.__ed.pid_pitch.setpoint - y) < 0.8 * abs(self.__ed.pid_pitch.setpoint - self.__p_end[0])
        # the drone didn't reached a position after the landing site
        pid_error_test2 = self.__ed.pid_pitch.setpoint - y > 5
        # the drone has the proper height
        pid_error_test3 = abs(self.__ed.pid_throttle.setpoint - z) < 3
        pid_error_test = pid_error_test1 and pid_error_test2 and pid_error_test3
        print(f"PID TESTS= {pid_error_test1} and {(self.__ed.pid_pitch.setpoint - y):.2f} > 5 and {abs(self.__ed.pid_throttle.setpoint - z):.2f} < 3", flush=True)
        #current speed < 10% of the maximum speed and the drone have moved at lest 20% forward
        #this 10% is just to ensure because sometimes the drone moves a little back
        if (speed_y < (0.01 * self.__max_speed_y))  and pid_error_test:

            self.__minimum_height= self.__ed.get_curr_pos_corrected()[2] + 4 * self.__drone_hook_center

            self.__ed.rc_control(pitch=0.5)
            time.sleep(0.5)
            if self.__ed.get_curr_pos_corrected()[2] < self.__ed.pid_throttle.setpoint:
                self.__ed.set_throttle(0.2)
            else:
                self.__ed.set_throttle(-0.5)
            time.sleep(0.5)
            
            self.__impact_time = time.time()
            
            print("-----------------------------------------------")
            print(f" STATE {self.__state} END")
            print("-----------------------------------------------")

            self.__state = self.S_LANDING_DETECTION
            self.__curr_state_method = self.state_landing_detection

        ut.draw_text(self.__image_s, f"landing_pos = {self.__dlp[0]:.1f} {self.__dlp[1]:.1f} {self.__dlp[2]:.1f} {self.__dlp[3]:.1f}", 0)
        ut.draw_text(self.__image_s, f"curr_pos    = {x:.1f} {y:.1f} {z:.1f} {yaw:.1f}", 1)
        ut.draw_text(self.__image_s, f"max_speed   = {np.round(self.__max_speed_y, 1)}", 2)
        ut.draw_text(self.__image_s, f"curr_speed  = {np.round(speed_y, 1)}", 3)
    
    def state_landing_detection(self):
        """
        Detection of the landing
        """
        self.__ed.rc_control(throttle=-0.8)

        curr_height = self.__ed.get_curr_pos_corrected()[2]
        dt = time.time() - self.__impact_time

        ut.draw_text(self.__image_s, f"      Height = {np.round(curr_height, 1)}", 0)
        ut.draw_text(self.__image_s, f"Height Limit = {np.round(self.__minimum_height, 1)}", 1)
        ut.draw_text(self.__image_s, f"  Time Count = {np.round(dt, 1)}", 2)
        
        print("-------------------------------------------------------")
        print(f"      Height = {np.round(curr_height, 1)}")
        print(f"Height Limit = {np.round(self.__minimum_height, 1)}")
        print(f"  Time Count = {np.round(dt, 1)}", flush=True)
        print("-------------------------------------------------------")

        if curr_height <= self.__minimum_height:
            print("*************************************************")
            print("*************************************************")
            print("******* LAND FAIL - DIDN'T HIT THE BRANCH *******")
            print(f"**************** HEIGHT ERROR ******************")
            print("*************************************************")
            print("*************************************************")
            self.__state = self.S_FAIL
            self.__curr_state_method = self.state_fail
            return
        
        if dt >= 5.0:
            self.__ed.rc_control()
            print("-----------------------------------------------")
            print(f" STATE {self.__state} END")
            print("-----------------------------------------------")
            self.__state = self.S_LANDING
            self.__curr_state_method = self.state_landing


    def state_landing(self):
        """
        Lands the drone after hitting the landing site
        
        It also saves the log into a file 
        (bad implementation, I should save the log from time to time)
        """
        print("*************************************************")
        print("*************************************************")
        print("*************** LAND COMPLETE *******************")
        print("*************************************************")
        print("*************************************************")
        self.__ed.land()
        time.sleep(5)

        print("-----------------------------------------------")
        print(f" STATE {self.__state} END")
        print("-----------------------------------------------")

        self.__state = self.S_SUCCESS
        self.__curr_state_method = self.state_success

    def write_odom(self):
        import io
        if isinstance(self.__odom_file, io.TextIOWrapper):
            f = self.__odom_file
            
            f.write("====================== HEADER ======================\n")
            f.write("Odometry data will folow format (time, x, y, z, yaw)\n")
            f.write("Landing site: (-1, x, y, z, yaw)\n")
            f.write("X = left and right\n")
            f.write("Y = foraward backward\n")
            f.write("Z = height\n")
            if self.__state == self.S_SUCCESS:
                f.write("====================== SUCCESS ======================\n")
            else:
                f.write("======================== FAIL =======================\n")

            for i in range(len(self.__odometry)):
                f.write(f"{self.__odometry[i]}\n")

            f.close()

    def state_success(self):
        """
        Informs a successful landing
        """
        self.write_odom()
        self.__ret_status = self.SUCCESS

    def state_fail(self):
        """
        Informs a failed landing
        """
        print("*************************************************")
        print("*************************************************")
        print("***************** LAND FAIL *********************")
        print("*************************************************")
        print("*************************************************")
        self.write_odom()
        self.__ret_status = self.FAIL

    def run(self, image, image_s):

        #Image used in data processing
        self.__image = image
            
        #Image used only to show
        self.__image_s = image_s
            
        #loop state ensure that when the state is updated the loop will restart before run the next state
        self.__curr_state_method()
        
        ut.draw_text(self.__image_s, f"Autonomous Landing - State {self.__state} of 9", -2)
        height, width, _ = self.__image_s.shape
        ut.draw_line(self.__image_s, (self.__cx, 0), (self.__cx, height))
        ut.draw_line(self.__image_s, (0, self.__cy), (width, self.__cy))

        return self.__ret_status