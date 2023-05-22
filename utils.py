import cv2
import numpy as np
import time

from simple_pid import PID
from tellopy import Tello
from easydrone import EasyDrone
from vision import Vision
from stereo import Stereo

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

def draw_line(image, start_point, end_point):
    color = (0, 255, 0)
    thickness = 1
    cv2.line(image, start_point, end_point, color, thickness)

def draw_dot(image, point):
    cv2.circle(image, point, radius=5, color=(0, 0, 255), thickness=-1)

def draw_polylines(image, points):
    color = (255, 0, 0)
    thickness = 3
    cv2.polylines(image, points, True, color, thickness)

def draw_rectangle(image, start_point, end_point):
    color = (255, 0, 0)
    thickness = 2
    cv2.rectangle(image, start_point, end_point, color, thickness)

def stereo_landing_pipeline(cx, cy, kp_ref, dsc_ref, td:Tello, ed:EasyDrone, v:Vision, s:Stereo, out_file):

    if kp_ref is None:
        return
    
    #pid_screen_yaw = PID(Kp=-1/960, Ki=-0, Kd=-0.25/960)
    pid_screen_yaw = PID(Kp=-1/960, Ki=-0.2/960, Kd=-0.1/960)
    pid_screen_yaw.output_limits = (-0.3, 0.3) 
    pid_screen_yaw.setpoint = 0
    pid_screen_yaw.sample_time = None
    pid_screen_yaw.set_auto_mode(True, last_output=0)

    pid_screen_height = PID(Kp=1/240, Ki=0.25/240, Kd=0.3/240) #
    pid_screen_height.output_limits = (-0.3, 0.3) 
    pid_screen_height.setpoint = 0
    pid_screen_height.sample_time = None
    pid_screen_height.set_auto_mode(True, last_output=0)

    pid_height = PID(Kp=1/30, Ki=1/40, Kd=1/60) #test sift with tihs params
    #pid_height = PID(Kp=1/30, Ki=0, Kd=0.05/25)
    pid_height.output_limits = (-0.3, 0.3) 
    pid_height.setpoint = 0
    pid_height.sample_time = None
    pid_height.set_auto_mode(True, last_output=0)

    pid_forward = PID(Kp=1/30, Ki=0, Kd=0.25/25) #test sift with tihs params
    #pid_height = PID(Kp=1/30, Ki=0, Kd=0.05/25)
    pid_forward.output_limits = (-0.2, 0.2) 
    pid_forward.setpoint = 0
    pid_forward.sample_time = None
    pid_forward.set_auto_mode(True, last_output=0)

    pid_roll = PID(Kp=1/100, Ki=1/800, Kd=1/1100) #test sift with tihs params
    #pid_height = PID(Kp=1/30, Ki=0, Kd=0.05/25)
    pid_roll.output_limits = (-0.3, 0.3) 
    pid_roll.setpoint = 0
    pid_roll.sample_time = None
    pid_roll.set_auto_mode(True, last_output=0)

    pid_yaw = PID(Kp=1/15, Ki=0, Kd=0.25/20) #test sift with tihs params
    #pid_height = PID(Kp=1/30, Ki=0, Kd=0.05/25)
    pid_yaw.output_limits = (-0.5, 0.5) 
    pid_yaw.setpoint = 0
    pid_yaw.sample_time = None
    pid_yaw.set_auto_mode(True, last_output=0)

    state = 0

    error_cx = float('inf')
    error_cy = float('inf')
    error_height = float('inf')
    max_speed_x = 0

    kp_start = None
    dsc_start = None
    kp_end = None
    dsc_end = None
    start_pos = None
    end_pos = None
    good_matches = None

    depth = None

    fps = 0
    time_start = 0
    landing_pos = None

    ###########################################
    ######### Odometry data variables #########
    ###########################################
    odometry = []
    odom_start_time = 0
    extra_print  = False
    # odometry data will folow format (time, x, y, z, yaw)
    # Landing site: (-1, x, y, z, yaw)
    # X = left and right
    # Y = foraward backward
    # Z = height
    ###########################################
    ###########################################

    while True:
        image = ed.get_curr_frame()
        if image is None:
            frame = np.zeros((10, 10, 3), dtype = np.uint8)
            time.sleep(0.1)
            cv2.imshow('Camera', frame)
            key = cv2.waitKey(1) & 0xFF
            continue

        image = s.rotateImage(image)
        image_s = image.copy()

        #state 0: the drone has to center a region of interest with cx and cy
        if state == 0:
            k2, d2 = v.detect_features(image)
            #image_s = cv2.drawKeypoints(image_s, k2, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

            k_curr, d_curr, error, _ = v.bf_matching_descriptors(dsc_ref, k2, d2, 0.65, (cx, cy))
            image_s = cv2.drawKeypoints(image_s, k_curr, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            if len(error) > 1:
                mean_error = np.mean(error, axis=0)
                error_cx = mean_error[0]
                error_cy = mean_error[1]

                ctrl_screen_yaw = np.round(pid_screen_yaw(error_cx), 2)
                ctrl_screen_height = np.round(pid_screen_height(error_cy), 2)

                draw_dot(image_s, (int(cx + error_cx), int(cy + error_cy)))

                #error ok, so stop and save keypoints and descriptors
                if (abs(error_cy) < 10) and (abs(error_cx) < 10) and abs(ctrl_screen_yaw) < 0.1 and abs(ctrl_screen_height) < 0.1:
                    #stop the drone
                    td.set_yaw(0)
                    td.set_throttle(0)
                    pid_screen_yaw.set_auto_mode(enabled=True, last_output=0)
                    pid_screen_height.set_auto_mode(enabled=True, last_output=0)

                    #save keypoints and descriptors
                    kp_start = k_curr
                    dsc_start = d_curr

                    #save current pose
                    start_pos = ed.get_curr_pos()
                    pid_height.setpoint = start_pos[2] + 15

                    print("********************************")
                    print(f"* {start_pos} *")
                    print("********************************")
                    state = 1
                    #break #remove after put another state
                    continue

                td.set_yaw(ctrl_screen_yaw)
                td.set_throttle(ctrl_screen_height)
            else:
                td.set_yaw(0)
                pid_screen_yaw.set_auto_mode(enabled=True, last_output=0)
                
                td.set_throttle(0)
                pid_screen_height.set_auto_mode(enabled=True, last_output=0)

        #state 1: center cx and move up
        if state == 1:

            error_cy = pid_height.setpoint - ed.get_curr_pos()[2] 
            ctrl_height = np.round(pid_height(ed.get_curr_pos()[2]), 2)
            td.set_throttle(ctrl_height)

            if abs(error_cy) < 2 and abs(ctrl_height) < 0.1:
                k2, d2 = v.detect_features(image)
                #image_s = cv2.drawKeypoints(image_s, k2, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

                k_curr, _, error, matches = v.bf_matching_descriptors(dsc_start, k2, d2, 0.65, (cx, cy))
                image_s = cv2.drawKeypoints(image_s, k_curr, 0, (255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
                if len(error) > 1:
                    mean_error = np.mean(error, axis=0)
                    error_cx = mean_error[0]
                    ctrl_screen_yaw = np.round(pid_screen_yaw(error_cx), 2)

                    draw_dot(image_s, (int(cx + error_cx), int(cy + error_cy)))

                    #error ok, so stop and save keypoints and descriptors
                    if (abs(error_cy) < 2) and (abs(error_cx) < 10) and abs(ctrl_height) < 0.05 and abs(ctrl_screen_yaw) < 0.1:
                        #stop the drone
                        td.set_yaw(0)
                        td.set_throttle(0)
                        pid_screen_yaw.set_auto_mode(enabled=True, last_output=0)
                        pid_height.set_auto_mode(enabled=True, last_output=0)

                        #save keypoints and descriptors
                        kp_end = k2
                        dsc_end = d2
                        good_matches = matches

                        #save current pose
                        end_pos = ed.get_curr_pos()
                        ed.save_quaternion()
                        
                        print("********************************")
                        print(f"* {end_pos} *")
                        print("********************************")
                        state = 2
                        #break #remove after put another state
                        continue

                    td.set_yaw(ctrl_screen_yaw)

                else: # YES, it is necessary, do not remove!
                    td.set_yaw(0)
                    pid_screen_yaw.set_auto_mode(enabled=True, last_output=0)
            
            else: # YES, it is also necessary, do not remove!
                td.set_yaw(0)
                pid_screen_yaw.set_auto_mode(enabled=True, last_output=0)
        
        #state 2: compute landing site position
        if state == 2:
            #compute relative position
            ty = end_pos[2] - start_pos[2]
            x_out, y_out, depth_out, yaw_out, roll_out  = s.compute_relative_depth(ty, kp_start, kp_end, good_matches)

            if abs(roll_out) > 15: #cant land
                print("*************************************************")
                print("*************************************************")
                print("************** ROLL ANGLE TOO BIG ***************")
                print("*************************************************")
                print("*************************************************")
                state = 7
                break
            else:
                print("*************************************************")
                print(f"*********** ROLL ANGLE: {roll_out:.2f} deg **************")
                print("*************************************************")
            
            #"""
            depth_mean = np.mean(depth_out)

            # calculate the difference array
            difference_array = np.absolute(depth_out-depth_mean)
            # find the index of minimum element from the array
            index = difference_array.argmin()
            #get the neart position of the mean
            x_mean = x_out[index]
            y_mean = y_out[index]
            #"""
            
            """
            x_min = np.min(x_out)
            x_max = np.max(x_out)
            # calculate the difference array 
            difference_array = np.absolute(x_out-((x_max - x_min) * 0.5))
            # find the index of minimum element from the array
            index = difference_array.argmin()
            #get the neart position of the mean
            x_mean = x_out[index]
            y_mean = y_out[index]
            depth_mean = depth_out[index]
            """

            mean_pos = (x_mean, y_mean, depth_mean, yaw_out)

            new_pos_drone = ed.rotate_pos(end_pos)

            yaw_odom    = ed.get_curr_yaw() ### ODOM DATA ###

            depth_adjust = 20 + np.abs(start_pos[0] - end_pos[0])
            depth  = new_pos_drone[0] + mean_pos[2] + depth_adjust
            x      = new_pos_drone[1] + mean_pos[0]
            height = new_pos_drone[2] + mean_pos[1] - 8
            yaw    = ed.get_curr_yaw() - mean_pos[3]

            landing_pos = (x, height, depth, yaw)
            
            pid_forward.setpoint = depth
            pid_height.setpoint  = height
            pid_roll.setpoint    = x
            pid_yaw.setpoint     = yaw

            ### ODOM DATA ###
            odom_start_time = time.time()
            odometry.append(("min", np.min(x_out) + new_pos_drone[1], np.min(depth_out) + new_pos_drone[0] + depth_adjust, np.min(y_out) + new_pos_drone[2] - 8))
            odometry.append(("max", np.max(x_out) + new_pos_drone[1], np.max(depth_out) + new_pos_drone[0] + depth_adjust, np.max(y_out) + new_pos_drone[2] - 8))
            odometry.append(("dlp", x, depth, height, yaw))
            odometry.append((0, new_pos_drone[1], new_pos_drone[0], new_pos_drone[2], yaw_odom))
            #################

            state = 3
            print("********************************")
            print(f"*STATE {state} ==> mean_pos: {mean_pos}*")
            print("********************************")

        #state 3: adjust height and x positions before start going forward
        if state == 3:
            (depth, x, height) = ed.get_curr_pos_corrected()

            ### ODOM DATA ###
            curr_time = time.time() - odom_start_time
            odometry.append((curr_time, x, depth, height, ed.get_curr_yaw()))
            #################

            ctrl_height = np.round(pid_height(height), 2)
            td.set_throttle(ctrl_height)

            ctrl_roll = np.round(pid_roll(x), 2)
            td.set_roll(ctrl_roll)

            error_cy = pid_height.setpoint - height
            error_cx = pid_roll.setpoint - x
            draw_text(image_s, f"error_x={np.round(error_cx, 1)}", 0)
            draw_text(image_s, f"error_y={np.round(error_cy, 1)}", 1)
            if abs(error_cy) < 2 and abs(ctrl_height) < 0.1 and abs(error_cx) < 1 and abs(ctrl_roll) < 0.1:
                td.set_throttle(0)
                pid_height.set_auto_mode(enabled=True, last_output=0)
                td.set_roll(0)
                pid_roll.set_auto_mode(enabled=True, last_output=0)
                state = 4

        #state 4: initial movment to ensure that max_speed_x > 0
        if state == 4:
            time.sleep(0.5)
            (depth, x, height) = ed.get_curr_pos_corrected()

            ### ODOM DATA ###
            curr_time = time.time() - odom_start_time
            odometry.append((curr_time, x, depth, height, ed.get_curr_yaw()))
            #################

            ctrl_height = np.round(pid_height(height), 2)
            td.set_throttle(ctrl_height)

            ctrl_pitch = np.round(pid_forward(depth), 2)
            td.set_pitch(ctrl_pitch)

            ctrl_roll = np.round(pid_roll(x), 2)
            td.set_roll(ctrl_roll)

            time.sleep(0.5)
            state = 5
            max_speed_x = ed.get_curr_speed_corrected()[0]

        #state 5: all controller to make the drone achieve the landing site
        if state == 5:
            (depth, x, height) = ed.get_curr_pos_corrected()
            curr_yaw = ed.get_curr_yaw()

            ### ODOM DATA ###
            if extra_print:
                curr_time = time.time() - odom_start_time
                odometry.append((curr_time, x, depth, height, curr_yaw, "landed"))
            else:
                curr_time = time.time() - odom_start_time
                odometry.append((curr_time, x, depth, height, curr_yaw, "fly"))  
            #################

            ctrl_height = np.round(pid_height(height), 2)
            td.set_throttle(ctrl_height)

            ctrl_pitch = np.round(pid_forward(depth), 2)
            td.set_pitch(ctrl_pitch)

            ctrl_roll = np.round(pid_roll(x), 2)
            td.set_roll(ctrl_roll)

            ctrl_yaw = np.round(pid_yaw(curr_yaw), 2)
            td.set_yaw(ctrl_yaw)

            speed_x = ed.get_curr_speed_corrected()[0]
            if speed_x > max_speed_x:
                max_speed_x = speed_x

            pid_error_test = abs(depth - pid_forward.setpoint) < 0.8 * abs(pid_forward.setpoint)
            #current spedd < 1% of the maximum speed and the drone have moved at lest 20% forward
            #this 20% is just to ensure because sometimes the drone moves a little back
            if (ed.get_curr_speed_corrected()[0] < 0.01 * max_speed_x and pid_error_test):
                state = 6
                td.set_pitch(0.5)
                time.sleep(1)
                td.set_throttle(-0.5)
                time.sleep(0.5)
                break

            #draw_text(image_s, f"ctrl_height={ctrl_height}", 0)
            #draw_text(image_s, f"ctrl_pitch={ctrl_pitch}", 1)
            #draw_text(image_s, f"ctrl_roll={ctrl_roll}", 2)
            #draw_text(image_s, f"ctrl_yaw={ctrl_yaw:.2f}", 3)
            #draw_text(image_s, f"vel_x={speed_x}", 4)
            draw_text(image_s, f"landing_pos={np.round(landing_pos, 1)}", 0)
            draw_text(image_s, f"curr_pos={np.round((x, height, depth, yaw), 1)}", 1)



        alpha = 0.1
        if time.time()-time_start > 0:
            fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
            time_start = time.time()

        draw_text(image_s, f"FPS={fps}", -1)     
        draw_line(image_s, (cx,0), (cx, 720))
        draw_line(image_s, (0,cy), (960, cy))
        cv2.imshow('Camera', image_s)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            ed.stop()
            break

        if key == ord(" "):
            extra_print = True


    td.set_pitch(0)
    td.set_yaw(0)
    td.set_throttle(0)
    td.set_roll(0)
    td.land()
    ed.stop()
    td.quit()

    if state == 6:
        print("*************************************************")
        print("*************************************************")
        print("*************** LAND COMPLETE *******************")
        print("*************************************************")
        print("*************************************************")

        f = out_file
        
        f.write("====================== HEADER ======================\n")
        f.write("Odometry data will folow format (time, x, y, z, yaw)\n")
        f.write("Landing site: (-1, x, y, z, yaw)\n")
        f.write("X = left and right\n")
        f.write("Y = foraward backward\n")
        f.write("Z = height\n")
        f.write("=====================================================\n")

        for i in range(len(odometry)):
            f.write(f"{odometry[i]}\n")
        f.close()
        
    else:
        print("*************************************************")
        print("*************************************************")
        print("***************** LAND FAIL *********************")
        print("*************************************************")
        print("*************************************************")
