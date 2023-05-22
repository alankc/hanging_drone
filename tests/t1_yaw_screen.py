import tellopy
import simple_pid
import numpy as np
import traceback
import sys
import av
import cv2
import time
import utils as ut
import pickle
from simple_pid import PID

from vision import Vision
from easydrone import EasyDrone
from threading import Event

"""

YAW PID CONTROLLER TESTS

*rect = selected_area()
k1, d1 = detect_features( img, rect)

while true:
    *img = read_img()
    *img = rotate_img(img)
    k2, d2 = detect_features(img)
    error = bf_matching_descriptors(k1, d1, k2, d2, 0.7)
    mean_error = mean(error(in x))
    pid.update(mean_error)

"""

select_rect = []
must_run = True

def mouse_click(event, x, y, flags, param):
    
    global select_rect

    if event == cv2.EVENT_LBUTTONDOWN:
        select_rect = [(x,y)]
    elif event == cv2.EVENT_LBUTTONUP:
        select_rect.append((x,y))

if __name__ == "__main__":
    
    drone = tellopy.Tello()

    e = Event()
    d = EasyDrone(drone, e, True)
    d.start()
    
    v = Vision()
    v.set_fast_detector(nonmaxSuppression=0, type=2, threshold=30)
    #v.set_sift_detector()
    v.set_bf_matcher()

    intrinsics_path = "drone/intrinsics.pkl"
    dist_path = "drone/dist.pkl"
    camera_pitch = -13

    #loading camera parammters
    intrinsics_file = open(intrinsics_path, "rb")
    intrinsics = pickle.load(intrinsics_file)
    [[fx,_,cx],[_,fy,cy],[_,_,_]] = intrinsics
    cy = 175

    pid_screen_yaw = PID(Kp=-1/960, Ki=-0, Kd=-0.25/960)
    pid_screen_yaw.output_limits = (-0.5, 0.5) 
    pid_screen_yaw.setpoint = 0
    pid_screen_yaw.sample_time = None
    pid_screen_yaw.set_auto_mode(True, last_output=0)

    pid_screen_height = PID(Kp=1/300, Ki=0, Kd=0.25/250)
    pid_screen_height.output_limits = (-0.5, 0.5) 
    pid_screen_height.setpoint = 0
    pid_screen_height.sample_time = None
    pid_screen_height.set_auto_mode(True, last_output=0)

    pid_height = PID(Kp=1/30, Ki=0, Kd=0.25/25)
    pid_height.output_limits = (-0.5, 0.5) 
    pid_height.setpoint = 0
    pid_height.sample_time = None
    pid_height.set_auto_mode(True, last_output=0)

    time.sleep(5) #wait camera

    try:        
        must_run = True
        test_print = False
        
        cv2.namedWindow("Camera")
        cv2.setMouseCallback("Camera", mouse_click)

        drone.wait_for_connection(60)
        drone.takeoff()
        #time.sleep(3)
        #drone.up(30)
        #time.sleep(2)
        #drone.up(0)

        k1 = None
        d1 = None
        k2 = None
        d2 = None
        
        time_start = time.time()
        fps = 0

        run_height = False
        run_yaw_scren = False
        run_height_scren = False

        while True:

            image = d.get_curr_frame()
            if image is None:
                frame = np.zeros((10, 10, 3), dtype = np.uint8)
                time.sleep(0.1)
                cv2.imshow('Camera', frame)
                key = cv2.waitKey(1) & 0xFF
                continue
            
            #image = cv2.blur(image,(5,5))
            image = v.rotateImage(image, 0, camera_pitch, 0, 0, 0, 1, fx, fy, cx, cy)
                
                
            alpha = 0.1
            if time.time()-time_start > 0:
                fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                time_start = time.time()
                
            image_s = image.copy()
            ut.draw_text(image_s, f"FPS: {int(round(fps,0))}", 0)
            ut.draw_text(image_s, f"Height: {pid_height.setpoint - d.get_curr_pos()[2]}", 4)
            
            if run_height:
                ctrl_height = np.round(pid_height(d.get_curr_pos()[2]), 2)
                drone.set_throttle(ctrl_height)
                ut.draw_text(image_s, f"Throtle: {ctrl_height}", 5)


            if (not (k1 is None)) and (run_yaw_scren or run_height_scren):
                k2, d2 = v.detect_features(image)
                error = v.bf_matching_descriptors(d1, k2, d2, 0.7, (cx, cy))

                if len(error) > 1:
                    mean_error = np.mean(error, axis=0)

                    if run_yaw_scren:
                        ctrl_screen_yaw = np.round(pid_screen_yaw(mean_error[0]), 2)
                        drone.set_yaw(ctrl_screen_yaw)

                    if run_height_scren:
                        ctrl_screen_height = np.round(pid_screen_height(mean_error[1]), 2)
                        drone.set_throttle(ctrl_screen_height)
                        
                    #ut.draw_text(image_s, f"Mean Error: {mean_error}", 1)
                    #ut.draw_text(image_s, f"CTRL: {ctrl}", 2)
                    #ut.draw_text(image_s, f"Matches: {len(error)}", 3)
                    ut.draw_line(image_s, (488,0), (488, 720))
                    ut.draw_line(image_s, (0,cy), (960, cy))
                    ut.draw_dot(image_s, (int(cx + mean_error[0]), int(cy + mean_error[1])))

                else:
                    if run_yaw_scren:
                        drone.set_yaw(0)
                        pid_screen_yaw.set_auto_mode(enabled=True, last_output=0)
                    
                    if run_height_scren:
                        drone.set_throttle(0)
                        pid_screen_height.set_auto_mode(enabled=True, last_output=0)
                    
            cv2.imshow('Camera', image_s)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                e.set()
                break

            if key == ord("1"): #enable yaw controller
                k1, d1 = v.detect_features_in_rect(image, select_rect)

            if key == ord("a"): #enable yaw controller
                run_yaw_scren = True
                run_height_scren = False
                run_height = False

            if key == ord("s"): #enable height screen controller
                run_height_scren = True
                run_height = False
                run_yaw_scren = False

            if key == ord("d"): #enable height mvo controller
                pid_height.setpoint = d.get_curr_pos()[2] + 15
                run_height = True
                run_yaw_scren = False
                run_height_scren = False

            if key == ord(" "): #stop all
                run_height = False
                run_yaw_scren = False
                run_height_scren = False
                drone.set_yaw(0)
                drone.set_throttle(0)
                    
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.land()
        drone.quit()
        cv2.destroyAllWindows()
