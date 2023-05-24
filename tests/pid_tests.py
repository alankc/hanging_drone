import sys
import numpy as np
import cv2
import time
import collections
from threading import Thread, Event

from simple_pid import PID
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation 

sys.path.insert(0, '../')
from easydrone import EasyDrone
import utils as ut

def nothing(x):
    pass

x_time = collections.deque(maxlen=300)
for t in range(300):
    x_time.append(t * 0.2)
y_setpoint = collections.deque(np.zeros(300))
y_read = collections.deque(np.zeros(300))

pid = PID(Kp=1/80, Ki=1/80, Kd=0)
pid.output_limits = (-0.5, 0.5) 
pid.setpoint = 0
pid.sample_time = None
pid.set_auto_mode(True, last_output=0)

run_control = False
ed = EasyDrone(True)
system_set_ctrl = lambda ctrl: ed.rc_control(throttle=ctrl)
system_get_out =  lambda : ed.get_curr_pos_corrected()[2]
e = Event()

def controller():
    global run_control, system_get_out, pid, system_set_ctrl, y_read, e
    while not e.is_set():
        if run_control:
            data = system_get_out()
            ctrl = pid(data)
            system_set_ctrl(ctrl)
            y_read.append(data)
        time.sleep(1/15)

if __name__ == "__main__":

    ed.connect()
    ed.start()

    ed.takeoff()

    cv2.namedWindow('Camera')
    cv2.createTrackbar('Variation', 'Camera', 0, 20, nothing)
    cv2.setTrackbarMin('Variation', 'Camera', -20)
    cv2.setTrackbarPos('Variation', 'Camera', 20)
    
    time_start = time.time()
    alpha = 0.1
    fps = 0

    Thread(target=controller, daemon=True).start()

    plt.ion()
    plt.show()
    plt.plot(x_time, y_setpoint)
    plt.plot(x_time, y_read)

    while True:
        image = ed.get_curr_frame()
        if image is None:
            frame = np.zeros((10, 10, 3), dtype = np.uint8)
            time.sleep(0.1)
            cv2.imshow('Camera', frame)
            cv2.waitKey(1)
            continue
        
        if time.time() - time_start > 0:
            fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
            time_start = time.time()

        ut.draw_text(image, f"FPS={fps}", -1)
        
        cv2.imshow('Camera', image)
        
        plt.clf()
        plt.plot(x_time, y_setpoint)
        plt.plot(x_time, y_read)


        key = cv2.waitKey(1) & 0xFF

        if key == ord("1"):
            setpoint = cv2.getTrackbarPos('Variation', 'Camera')
            cv2.setTrackbarPos('Variation', 'Camera', 0)
            ed.save_quaternion()
            data = system_get_out()
            pid.setpoint = data + setpoint
            
            y_read = collections.deque(maxlen=300)
            y_setpoint = collections.deque(maxlen=300)
            for i in range(300):
                y_setpoint.append(pid.setpoint)
                y_read.append(data)

            run_control = True
        
        if key == ord("2"):
            run_control = False

        if key == ord("q"):
            e.set()
            plt.close()
            cv2.destroyAllWindows()
            ed.land()
            time.sleep(5)
            ed.quit()
            break
    
    