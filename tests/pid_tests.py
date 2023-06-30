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

"""
throttle
KP = 1/80
TI = 2
TD = 2
pid = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD)
"""

"""
yaw

KP = 1/10
TI = 1
TD = 0.1
"""

"""
pitch
KP = 1/150
TI = 8
TD = 1
pid = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD)
pid.output_limits = (-1, 0.2) 

"""

"""
roll

KP = 1/150
TI = 5
TD = 2
"""

"""
new throttle
KP = 1/50
TI = 5
TD = 3
pid = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD)
"""

"""
Interesting link: https://www.mstarlabs.com/control/znrule.html
Rule Name	            Tuning Parameters
Classic Ziegler-Nichols Kp = 0.6 Ku     Ti = 0.5 Tu     Td = 0.125 Tu
Pessen Integral Rule    Kp = 0.7 Ku     Ti = 0.4 Tu     Td = 0.15 Tu
Some Overshoot          Kp = 0.33 Ku    Ti = 0.5 Tu     Td = 0.33 Tu
No Overshoot            Kp = 0.2 Ku     Ti = 0.5 Tu     Td = 0.33 Tu
KP= 0.01
KI= 0.008
KD= 0.00825
"""

KP  = 0.33 / 20
PCR = 2.5
TI  = PCR * 0.5
TD  = PCR * 0.33

print(f"KP= {KP}")
print(f"KI= {KP/TI}")
print(f"KD= {KP*TD}")


pid = PID(Kp=KP, Ki=KP/TI, Kd=KP*TD, proportional_on_measurement=False, differential_on_measurement=False)
pid.output_limits = (-0.3, 0.3) 
pid.setpoint = 0
pid.sample_time = None
pid.set_auto_mode(True, last_output=0)

run_control = False
ed = EasyDrone(True)
system_set_ctrl = lambda ctrl: ed.rc_control(throttle=ctrl)
system_get_out =  lambda : ed.get_curr_pos_corrected()[2]
e = Event()

freq = 25
fps_controller = 0
x_time = collections.deque(maxlen=300)
for t in range(300):
    x_time.append(np.round(t * 1.0/freq, 2))
y_setpoint = collections.deque(np.zeros(300))
y_read = collections.deque(np.zeros(300))

def controller():
    global run_control, system_get_out, pid, system_set_ctrl, y_read, e, freq, fps_controller
    
    while not e.is_set():

        if run_control:

            curr_dt = time.time()-time_start
            if curr_dt < 1.0/freq:
                t_delay = 1.0/freq - curr_dt
                time.sleep(t_delay / 10)
                continue
            
            fps_controller = 0.9 * fps_controller + 0.1 / curr_dt
            time_start = time.time()

            data = system_get_out()
            ctrl = pid(data)
            system_set_ctrl(ctrl)
            y_read.append(data)
        
        else:
            time_start = time.time()
            time.sleep(1/50)

if __name__ == "__main__":

    ed.connect()
    ed.start()
    time.sleep(5)
    ed.takeoff()

    cv2.namedWindow('Camera')
    cv2.createTrackbar('Variation', 'Camera', 0, 40, nothing)
    cv2.setTrackbarMin('Variation', 'Camera', -40)
    cv2.setTrackbarPos('Variation', 'Camera', 40)

    cv2.createTrackbar('Hz', 'Camera', 0, 60, nothing)
    cv2.setTrackbarMin('Hz', 'Camera', 10)
    cv2.setTrackbarPos('Hz', 'Camera', 25)
    
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

        ut.draw_text(image, f"    FPS SCREEN = {fps:.2f}", -1)
        ut.draw_text(image, f"FPS CONTROLLER = {fps_controller:.2f}", -2)
        
        cv2.imshow('Camera', image)
        
        plt.clf()
        plt.plot(x_time, y_setpoint)
        plt.plot(x_time, y_read)
        #plt.pause(0.001)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("1"):
            setpoint = cv2.getTrackbarPos('Variation', 'Camera')
            cv2.setTrackbarPos('Variation', 'Camera', 0)
            freq = cv2.getTrackbarPos('Hz', 'Camera')
            cv2.setTrackbarPos('Hz', 'Camera', 25)

            ed.save_quaternion()
            data = system_get_out()
            pid.setpoint = data + setpoint
            
            y_read = collections.deque(maxlen=300)
            y_setpoint = collections.deque(maxlen=300)
            for i in range(300):
                y_setpoint.append(pid.setpoint)
                y_read.append(data)

            x_time = collections.deque(maxlen=300)
            for t in range(300):
                x_time.append(np.round(t * 1.0/freq, 2))

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
    
    print(f"KP={KP}")
    print(f"KI={KP/TI}")
    print(f"KD={KP*TD}")
    