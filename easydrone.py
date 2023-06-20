import tellopy
from threading import Thread, Event
import av
import time
import traceback
import sys
import cv2
import numpy as np
from pyquaternion import Quaternion
from simple_pid import PID

#To use the Fakewebcam use:
# sudo depmod -a
# sudo modprobe -r v4l2loopback
# sudo modprobe v4l2loopback  or sudo modprobe v4l2loopback video_nr=28
# sudo v4l2-ctl --list-device

from pyfakewebcam import FakeWebcam

class EasyDrone(Thread):
    def __init__(self, get_log_data:bool = True, pub_video_stream:str = None, pid:dict = {}) -> None:
        
        super(EasyDrone, self).__init__()

        self.__drone = None
        self.__curr_frame = None
        self.__event = Event()
        self.mvo = None
        self.imu = None
        self.__get_log_data = get_log_data
        self.__q = Quaternion(1,0,0,0) #No rotation
        self.__start_yaw = 0

        if not (pub_video_stream is None):
            self.__camera = FakeWebcam(pub_video_stream, 960, 720)
        else:
            self.__camera = None

        #Dear Programmer from the future, I know that putting all these PIDs here is a trash practice,
        #but it was a fast way, sorry and good luck ;D
        dpid = pid['screen_yaw']
        self.pid_s_yaw = PID(Kp=dpid['kp'], Ki=dpid['ki'], Kd=dpid['kd'], proportional_on_measurement=False, differential_on_measurement=False)
        self.pid_s_yaw.output_limits = (dpid['min'], dpid['max']) 
        self.pid_s_yaw.setpoint = 0
        self.pid_s_yaw.sample_time = None
        self.pid_s_yaw.set_auto_mode(True, last_output=0)

        dpid = pid['screen_throttle']
        self.pid_s_throttle = PID(Kp=dpid['kp'], Ki=dpid['ki'], Kd=dpid['kd'], proportional_on_measurement=False, differential_on_measurement=False)
        self.pid_s_throttle.output_limits = (dpid['min'], dpid['max']) 
        self.pid_s_throttle.setpoint = 0
        self.pid_s_throttle.sample_time = None
        self.pid_s_throttle.set_auto_mode(True, last_output=0)

        dpid = pid['throttle']
        self.pid_throttle = PID(Kp=dpid['kp'], Ki=dpid['ki'], Kd=dpid['kd'], proportional_on_measurement=False, differential_on_measurement=False)
        self.pid_throttle.output_limits = (dpid['min'], dpid['max']) 
        self.pid_throttle.setpoint = 0
        self.pid_throttle.sample_time = None
        self.pid_throttle.set_auto_mode(True, last_output=0)
        
        dpid = pid['pitch']
        self.pid_pitch = PID(Kp=dpid['kp'], Ki=dpid['ki'], Kd=dpid['kd'], proportional_on_measurement=False, differential_on_measurement=False)
        self.pid_pitch.output_limits = (dpid['min'], dpid['max']) 
        self.pid_pitch.setpoint = 0
        self.pid_pitch.sample_time = None
        self.pid_pitch.set_auto_mode(True, last_output=0)

        dpid = pid['roll']
        self.pid_roll = PID(Kp=dpid['kp'], Ki=dpid['ki'], Kd=dpid['kd'], proportional_on_measurement=False, differential_on_measurement=False)
        self.pid_roll.output_limits = (dpid['min'], dpid['max']) 
        self.pid_roll.setpoint = 0
        self.pid_roll.sample_time = None
        self.pid_roll.set_auto_mode(True, last_output=0)
        
        dpid = pid['yaw']
        self.pid_yaw = PID(Kp=dpid['kp'], Ki=dpid['ki'], Kd=dpid['kd'], proportional_on_measurement=False, differential_on_measurement=False)
        self.pid_yaw.output_limits = (dpid['min'], dpid['max'])  
        self.pid_yaw.setpoint = 0
        self.pid_yaw.sample_time = None
        self.pid_yaw.set_auto_mode(True, last_output=0)

    def PID_reset(self):
        self.pid_s_yaw.set_auto_mode(True, last_output=0)
        self.pid_s_throttle.set_auto_mode(True, last_output=0)
        self.pid_throttle.set_auto_mode(True, last_output=0)
        self.pid_pitch.set_auto_mode(True, last_output=0)
        self.pid_roll.set_auto_mode(True, last_output=0)
        self.pid_yaw.set_auto_mode(True, last_output=0)

    def connect(self):
        self.__drone = tellopy.Tello()
        self.__drone.connect()
        self.__drone.wait_for_connection(60.0)

    def takeoff(self):
        self.__drone.takeoff()

    def cooler_on(self):
        self.__drone.manual_takeoff()

    def land(self):
        self.__drone.land()

    def rc_control(self, throttle:float = 0, pitch:float = 0, roll:float = 0, yaw:float = 0):
        self.__drone.set_throttle(throttle)
        self.__drone.set_pitch(pitch)
        self.__drone.set_roll(roll)
        self.__drone.set_yaw(yaw)
    
    def set_throttle(self, throttle:float):
        self.__drone.set_throttle(throttle)

    def set_pitch(self, pitch:float):
        self.__drone.set_pitch(pitch)

    def set_roll(self, roll:float):
        self.__drone.set_roll(roll)

    def set_yaw(self, yaw:float):
        self.__drone.set_yaw(yaw)

    def set_destination(self, x, y, z, yaw):
        self.save_quaternion()
        (cy, cx, cz) = self.get_curr_pos_corrected()
        cyaw = self.get_curr_yaw()

        self.pid_roll.setpoint = cx + x
        self.pid_pitch.setpoint = cy + y
        self.pid_throttle.setpoint = cz + z
        self.pid_yaw.setpoint = cyaw + yaw

        self.pid_throttle.set_auto_mode(True, last_output=0)
        self.pid_pitch.set_auto_mode(True, last_output=0)
        self.pid_roll.set_auto_mode(True, last_output=0)
        self.pid_yaw.set_auto_mode(True, last_output=0)
    
    #pt: position tolerance
    #yt: yaw tolerance
    def update_control(self, pt = 10, yt = 5):
        
        (cy, cx, cz) = self.get_curr_pos_corrected()
        cyaw = self.get_curr_yaw()

        ex = self.pid_roll.setpoint - cx
        ey = self.pid_pitch.setpoint - cy
        ez = self.pid_throttle.setpoint - cz
        eyaw = abs(self.pid_yaw.setpoint - cyaw)
        pe = (ex**2 + ey**2 + ez**2) ** 0.5
        
        if pe <= pt and eyaw <= yt:
            self.rc_control()
            return True

        throttle = self.pid_throttle(cz)
        pitch = self.pid_pitch(cy)
        roll = self.pid_roll(cx)
        yaw = self.pid_yaw(cyaw)

        self.__drone.set_throttle(throttle)
        self.__drone.set_pitch(pitch)
        self.__drone.set_roll(roll)
        self.__drone.set_yaw(yaw)

        return False

    def quit(self):
        self.__event.set()
        self.__drone.quit()

    def handler_log_data(self, event, sender, data, **args):
        drone = sender
        if event is drone.EVENT_LOG_DATA:      
            if data:
                self.mvo = data.mvo
                self.imu = data.imu

    def get_curr_frame(self):
        return self.__curr_frame
    
    def save_quaternion(self):
        self.__q = Quaternion(self.imu.q0, self.imu.q1, self.imu.q2, self.imu.q3)
        self.__q = self.__q.inverse

        q0 = self.__q.w
        q1 = self.__q.x
        q2 = self.__q.y
        q3 = self.__q.z
        self.__start_yaw =  np.arctan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2) * 180 /  np.pi
    
    def get_curr_pos(self):
        return (self.mvo.pos_x*100, self.mvo.pos_y*100, -self.mvo.pos_z*100)
    
    def get_curr_pos_corrected(self):
        curr_pos = (self.mvo.pos_x*100, self.mvo.pos_y*100, -self.mvo.pos_z*100)
        return self.__q.rotate(curr_pos)
    
    def get_curr_yaw(self):
        q_read = Quaternion(self.imu.q0, self.imu.q1, self.imu.q2, self.imu.q3) #current quaternion
        q0 = q_read.w
        q1 = q_read.x
        q2 = q_read.y
        q3 = q_read.z
        curr_yaw = np.arctan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2) * 180 /  np.pi
        return curr_yaw - self.__start_yaw

    def get_curr_speed_corrected(self):
        curr_speed = (self.mvo.vel_x * 10, self.mvo.vel_y * 10, -self.mvo.vel_z * 10)
        return self.__q.rotate(curr_speed)
    
    def rotate_pos(self, pos):
        return self.__q.rotate(pos)
    
    def run(self):
        kernel = np.array([[ 0,-1, 0], 
                        [-1, 5,-1],
                        [ 0,-1, 0]])
        
        try:
            self.__drone.set_video_encoder_rate(4)
            #self.__drone.set_exposure(2)

            if self.__get_log_data:
                self.__drone.subscribe(self.__drone.EVENT_LOG_DATA, self.handler_log_data)

            retry = 3
            container = None
            while container is None and 0 < retry:
                retry -= 1
                try:
                    container = av.open(self.__drone.get_video_stream())
                except av.AVError as ave:
                    print(ave)
                    print('retry...')
        
            frame_skip = 300

            #total_run_time = time.time()
            while not self.__event.is_set():
                for frame in container.decode(video=0):
                    if 0 < frame_skip:
                        frame_skip = frame_skip - 1
                        continue

                    if self.__event.is_set():
                        break

                    start_time = time.time()
                    
                    #original
                    #self.__curr_frame = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    
                    #"""
                    ###############################
                    ## correcting contrast image ##
                    ###############################
                    img = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    #g = cv2.GaussianBlur(img, (7,7), 0)
                    #new_frame = cv2.addWeighted(img, 2, g, -1, 0) #Unsharp masking
                    #new_frame = cv2.bilateralFilter(new_frame, d=5, sigmaColor=80, sigmaSpace=80)
                    new_frame = cv2.filter2D(img, -1, kernel)
                    new_frame = cv2.bilateralFilter(new_frame, d=5, sigmaColor=80, sigmaSpace=80)
                    self.__curr_frame = new_frame
                    ###############################
                    ############# END #############
                    ###############################
                    #"""
                    
                    #publishing video
                    if not (self.__camera is None):
                        self.__camera.schedule_frame(cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB))
                    
                    if frame.time_base < 1.0/60:
                        time_base = 1.0/60
                    else:
                        time_base = frame.time_base
                    frame_skip = int((time.time() - start_time)/time_base)
                    
        except Exception as ex:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            print(ex)
            
            """
            total_run_time = time.time() - total_run_time
            self.__event.set()
            print("===========================================")
            print(f"=== Total time: {total_run_time} ===")
            print("===========================================")
            self.__drone.quit()
            """



"""
Test
"""

if __name__ == "__main__":
    drone = tellopy.Tello()

    e = Event()
    dc = EasyDrone(drone, e, True)
    dc.setDaemon(True)

    dc.start()

    import utils as ut

    while True:
        frame = dc.get_curr_frame()
        if frame is None:
            frame = np.zeros((10, 10, 3), dtype = np.uint8)
        
        ut.draw_text(frame, f"{dc.mvo}", 0)
        ut.draw_text(frame, f"{dc.imu}", 1)
        cv2.imshow("Teste", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            e.set()
            break

    drone.quit()
    