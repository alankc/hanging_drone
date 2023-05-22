import tellopy
from threading import Thread, Event
import av
import time
import traceback
import sys
import cv2
import numpy as np
from pyquaternion import Quaternion

class EasyDrone(Thread):
    def __init__(self, get_log_data:bool = True) -> None:
        
        super(EasyDrone, self).__init__()

        self.__drone = None
        self.__curr_frame = None
        self.__event = Event()
        self.mvo = None
        self.imu = None
        self.__get_log_data = get_log_data
        self.__q = Quaternion(1,0,0,0) #No rotation
        self.__start_yaw = 0
    
    def connect(self):
        self.__drone = tellopy.Tello()
        self.__drone.connect()
        self.__drone.wait_for_connection(60.0)

    def takeoff(self):
        self.__drone.takeoff()

    def land(self):
        self.__drone.land()

    def rc_control(self, throttle:float = 0, pitch:float = 0, roll:float = 0, yaw:float = 0):
        self.__drone.set_throttle(throttle)
        self.__drone.set_pitch(pitch)
        self.__drone.set_roll(roll)
        self.__drone.set_yaw(yaw)

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
    