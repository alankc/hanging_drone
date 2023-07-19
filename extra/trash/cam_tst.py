import cv2
import tellopy
import numpy as np
import time
import utils as ut
from easydrone import EasyDrone


if __name__ == "__main__":
           
    td = tellopy.Tello()
    td.connect()
    td.wait_for_connection(60.0)

    ed = EasyDrone(td, True)
    ed.start()

    start_time = time.time() + 10000
    count = 0

    while True:

        if td.state == td.STATE_DISCONNECTED:
            print("OFF")
            break

        image = ed.get_curr_frame()

        if (time.time() - start_time) > 5:
            value = -1 + 0.1 * (-1) ** count
            count = count + 1
            print(f"set_throttle({value})")
            td.set_throttle(value)
            start_time = time.time()

        if image is None:
            frame = np.zeros((10, 10, 3), dtype = np.uint8)
            time.sleep(0.1)
            cv2.imshow('Camera', frame)
            key = cv2.waitKey(1) & 0xFF
            continue

        cv2.imshow("Camera", image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or not ed.is_alive():
            ed.stop()
            break

        if key == ord("1"):
            start_time =  time.time()
            td.manual_takeoff()
            
            time.sleep(5)
            
            td.set_pitch(0)
            td.set_roll(0)
            td.set_yaw(0)
            td.set_throttle(0)

        if key == ord("2"):
            td.land()

    ed.stop()
    td.quit()
