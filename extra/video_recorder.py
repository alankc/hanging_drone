import argparse
import os
import yaml
import sys
import cv2
import time
sys.path.insert(0, '../')

from easydrone import EasyDrone
from stereo import Stereo
import utils as ut

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
   
    parser.add_argument('-p', '--parameters',
                        required=False,
                        default="parameters.yaml",
                        help='System parameters'
                        )
    
    parser.add_argument('-v', '--video_file',
                        required=False,
                        default="video",
                        help='Video File'
                        )
    
    parser.add_argument('-f', '--out_folder',
                        required=False,
                        default="out",
                        help=''
                        )
    
    args = parser.parse_args()

    if not os.path.exists(args.parameters):
        print("======================================================")
        print("============= Parameters file required ===============")
        print("======================================================")
        exit(0)

    os.makedirs(args.out_folder, exist_ok=True)

    video_path = str(args.out_folder) + "/" + str(args.video_file) + ".mp4"

    with open(f'{args.parameters}','r') as f:
        parameters_file = yaml.safe_load(f)

    ed = EasyDrone(True, parameters_file['Camera']['stream'], parameters_file['Control']['pid'], parameters_file['WiFi'])
    ed.connect()
    ed.start()
    
    desired_fps = parameters_file['Control']['desired_fps']

    s = Stereo()
    sp = parameters_file['Camera']
    s.set_camera_params(sp['fx'], sp['fy'], sp['pitch'], sp['cx'], sp['cy_aligned'])

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_path, fourcc, desired_fps, (960, 720))
    
    # loop runs if capturing has been initialized.
    time_start = time.time()
    alpha = 0.1
    fps = 0
    
    while(True):

        curr_dt = time.time()-time_start
        if curr_dt < 1.0/desired_fps:
            t_delay = 1.0/desired_fps - curr_dt
            time.sleep(t_delay / 10)
            continue


        fps = (1 - alpha) * fps + alpha * 1 / curr_dt  # exponential moving average
        time_start = time.time()

        # reads frames from a camera 
        # ret checks return at each frame
        frame = ed.get_curr_frame()
        if frame is None:
            continue

        frame = s.rotateImage(frame)
    
        out.write(frame) 
        
        # The original input frame is shown in the window 
        cv2.imshow('Original', frame)
  
        
        key = cv2.waitKey(20) & 0xFF
        if key == 27:
            ed.land()
            break

        elif key == ord("q"):
            ed.land()
            print("*************************************************")
            print("******************** LAND ***********************")
            print("*************************************************", flush=True)

        elif key == ord("e"):
            ed.takeoff()
            print("*************************************************")
            print("****************** TAKE OFF *********************")
            print("*************************************************", flush=True)

        else:
            ut.rc_control(key, ed)
    
    # After we release our webcam, we also release the output
    out.release()
    ed.quit()
    
    # De-allocate any associated memory usage 
    cv2.destroyAllWindows()

    vidcap = cv2.VideoCapture(video_path)
    num_frames = vidcap.get(cv2.CAP_PROP_FRAME_COUNT)

    success,image = vidcap.read()
    count = 0
    f_count = 0
    while success:
        
        if count % 25 == 0:
            cv2.imwrite(f"{str(args.out_folder)}/{str(args.video_file)}_{f_count}.png", image)     # save frame as JPEG file
            f_count += 1
        
        success,image = vidcap.read()
        perc = int(10 * count / (num_frames - 1))
        outstr = "*" * perc
        while len(outstr) < 10:
            outstr += " "

        print(f"\rProcessing [{outstr}]",end="")
        count += 1

    print("\nFinished")
    