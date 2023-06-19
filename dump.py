import yaml
import pickle

    
intrinsics_path = "drone/intrinsics.pkl"
dist_path = "drone/dist.pkl"
camera_pitch = -13

 #loading camera parammters
intrinsics_file = open(intrinsics_path, "rb")
intrinsics = pickle.load(intrinsics_file)
[[fx,_,cx],[_,fy,cy],[_,_,_]] = intrinsics
cy = 175

data = {
    'Vision': {'fast':{'nonmaxSuppression':1, 'type':2, 'threshold':10}},
    'Camera': {'fx':912.8844558325202, 'fy':916.1788304640794, 'cx':488.3668390339133, 'cy':377.42470274611975, 'cy_aligned':175, 'pitch':-13},
    'YOLO': {'path':"yolo_dataset/runs/detect/train6/weights/best.pt"},
    'PID': {
            'screen_yaw':{'kp':-1/960, 'ki':-0.2/960, 'kd':-0.1/960, 'min':-0.3, 'max':0.3}, 
            'screen_throttle':{'kp':1/240, 'ki':0.25/240, 'kd':0.3/240, 'min':-0.3, 'max':0.3},
            'throttle':{'kp':1/30, 'ki':1/40, 'kd':1/60, 'min':-0.3, 'max':0.3},
            'pitch':{'kp':1/150, 'ki':1/(8*150), 'kd':1/150, 'min':-0.2, 'max':0.2},
            'roll':{'kp':1/150, 'ki':1/(5*150), 'kd':2/150, 'min':-0.7, 'max':0.7},
            'yaw':{'kp':1/10, 'ki':1/10, 'kd':0.1*1/10, 'min':-0.5, 'max':0.5},
            },
    'WiFi': {'interface':'wlxd8ec5e0a30b5', 'ssid':'TELLO-98FD38', 'password':'TELLOBISG'},
    'RechargeStation': {'ip':"192.168.1.122", 'port':2810}
}

yaml_output = yaml.dump(data, sort_keys=False) 

def write_yaml_to_file(py_obj,filename):
    with open(f'{filename}.yaml', 'w',) as f :
        yaml.dump(py_obj,f,sort_keys=False) 
    print('Written to file successfully')

def read_one_block_of_yaml_data(filename):
    with open(f'{filename}.yaml','r') as f:
        output = yaml.safe_load(f)
    print(output) 

#write_yaml_to_file(data, 'parameters') 
read_one_block_of_yaml_data('parameters')

