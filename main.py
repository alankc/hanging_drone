import argparse
import os
import yaml
from main_state_machine import MainStateMachine

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
   
    parser.add_argument('-p', '--parameters',
                        required=False,
                        default="parameters.yaml",
                        help='System parameters'
                        )
    
    args = parser.parse_args()

    if not os.path.exists(args.parameters):
        print("======================================================")
        print("============= Parameters file required ===============")
        print("======================================================")
        exit(0)

    with open(f'{args.parameters}','r') as f:
        parameters_file = yaml.safe_load(f)
    
    msm = MainStateMachine(parameters_file)
    msm.start()