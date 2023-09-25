import os
from rosutility.util.file_loader import load_json_file

cwd = os.getcwd()
file_path = cwd.split('ros2_ws')[0] + 'ros2_ws/src/rosutility/rosutility/config/params/params_config.json'
network_config = load_json_file(file_path)["network"]

def get_connection_pruning_period():
    return network_config["connection"]["pruning-period"]

def get_heartbeat_period():
    return network_config["heartbeat"]["period"]
