import os
from rosutility.util.file_loader import load_json_file

cwd = os.getcwd()
file_path = cwd.split("ros2_ws")[0] + ("ros2_ws/src/rosutility/rosutility/"
                                       "config/system_message/"
                                       "routing_data.json")
config_data = load_json_file(file_path)

def get_destination_agent_id_size():
    return config_data["destination_agents"]["item_size"]

def get_destination_group_id_size():
    return config_data["destination_groups"]["item_size"]

def get_num_destination_agents():
    return config_data["destination_agents"]["num_items"]

def get_num_destination_groups():
    return config_data["destination_groups"]["num_items"]
