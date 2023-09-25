import os
from rosutility.util.file_loader import load_json_file

cwd = os.getcwd()
file_path = cwd.split("ros2_ws")[0] + ("ros2_ws/src/rosutility/rosutility/"
                                       "config/params/params_config.json")
agents_config = load_json_file(file_path)["agents"]

def get_agent_type_by_name(agent_name, type_as_str):
    agent_config = agents_config[agent_name]
    if type_as_str:
        return agent_config["type-str"]
    return agent_config["type"]

def get_agent_types(type_as_str=False):
    agent_types = []
    key = "type-str" if type_as_str else "type"
    for agent_config in agents_config.values():
        agent_types.append(agent_config[key])
    return agent_types

def get_ros_agent_type(type_as_str=False):
    if type_as_str:
        return agents_config["ros"]["type-str"]
    return agents_config["ros"]["type"]

def get_unity_agent_type(type_as_str=False):
    if type_as_str:
        return agents_config["unity"]["type-str"]
    return agents_config["unity"]["type"]
