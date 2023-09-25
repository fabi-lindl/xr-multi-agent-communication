import json
import struct

from rosutility.message.serializer import variable_format_serializer as VariableFormatSerializer

def bytes_to_dict(data, create_snake_case_keys=False):
    if data == b'':
        return {}
    data_str = data.decode("utf-8")
    data_dict = json.loads(data_str)
    if create_snake_case_keys:
         return _keys_to_snake_case_keys(data_dict)
    return data_dict

def _keys_to_snake_case_keys(data: dict):
    return_data = {}
    for key, value in data.items():
        snake_case_key = VariableFormatSerializer.camel_to_snake_case(key)
        return_data[snake_case_key] = value
    return return_data

def bytes_to_uint(value):
    return struct.unpack("<I", value)[0]
