import re

def camel_case_str_to_attr_name(key, is_private_attr=True):
    snake_case_key = camel_to_snake_case(key)
    if is_private_attr:
        return "_" + snake_case_key
    return snake_case_key

def camel_to_snake_case(camel_case_value: str):
    upper_camel_case_value = camel_top_upper_camel_case(camel_case_value)
    camel_case_split = re.findall("[A-Z][^A-Z]*", upper_camel_case_value)
    snake_case_value = "_".join(camel_case_split).lower()
    return snake_case_value

def snake_to_camel_case(snake_case_value: str):
    underscore_split = snake_case_value.split("_")
    camel_case_value = underscore_split[0]
    split_length = len(underscore_split)
    for i in range(1, split_length):
        camel_case_value += underscore_split[i].capitalize()
    return camel_case_value

def camel_top_upper_camel_case(camel_case_value: str):
    first_char_upper = camel_case_value[0].upper()
    upper_camel_case_value = first_char_upper + camel_case_value[1:]
    return upper_camel_case_value
