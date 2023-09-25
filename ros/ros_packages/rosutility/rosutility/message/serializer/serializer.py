import json
import struct

from rosutility.message.serializer import struct_format_serializer as StructFormatSerializer

def dict_to_bytes(value):
    json_value = json.dumps(value)
    str_value = json_value.encode("utf-8")
    return str_to_bytes(str_value)

def str_to_bytes(value):
    length = len(value)
    return struct.pack(f"<{length}s", value)

def uint8_to_bytes(value):
    return struct.pack("<B", value)

def uint16_to_bytes(value):
    return struct.pack("<H", value)

def uint_to_bytes(value):
    return struct.pack("<I", value)

def uintx_to_bytes(value, item_size):
    format_char = StructFormatSerializer.num_bytes_to_format_char(item_size)
    return struct.pack(f"<{format_char}", value)

def uint8_iterable_to_bytes(iterable, fixed_num_items=0):
    format_char = "B"
    return _iterable_to_bytes(iterable, format_char, fixed_num_items)

def uint16_iterable_to_bytes(iterable, fixed_num_items=0):
    format_char = "H"
    return _iterable_to_bytes(iterable, format_char, fixed_num_items)

def uint_iterable_to_bytes(iterable, fixed_num_items=0):
    format_char = "I"
    return _iterable_to_bytes(iterable, format_char, fixed_num_items)

def uintx_iterable_to_bytes(iterable, item_size, fixed_num_items=0):
    format_char = StructFormatSerializer.num_bytes_to_format_char(item_size)
    return _iterable_to_bytes(iterable, format_char, fixed_num_items)

def _iterable_to_bytes(iterable, format_char, fixed_num_items=0):
    num_items = len(iterable)
    num_pad_bytes = _num_pad_bytes(num_items, fixed_num_items, format_char)
    format_ = f"<{num_items}{format_char}{num_pad_bytes}x"
    return struct.pack(format_, *iterable)

def _num_pad_bytes(num_items, fixed_num_items, format_char):
    if num_items > fixed_num_items:
        return 0
    num_missing_items = fixed_num_items - num_items
    item_size = StructFormatSerializer.format_char_to_num_bytes(format_char)
    num_pad_bytes = num_missing_items * item_size
    return num_pad_bytes
