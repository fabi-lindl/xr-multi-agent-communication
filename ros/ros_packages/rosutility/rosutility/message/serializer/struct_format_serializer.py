_format_char_to_item_size = {
    "B": 1,
    "H": 2,
    "I": 4
}

_item_size_to_format_char = {v:k for k, v in _format_char_to_item_size.items()}

def format_char_to_num_bytes(format_char):
    return _format_char_to_item_size[format_char]

def num_bytes_to_format_char(num_bytes):
    return _item_size_to_format_char[num_bytes]
