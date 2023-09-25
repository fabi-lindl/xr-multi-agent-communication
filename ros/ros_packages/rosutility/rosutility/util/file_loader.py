import json

def load_json_file(file_path):
    f = open(file_path)
    json_data = json.load(f)
    f.close()
    return json_data
