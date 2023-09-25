from queue import Queue, Empty

from rosutility.config.params.getter import network_param_getter as NetworkParamGetter

heartbeat_period = NetworkParamGetter.get_heartbeat_period()
buffer = Queue()

def enqueue(data):
    buffer.put(data)

def dequeue():
    try:
        return buffer.get(timeout=heartbeat_period)
    except Empty:
        return None

def get_size():
    return buffer.qsize()

def is_empty():
    return buffer.empty()
