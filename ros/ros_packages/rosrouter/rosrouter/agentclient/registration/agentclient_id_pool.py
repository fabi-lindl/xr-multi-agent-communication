from queue import Queue, Empty

class AgentclientIdPool:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentclientIdPool, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._id_pool = Queue()
    
    def enqueue(self, request_id):
        self._id_pool.put(request_id)
    
    def dequeue(self):
        try:
            request_id = self._id_pool.get(block=False)
            return request_id
        except Empty:
            return None
    