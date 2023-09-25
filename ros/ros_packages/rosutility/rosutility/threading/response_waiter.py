import threading

class ResponseWaiter:

    def __init__(self):
        self._cv = threading.Condition()
        self._response = None
    
    def wait_for_response(self):
        with self._cv:
            self._cv.wait()
    
    def resume(self, response):
        self._response = response
        with self._cv:
            self._cv.notify()
    
    def get_response(self):
        return self._response
