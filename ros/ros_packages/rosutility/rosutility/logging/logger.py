from rclpy.node import Node

class Logger(Node):

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, node_name="logger"):
        if self._initialized:
            return
        super().__init__(node_name)
        self._initialized = True
        self.node_logger = self.get_logger()

    def log_start_workers(self):
        self.log_info("Starting worker threads ...")

    def log_info(self, message):
        self.node_logger.info(message)

    def log_warning(self, message):
        self.node_logger.warning(message)

    def log_error(self, message):
        self.node_logger.error(message)
