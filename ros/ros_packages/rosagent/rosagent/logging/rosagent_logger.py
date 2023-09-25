from rosutility.logging.logger import Logger

class RosagentLogger(Logger):

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosagentLogger, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__("rosagent_logger")
        self._initialized = True

    def log_configure_agent(self, id_):
        self.log_info(f"Configure rosagent of id {id_} ...")

    def log_send_config_message(self):
        self.log_info("Sending rosagent config message ...")

    def log_remove_unity_agents_from_interface_nodes(self):
        message = ("Remove ids of unity agents that went offline from all "
                   "interface nodes ...")
        self.log_info(message)
