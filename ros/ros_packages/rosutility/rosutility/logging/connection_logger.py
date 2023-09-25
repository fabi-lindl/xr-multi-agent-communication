from rosutility.logging.logger import Logger

class ConnectionLogger(Logger):

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConnectionLogger, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__("connection_logger")
        self._initialized = True

    def log_server_is_listening(self, ip_address, port):
        message = (f"\nServer has IP address: {ip_address}\n"
                   f"Server is listening on port: {port}")
        self.log_info(message)

    def log_server_new_connection_accepted(self, ip_address, port):
        message = (f"New connection from (IP address: {ip_address}, Port: "
                   f"{port}) accepted.")
        self.log_info(message)

    def log_connection_closed(self):
        message = "Socket connection was closed."
        self.log_warning(message)

    def log_next_retry(self, seconds):
        message = f"Next retry in {seconds} ..."
        self.log_info(message)

    def log_give_up_connecting(self):
        message = "Give up on connecting to the rosrouter."
        self.log_warning(message)

    def log_pruned_expired_agentclient_conn_request(self, agent_id, agent_ip,
                                                    agent_port):
        message = (f"Pruned expired connection ({agent_ip}, {agent_port}) of "
                   f"configuration request for agentclient of id {agent_id} "
                   "and deleted stored request.")
        self.log_warning(message)

    def log_pruned_expired_agentclient_connection(self, agent_id, agent_ip,
                                                 agent_port):
        message = (f"Pruned expired connection ({agent_ip}, {agent_port}) "
                   f"and removed its agentclient (id: {agent_id}) from the "
                   "registry.")
        self.log_warning(message)

    def log_rosagent_connected_to_router(self, ip_address, port):
        message = (f"Rosagent connected to rosrouter (IP address: "
                   f"{ip_address}, Port: {port}).")
        self.log_info(message)

    def log_rosagent_connecting_to_router_failed(self, ip_address, port):
        message = (f"Could not connect to rosrouter (IP address: "
                   f"{ip_address}, Port {port})!")
        self.log_error(message)
