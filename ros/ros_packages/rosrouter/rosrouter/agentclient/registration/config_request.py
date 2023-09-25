from datetime import datetime

class ConfigRequest:

    def __init__(self, request_id, agentclient_id, conn_manager):
        self._request_id = request_id
        self._agentclient_id = agentclient_id
        self._conn_manager = conn_manager
        self._timestamp = datetime.now()
    
    def get_request_id(self):
        return self._request_id

    def get_agentclient_id(self):
        return self._agentclient_id

    def get_conn_manager(self):
        return self._conn_manager
    
    def get_timestamp(self):
        return self._timestamp
    