import time
from datetime import datetime
from threading import Thread

from rosrouter.agentclient.registration.agentclient_id_pool import AgentclientIdPool

from rosutility.logging.connection_logger import ConnectionLogger

class ConfigRequestsSurveillancer(Thread):

    _instance = None
    _cleanup_period = 60 # seconds

    def __new__(cls, *args):
        if cls._instance is None:
            cls._instance = (super(ConfigRequestsSurveillancer, cls)
                             .__new__(cls))
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self, pending_requests):
        if self._initialized: return
        Thread.__init__(self)
        self.daemon = True
        self._pending_requests = pending_requests
        self._agentclientIdPool = AgentclientIdPool()
        self._logger = ConnectionLogger()

    def run(self):
        while True:
            self._wait_for_prune_period()
            self._prune_expired_config_requests()

    def _wait_for_prune_period(self):
        time.sleep(ConfigRequestsSurveillancer._cleanup_period)
    
    def _prune_expired_config_requests(self):
        pruned_request_ids = []
        for request_id, request in self._pending_requests.items():
            if self._has_request_expired(request):
                self._prune_request_conn(request)
                pruned_request_ids.append(request_id)
        self._delete_pruned_requests(pruned_request_ids)
        self._pool_agentclient_ids(pruned_request_ids)

    def _has_request_expired(self, request):
        now = datetime.now()
        request_timestamp = request.get_timestamp()
        delta_time = (now - request_timestamp).total_seconds()
        return delta_time > ConfigRequestsSurveillancer._cleanup_period

    def _prune_request_conn(self, request):
        conn_manager = request.get_conn_manager()
        conn_manager.close_connection()
        self._log_prune_message(request.get_agentclient_id(), conn_manager)

    def _log_prune_message(self, agentclient_id, conn_manager):
        ip_address = conn_manager.get_ip_address()
        port = conn_manager.get_port()
        self._logger.log_pruned_expired_agentclient_conn_request(
            agentclient_id, ip_address, port)

    def _delete_pruned_requests(self, request_ids):
        for request_id in request_ids:
            del self._pending_requests[request_id]

    def _pool_agentclient_ids(self, agentclient_ids):
        for agentclient_id in agentclient_ids:
            self._agentclientIdPool.enqueue(agentclient_id)
