import time
from threading import Thread

from rosrouter.agentclient.registry import agentclient_registry as Registry
from rosrouter.management.sending import message_distributor as MessageDistributor
from rosrouter.message.egress_system_message import EgressSystemMessage

from rosutility.message.serializer import serializer as Serializer
from rosutility.message.system_message_metadata import SystemMessageMetadata

class AgentclientConnectionSurveillancer(Thread):
    """
    Prunes connections that did not have any ingress traffic for a specified
    amount of time.
    Removes the agentclients that own the pruned connections from the registry.
    Sends a broadcast message with the ids of pruned unity agents to allow
    agents to cleanup their respective nodes interfaces.
    """
    _instance = None
    _cleanup_period = 60 # seconds
    _message_manager = "PrunedUnityAgentsInterfaceNodesCleaner"

    def __new__(cls):
        if cls._instance is None:
            cls._instance = (super(AgentclientConnectionSurveillancer, cls)
                             .__new__(cls))
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized: return
        Thread.__init__(self)
        self.daemon = True
    
    def run(self):
        while True:
            self._wait_for_prune_period()
            self._prune()

    def _wait_for_prune_period(self):
        time.sleep(AgentclientConnectionSurveillancer._cleanup_period)

    def _prune(self):
        expired_agentclients = self._collect_expired_agentclients()
        self.prune_agentclients(expired_agentclients)

    def _collect_expired_agentclients(self):
        expired_agentclients = []
        types = Registry.get_agentclient_types()
        for type_ in types:
            agentclients = Registry.get_agentclients_of_type(type_)
            for agentclient in agentclients:
                if not agentclient.is_connection_alive():
                    expired_agentclients.append(agentclient)
        return expired_agentclients

    def prune_agentclient(self, agentclient):
        self.prune_agentclients([agentclient])

    def prune_agentclients(self, agentclients):
        self._prune_agentclient_connections(agentclients)
        self._remove_agentclients_from_registry(agentclients)
        self._broadcast_pruned_info_message(agentclients)

    def _prune_agentclient_connections(self, expired_agentclients):
        for expired_agentclient in expired_agentclients:
            expired_agentclient.close_connection()

    def _remove_agentclients_from_registry(self, expired_agentclients):
        for expired_agentclient in expired_agentclients:
            Registry.remove(expired_agentclient)

    def _broadcast_pruned_info_message(self, pruned_agentclients):
        if self._has_message_to_send(pruned_agentclients):
            agents_data = self._create_agents_data(pruned_agentclients)
            message = self._create_pruned_info_message(agents_data)
            MessageDistributor.broadcast_message(message)

    def _create_agents_data(self, pruned_agentclients):
        agents_data = []
        for pruned_agentclient in pruned_agentclients:
            agent_data = self._create_agent_data(pruned_agentclient)
            agents_data.append(agent_data)
        return agents_data

    def _create_agent_data(self, pruned_agentclient):
        return {
            "agent_type": pruned_agentclient.get_agent_type(),
            "agent_id": pruned_agentclient.get_agent_id()
        }

    def _has_message_to_send(self, pruned_agentclients):
        return len(pruned_agentclients) > 0

    def _create_pruned_info_message(self, agents_data) -> bytes:
        metadata = SystemMessageMetadata(message_manager=
            AgentclientConnectionSurveillancer._message_manager)
        data = self._create_message_data(agents_data)
        esm = EgressSystemMessage(metadata, data)
        esm_bytes = esm.serialize()
        return esm_bytes

    def _create_message_data(self, agents_data) -> bytes:
        data = {"agents": agents_data}
        data_bytes = Serializer.dict_to_bytes(data)
        return data_bytes
