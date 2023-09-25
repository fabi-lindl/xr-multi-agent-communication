from rosrouter.agentclient.registration.agentclient_registrar import AgentclientRegistrar
from rosrouter.management.management_message_manager import ManagementMessageManager

class AgentclientConfigurer(ManagementMessageManager):

    def __init__(self):
        super().__init__()

    def process_ism(self, ism):
        registrar = AgentclientRegistrar()
        registrar.create_agentclient(ism.get_management_data())
