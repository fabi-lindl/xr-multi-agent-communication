from rosrouter.agentclient.registry import singles_registry_message_sender as Sender

class RosAgentSubscriberDataForwarder:

    def process_ism(self, ism):
        management_data = ism.get_management_data()
        dest_agents = management_data["dest_agents"]
        message = ism.get_message()
        for dest_agent_type, dest_agent_ids in dest_agents.items():
            Sender.send_to_agentclients(message, dest_agent_type, dest_agent_ids)
