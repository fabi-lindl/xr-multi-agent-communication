from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class RosAgentNodeRegistration:

    _agent_type = AgentParamGetter.get_ros_agent_type()

    def __init__(self, registrant_agent_id, registrar_agent_ids, registrar_group_ids,
                 node_type: int, node_name, tos_name, data_exchange_type: str,
                 **kwargs):
        self.registrant_agent_id = registrant_agent_id
        self.registrar_agent_ids = registrar_agent_ids
        self.registrar_group_ids = registrar_group_ids
        self.node_type = node_type
        self.node_name = node_name
        self.tos_name = tos_name
        self.data_exchange_type = data_exchange_type

    def get_agent_type(self):
        return RosAgentNodeRegistration._agent_type
