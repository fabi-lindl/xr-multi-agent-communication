from rosrouter.management.util.ros_agent_node_registration import RosAgentNodeRegistration
from rosrouter.agentclient.registry import agentclient_registry as AgentclientRegistry

from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class RosAgentNodeRegistrationForwarder:

    def process_ism(self, ism):
        management_data = ism.get_management_data()
        self._node_registration = RosAgentNodeRegistration(**management_data)
        self._message = ism.get_message()
        self._process_registration()

    def _process_registration(self):
        self._process_single_agentclients_registration()
        self._process_agentclient_groups_registration()

    def _process_single_agentclients_registration(self):
        ros_agent_type = AgentParamGetter.get_ros_agent_type()
        for registrar_agent_id in self._node_registration.registrar_agent_ids:
            agentclient = AgentclientRegistry.get(ros_agent_type,
                                                  registrar_agent_id)
            if agentclient is None: continue
            agentclient.send(self._message)

    def _process_agentclient_groups_registration(self):
        for registrar_group_id in self._node_registration.registrar_group_ids:
            group = AgentclientRegistry.get_group(registrar_group_id)
            if group is None: continue
            self._process_registration_of_group(group)

    def _process_registration_of_group(self, group):
        agent_type = self._node_registration.get_agent_type()
        agentclients = group.get_agentclients_of_type(agent_type)
        if agentclients is None: return
        for agentclient in agentclients:
            agentclient.send(self._message)
