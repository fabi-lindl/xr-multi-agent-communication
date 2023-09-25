from rosutility.config.params.getter import agent_param_getter as AgentParamGetter

class RosAgentConfiguration:

    _instance = None

    def __new__(cls, router_ip_address=None, router_port=None, group_id=0):
        if cls._instance is None:
            cls._instance = super(RosAgentConfiguration, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self, router_ip_address=None, router_port=None, group_id=0):
        if self._initialized:
            return
        self._initialized = True
        self._router_ip_address = router_ip_address
        self._router_port = router_port
        self._agent_type = AgentParamGetter.get_ros_agent_type()
        self._agent_id = None # Is set onconnect with RosRouter.
        self._group_id = group_id
    
    def get_router_ip_address(self):
        return self._router_ip_address
    
    def get_router_port(self):
        return self._router_port
    
    def get_router_address(self):
        return (self._router_ip_address, self._router_port)

    def get_agent_type(self):
        return self._agent_type

    def get_agent_id(self):
        return self._agent_id

    def get_group_id(self):
        return self._group_id

    def set_agent_id(self, agent_id):
        self._agent_id = agent_id

    def belongs_to_group(self):
        return bool(self._group_id)
