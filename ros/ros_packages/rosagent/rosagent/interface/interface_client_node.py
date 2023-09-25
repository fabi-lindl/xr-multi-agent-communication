import threading

from rclpy.serialization import deserialize_message

from rosagent.config.ros_agent_configuration import RosAgentConfiguration
from rosagent.interface.interface_sender_node import InterfaceSenderNode
from rosagent.message.egress_system_message import EgressSystemMessage
from rosagent.util import data_exchange_type_module_importer as Importer

from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo
from rosutility.threading.response_waiter import ResponseWaiter

class InterfaceClientNode(InterfaceSenderNode):

    NODE_TYPE = 4

    def __new__(cls, data_exchange_type, *args, **kwargs):
        class_ = Importer.get_service_type_class(data_exchange_type)
        if class_ is None: return None
        return super().__new__(cls)

    def __init__(self, registrant_agent_type, registrant_agent_id,
                 node_name, tos_name, data_exchange_type,
                 buffer_size=10, drop_new=True, **kwargs):
        super().__init__(registrant_agent_type, registrant_agent_id,
                         node_name, InterfaceClientNode.NODE_TYPE, tos_name,
                         data_exchange_type, buffer_size, drop_new)
        class_ = Importer.get_service_type_class(data_exchange_type)
        self._service_agent_type = registrant_agent_type
        self._service_agent_id = registrant_agent_id
        self._service = self.create_service(class_,
                                            tos_name,
                                            self._service_function)
        self._response_message_type = type(class_.Response())
        self._next_request_id = 1
        self._pending_responses = {}
        self._lock = threading.Lock()
        self._rosagent_config = RosAgentConfiguration()

    def get_service_agent_id(self):
        return self._service_agent_id

    def get_service_name(self):
        return self.get_tos_name()

    def has_callback_group(self):
        return True

    def process_ism(self, ism):
        request_id = ism.get_request_id()
        response_waiter = self.get_response_waiter(request_id)
        if response_waiter is not None:
            data = ism.get_data()
            response_waiter.resume(data)

    def get_response_waiter(self, request_id, del_pending_response=True):
        if self.has_request_pending_response(request_id):
            response_waiter = self._pending_responses[request_id]
            if del_pending_response:
                self.delete_pending_response(request_id)
            return response_waiter
        return None 
        
    def has_request_pending_response(self, request_id):
        return request_id in self._pending_responses

    def delete_pending_response(self, request_id):
        del self._pending_responses[request_id]

    def _service_function(self, request, other):
        request_id = self._get_request_id()
        response_waiter = ResponseWaiter()
        self._create_pending_response(request_id, response_waiter)
        
        esm = self._create_esm(request_id, request)
        self._send_esm(esm)

        response_waiter.wait_for_response()
        response_bytes = response_waiter.get_response()
        response = deserialize_message(response_bytes,
                                       self._response_message_type)
        return response

    def _get_request_id(self):
        with self._lock:
            request_id = self._next_request_id
            self._increment_next_request_id()
        return request_id

    def _increment_next_request_id(self):
        self._next_request_id += 1

    def _create_pending_response(self, request_id, response_waiter):
        self._pending_responses[request_id] = response_waiter

    def _create_esm(self, request_id, request):
        routing_info = self._create_routing_info()
        metadata = self._create_esm_metadata(request_id=request_id)
        data = self._serialize_ros_message(request)
        esm = EgressSystemMessage(routing_info, metadata, data)
        return esm

    def _create_routing_info(self):
        routing_data = SystemMessageRoutingData(
            agent_type=self._service_agent_type,
            destination_agents=[self._service_agent_id])
        routing_info = SystemMessageRoutingInfo(routing_data=routing_data)
        return routing_info
