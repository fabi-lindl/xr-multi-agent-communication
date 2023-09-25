from rosagent.buffers import egress_buffer as EgressBuffer
from rosagent.logging.rosagent_logger import RosagentLogger as Logger
from rosagent.message.egress_system_message_buffer import EgressSystemMessageBuffer

from rosutility.message.serializer import deserializer as Deserializer

class ManagementMessageManager:
    
    def __init__(self):
        self._logger = Logger()

    def _send_esm(self, esm):
        esm_buffer = EgressSystemMessageBuffer(size=1)
        esm_buffer.enqueue(esm)
        EgressBuffer.enqueue(esm_buffer)

    def _deserialize_management_message_data(self, ism):
        data = ism.get_data()
        management_data = Deserializer.bytes_to_dict(
            data, create_snake_case_keys=True)
        return management_data
