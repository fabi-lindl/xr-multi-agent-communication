from rosrouter.management.management_message_manager import ManagementMessageManager

class HeartbeatHandler(ManagementMessageManager):

    def __init__(self):
        super().__init__()
    
    def process_ism(self, ism):
        pass # Heart beats do not require any processing.
