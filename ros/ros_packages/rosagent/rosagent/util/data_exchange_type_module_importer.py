import importlib

from rosutility.logging.logger import Logger

def get_message_type_class(message_type):
    return get_data_exchange_type_class(message_type, "msg")

def get_service_type_class(message_type):
    return get_data_exchange_type_class(message_type, "srv")

def get_data_exchange_type_class(message_type, identifier):
    package_name, class_name = split_message_type(message_type)
    module_resource_path = package_name + '.' + identifier
    try:
        module = importlib.import_module(module_resource_path)
        class_ = getattr(module, class_name)
    except ModuleNotFoundError:
        log_message = f"Data exchange type module '{module}' was not found."
        Logger().log_warning(log_message)
        class_ = None
    except AttributeError:
        log_message = f"Data exchange type class '{class_name}' was not found."
        Logger().log_warning(log_message)
        class_ = None
    return class_

def split_message_type(message_type):
    split_message_type = message_type.split("/")
    assert len(split_message_type) == 2
    package_name = split_message_type[0]
    class_name = split_message_type[1]
    return package_name, class_name
