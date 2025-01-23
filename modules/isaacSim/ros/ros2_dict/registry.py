import json
import dataclasses
import numpy as np
import rclpy # ROS2

from . import converter

class NpEncoder(json.JSONEncoder):
    """
    A JSON encoder that handles NumPy data types, dataclasses, and nested objects.
    
    Features:
        - Converts NumPy integer and floating-point numbers to Python native types.
        - Converts NumPy arrays to lists.
        - Converts dataclass instances to dictionaries.
    """
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif dataclasses.is_dataclass(obj):
            return dataclasses.asdict(obj)
        return super(NpEncoder, self).default(obj)



def convert_ros_message_to_json(message):
    """
    Convert a ROS message to a JSON-formatted string.

    Args:
        message: A ROS message instance.

    Returns:
        str: A JSON-formatted string representing the ROS message.

    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        json_message = convert_ros_message_to_json(ros_message)
        # json_message = '{"data": "Hello, Robot"}'
    """
    dictionary = converter.ros_message_to_dict(message)
    json_message = json.dumps(dictionary)
    return json_message


def dictify(message):
    """
    Parse the `data` field of a ROS message into a Python dictionary.

    Args:
        message: A ROS message containing a `data` field with JSON content.

    Returns:
        dict: A Python dictionary parsed from the `data` field. 
              Returns an empty dictionary if parsing fails.

    Example:
        ros_message = std_msgs.msg.String(data='{"key": "value"}')
        parsed_dict = dictify(ros_message)
        # parsed_dict = {"key": "value"}
    """
    data = ''
    try:
        data = json.loads(message.data)
    except json.JSONDecodeError as e:
        # Log the error or handle it (e.g., return an empty dictionary).
        rclpy.logerr(f"Failed to parse JSON: {e}")
    return data

def convert_json_to_ros_message(message_type, json_message, strict_mode=True):
    """
    Convert a JSON-formatted string to a ROS message.

    Args:
        message_type (str): The type of the ROS message (e.g., "std_msgs/String").
        json_message (str): The JSON-formatted string to be converted.
        strict_mode (bool): If True, throws an exception for extra fields in the JSON.

    Returns:
        ROS message: An instance of the specified ROS message type.

    Example:
        message_type = "std_msgs/String"
        json_message = '{"data": "Hello, Robot"}'
        ros_message = convert_json_to_ros_message(message_type, json_message)
    """
    dictionary = json.loads(json_message)
    return converter.dict_to_ros_message(message_type, dictionary, strict_mode=strict_mode)


def msgify(message_type, dict_obj, strict_mode=True):
    """
    Convert a dictionary to a JSON string and embed it in a ROS message.

    Args:
        message_type (str): The type of the ROS message (e.g., "std_msgs/String").
        dict_obj (dict): A dictionary containing the data to be converted.
        strict_mode (bool): If True, ensures strict mapping of fields.

    Returns:
        ROS message: A ROS message with the serialized dictionary data.

    Example:
        message_type = "std_msgs/String"
        dict_obj = {"field1": "value1", "field2": "value2"}
        ros_message = msgify(message_type, dict_obj)
    """
    data = json.dumps(dict_obj, cls=NpEncoder)
    return converter.dict_to_ros_message(message_type, {"data": data}, strict_mode=strict_mode)
