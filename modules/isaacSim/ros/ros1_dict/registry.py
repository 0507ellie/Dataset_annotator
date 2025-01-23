import json
import collections
import dataclasses
import rospy  # ROS1
import numpy as np

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

	
def dictify(msg):
	"""
	Parse the `data` field of a ROS message into a Python dictionary.

	Args:
		msg: A ROS message with a `data` field containing JSON content.

	Returns:
		dict: A Python dictionary parsed from the JSON data. If parsing fails,
			  logs the error and returns an empty dictionary.

	Example:
		ros_message = std_msgs.msg.String(data='{"key": "value"}')
		parsed_dict = dictify(ros_message)
		# parsed_dict = {"key": "value"}
	"""
	data = ''
	try:
		data = json.loads(msg.data)
	except json.JSONDecodeError as e:
		rospy.logerr(f"Failed to parse JSON: {e}")
	return data
   
def msgify(msg_type, dict_obj):
	"""
	Convert a dictionary into a JSON string and embed it into a ROS message.

	Args:
		msg_type (str): The ROS message type (e.g., "std_msgs/String").
		dict_obj (dict): A Python dictionary to serialize and convert.

	Returns:
		ROS message: A ROS message populated with the serialized dictionary.

	Example:
		msg_type = "std_msgs/String"
		dict_obj = {"field1": "value1", "field2": "value2"}
		ros_message = msgify(msg_type, dict_obj)
	"""
	data = json.dumps(dict_obj, cls=NpEncoder)
	return converter.dict_to_ros_message(msg_type, {"data": data})
