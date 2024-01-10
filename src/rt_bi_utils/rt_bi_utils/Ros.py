import logging
from functools import partial
from math import isnan, nan
from typing import AbstractSet, Any, Callable, Dict, List, Sequence, Tuple, TypeVar, Union

import rclpy
from builtin_interfaces.msg import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import LoggingSeverity
from rclpy.node import Client, Node, Publisher, Subscription, Timer

Topic = TypeVar("Topic")
QueryRequest = TypeVar("QueryRequest")

NAMESPACE = "rt_bi_core"
__LOGGER: Union[RcutilsLogger, None] = None
# In case of failure to obtain a ROS logger, the default Python logger will be used, which most likely will log to std stream.
logging.basicConfig(format="[+][%(levelname)s]: %(message)s", force=True)
__rtBiLog: Callable[[str], bool] = lambda m: False
__strNameToIdNum: Dict[str, int] = dict()
"""This map helps us assume nothing about the meaning of the names assigned to any regions."""
NANO_CONVERSION_CONSTANT = 10 ** 9


def SetLogger(logger: RcutilsLogger, defaultSeverity: LoggingSeverity) -> None:
	global __LOGGER
	global __rtBiLog
	__LOGGER = logger
	__rtBiLog = partial(__LOGGER.log, severity=defaultSeverity)
	return

def Logger() -> Union[RcutilsLogger, logging.Logger]:
	global __LOGGER
	if __LOGGER is None:
		context = rclpy.get_default_context()
		if not context.ok():
			logging.error("ROS context not OK!")
			return logging.getLogger()
		if not context._logging_initialized:
			logging.error("Logging not initialized!")
			return logging.getLogger()
		rosNodes = rclpy.get_global_executor().get_nodes()
		if len(rosNodes) == 0:
			logging.warn("No ROS nodes added to executor! Defaulting to python logger.")
			return logging.getLogger()
		__LOGGER = rosNodes[0].get_logger()
	return __LOGGER

def Log(msg: str) -> bool:
	global __rtBiLog
	return __rtBiLog(msg)

def RosTimeStamp() -> Time:
	context = rclpy.get_default_context()
	if not context.ok():
		raise RuntimeError("ROS context not OK!")
	return rclpy.get_global_executor()._clock.now().to_msg()

def TimeToInt(t: Time) -> int:
	return ((t.sec * NANO_CONVERSION_CONSTANT) + t.nanosec) # CSpell: ignore nanosec

def CreatePublisher(node: Node, topic: Topic, topicName: str, callbackFunc: Callable[[Topic], None] = lambda _: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
	"""
	Create and return the tuple of `(Publisher, Timer | None)`.

	Parameters
	----------
	`node : Node`
		The node who publishes the topic
	`topic : Topic`
		The data structure of the topic
	`topicName : str`
		The string name of the topic
	`callbackFunc : Callable[[], None]`
		The function callback that would be called to publish the topic
	`interval : int`
		The interval in seconds between each time the topic is published.
		If `nan` (not an number) is given, topics must be published manually.

	Returns
	-------
	`Tuple[Publisher, Union[Timer, None]]`
	"""
	publisher = node.create_publisher(topic, topicName, 10)
	timer = None if isnan(intervalSecs) else node.create_timer(intervalSecs, callbackFunc)
	try:
		freq = " @ %.2fHz" % (1 / intervalSecs)
	except:
		freq = ""
	node.get_logger().debug(f"{node.get_fully_qualified_name()} publishes topic \"{topicName}\"{freq}")
	return (publisher, timer)

def CreateSubscriber(node: Node, topic: Topic, topicName: str, callbackFunc: Callable[[Topic], None]) -> Subscription:
	"""
	Create and return the `Subscription`.

	Parameters
	----------
	`node : Node`
		The node who publishes the topic
	`topic : Topic`
		The data structure of the topic
	`topicName : str`
		The string name of the topic
	`callbackFunc : Callable[[], None]`
		The function callback that would be called when the topic is received

	Returns
	-------
	`Subscription`
	"""
	subscription = node.create_subscription(topic, topicName, callbackFunc, 10)
	node.get_logger().debug("%s subscribed to \"%s\"" % (node.get_fully_qualified_name(), topicName))
	return subscription

def CreateTopicName(shortTopicName: str) -> str:
	"""
	Given the short version of the topic name, this function produces the topic name including the namespace etc.

	Parameters
	----------
	shortTopicName : str
		The short name of the publisher's topic.

		For example, `rviz`.

	Returns
	-------
	str
		The full topic name to be used when creating the publisher/subscriber.

		For example, given `rviz` the returned topic would be `/namespace_prefix/rviz`.
	"""
	return "/%s/%s" % (NAMESPACE, shortTopicName)

def SendClientRequest(node: Node, client: Client, request: QueryRequest, responseCallback: Callable[[QueryRequest, Any], Any]) -> None:
	future = client.call_async(request)
	rclpy.spin_until_future_complete(node, future)
	responseCallback(request, future.result())
	# FIXME: Add try catch for error handling of service request and return true or false for indication
	return None

def WaitForServicesToStart(node: Node, client: Union[Client, None] = None) -> None:
	clientList = node.clients if client is None else [client]
	for client in clientList:
		while not client.wait_for_service():
			node.get_logger().debug("Client %s is waiting for service %s" % (node.get_name(), client.srv_name))
	node.get_logger().debug("Client %s is ready"% node.get_name())
	return

def AppendMessage(array: Union[Sequence[Topic], AbstractSet[Topic], List[Topic]], msg: Topic) -> None:
	"""Appends a message to a message array.

	Parameters
	----------
	array : Union[Sequence, AbstractSet, UserList]
		The array.
	msg : Any
		The message.
	"""
	assert isinstance(array, List), ("Failed to append messages to array. Array type: %s" % type(array))
	array.append(msg)

def ConcatMessageArray(array: Union[Sequence[Topic], AbstractSet[Topic], List[Topic]], toConcat: Sequence[Topic]) -> List[Topic]:
	"""Concatenates an array of messages to another message array.

	Parameters
	----------
	array : Union[Sequence, AbstractSet, UserList]
		The array.
	msg : Any
		The message.
	"""
	assert isinstance(array, List), ("Failed to append messages to array. Array type: %s" % type(array))
	array += toConcat
	return array

def RegisterRegionId(featureName: str) -> int:
	if featureName in __strNameToIdNum:
		idNum = __strNameToIdNum[featureName]
		Logger().debug("Duplicate region name encountered: %s... same id assigned %d." % (featureName, idNum))
		return __strNameToIdNum[featureName]
	idNum = len(__strNameToIdNum)
	__strNameToIdNum[featureName] = idNum
	return idNum

def CreateTimer(node: Node, callback: Callable, intervalNs = 1000) -> Timer:
	return Timer(callback, None, intervalNs, node.get_clock(), context=node.context)
