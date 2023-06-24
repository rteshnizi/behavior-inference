from math import isnan, nan
from typing import Callable, Tuple, TypeVar, Union

import rclpy
from builtin_interfaces.msg import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node, Publisher, Subscription, Timer

Topic = TypeVar("Topic")

NAMESPACE = "rt_bi_core"
__LOGGER: Union[RcutilsLogger, None] = None

def Logger() -> RcutilsLogger:
	global __LOGGER
	if __LOGGER is None:
		context = rclpy.get_default_context()
		if not context.ok():
			raise RuntimeError("ROS context not OK!")
		if not context._logging_initialized:
			raise RuntimeError("Logging not initialized!")
		rosNodes = rclpy.get_global_executor().get_nodes()
		if len(rosNodes) == 0:
			raise RuntimeError("No ROS nodes added to executor!")
		__LOGGER = rosNodes[0].get_logger()
	return __LOGGER

def RosTimeStamp() -> Time:
	context = rclpy.get_default_context()
	if not context.ok():
		raise RuntimeError("ROS context not OK!")
	return rclpy.get_global_executor()._clock.now().to_msg()

def CreatePublisher(node: Node, topic: Topic, topicName: str, callbackFunc: Callable = lambda x: x, interval: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
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
	timer = None if isnan(interval) else node.create_timer(interval, callbackFunc)
	try:
		freq = " @ %.2fHz" % 1 / interval
	except:
		freq = ""
	node.get_logger().info("Node %s publishes topic \"%s\"%s" % (node.get_name(), topicName, freq))
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
	node.get_logger().info("Node %s subscribed to \"%s\"" % (node.get_name(), topicName))
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
