from collections.abc import Callable
from typing import Tuple, TypeVar, Union

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node, Publisher, Subscription, Timer


class RosUtils:
	"""
	Reusable utility types and functions for ROS2
	"""

	Topic = TypeVar("Topic")

	__LOGGER: Union[RcutilsLogger, None] = None
	@property
	def Logger() -> RcutilsLogger:
		if RosUtils.__LOGGER is None:
			raise RuntimeError("How is this possible?")
		return RosUtils.__LOGGER

	@staticmethod
	def CreatePublisher(
			node: Node,
			topic: Topic,
			topicName: str,
			callbackFunc: Callable,
			interval: int
		) -> Tuple[Publisher, Timer]:
		"""
		Create and return the tuple of `(Publisher, Timer)`.

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
			The interval in seconds between each published topic

		Returns
		-------
		`Tuple[Publisher, Timer]`
		"""
		publisher = node.create_publisher(topic, topicName, 10)
		timer = node.create_timer(interval, callbackFunc)
		return (publisher, timer)

	@staticmethod
	def CreateSubscriber(
			node: Node,
			topic: Topic,
			topicName: str,
			callbackFunc: Callable[[Topic], None],
		) -> Subscription:
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
		return node.create_subscription(topic, topicName, callbackFunc, 10)
