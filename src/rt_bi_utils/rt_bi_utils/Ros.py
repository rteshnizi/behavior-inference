"""
Reusable utility classes and functions for ROS2
"""
from collections.abc import Callable
from rclpy.node import Node, Publisher, Timer
from typing import TypeVar, Tuple

Topic = TypeVar("Topic")

def CreatePublisher(
		node: Node,
		topic: Topic,
		topicName: str,
		callbackFunc: Callable[[], None],
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
	"""
	publisher = node.create_publisher(topic, topicName, 10)
	timer = node.create_timer(interval, callbackFunc)
	return (publisher, timer)
