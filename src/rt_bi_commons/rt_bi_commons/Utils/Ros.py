"""This module must not import any `rt_bi` modules."""
import csv
import datetime
import logging
import os
from functools import partial
from math import inf, isnan, nan
from pathlib import Path
from typing import AbstractSet, Any, Callable, Iterable, Sequence, TypeAlias, TypeVar, cast

import rclpy
from rclpy.clock import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import LoggingSeverity
from rclpy.node import Client, Node, Publisher, Service, Subscription, Timer

__Topic = TypeVar("__Topic")

NAMESPACE = "REZA_TESHNIZI_NS"
__LOGGER: RcutilsLogger | None = None
__DEFAULT_NODE: Node
logging.basicConfig(format="[+][%(levelname)s]: %(message)s", force=True)
__rtBiLog: Callable[[str], bool] = lambda m: False

Array: TypeAlias = Sequence[__Topic] | AbstractSet[__Topic] | list[__Topic]

def IsProfiling() -> bool:
	return os.getenv("RT_BI_PROFILE") == "true"

def SetLogger(node: Node, logger: RcutilsLogger, defaultSeverity: LoggingSeverity, isProfiling: bool) -> None:
	global __LOGGER
	global __DEFAULT_NODE
	global __rtBiLog
	__LOGGER = logger
	__DEFAULT_NODE = node
	__rtBiLog = partial(__LOGGER.log, severity=defaultSeverity)
	if isProfiling: os.environ["RT_BI_PROFILE"] = "true"
	return

def Logger() -> RcutilsLogger | logging.Logger:
	"""
	The logger object.

	:return: The default ROS client Logger object.
	This is most likely the logger that belong to the node that owns the process.
	In case of failure to obtain a ROS logger, the default Python logger will be used,
	which most likely will log to std stream.
	:rtype: `RcutilsLogger` or `logging.Logger`
	"""
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

def Log(msg: str, l: Iterable | None = None, indentStr = "\t\t", severity: LoggingSeverity | None = None) -> bool:
	global __IS_PROFILING
	if IsProfiling(): return True
	if l is not None:
		sep = f"\n{indentStr}"
		if isinstance(l, str): l = [l]
		else: l = [m if isinstance(m, str) else repr(m) for m in l]
		indentedStr = sep.join(l)
		msg = f"{msg}:{sep}{indentedStr}"
	if severity is None:
		global __rtBiLog
		return __rtBiLog(msg)
	else:
		__logger = Logger()
		if isinstance(__logger, RcutilsLogger): return __logger.log(msg, severity)
		__logger.log(severity, msg)
	return False

def Now(node: Node | None) -> Time:
	if node is not None:
		return node.get_clock().now()

	context = rclpy.get_default_context()
	if not context.ok():
		raise RuntimeError("ROS context not OK!")
	return rclpy.get_global_executor()._clock.now()

def CreatePublisher(node: Node, topic: type[__Topic], topicName: str, callbackFunc: Callable = lambda _: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
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
		freq = f" @ {(1 / intervalSecs):.2f}Hz"
	except:
		freq = ""
	node.get_logger().debug(f"{node.get_fully_qualified_name()} publishes topic \"{topicName}\"{freq}")
	return (publisher, timer)

def CreateSubscriber(node: Node, topic: type[__Topic], topicName: str, callbackFunc: Callable[[__Topic], None]) -> Subscription:
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
	node.get_logger().debug(f"{node.get_fully_qualified_name()} subscribed to \"{topicName}\"")
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
	return f"/{NAMESPACE}/{shortTopicName}"

def AsList(array: Sequence[__Topic] | AbstractSet[__Topic] | list[__Topic], t: type[__Topic]) -> list[__Topic]:
	assert isinstance(array, list), (f"Failed to cast messages to array. Array type: {type(array)}")
	return array

def PopMessage(array: Sequence[__Topic] | AbstractSet[__Topic] | list[__Topic], i: int, t: type[__Topic]) -> __Topic:
	assert isinstance(array, list), (f"Failed to append messages to array. Array type: {type(array)}")
	return array.pop(i)

def AppendMessage(array: Sequence[__Topic] | AbstractSet[__Topic] | list[__Topic], msg: __Topic) -> None:
	"""Appends a message to a message array.

	Parameters
	----------
	array : Union[Sequence, AbstractSet, UserList]
		The array.
	msg : Any
		The message.
	"""
	assert isinstance(array, list), (f"Failed to append messages to array. Array type: {type(array)}")
	array.append(msg)

def GetMessage(array: Sequence[__Topic] | AbstractSet[__Topic] | list[__Topic], i: int, t: type[__Topic]) -> __Topic:
	assert isinstance(array, list), (f"Failed to append messages to array. Array type: {type(array)}")
	return array[i]

def ConcatMessageArray(first: Sequence[__Topic] | AbstractSet[__Topic] | list[__Topic], second: Sequence[__Topic]) -> None:
	"""Creates a new array by concatenating first to second.
	This ensures the type-checker error is managed from ROS arrays.

	:param first: The first part of the new array.
	:type array: `Sequence[__Topic]` or `AbstractSet[__Topic]` or `list[__Topic]`
	:param toConcat: The second part of the new array.
	:type toConcat: `Sequence[__Topic]`
	"""
	assert isinstance(first, list), (f"Failed to append messages to array. Array type: {type(first)}")
	first += second
	return

def CreateTimer(node: Node, callback: Callable, intervalNs = 1000) -> Timer:
	return Timer(callback, None, intervalNs, node.get_clock(), context=node.context)

__ServiceInterface_Request = TypeVar("__ServiceInterface_Request")
__ServiceInterface_Response = TypeVar("__ServiceInterface_Response")
__ServiceInterface = TypeVar("__ServiceInterface")

def CreateService(node: Node, interface: __ServiceInterface, serviceName: str, callbackFunc: Callable[[__ServiceInterface_Request, __ServiceInterface_Response], __ServiceInterface_Response]) -> Service: # pyright: ignore[reportInvalidTypeVarUse]
	l = list(filter(lambda s: s.srv_name == serviceName, node.services))
	if len(l) == 0: return node.create_service(interface, serviceName, callbackFunc)
	if len(l) == 1: return l[0]
	raise RuntimeError("This should never happen.")

def CreateClient(node: Node, interface: __ServiceInterface, serviceName: str) -> Client: # pyright: ignore[reportInvalidTypeVarUse]
	l = list(filter(lambda s: s.srv_name == serviceName, node.clients))
	if len(l) == 0: return node.create_client(interface, serviceName)
	if len(l) == 1: return l[0]
	raise RuntimeError("This should never happen.")

def SendClientRequest(node: Node, client: Client, request: __ServiceInterface_Request, responseCallback: Callable[[__ServiceInterface_Request, __ServiceInterface_Response], __ServiceInterface_Response] | None = None) -> None:
	if responseCallback is None: responseCallback = lambda _1, _2: _2
	future = client.call_async(request)
	rclpy.spin_until_future_complete(node, future)
	responseCallback(request, cast(__ServiceInterface_Response, future.result()))
	return None

def GetSubscriberNames(node: Node, topic: str) -> set[str]:
	subs = node.get_subscriptions_info_by_topic(topic)
	return {f"{s.node_namespace}/{s.node_name}" for s in subs}

def Wait(node: Node, timeoutSec: float = inf) -> None:
	sleepTime = 0.05
	while node.context.ok() and timeoutSec > 0.0:
		rclpy.spin_once(node, timeout_sec=sleepTime)
		timeoutSec -= sleepTime
	return

def WaitForSubscriber(node: Node, topic: str, subscriberFullName: str) -> None:
	subsName = GetSubscriberNames(node, topic)
	node.log(f"Publisher {node.get_name()} is waiting for subscriber {subscriberFullName}.") # pyright: ignore[reportAttributeAccessIssue]
	while subscriberFullName not in subsName:
		Wait(node, 1.0)
		subsName = GetSubscriberNames(node, topic)
	return

def WaitForServiceToStart(node: Node, client: Client) -> None:
	node.log(f"Client {node.get_name()} is waiting for service {client.srv_name}.") # pyright: ignore[reportAttributeAccessIssue]
	while not client.wait_for_service(): pass
	return

ReductionStats: list[tuple[int, int, int]] = []
"""[(`time`, `poly verts`, `graph V+E`)]"""

MessageStats: dict[str, list[tuple[int, int]]] = {}
"""`topic`: [(`timeNS`, `count`)]"""

def Publish(publisher: Publisher, msg: Any) -> None:
	if publisher.topic not in MessageStats:
		MessageStats[publisher.topic] = []
		counter = 1
	else:
		counter = MessageStats[publisher.topic][-1][1] + 1
	global __DEFAULT_NODE
	MessageStats[publisher.topic].append((Now(__DEFAULT_NODE).nanoseconds, counter))
	publisher.publish(msg)
	return

def LogMessageStats() -> None:
	if len(MessageStats) == 0: return
	now = datetime.datetime.now()
	date = now.strftime("%Y-%m-%d--%H-%M-%S")
	dir = f"/home/reza/git/behavior-inference/log-rtbi/stats/{date}"
	for topic in MessageStats:
		if len(MessageStats[topic]) == 0: continue
		fileName = f"{topic}.csv"
		filePath = Path(f"{dir}{fileName}").absolute()
		filePath.parent.mkdir(parents=True, exist_ok=True)
		# Write the dictionary to a CSV file
		with open(filePath, mode="w", newline="") as file:
			writer = csv.writer(file)
			for stat in MessageStats[topic]:
				writer.writerow(stat)
	return

def RecordReductionStats(timeNS: int, numVerts: int, numGVE: int) -> None:
	ReductionStats.append((timeNS, numVerts, numGVE))
	return

def LogReductionStats() -> None:
	if len(MessageStats) == 0: return
	now = datetime.datetime.now()
	date = now.strftime("%Y-%m-%d--%H-%M-%S")
	dir = f"/home/reza/git/behavior-inference/log-rtbi/stats/{date}"
	fileName = "reduction.csv"
	filePath = Path(f"{dir}/{fileName}").absolute()
	filePath.parent.mkdir(parents=True, exist_ok=True)
	# Write the dictionary to a CSV file
	with open(filePath, mode="w", newline="") as file:
		writer = csv.writer(file)
		for stat in ReductionStats:
			writer.writerow(stat)
	return
