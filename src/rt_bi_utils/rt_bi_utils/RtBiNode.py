from abc import ABCMeta, abstractmethod

from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from rt_bi_utils.Ros import SetLogger


class RtBiNode(Node, metaclass=ABCMeta):
	def __init__(self, loggingLevel: LoggingSeverity = LoggingSeverity.DEBUG, **kwArgs):
		newKw = { "node_name": "rt_bi_util_node_base", **kwArgs}
		super().__init__(**newKw)
		SetLogger(self.get_logger())
		self.__loggingLevel: LoggingSeverity = loggingLevel
		self.log("%s is initializing." % self.get_fully_qualified_name())

	def log(self, msg: str) -> bool:
		"""Shorthand for `self.get_logger().log(msg, self.__loggingLevel)`"""
		return self.get_logger().log(msg, self.__loggingLevel)

	@abstractmethod
	def declareParameters(self) -> None: ...

	@abstractmethod
	def parseConfigFileParameters(self) -> None: ...

	@abstractmethod
	def render(self) -> None: ...
