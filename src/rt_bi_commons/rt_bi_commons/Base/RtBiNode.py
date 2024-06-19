import cProfile
import datetime
from abc import ABC, abstractmethod
from pathlib import Path
from typing import TypeVar, final

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from rt_bi_commons.Utils import Ros


class RtBiNode(Node, ABC):
	def __init__(self, loggingSeverity: LoggingSeverity = LoggingSeverity.DEBUG, **kwArgs) -> None:
		newKw = { "node_name": "node_base", **kwArgs}
		super().__init__(**newKw)
		self.__defaultLoggingSeverity: LoggingSeverity = loggingSeverity
		self.declare_parameter("render", False)
		self.shouldRender: bool = self.get_parameter("render").get_parameter_value().bool_value
		self.declare_parameter("profile", False)
		self.isProfiling: bool = self.get_parameter("profile").get_parameter_value().bool_value
		Ros.SetLogger(self, self.get_logger(), self.__defaultLoggingSeverity, self.isProfiling)
		self.log("%s is initializing." % self.get_fully_qualified_name())

	def log(self, msg: str) -> bool:
		"""Shorthand for `self.get_logger().log(msg, self.__defaultLoggingSeverity)`"""
		return self.get_logger().log(msg, self.__defaultLoggingSeverity)

	@property
	def defaultServiceName(self) -> str:
		return RtBiNode.toServiceName(self.get_name(), "")

	@staticmethod
	def toServiceName(nodeName: str, param: str) -> str:
		pStr = "" if param == "" else f"/{param}"
		return f"__{nodeName}{pStr}"

	@abstractmethod
	def declareParameters(self) -> None:
		self.log(f"{self.declareParameters.__name__}() has no implementation for {self.get_fully_qualified_name()}")
		return

	@abstractmethod
	def parseParameters(self) -> None:
		self.log(f"{self.parseParameters.__name__}() has no implementation for {self.get_fully_qualified_name()}")
		return

	@final
	def spinNode(self) -> None:
		try:
			if self.isProfiling:
				self.get_logger().warn(f"{self.get_fully_qualified_name()} is collecting profiling statistics.")
				date = datetime.datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
				nodeName = self.get_fully_qualified_name().replace("/", "X")
				outputFile = f"/home/reza/git/behavior-inference/log-rtbi/profiler/{date}/{nodeName}.prof"
				Path(outputFile).parent.mkdir(parents=True, exist_ok=True)
				cProfile.runctx("rclpy.spin(self)",globals(), locals(), outputFile)
			else:
				rclpy.spin(self)
		except KeyboardInterrupt as e:
			pass
		except Exception as e:
			raise e
		return

	@final
	def destroy_node(self) -> None:
		Ros.LogMessageStats()
		return super().destroy_node()

	@abstractmethod
	def render(self) -> None:
		self.log(f"{self.render.__name__}() has no implementation for {self.get_fully_qualified_name()}")
		return

	@classmethod
	def Main(cls, args=None) -> None:
		rclpy.init(args=args)
		node = cls()
		node.spinNode()
		node.destroy_node()
		# rclpy.shutdown()
		return
