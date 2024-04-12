import json
from abc import ABC

from rclpy.exceptions import ParameterUninitializedException
from rclpy.logging import LoggingSeverity
from rclpy.node import Service
from rclpy.parameter import Parameter
from typing_extensions import Any, Generic, Iterator, LiteralString, TypeVar, cast, final

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.RosParamParsers.ParamParser import ParserBase
from rt_bi_commons.RosParamParsers.ReferenceParser import REFERENCES_PARAM_NAME, ListReferenceParser
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.msg import DataRefRequest, DataRefResponse
from rt_bi_interfaces.srv import DataReference

_K = TypeVar("_K", bound=LiteralString)
_P = TypeVar("_P", bound=ParserBase)

class DataDictionaryNode(Generic[_K], RtBiNode, ABC):
	"""
	### Note
	This class was made to make ROS nodes act like RDF resources.
	After we used RDF, the reference parsing became irrelevant.
	However, the code is good, so I am keeping it.
	I also noticed that ROS may have this as built-in feature.

	(＾皿＾)
	### Description
	The parameters of an instance of this ROS node are available through ROS services.
	"""
	def __init__(self, parsers: dict[_K, ParserBase[_K, Any, Any]], **kwArgs) -> None:
		newKw = { "node_name": "dd_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__srcs: dict[_K, str]
		self.parsers = parsers
		self.__declareRefSources()
		self.declareParameters()
		self.parseParameters()
		self.ddServices: dict[_K, Service] = {}
		for paramName in self.parsers:
			self.ddServices[paramName] = RtBiInterfaces.createDataReferenceService(self, paramName, self.resolveRequest)
		return

	def __getitem__(self, k: _K) -> list:
		if self.isListParam(k): return cast(list, self.parsers[k].value)
		return [self.parsers[k].value]

	@final
	def __iter__(self) -> Iterator[str]:
		return iter(self.parsers)

	def getParser(self, p: _K, castTo: type[_P]) -> _P:
		if self.parsers[p] is None: raise KeyError(f"Parser not available for param {p}")
		parser = cast(castTo, self.parsers[p])
		return parser

	def __declareRefSources(self) -> None:
		self.declare_parameter(REFERENCES_PARAM_NAME, Parameter.Type.STRING)
		try:
			self.__srcs: dict[_K, str] = json.loads(self.get_parameter(REFERENCES_PARAM_NAME).get_parameter_value().string_value)
		except ParameterUninitializedException as e:
			self.log(f"Node {self.get_fully_qualified_name()} does not have refs.")
			self.__srcs = {}
		except Exception as e:
			raise e
		for p in self.parsers:
			if self.isRefParam(p):
				cast(ListReferenceParser, self.parsers[p]).srcs = self.__srcs[p]
		return

	def isAtomicParam(self, p: _K) -> bool:
		return p not in self.__srcs

	def isListParam(self, p: _K) -> bool:
		return self.parsers[p].isListParser

	def isRefParam(self, p: _K) -> bool:
		return p in self.__srcs

	def have(self, req: DataReference.Request) -> bool:
		if req.service_name != self.get_fully_qualified_name(): return False
		return True

	@final
	def declareParameters(self) -> None:
		for paramName in self.parsers:
			self.parsers[paramName].declareParameters()
		return

	@final
	def parseParameters(self) -> None:
		for paramName in self.parsers:
			self.parsers[paramName].parseParameters()
		return

	def resolveRequest(self, req: DataReference.Request, res: DataReference.Response) -> DataReference.Response:
		for i in range(len(req.references)):
			ref = Ros.GetMessage(req.references, i, DataRefRequest)
			if not self.have(req):
				self.get_logger().warn(f"Ignoring request for {req.service_name}, as it does not belong to {self.get_fully_qualified_name()}")
				Ros.AppendMessage(res.resolutions, DataRefResponse())
			else:
				paramName = cast(_K, ref.param_name)
				vals = self[paramName]
				respMsg = DataRefResponse()
				respMsg.values = [self.parsers[paramName].stringifyValue(vals[ind]) for ind in ref.ref_indices]
				Ros.AppendMessage(res.resolutions, respMsg)
		return res
