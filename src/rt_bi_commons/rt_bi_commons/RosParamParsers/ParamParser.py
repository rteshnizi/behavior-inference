from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Generic, Iterator, Literal, TypeVar

from rclpy.parameter import Parameter, ParameterValue
from typing_extensions import LiteralString

from rt_bi_commons.Base.RtBiNode import RtBiNode

_ParsedItemType = TypeVar("_ParsedItemType")
_RosParamType = TypeVar("_RosParamType", bound=Parameter.Type)
_YamlParamName = TypeVar("_YamlParamName", bound=LiteralString)

@dataclass
class YamlParamDef(Generic[_YamlParamName, _RosParamType]):
	name: _YamlParamName
	rosType: _RosParamType

class ParserBase(Generic[_YamlParamName, _RosParamType, _ParsedItemType], ABC):
	def __init__(self, node: RtBiNode, paramName: _YamlParamName, paramType: _RosParamType) -> None:
		self.node: RtBiNode = node
		self.paramDef: YamlParamDef[_YamlParamName, _RosParamType] = YamlParamDef[_YamlParamName, _RosParamType](name=paramName, rosType=paramType)
		self.__value: _ParsedItemType | None
		return

	@staticmethod
	def isListParam(p: Parameter.Type) -> bool:
		if p == Parameter.Type.INTEGER_ARRAY: return True
		if p == Parameter.Type.STRING_ARRAY: return True
		if p == Parameter.Type.DOUBLE_ARRAY: return True
		if p == Parameter.Type.BOOL_ARRAY: return True
		if p == Parameter.Type.BYTE_ARRAY: return True
		return False

	@property
	def value(self) -> _ParsedItemType | None:
		return self.__value

	@value.setter
	def value(self, val: _ParsedItemType | None) -> None:
		self.__value = val

	@property
	def isListParser(self) -> bool:
		return ParserBase.isListParam(self.paramDef.rosType)

	def declareParameters(self) -> None:
		self.node.declare_parameter(self.paramDef.name, self.paramDef.rosType)
		return

	def parseParameters(self) -> None:
		paramVal = self.node.get_parameter(self.paramDef.name).get_parameter_value()
		self._parseImpl(paramVal, self.paramDef)
		return

	@abstractmethod
	def _parseImpl(self, value: ParameterValue, paramDef: YamlParamDef) -> _ParsedItemType: ...

	@abstractmethod
	def stringifyValue(self, val: _ParsedItemType) -> str: ...

_ListRosParamType = TypeVar("_ListRosParamType",
	Literal[Parameter.Type.INTEGER_ARRAY],
	Literal[Parameter.Type.STRING_ARRAY],
	Literal[Parameter.Type.DOUBLE_ARRAY],
	Literal[Parameter.Type.BOOL_ARRAY],
	Literal[Parameter.Type.BYTE_ARRAY],
)
class ListParserBase(
	Generic[_YamlParamName, _ListRosParamType, _ParsedItemType],
	ParserBase[_YamlParamName, _ListRosParamType, list[_ParsedItemType]],
	ABC
):
	def __init__(self, node: RtBiNode, paramName: _YamlParamName, paramType: _ListRosParamType) -> None:
		super().__init__(node, paramName, paramType)

	def __len__(self) -> int:
		if self.value is None: raise RuntimeError("item is not parsed.")
		return len(self.value)

	def __getitem__(self, ind: int) -> _ParsedItemType:
		if self.value is None: raise RuntimeError("item is not parsed.")
		return self.value[ind]

	def __setitem__(self, ind: int, val: _ParsedItemType) -> None:
		if self.value is None: raise RuntimeError("item is not parsed.")
		self.value[ind] = val
		return

	def __iter__(self) -> Iterator[_ParsedItemType]:
		if self.value is None: raise RuntimeError("item is not parsed.")
		return iter(self.value)

	@abstractmethod
	def stringifyValue(self, val: _ParsedItemType) -> str: ...
