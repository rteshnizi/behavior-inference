import json
from abc import ABC
from typing import Generic, Literal, TypeVar, cast

from rclpy.parameter import Parameter, ParameterValue
from typing_extensions import LiteralString

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.RosParamParsers.ParamParser import ListParserBase, ParserBase, YamlParamDef

_K = TypeVar("_K", bound=LiteralString)
_V = TypeVar("_V", int, str, bool, float, bytes)
_AtomicRosParamType = TypeVar("_AtomicRosParamType",
	Literal[Parameter.Type.INTEGER],
	Literal[Parameter.Type.STRING],
	Literal[Parameter.Type.DOUBLE],
	Literal[Parameter.Type.BOOL]
)
class __AtomicTypeParserBase(Generic[_K, _AtomicRosParamType, _V], ParserBase[_K, _AtomicRosParamType, _V]):
	def __init__(self, node: RtBiNode, paramName: _K, paramType: _AtomicRosParamType) -> None:
		super().__init__(node, paramName, paramType)
		return

	def _parseImpl(self, value: ParameterValue, paramDef: YamlParamDef) -> None:
		if paramDef.rosType == Parameter.Type.INTEGER:
			val = value.integer_value
		elif paramDef.rosType == Parameter.Type.STRING:
			val = value.string_value
		elif paramDef.rosType == Parameter.Type.DOUBLE:
			val = value.double_value
		elif paramDef.rosType == Parameter.Type.BOOL:
			val = value.bool_value
		else:
			raise TypeError(f"Parameter {repr(paramDef)} is not expected.")
		self.value = cast(_V, val)
		return

	def stringifyValue(self, val: _V) -> str:
		return json.dumps(val)

class IntParser(__AtomicTypeParserBase[_K, Literal[Parameter.Type.INTEGER], int]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.INTEGER)

class StrParser(__AtomicTypeParserBase[_K, Literal[Parameter.Type.STRING], str]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.STRING)

class DoubleParser(__AtomicTypeParserBase[_K, Literal[Parameter.Type.DOUBLE], float]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.DOUBLE)

class BoolParser(__AtomicTypeParserBase[_K, Literal[Parameter.Type.BOOL], bool]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.BOOL)

AtomicParser = IntParser[_K] | StrParser[_K] | DoubleParser[_K] | BoolParser[_K]

_ListRosParamType = TypeVar("_ListRosParamType",
	Literal[Parameter.Type.INTEGER_ARRAY],
	Literal[Parameter.Type.STRING_ARRAY],
	Literal[Parameter.Type.DOUBLE_ARRAY],
	Literal[Parameter.Type.BOOL_ARRAY],
	Literal[Parameter.Type.BYTE_ARRAY],
)
class __ListAtomicTypeParserBase(Generic[_K, _ListRosParamType, _V], ListParserBase[_K, _ListRosParamType, _V], ABC):
	def __init__(self, node: RtBiNode, paramName: _K, paramType: _ListRosParamType) -> None:
		super().__init__(node, paramName, paramType)

	def _parseImpl(self, value: ParameterValue, paramDef: YamlParamDef) -> None:
		if paramDef.rosType == Parameter.Type.INTEGER_ARRAY:
			val = value.integer_array_value
		elif paramDef.rosType == Parameter.Type.STRING_ARRAY:
			val = value.string_array_value
		elif paramDef.rosType == Parameter.Type.DOUBLE_ARRAY:
			val = value.double_array_value
		elif paramDef.rosType == Parameter.Type.BOOL_ARRAY:
			val = value.bool_array_value
		elif paramDef.rosType == Parameter.Type.BYTE_ARRAY:
			val = value.byte_array_value
		else:
			raise TypeError(f"Parameter {repr(paramDef)} is not expected.")
		self.value = cast(list[_V], val)
		return

	def stringifyValue(self, val: _V) -> str:
		return json.dumps(val)

class ListIntParser(Generic[_K], __ListAtomicTypeParserBase[_K, Literal[Parameter.Type.INTEGER_ARRAY], int]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.INTEGER_ARRAY)

class ListStrParser(Generic[_K], __ListAtomicTypeParserBase[_K, Literal[Parameter.Type.STRING_ARRAY], str]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.STRING_ARRAY)

class ListDoubleParser(Generic[_K], __ListAtomicTypeParserBase[_K, Literal[Parameter.Type.DOUBLE_ARRAY], float]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.DOUBLE_ARRAY)

class ListBoolParser(Generic[_K], __ListAtomicTypeParserBase[_K, Literal[Parameter.Type.BOOL_ARRAY], bool]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.BOOL_ARRAY)

class ListByteParser(Generic[_K], __ListAtomicTypeParserBase[_K, Literal[Parameter.Type.BYTE_ARRAY], bytes]):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		super().__init__(node, paramName, Parameter.Type.BYTE_ARRAY)

AtomicListParser =\
	ListIntParser[_K] |\
	ListStrParser[_K] |\
	ListDoubleParser[_K] |\
	ListBoolParser[_K] |\
	ListByteParser[_K]
