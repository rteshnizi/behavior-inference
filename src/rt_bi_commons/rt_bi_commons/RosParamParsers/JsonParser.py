import json
from typing import Any, Generic, Literal, cast

from rclpy.parameter import Parameter, ParameterValue
from typing_extensions import LiteralString, TypeVar

from rt_bi_commons.Base.ParamParserBase import ListParserBase, YamlParamDef
from rt_bi_commons.Base.RtBiNode import RtBiNode

_IntermediateDataType = TypeVar("_IntermediateDataType", default=Any)
_ParsedItemType = TypeVar("_ParsedItemType")
_ParsedJsonType = TypeVar("_ParsedJsonType")
_YamlParamName = TypeVar("_YamlParamName", bound=LiteralString)
class ListJsonParser(
	Generic[_YamlParamName, _ParsedItemType, _ParsedJsonType, _IntermediateDataType],
	ListParserBase[_YamlParamName, Literal[Parameter.Type.STRING_ARRAY], _ParsedItemType],
):
	def __init__(self, node: RtBiNode, paramName: _YamlParamName) -> None:
		self.__parsedJsonList: list[_ParsedJsonType]
		super().__init__(node, paramName, Parameter.Type.STRING_ARRAY)
		return

	def convertToValue(self, intermediateVal: _IntermediateDataType) -> _ParsedItemType:
		return cast(_ParsedItemType, intermediateVal)

	def parseJsonResults(self, item: _ParsedJsonType) -> _IntermediateDataType:
		return cast(_IntermediateDataType, item)

	def _parseImpl(self, value: ParameterValue, _: YamlParamDef) -> None:
		self.__parsedJsonList = [json.loads(jsonStr) for jsonStr in value.string_array_value]
		self.value = cast(list[_ParsedItemType], [None] * len(self.__parsedJsonList))
		for i in range(len(self.__parsedJsonList)):
			parsedJson = self.__parsedJsonList[i]
			intermediateData = self.parseJsonResults(parsedJson)
			self.value[i] = self.convertToValue(intermediateData)
		return

	def stringifyValue(self, val: _ParsedItemType) -> str:
		return json.dumps(val)
