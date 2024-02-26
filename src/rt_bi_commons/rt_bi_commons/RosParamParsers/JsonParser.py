import json
from typing import Any, Generic, Literal, TypeVar, cast

from rclpy.parameter import Parameter, ParameterValue
from typing_extensions import LiteralString

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.RosParamParsers.ParamParser import ListParserBase, YamlParamDef

IntermediateDataType = TypeVar("IntermediateDataType")
ParsedItemType = TypeVar("ParsedItemType")
ParsedJsonType = TypeVar("ParsedJsonType")
YamlParamName = TypeVar("YamlParamName", bound=LiteralString)

class ListJsonParser(
	Generic[YamlParamName, ParsedItemType, ParsedJsonType, IntermediateDataType],
	ListParserBase[YamlParamName, Literal[Parameter.Type.STRING_ARRAY], ParsedItemType],
):
	def __init__(self, node: RtBiNode, paramName: YamlParamName) -> None:
		self.__parsedJsonList: list[ParsedJsonType]
		super().__init__(node, paramName, Parameter.Type.STRING_ARRAY)
		return

	def convertToValue(self, intermediateVal: IntermediateDataType) -> ParsedItemType:
		return cast(ParsedItemType, intermediateVal)

	def parseJsonResults(self, item: ParsedJsonType) -> IntermediateDataType:
		return cast(IntermediateDataType, item)

	def _parseImpl(self, value: ParameterValue, _: YamlParamDef) -> None:
		self.__parsedJsonList = [json.loads(jsonStr) for jsonStr in value.string_array_value]
		self.value = cast(list[ParsedItemType], [None] * len(self.__parsedJsonList))
		for i in range(len(self.__parsedJsonList)):
			parsedJson = self.__parsedJsonList[i]
			intermediateData = self.parseJsonResults(parsedJson)
			self.value[i] = self.convertToValue(intermediateData)
		return

	def stringifyValue(self, val: ParsedItemType) -> str:
		return json.dumps(val)
