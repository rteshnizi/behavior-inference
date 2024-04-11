import json
from functools import partial

from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Timer
from rclpy.parameter import Parameter
from typing_extensions import Final, Generic, LiteralString, TypeVar, cast

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.RosParamParsers.JsonParser import ListJsonParser
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import ReferenceDescriptor, RtBiInterfaces
from rt_bi_interfaces.msg import DataRefRequest, DataRefResponse
from rt_bi_interfaces.srv import DataReference

REFERENCES_PARAM_NAME: Final = "param_src"

_V = TypeVar("_V")
_I = TypeVar("_I")
_K = TypeVar("_K", bound=LiteralString)
class ListReferenceParser(
	Generic[_K, _V, _I],
	ListJsonParser[_K, _V, list[int], _I],
):
	def __init__(self, node: RtBiNode, paramName: _K) -> None:
		self.__srcs: list[ReferenceDescriptor] | None
		self.__intermediateData: dict[int, list[_I]] = {}
		self.__remoteRequests: dict[str, DataReference.Request] = {}
		self.__conversionTimer: Timer | None
		super().__init__(node, paramName)
		return

	@property
	def srcs(self) -> list[ReferenceDescriptor]:
		if self.__srcs is None: raise RuntimeError(f"The node {self.node.get_fully_qualified_name()} must set self.__srcs during the parsing of its parameters.")
		return self.__srcs

	@srcs.setter
	def srcs(self, srcStr: str) -> None:
		sParts = srcStr.split(",")
		sParts = [s.strip().lower() for s in sParts]
		self.__srcs = [ReferenceDescriptor.fromRefStr(s) for s in sParts]
		return

	def getSrc(self, ind: int) -> ReferenceDescriptor:
		if len(self.srcs) == 1: return self.srcs[0]
		return self.srcs[ind]

	def resolveLocalRefs(self, srcInd: int, refInd: int) -> int | str | float | bool | bytes:
		if self.value is None: raise RuntimeError("See why value is not set yet.")
		ref = self.getSrc(refInd)
		param = self.node.get_parameter(ref.param).get_parameter_value()
		if param.type == Parameter.Type.INTEGER_ARRAY:
			val = cast(list[int], param.integer_array_value)[refInd]
		elif param.type == Parameter.Type.STRING_ARRAY:
			val = cast(list[str], param.string_array_value)[refInd]
		elif param.type == Parameter.Type.DOUBLE_ARRAY:
			val = cast(list[float], param.double_array_value)[refInd]
		elif param.type == Parameter.Type.BOOL_ARRAY:
			val = cast(list[bool], param.bool_array_value)[refInd]
		elif param.type == Parameter.Type.BYTE_ARRAY:
			val = cast(list[bytes], param.byte_array_value)[refInd]
		elif param.type == Parameter.Type.INTEGER:
			val = cast(int, param.integer_value)
		elif param.type == Parameter.Type.STRING:
			val = cast(str, param.string_value)
		elif param.type == Parameter.Type.DOUBLE:
			val = cast(float, param.double_value)
		elif param.type == Parameter.Type.BOOL:
			val = cast(bool, param.bool_value)
		else:
			raise RuntimeError(f"Unexpected parameter type value: {param.type}")
		return val

	def resolveRemoteResponse(self, req: DataReference.Request, res: DataReference.Response) -> DataReference.Response:
		if req.src_param_name != self.paramDef.name: raise RuntimeError(f"Received response meant for source {req.src_param_name}")
		if self.value is None: raise RuntimeError("See why value is not set yet.")
		self.__remoteRequests.pop(req.service_name)
		for i in range(len(req.references)):
			refReq = Ros.GetMessage(req.references, i, DataRefRequest)
			refRes = Ros.GetMessage(res.resolutions, i, DataRefResponse)
			for j in range(len(refReq.value_indices)):
				valueInd = refReq.value_indices[j]
				strVal = Ros.GetMessage(refRes.values, j, str)
				self.__intermediateData[req.src_param_index][valueInd] = json.loads(strVal)
		if len(self.__remoteRequests) == 0: self.convertToValue(self.__intermediateData[req.src_param_index], req.src_param_index)
		return res

	def convertToValue(self, intermediateVal: list[_I], srcInd: int) -> _V:
		if self.__conversionTimer is not None:
			self.__conversionTimer.destroy()
			self.__conversionTimer = None
		if intermediateVal is None:
			boundFunc = partial(self.convertToValue, intermediateVal, srcInd)
			self.__conversionTimer = Ros.CreateTimer(self.node, boundFunc, 2000)
			return cast(_V, None)
		return cast(_V, intermediateVal)

	def parseJsonResults(self, indices: list[int], srcInd: int) -> list[_I]:
		if self.value is None: raise RuntimeError("See why value is not set yet.")
		if len(self.srcs) != 1 and len(indices) != len(self.srcs):
			raise RuntimeError("Either one source or equal number of sources and indices required.")

		self.__intermediateData[srcInd] = cast(list[_I], [None] * len(indices))
		for valueInd in range(len(indices)):
			refInd = indices[valueInd]
			if self.isLocalRefParam(refInd):
				self.__intermediateData[srcInd][valueInd] = cast(_I, self.resolveLocalRefs(srcInd, refInd))
			else:
				ref = self.getSrc(valueInd)
				if ref.serviceName not in self.__remoteRequests:
					req = DataReference.Request()
					req.service_name = ref.serviceName
					req.src_param_name = self.paramDef.name
					req.src_param_index = srcInd
					self.__remoteRequests[req.service_name] = req
				else:
					req = self.__remoteRequests[ref.serviceName]
				refReq: DataRefRequest | None = None
				for r in self.__remoteRequests[req.service_name].references:
					if r.param_name == ref.param:
						refReq = r
						break
				if refReq is None:
					refReq = DataRefRequest()
					refReq.param_name = ref.param
					Ros.AppendMessage(req.references, refReq)
				Ros.AppendMessage(refReq.ref_indices, refInd)
				Ros.AppendMessage(refReq.value_indices, valueInd)

		if len(self.__remoteRequests) > 0:
			client = RtBiInterfaces.createDataReferenceClient(self.node, self.getSrc(srcInd))
			Ros.WaitForServiceToStart(self.node, client)
			for serviceName in self.__remoteRequests:
				Ros.SendClientRequest(self.node, client, self.__remoteRequests[serviceName], self.resolveRemoteResponse)
		return self.__intermediateData[srcInd]

	def isLocalRefParam(self, ind: int) -> bool:
		ref = self.srcs[0] if len(self.srcs) == 1 else self.srcs[ind]
		if ref.node != self.node.get_name(): return False
		return True

	def declareParameters(self) -> None:
		try:
			self.node.get_parameter(REFERENCES_PARAM_NAME)
		except ParameterNotDeclaredException as e:
			raise ParameterNotDeclaredException(f"Node {self.node.get_fully_qualified_name()} does not appear to be a DataDictionaryNode.")
		except Exception as e:
			raise e
		return super().declareParameters()
