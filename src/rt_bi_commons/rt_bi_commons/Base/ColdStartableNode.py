from abc import ABC, abstractmethod
from json import dumps, loads
from typing import Any, Callable, Literal, TypeVar, final, overload

from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.msg import ColdStart

_K = Literal["nodeName", "done", "phase", "predicates", "dynamic", "affine"]
# "node_name" is not set in cold start communications since it is in the ROS msg,
# "node_name" is added here for the RDF node's benefit.
class ColdStartPayload(dict[_K, Any]):
	__V = TypeVar("__V")
	@overload
	def __init__(self, payload: dict[_K, Any]) -> None: ...
	@overload
	def __init__(self, payload: str) -> None: ...
	@overload
	def __init__(self, payload: "ColdStartPayload") -> None: ...

	def __init__(self, payload: Any) -> None:
		if isinstance(payload, str):
			if payload == "":
				super().__init__()
				return
			try:
				d: dict[_K, Any] = loads(payload)
				super().__init__(d)
				return
			except Exception as e:
				Ros.Log(f"Exception occurred while parsing json payload string \"{payload}\"")
				raise e
		if isinstance(payload, dict):
			super().__init__(payload)
			return

	@property
	def nodeName(self) -> str:
		return self.get("nodeName", "")

	@property
	def done(self) -> bool:
		return self.get("done", False)

	@property
	def phase(self) -> int:
		return self.get("phase", 0)

	@property
	def predicates(self) -> set[str]:
		return self.__getListAsSet("predicates", str)

	@property
	def dynamic(self) -> set[str]:
		return self.__getListAsSet("dynamic", str)

	@property
	def affine(self) -> set[str]:
		return self.__getListAsSet("affine", str)

	def stringify(self) -> str:
		try:
			return dumps(self)
		except Exception as e:
			Ros.Log(f"Exception occurred while serializing json payload object: \"{repr(self)}\"")
			raise e

	def __getListAsSet(self, var: _K, _: type[__V]) -> set[__V]:
		val = self.get(var, None)
		if val is None: return set()
		return set(val)

class ColdStartableNode(RtBiNode, ABC):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "cold_startable_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__coldStartPublisher = RtBiInterfaces.createColdStartPublisher(self)
		self.__coldStartPermitted = False
		self.__coldStartPayload = ColdStartPayload({})
		RtBiInterfaces.subscribeToColdStart(self, self.__onColdStartPermission)
		return

	@abstractmethod
	def onColdStartAllowed(self, payload: ColdStartPayload) -> None: ...

	@final
	def __onColdStartPermission(self, msg: ColdStart) -> None:
		if msg.node_name != self.get_fully_qualified_name(): return
		payload = ColdStartPayload(msg.json_payload)
		if payload.done: return
		self.__coldStartPayload = payload
		self.log(f"Received cold start for {msg.node_name} ")
		self.__coldStartPermitted = True
		return

	@final
	def waitForColdStartPermission(self, coldStartFunc: Callable[[ColdStartPayload], None]) -> None:
		while not self.__coldStartPermitted: Ros.Wait(self, 0.1)
		coldStartFunc(self.__coldStartPayload)
		return

	@final
	def coldStartCompleted(self, payload: dict[_K, Any] | ColdStartPayload) -> None:
		payload = ColdStartPayload(payload)
		coldStartAck = ColdStart(node_name=self.get_fully_qualified_name(), json_payload=payload.stringify())
		self.__coldStartPublisher.publish(coldStartAck)
		self.log(f"Announcing cold start completion for {self.get_fully_qualified_name()}.")
		return
