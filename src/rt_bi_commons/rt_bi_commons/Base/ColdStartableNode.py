from abc import ABC
from typing import Callable, final

from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.msg import ColdStart


class ColdStartableNode(RtBiNode, ABC):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "cold_startable_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.coldStartPublisher = RtBiInterfaces.createColdStartPublisher(self)
		self.coldStartPermitted = False
		RtBiInterfaces.subscribeToColdStart(self, self.__onColdStartPermission)
		return

	@final
	def __onColdStartPermission(self, msg: ColdStart) -> None:
		if msg.done or msg.node_name != self.get_fully_qualified_name(): return
		self.log(f"Received cold start for {msg.node_name} ")
		self.coldStartPermitted = True
		return

	@final
	def waitForColdStartPermission(self, coldStartFunc = Callable[[None], None]) -> None:
		while not self.coldStartPermitted: Ros.Wait(self, 0.1)
		coldStartFunc()
		return

	@final
	def coldStartCompleted(self) -> None:
		coldStartAck = ColdStart(node_name=self.get_fully_qualified_name(), done=True)
		self.coldStartPublisher.publish(coldStartAck)
		self.log(f"Cold start completed for {self.get_fully_qualified_name()}.")
		return
