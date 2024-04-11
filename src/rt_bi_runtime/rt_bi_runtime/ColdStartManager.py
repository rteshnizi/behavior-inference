import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from rt_bi_commons.Base.ColdStartableNode import ColdStartPayload
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces


class ColdStartManager(RtBiNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "cs_mgr", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.__awaitingColdStart: list[str] = []
		self.__allPredicates: set[str] = set()
		self.parseParameters()
		self.__coldStartPublisher = RtBiInterfaces.createColdStartPublisher(self)
		RtBiInterfaces.subscribeToColdStart(self, self.__onColdStartDone)
		self.__triggerNextColdStart()
		return

	def __triggerNextColdStart(self) -> None:
		if len(self.__awaitingColdStart) > 0:
			nodeName = self.__awaitingColdStart.pop(0)
			topic = RtBiInterfaces.TopicNames.RT_BI_RUNTIME_COLD_START.value
			Ros.WaitForSubscriber(self, topic, nodeName)
			self.log(f"Sending cold start to node {nodeName}.")
			if nodeName.startswith(RtBiInterfaces.BA_NODE_PREFIX):
				payload = ColdStartPayload({}).stringify()
			elif nodeName.startswith(RtBiInterfaces.KNOWN_REGION_NODE_PREFIX):
				payload = ColdStartPayload({}).stringify()
			else:
				payload = ColdStartPayload({ "predicates": list(self.__allPredicates) }).stringify()
			msg = Msgs.RtBi.ColdStart(node_name=nodeName, json_payload=payload)
			self.__coldStartPublisher.publish(msg)
		return

	def __onColdStartDone(self, msg: Msgs.RtBi.ColdStart) -> None:
		if msg.json_payload:
			payload = ColdStartPayload(msg.json_payload)
			if not payload.done: return
			self.log(f"Cold start done for node {msg.node_name}.")
			if msg.node_name.startswith(RtBiInterfaces.BA_NODE_PREFIX):
				self.__allPredicates |= payload.predicates
			self.__triggerNextColdStart()
		return

	def declareParameters(self) -> None:
		self.log("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("await_cold_start", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__awaitingColdStart = list(self.get_parameter("await_cold_start").get_parameter_value().string_array_value)
		return

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	mgr = ColdStartManager()
	rclpy.spin(mgr)
	mgr.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
