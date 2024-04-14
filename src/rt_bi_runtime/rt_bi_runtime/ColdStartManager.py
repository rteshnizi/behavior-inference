import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartPayload
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces


class ColdStartManager(RtBiNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "cs_mgr", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__awaitingColdStart: list[str] = [
			# The order in this list is significant
			"/rt_bi_runtime/ba1",
			"/rt_bi_emulator/dynamic_map",
		]
		self.__allPredicates: set[str] = set()
		self.__coldStartPublisher = RtBiInterfaces.createColdStartPublisher(self)
		self.__coldStartPhase = 0
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
				payload = ColdStartPayload({ "phase": self.__coldStartPhase })
			elif nodeName.startswith(RtBiInterfaces.KNOWN_REGION_NODE_PREFIX):
				# TODO: fetch and assign predicates
				payload = ColdStartPayload({ "phase": self.__coldStartPhase })
			else: # Dynamic Map Cold Start
				payload = ColdStartPayload({
					"phase": self.__coldStartPhase,
					"predicates": list(self.__allPredicates),
				})
			msg = Msgs.RtBi.ColdStart(node_name=nodeName, json_payload=payload.stringify())
			self.__coldStartPublisher.publish(msg)
		return

	def __onColdStartDone(self, msg: Msgs.RtBi.ColdStart) -> None:
		if msg.json_payload:
			payload = ColdStartPayload(msg.json_payload)
			self.log(msg.json_payload)
			if not payload.done: return
			self.log(f"Cold start done for node {msg.node_name}.")
			if msg.node_name.startswith(RtBiInterfaces.BA_NODE_PREFIX):
				self.__coldStartPhase = 0
				self.__allPredicates |= payload.predicates
			if msg.node_name.startswith(RtBiInterfaces.KNOWN_REGION_NODE_PREFIX):
				self.__coldStartPhase = 0
				pass # TODO:
			else: # Dynamic Map
				pass
				# if payload.phase == 0:
				# 	pass # Fetch dynamic region data so that the map can trigger reachability events
				# 	self.__coldStartPhase = 1
				# elif payload.phase == 1:
				# 	self.__coldStartPhase = 0
			self.__triggerNextColdStart()
		return

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
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
