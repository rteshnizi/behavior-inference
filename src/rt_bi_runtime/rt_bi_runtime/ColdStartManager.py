from rt_bi_commons.Base.ColdStartableNode import ColdStartPayload
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces


class ColdStartManager(RtBiNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "cs_mgr", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__awaitingColdStart: list[str] = [
			# The order in this list is significant
			"/rt_bi_behavior/ba1",
			"/rt_bi_emulator/dynamic_map",
			"/rt_bi_eventifier/eventifier",
		]
		self.__predicates: set[str] = set()
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
				payload = ColdStartPayload({})
			elif nodeName.startswith(RtBiInterfaces.KNOWN_REGION_NODE_PREFIX):
				# TODO: fetch predicates
				payload = ColdStartPayload({})
			elif nodeName.startswith("/rt_bi_emulator/dynamic_map"):
				payload = ColdStartPayload({
					"predicates": list(self.__predicates),
				})
			elif nodeName.startswith("/rt_bi_eventifier/eventifier"):
				# FIXME: Tell it which affine regions to subscribe to
				# This is the right thing to do, but RegionSubscriber and ColdStartableNode have conflicting publishers.
				payload = ColdStartPayload({})
				pass
			else:
				raise NotImplementedError(f"ColdStart procedure is not implemented for {nodeName}.")
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
				self.__predicates |= payload.predicates
			elif msg.node_name.startswith(RtBiInterfaces.KNOWN_REGION_NODE_PREFIX):
				pass # TODO: assign predicates
			elif msg.node_name.startswith("/rt_bi_emulator/dynamic_map"):
				pass
			elif msg.node_name.startswith("/rt_bi_eventifier/eventifier"):
				pass
			else:
				self.get_logger().error(f"ColdStart completion is not expected for {msg.node_name}.")
			self.__triggerNextColdStart()
		return

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

	def render(self) -> None:
		return super().render()

def main(args=None) -> None:
	return ColdStartManager.Main(args)

if __name__ == "__main__":
	main()
