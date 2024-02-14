import rclpy
from rclpy.clock import Duration
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.msg import ColdStart


class RuntimeManager(RtBiNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "runtime_mgr", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.__awaitingColdStart: list[str] = []
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
			self.log(f"Sending cold start for {nodeName}.")
			msg = ColdStart(node_name=nodeName, done=False)
			self.__coldStartPublisher.publish(msg)
		return

	def __onColdStartDone(self, msg: ColdStart) -> None:
		if msg.done:
			self.log(f"Cold start done for {msg.node_name}.")
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
	mgr = RuntimeManager()
	rclpy.spin(mgr)
	mgr.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
