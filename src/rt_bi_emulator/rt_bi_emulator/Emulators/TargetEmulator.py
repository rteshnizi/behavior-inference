import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_emulator.Shared.DynamicRegionNode import DynamicRegionNode


class TargetEmulator(DynamicRegionNode):
	def __init__(self):
		super().__init__(loggingSeverity=LoggingSeverity.DEBUG, node_name="em_target")
		(self.__publisher, _) = RtBiInterfaces.createTargetPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		msgDy = self.createDynamicRegionMsg()
		self.__publisher.publish(msgDy)

def main(args=None):
	rclpy.init(args=args)
	targetNode = TargetEmulator()
	rclpy.spin(targetNode)
	targetNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
