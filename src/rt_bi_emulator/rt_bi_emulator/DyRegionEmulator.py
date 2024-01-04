import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_emulator.Model.DynamicRegionNode import DynamicRegionBase
from rt_bi_utils.RtBiEmulator import RtBiEmulator


class DynamicRegionEmulator(DynamicRegionBase):
	def __init__(self):
		super().__init__(loggingLevel=LoggingSeverity.DEBUG, node_name="rt_bi_emulator_dy")
		(self.__publisher, _) = RtBiEmulator.createDyRegPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		msg = self.createRobotStateMsg()
		self.__publisher.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	dynRegNode = DynamicRegionEmulator()
	rclpy.spin(dynRegNode)
	dynRegNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
