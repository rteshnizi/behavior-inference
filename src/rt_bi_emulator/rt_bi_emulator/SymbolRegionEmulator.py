import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_emulator.Model.DynamicRegionNode import DynamicRegionNode
from rt_bi_utils.RtBiInterfaces import RtBiInterfaces


class SymbolRegionEmulator(DynamicRegionNode):
	def __init__(self, loggingLevel: LoggingSeverity, **kwArgs):
		newKw = { "node_name": "rt_bi_emulator_dy", "loggingSeverity": loggingLevel, **kwArgs}
		super().__init__(**newKw)
		(self.__publisher, _) = RtBiInterfaces.createSymbolPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		dyMsg = self.createDynamicRegionMsg()
		self.__publisher.publish(dyMsg)
		return


def main(args=None):
	rclpy.init(args=args)
	dynRegNode = SymbolRegionEmulator(LoggingSeverity.INFO)
	rclpy.spin(dynRegNode)
	dynRegNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
