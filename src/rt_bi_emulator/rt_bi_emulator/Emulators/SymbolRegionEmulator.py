import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_emulator.Shared.DynamicRegionNode import DynamicRegionNode


class SymbolRegionEmulator(DynamicRegionNode):
	def __init__(self, **kwArgs):
		newKw = { "node_name": "em_symbol", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		(self.__publisher, _) = RtBiInterfaces.createSymbolPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		dyMsg = self.createDynamicRegionMsg()
		self.__publisher.publish(dyMsg)
		return


def main(args=None):
	rclpy.init(args=args)
	dynRegNode = SymbolRegionEmulator()
	rclpy.spin(dynRegNode)
	dynRegNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
