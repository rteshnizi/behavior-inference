import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class TargetEmulator(AffineRegionEmulator):
	def __init__(self):
		newKw = { "node_name": "emulator_target", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(**newKw)
		(self.__publisher, _) = RtBiInterfaces.createTargetPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		msgArr = self.asRegularSpaceArrayMsg(Msgs.RtBi.RegularSet.TARGET)
		self.__publisher.publish(msgArr)

def main(args=None):
	rclpy.init(args=args)
	targetNode = TargetEmulator()
	rclpy.spin(targetNode)
	targetNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
