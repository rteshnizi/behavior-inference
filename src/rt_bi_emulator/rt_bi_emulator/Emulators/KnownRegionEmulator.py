import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class KnownRegionEmulator(AffineRegionEmulator):
	""" This class provides a ROS node which emulates a *known* moving region. """
	def __init__(self):
		newKw = { "node_name": "emulator_map_region", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(**newKw)
		(self.__publisher, _) = RtBiInterfaces.createKnownRegionPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		msg = self.asRegularSpaceArrayMsg(Msgs.RtBi.RegularSet.AFFINE)
		self.__publisher.publish(msg)
		return


def main(args=None):
	rclpy.init(args=args)
	node = KnownRegionEmulator()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
