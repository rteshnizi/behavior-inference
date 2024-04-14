import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class KnownRegionEmulator(AffineRegionEmulator):
	""" This class provides a ROS node which emulates a *known* moving region. """
	def __init__(self):
		newKw = { "node_name": "emulator_map_region", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(AffinePolygon, **newKw)
		(self.__publisher, _) = RtBiInterfaces.createKnownRegionPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		timeNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		arr = Msgs.RtBi.RegularSetArray()
		arr.sets = [self.asRegularSpaceMsg(timeNanoSecs)]
		self.__publisher.publish(arr)
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
