import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Spatial.TargetPolygon import TargetPolygon
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class TargetEmulator(AffineRegionEmulator):
	def __init__(self):
		newKw = { "node_name": "emulator_target", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(TargetPolygon, **newKw)
		(self.__publisher, _) = RtBiInterfaces.createTargetPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		timeNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		msgArr = Msgs.RtBi.RegularSetArray()
		msgArr.sets = [self.asRegularSpaceMsg(timeNanoSecs)]
		Ros.Publish(self.__publisher, msgArr)

def main(args=None):
	rclpy.init(args=args)
	node = TargetEmulator()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt as e:
		pass
	except Exception as e:
		raise e
	node.destroy_node()
	# rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
