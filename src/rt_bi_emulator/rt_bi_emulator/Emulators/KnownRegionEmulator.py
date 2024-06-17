from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class KnownRegionEmulator(AffineRegionEmulator):
	""" This class provides a ROS node which emulates a *known* moving region. """
	def __init__(self):
		newKw = { "node_name": "emulator_map_region", "loggingSeverity": Ros.LoggingSeverity.INFO }
		super().__init__(AffinePolygon, **newKw)
		(self.__publisher, _) = RtBiInterfaces.createKnownRegionPublisher(self, self.publishUpdate, self.updateInterval)

	def publishUpdate(self) -> None:
		timeNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		arr = Msgs.RtBi.RegularSetArray()
		arr.sets = [self.asRegularSpaceMsg(timeNanoSecs)]
		Ros.Publish(self.__publisher, arr)
		return

def main(args=None) -> None:
	return KnownRegionEmulator.Main(args)

if __name__ == "__main__":
	main()
