import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import TargetSubscriber
from rt_bi_core.Spatial import TargetPolygon
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class SensorEmulator(AffineRegionEmulator, TargetSubscriber):
	def __init__(self):
		newKw = { "node_name": "emulator_sensor", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(**newKw)
		self.__observedTargetIds = set()
		(self.__regionPublisher, _) = RtBiInterfaces.createSensorPublisher(self, self.__publishUpdate, self.updateInterval)
		(self.__estPublisher, _) = RtBiInterfaces.createEstimationPublisher(self)

	def __publishUpdate(self) -> None:
		msg = self.asRegularSpaceArrayMsg(Msgs.RtBi.RegularSet.SENSING)
		self.__regionPublisher.publish(msg)

	def onPolygonUpdated(self, polygon: TargetPolygon) -> None:
		for regionId in self.targetRegions:
			targetPolySeq = self.targetRegions[regionId]
			target = targetPolySeq[-1] # take the latest
			trackletMsg = Msgs.RtBi.Tracklet(entered=False, exited=False)
			trackletMsg.id = regionId
			trackletMsg.pose = Msgs.toStdPose(target.centroid)
			if GeometryLib.intersects(target.interior, self.getRegionAtTime(target.timeNanoSecs).interior):
				estMsg = Msgs.RtBi.Estimation()
				estMsg.sensor = self.asRegularSpaceMsg(Msgs.RtBi.RegularSet.SENSING)
				if target.id.sansTime() not in self.__observedTargetIds:
					self.log(f"TG {target.id} entered.")
					trackletMsg.entered = True
					self.__observedTargetIds.add(target.id.sansTime())
				Ros.AppendMessage(estMsg.estimations, trackletMsg)
				self.__estPublisher.publish(estMsg)
				return
			else:
				if target.id.sansTime() in self.__observedTargetIds:
					self.log(f"TG {target.id} exited.")
					estMsg = Msgs.RtBi.Estimation()
					estMsg.sensor = self.asRegularSpaceMsg(Msgs.RtBi.RegularSet.SENSING)
					trackletMsg.exited = True
					self.__observedTargetIds.remove(target.id.sansTime())
					Ros.AppendMessage(estMsg.estimations, trackletMsg)
					self.__estPublisher.publish(estMsg)
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		raise AssertionError("Emulators do not render")

def main(args=None):
	rclpy.init(args=args)
	avNode = SensorEmulator()
	rclpy.spin(avNode)
	avNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
