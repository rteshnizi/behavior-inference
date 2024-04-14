from typing import Literal

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import TargetSubscriber
from rt_bi_core.Spatial import TargetPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class SensorEmulator(AffineRegionEmulator[SensingPolygon], TargetSubscriber):
	def __init__(self):
		newKw = { "node_name": "emulator_sensor", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(SensingPolygon, **newKw)
		self.__observedTargets: dict[str, Literal["entered", "exited", "stayed"]] = {}
		(self.__locationPublisher, _) = RtBiInterfaces.createSensorPublisher(self, self.__publishUpdateNow, self.updateInterval)

	def __emulateEstimation(self, sensor: SensingPolygon, target: TargetPolygon) -> None:
		targetPt = GeometryLib.toPoint(target.centroid)
		targetId = target.id.regionId
		if GeometryLib.intersects(targetPt, sensor.interior):
			if targetId not in self.__observedTargets:
				self.__observedTargets[targetId] = "entered"
			else:
				self.__observedTargets[targetId] = "stayed"
		else:
			if targetId in self.__observedTargets:
				self.__observedTargets[targetId] = "exited"
		return

	def __publishUpdate(self, timeNanoSecs: int) -> None:
		sensor = self.getRegionAtTime(timeNanoSecs)
		updateMsg = sensor.asRegularSetMsg()
		for targetId in self.targetRegions:
			self.__emulateEstimation(sensor, self.targetRegions[targetId][-1])
		targetIds = list(self.__observedTargets.keys())
		for targetId in targetIds:
			target = self.targetRegions[targetId][-1]
			entered = self.__observedTargets[targetId] == "entered"
			exited = self.__observedTargets[targetId] == "exited"
			trackletMsg = Msgs.RtBi.Tracklet(entered=entered, exited=exited)
			trackletMsg.id = targetId
			trackletMsg.pose = Msgs.toStdPose(target.centroid)
			Ros.AppendMessage(updateMsg.estimations, trackletMsg)
			# Now that we published targets, remove the ones who exited
			if exited: self.__observedTargets.pop(targetId, None)
		arr = Msgs.RtBi.RegularSetArray()
		arr.sets = [updateMsg]
		self.__locationPublisher.publish(arr)
		return

	def __publishUpdateNow(self) -> None:
		timeNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		self.__publishUpdate(timeNanoSecs)

	def onTargetUpdated(self, target: TargetPolygon) -> None:
		sensor = self.getRegionAtTime(target.timeNanoSecs)
		self.__emulateEstimation(sensor, target)
		self.__publishUpdate(target.timeNanoSecs)

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
