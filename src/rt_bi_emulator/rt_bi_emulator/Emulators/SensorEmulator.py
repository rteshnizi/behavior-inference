from typing import Literal

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.TargetPolygon import TargetPolygon
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class SensorEmulator(AffineRegionEmulator[SensingPolygon], RegionsSubscriber):
	def __init__(self):
		newKw = { "node_name": "emulator_sensor", "loggingSeverity": Ros.LoggingSeverity.INFO }
		super().__init__(SensingPolygon, **newKw)
		self.__observedTargets: dict[str, Literal["entered", "exited", "stayed"]] = {}
		(self.__locationPublisher, _) = RtBiInterfaces.createSensorPublisher(self, self.__publishUpdateNow, self.updateInterval)
		RtBiInterfaces.subscribeToTargets(self, self.enqueueUpdate)

	def __emulateEstimation(self, sensor: SensingPolygon, target: TargetPolygon) -> None:
		targetPt = GeometryLib.toPoint(target.centroid)
		targetId = target.id.regionId
		if GeometryLib.intersects(targetPt, sensor.interior):
			if targetId not in self.__observedTargets:
				self.__observedTargets[targetId] = "entered"
		else:
			if targetId in self.__observedTargets:
				self.__observedTargets[targetId] = "exited"
		return

	def __publishUpdate(self, sensor: SensingPolygon) -> None:
		updateMsg = sensor.asRegularSetMsg()
		targetIds = list(self.__observedTargets.keys())
		for targetId in targetIds:
			target = self.targetRegions[targetId][-1]
			entered = self.__observedTargets[targetId] == "entered"
			exited = self.__observedTargets[targetId] == "exited"
			trackletMsg = Msgs.RtBi.Tracklet(entered=entered, exited=exited)
			trackletMsg.id = targetId
			trackletMsg.pose = Msgs.toPoseMsg(target.centroid)
			Ros.AppendMessage(updateMsg.estimations, trackletMsg)
			# Now that we published the enter event, change it so it doesn't affect next updates
			if entered: self.__observedTargets[targetId] = "stayed"
			# Now that we published targets, remove the ones who exited
			if exited: self.__observedTargets.pop(targetId, None)
		arr = Msgs.RtBi.RegularSetArray()
		arr.sets = [updateMsg]
		Ros.Publish(self.__locationPublisher, arr)
		return

	def __publishUpdateNow(self) -> None:
		timeNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		sensor = self.getRegionAtTime(timeNanoSecs)
		for targetId in self.targetRegions:
			self.__emulateEstimation(sensor, self.targetRegions[targetId][-1])
		self.__publishUpdate(sensor)

	def onMapUpdated(self, _) -> None:
		return

	def onSensorUpdated(self, _) -> None:
		return

	def onTargetUpdated(self, target: TargetPolygon) -> None:
		# FIXME: It would be more accurate to do this BUT
		# when I did it, EXIT events went missing on the eventifier side (They are rendered)
		# This probably means, the event is not sent as an update properly
		# The target position is updated via region subscriber regardless.
		# sensor = self.getRegionAtTime(target.timeNanoSecs)
		# self.__emulateEstimation(sensor, target)
		# self.__publishUpdate(sensor)
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		raise AssertionError("Emulators do not render")

def main(args=None) -> None:
	return SensorEmulator.Main(args)

if __name__ == "__main__":
	main()
