import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.RegionsSubscriber import TargetSubscriber
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_emulator.Emulators.AffineRegionEmulator import AffineRegionEmulator


class SensorEmulator(AffineRegionEmulator, TargetSubscriber):
	def __init__(self):
		newKw = { "node_name": "emulator_sensor", "loggingSeverity": LoggingSeverity.INFO }
		super().__init__(**newKw)
		self.__observedTargetIds = set()
		(self.__regionPublisher, _) = RtBiInterfaces.createSensorPublisher(self, self.__publishUpdate, self.updateInterval)
		(self.__estPublisher, _) = RtBiInterfaces.createEstimationPublisher(self)

	def __publishUpdate(self) -> None:
		msg = self.asRegularSpaceArrayMsg()
		self.__regionPublisher.publish(msg)

	def onTargetsUpdated(self) -> None:
		for targetId in self.targets:
			region = self.targets[targetId]
			if len(region) > 1: raise RuntimeError(f"We do not expect a target region with more than one member.")
			target = region[region.regionIds[0]]
			trackletMsg = Msgs.RtBi.Tracklet(spawned=False, vanished=False)
			trackletMsg.id = target.id
			trackletMsg.pose = Msgs.toStdPose(target.interior.centroid)
			if GeometryLib.intersects(target.interior, self.getRegionAtTime(target.timeNanoSecs)):
				estMsg = Msgs.RtBi.Estimation()
				estMsg.robot = self.asRegularSpaceMsg()
				if target.id not in self.__observedTargetIds:
					self.log(f"TG {target.id} spawned.")
					trackletMsg.spawned = True
					self.__observedTargetIds.add(target.id)
				Ros.AppendMessage(estMsg.estimations, trackletMsg)
				self.__estPublisher.publish(estMsg)
				return
		return

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	avNode = SensorEmulator()
	rclpy.spin(avNode)
	avNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
