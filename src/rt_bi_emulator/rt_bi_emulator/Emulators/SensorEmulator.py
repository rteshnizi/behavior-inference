import rclpy
from rclpy.logging import LoggingSeverity
from sa_msgs.msg import RobotState

import rt_bi_commons.Utils.Ros as RosUtils
from rt_bi_commons.Shared.Pose import Pose
from rt_bi_commons.Utils.Geometry import Geometry, Polygon
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.SaMsgs import SaMsgs
from rt_bi_emulator.Shared.Body import Body
from rt_bi_emulator.Shared.DynamicRegionNode import DynamicRegionNode
from rt_bi_interfaces.msg import DynamicRegion, EstimationMsg, PoseEstimation


class Av(Body):
	def __repr__(self) -> str:
		return super().__repr__().replace("BD", "AV")

class AvEmulator(DynamicRegionNode):
	def __init__(self):
		super().__init__(loggingSeverity=LoggingSeverity.DEBUG, node_name="em_av")
		self.__targetIds = set()
		(self.__regionPublisher, _) = RtBiInterfaces.createSensorPublisher(self, self.__publishUpdate, self.updateInterval)
		(self.__estPublisher, _) = RtBiInterfaces.createEstimationPublisher(self)
		RtBiInterfaces.subscribeToTarget(self, self.__onTargetUpdate)

	def __publishUpdate(self) -> None:
		msgDy = self.createDynamicRegionMsg()
		self.__regionPublisher.publish(msgDy)

	def __onTargetUpdate(self, msg: DynamicRegion) -> None:
		if msg is None:
			self.get_logger().warn("Received empty Target message!")
			return

		timeNanoSecs = self.get_clock().now().nanoseconds
		coordsList = RtBiInterfaces.fromStdPoints32ToCoordsList(msg.region.points)
		cor = RtBiInterfaces.fromStdPointToCoords(msg.center_of_rotation)
		angleRad = Geometry.quatToAngle((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
		target = Body(msg.id, Pose(timeNanoSecs, msg.pose.position.x, msg.pose.position.y, angleRad), cor, coordsList)
		targetLocation = Polygon(coordsList)
		fov = self.getRegionAtTime(timeNanoSecs)
		if Geometry.intersects(targetLocation, fov):
			estMsg = EstimationMsg()
			estMsg.detection_time = float(timeNanoSecs)
			robotStateMsg = RobotState()
			robotStateMsg.robot_id = self.id
			pose = self.getPoseAtTime(timeNanoSecs)
			robotStateMsg.pose = SaMsgs.createSaPoseMsg(pose.x, pose.y)
			robotStateMsg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
			estMsg.robot_state = robotStateMsg
			poseEstMsg = PoseEstimation(spawned=False, vanished=False)
			if target.id not in self.__targetIds:
				self.log("TG#%d spawned." % msg.id)
				poseEstMsg.spawned = True
				self.__targetIds.add(target.id)
			poseEstMsg.trajectory_id = target.id
			poseEstMsg.pose = SaMsgs.createSaPoseMsg(target.location.x, target.location.y, target.location.angleFromX)
			RosUtils.AppendMessage(estMsg.pose_estimations, poseEstMsg)
			self.__estPublisher.publish(estMsg)
			return
		if target.id in self.__targetIds:
			self.log("TG#%d vanished." % msg.id)
			estMsg = EstimationMsg()
			estMsg.detection_time = float(timeNanoSecs)
			robotStateMsg = RobotState()
			robotStateMsg.robot_id = self.id
			pose = self.getPoseAtTime(timeNanoSecs)
			robotStateMsg.pose = SaMsgs.createSaPoseMsg(pose.x, pose.y)
			robotStateMsg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
			estMsg.robot_state = robotStateMsg
			poseEstMsg = PoseEstimation(spawned=False, vanished=False)
			poseEstMsg.vanished = True
			self.__targetIds.remove(target.id)
			poseEstMsg.trajectory_id = target.id
			poseEstMsg.pose = SaMsgs.createSaPoseMsg(target.location.x, target.location.y, target.location.angleFromX)
			RosUtils.AppendMessage(estMsg.pose_estimations, poseEstMsg)
			self.__estPublisher.publish(estMsg)
			return
		return

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	avNode = AvEmulator()
	rclpy.spin(avNode)
	avNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
