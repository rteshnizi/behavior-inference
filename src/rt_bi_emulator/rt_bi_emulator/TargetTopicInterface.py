from typing import Dict, List, Union

import rclpy
from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.TargetRegion import TargetRegion
from rt_bi_interfaces.msg import DynamicRegion
from rt_bi_utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_utils.RtBiNode import RtBiNode
from rt_bi_utils.RViz import ColorNames, RViz


class TargetTopicInterface(RtBiNode):
	""" This Node listens to all the messages published on the topics related to targets and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "rt_bi_emulator_target", **kwArgs}
		super().__init__(loggingSeverity=LoggingSeverity.INFO, **newKw)
		self.__targets: Union[Dict[int, TargetRegion], None] = None
		RtBiInterfaces.subscribeToTargetTopic(self, self.__onTargetUpdate)
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))
		self.__renderColor = ColorNames.ORANGE

	def __onTargetUpdate(self, update: DynamicRegion) -> None:
		if update is None:
			self.log("Skipping None update.")
			return
		if self.__targets is None:
			self.__targets = {}

		timeNanoSecs = self.get_clock().now().nanoseconds
		self.log(f"Updating region type {TargetRegion.RegionType.TARGET} id {update.id} definition @{timeNanoSecs}.")
		coords = RtBiInterfaces.fromStdPoints32ToCoordsList(update.region.points) # type: ignore
		cor = RtBiInterfaces.fromStdPointToCoords(update.center_of_rotation)
		target = TargetRegion(centerOfRotation=cor, idNum=update.id, envelope=coords, envelopeColor=self.__renderColor, timeNanoSecs=timeNanoSecs)
		self.__targets[update.id] = target
		self.render([target])
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseConfigFileParameters(self) -> None:
		return super().parseConfigFileParameters()

	def render(self, regions: List[TargetRegion]) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.log("Skipping map render... RViz is not ready yet to receive messages.")
			return
		message = MarkerArray()
		for region in regions:
			RosUtils.ConcatMessageArray(message.markers, region.render())
		self.__rvizPublisher.publish(message)
		return

def main(args=None):
	rclpy.init(args=args)
	node = TargetTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
