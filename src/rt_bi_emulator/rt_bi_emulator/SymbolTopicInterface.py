from typing import Any, Dict, List, Union

import rclpy
from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_interfaces.msg import DynamicRegion
from rt_bi_utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_utils.RtBiNode import RtBiNode
from rt_bi_utils.RViz import RViz


class SymbolTopicInterface(RtBiNode):
	""" This Node listens to all the messages published on the topics related to dynamic symbols and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "rt_bi_emulator_isy", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__regions: Union[Dict[int, SymbolRegion], None] = None
		RtBiInterfaces.subscribeToSymbolTopic(self, self.__onTargetUpdate)
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))

	def __onTargetUpdate(self, update: DynamicRegion) -> None:
		if update is None:
			self.log("Skipping None update.")
			return
		if self.__regions is None: self.__regions = {}

		timeNanoSecs = self.get_clock().now().nanoseconds
		self.log(f"Updating region type {SymbolRegion.RegionType.SYMBOL} id {update.id} definition @{timeNanoSecs}.")
		coords = RtBiInterfaces.fromStdPoints32ToCoordsList(update.region.points) # type: ignore
		cor = RtBiInterfaces.fromStdPointToCoords(update.center_of_rotation)
		target = SymbolRegion(centerOfRotation=cor, idNum=update.id, envelope=coords, timeNanoSecs=timeNanoSecs, overlappingRegionId=0, overlappingRegionType=SymbolRegion.RegionType.SYMBOL, index=0)
		self.__regions[update.id] = target
		self.render([target])
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseConfigFileParameters(self) -> None:
		return super().parseConfigFileParameters()

	def render(self, regions: List[Any]) -> None:
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
	node = SymbolTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
