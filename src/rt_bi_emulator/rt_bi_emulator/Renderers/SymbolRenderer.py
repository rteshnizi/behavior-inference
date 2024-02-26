from typing import Any, Dict, List, Union

import rclpy
from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

import rt_bi_commons.Utils.Ros as RosUtils
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.SymbolRegion import SymbolRegion
from rt_bi_interfaces.msg import DynamicRegion


class SymbolRenderer(RtBiNode):
	""" This Node listens to all the messages published on the topics related to dynamic symbols and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_symbol", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__regions: Union[Dict[int, SymbolRegion], None] = None
		RtBiInterfaces.subscribeToSymbol(self, self.__onTargetUpdate)
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))

	def __onTargetUpdate(self, update: DynamicRegion) -> None:
		if update is None:
			self.log("Skipping None update.")
			return
		if self.__regions is None: self.__regions = {}

		timeNanoSecs = self.get_clock().now().nanoseconds
		self.log(f"Updating region type {SymbolRegion.RegionType.SYMBOL} id {update.id} definition @{timeNanoSecs}.")
		coords = RtBiInterfaces.fromStdPoints32ToCoordsList(update.region.points)
		cor = RtBiInterfaces.fromStdPointToCoords(update.center_of_rotation)
		target = SymbolRegion(centerOfRotation=cor, id=update.id, envelope=coords, timeNanoSecs=timeNanoSecs, overlappingRegionId=0, overlappingRegionType=SymbolRegion.RegionType.SYMBOL)
		self.__regions[update.id] = target
		self.render([target])
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseParameters(self) -> None:
		return super().parseParameters()

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
	node = SymbolRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
