import rclpy

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import TargetSubscriber
from rt_bi_core.Spatial import TargetPolygon


class TargetRenderer(TargetSubscriber):
	""" This Node listens to all the messages published on the topics related to targets and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_target", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(pauseQueuingMsgs=False, **newKw)

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for regionId in self.targetRegions:
			polys = self.targetRegions[regionId]
			for poly in polys:
				Ros.ConcatMessageArray(markers, poly.createMarkers(durationNs=-1, stamped=False))
		return markers

	def onPolygonUpdated(self, rType: TargetPolygon.Type, polygon: TargetPolygon) -> None:
		self.log("Targets updated.")
		self.render()
		return

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

def main(args=None):
	rclpy.init(args=args)
	node = TargetRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
