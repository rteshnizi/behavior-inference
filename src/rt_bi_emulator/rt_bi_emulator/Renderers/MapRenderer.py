import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import MapSubscriber
from rt_bi_core.Spatial import MapPolygon


class MapRenderer(MapSubscriber):
	""" This Node listens to all static and dynamic map region updates and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_map", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for regionId in self.mapRegions:
			polys = self.mapRegions[regionId]
			for poly in polys:
				marker = poly.createMarkers(durationNs=-1, stamped=False)
				Ros.ConcatMessageArray(markers, marker)
		return markers

	def onMapUpdated(self, polygon: MapPolygon) -> None:
		self.log(f"Map updated region {repr(polygon.id)}")
		return self.render()

def main(args=None):
	rclpy.init(args=args)
	node = MapRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
