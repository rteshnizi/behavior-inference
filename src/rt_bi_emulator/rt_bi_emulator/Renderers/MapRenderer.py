from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber
from rt_bi_core.Spatial import MapPolygon


class MapRenderer(RegionsSubscriber):
	""" This Node listens to all static and dynamic map region updates and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_map", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		RtBiInterfaces.subscribeToProjectiveMap(self, self.enqueueUpdate)
		RtBiInterfaces.subscribeToAffineMap(self, self.enqueueUpdate)

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

	def onSensorUpdated(self, _) -> None:
		return

	def onTargetUpdated(self, _) -> None:
		return

def main(args=None) -> None:
	return MapRenderer.Main(args)

if __name__ == "__main__":
	main()
