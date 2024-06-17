from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber


class TargetRenderer(RegionsSubscriber):
	""" This Node listens to all the messages published on the topics related to targets and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_target", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		RtBiInterfaces.subscribeToTargets(self, self.enqueueUpdate)

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for regionId in self.targetRegions:
			polys = self.targetRegions[regionId]
			for poly in polys:
				Ros.ConcatMessageArray(markers, poly.createMarkers(durationNs=-1, stamped=False))
		return markers

	def onTargetUpdated(self, _) -> None:
		self.log("Targets updated.")
		self.render()
		return

	def onMapUpdated(self, _) -> None:
		return

	def onSensorUpdated(self, _) -> None:
		return

def main(args=None) -> None:
	return TargetRenderer.Main(args)

if __name__ == "__main__":
	main()
