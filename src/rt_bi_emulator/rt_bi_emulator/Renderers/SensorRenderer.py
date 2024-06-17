from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber


class SensorRenderer(RegionsSubscriber):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_sensor", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		RtBiInterfaces.subscribeToSensors(self, self.enqueueUpdate)

	def onSensorUpdated(self, _) -> None:
		self.log("Sensors updated.")
		self.render()
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for regionId in self.sensorRegions:
			polys = self.sensorRegions[regionId]
			for poly in polys:
				marker = poly.createMarkers(durationNs=-1, stamped=False, renderTracklet=False)
				Ros.ConcatMessageArray(markers, marker)
		return markers

	def onMapUpdated(self, _) -> None:
		return

	def onTargetUpdated(self, _) -> None:
		return

def main(args=None) -> None:
	return SensorRenderer.Main(args)

if __name__ == "__main__":
	main()
