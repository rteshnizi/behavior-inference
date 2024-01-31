from abc import ABC, abstractmethod
from typing import cast

from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.TimeInterval import TimeInterval
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.MapRegion import MapRegion
from rt_bi_interfaces.msg import MapRegion as MapRegionMsg, MapRegions as MapRegionsMsg


class MapRegionsSubscriber(RtBiNode, ABC):
	""" This Node listens to all the messages published on the topics related to the map and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "map_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.mapRegions: list[MapRegion] = []
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName("map"))
		RtBiInterfaces.subscribeToMapRegions(self, self.parseMapRegions)

	def parseMapRegions(self, msg: MapRegionsMsg) -> None:
		for regionMsg in msg.regions:
			regionMsg = cast(MapRegionMsg, regionMsg)
			region = MapRegion(
				idNum=Ros.RegisterRegionId(regionMsg.id),
				envelope=RtBiInterfaces.fromStdPoints32ToCoordsList(regionMsg.region.points),
				envelopeColor=ColorNames.fromString(regionMsg.spec.color),
				offIntervals=[TimeInterval.fromMsg(interval) for interval in regionMsg.spec.off_intervals]
			)
			self.mapRegions.append(region)
		self.onMapUpdated()
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseParameters(self) -> None:
		return super().parseParameters()

	def render(self) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.log("Skipping map render... RViz is not ready yet to receive messages.")
			return
		if len(self.mapRegions) == 0:
			self.log("Skipping map render... No regions to render.")
			return
		message = MarkerArray()
		for region in self.mapRegions:
			Ros.ConcatMessageArray(message.markers, region.render())
		self.__rvizPublisher.publish(message)
		return

	@abstractmethod
	def onMapUpdated(self) -> None: ...
