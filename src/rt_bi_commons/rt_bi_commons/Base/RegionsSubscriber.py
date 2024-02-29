from abc import ABC, abstractmethod
from typing import TypeVar, cast, final

from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Polygons.DynamicPolygon import DynamicPolygon
from rt_bi_core.Polygons.SensingPolygon import SensingPolygon
from rt_bi_core.Polygons.StaticPolygon import StaticPolygon
from rt_bi_core.Polygons.TargetPolygon import TargetPolygon
from rt_bi_core.RegularRegions.DynamicRegion import DynamicRegion
from rt_bi_core.RegularRegions.SpatialRegion import SpatialRegion


class RegionsSubscriberBase(RtBiNode, ABC):
	"""
	This Node provides an API to listen to all the messages about polygonal regions.
	It also creates a  publisher for RViz and a :meth:`~RegionsSubscriberBase.render` method.

	**NOTICE**
	* Subclasses of this class must subscribe to the relevant topics.
	* Subclasses do not need to create a publisher to RViz. Just call :meth:`~RegionsSubscriberBase.render`
	"""
	def __init__(self, **kwArgs):
		newKw = { "node_name": "map_base", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)
		self.mapRegions: dict[str, SpatialRegion[DynamicPolygon | StaticPolygon]] = {}
		self.sensors: dict[str, DynamicRegion[SensingPolygon]] = {}
		self.targets: dict[str, DynamicRegion[TargetPolygon]] = {}
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName("map"))

	def __appendPolygonToRegion(self, regionId: str, poly: DynamicPolygon | StaticPolygon | SensingPolygon | TargetPolygon) -> None:
		match poly.type:
			case StaticPolygon.type | DynamicPolygon.type:
				poly = cast(StaticPolygon | DynamicPolygon, poly)
				if regionId not in self.mapRegions:
					regRegion = SpatialRegion[DynamicPolygon | StaticPolygon]()
					self.mapRegions[regionId] = regRegion
				else:
					regRegion = self.mapRegions[regionId]
				regRegion.addConnectedComponent(poly)
			case SensingPolygon.type:
				poly = cast(SensingPolygon, poly)
				if regionId not in self.sensors:
					regRegion = DynamicRegion[SensingPolygon]()
					self.sensors[regionId] = regRegion
				else:
					regRegion = self.sensors[regionId]
				regRegion.addConnectedComponent(poly)
			case TargetPolygon.type:
				poly = cast(TargetPolygon, poly)
				if regionId not in self.sensors:
					regRegion = DynamicRegion[TargetPolygon]()
					self.targets[regionId] = regRegion
				else:
					regRegion = self.targets[regionId]
				regRegion.addConnectedComponent(poly)
			case _:
				raise RuntimeError(f"Unexpected region type: {poly.type}")
		return

	def __createPolygon(self, rType: DynamicPolygon.Types, region: Msgs.RtBi.RegularSpace, polyMsg: Msgs.RtBi.Polygon) -> DynamicPolygon | StaticPolygon | SensingPolygon | TargetPolygon:
		match rType:
			case StaticPolygon.type:
				poly = StaticPolygon(
					polygonId=polyMsg.id,
					regionId=region.id,
					envelope=Msgs.toCoordsList(polyMsg.region.points),
				)
			case DynamicPolygon.type:
				poly = DynamicPolygon(
					polygonId=polyMsg.id,
					regionId=region.id,
					envelope=Msgs.toCoordsList(polyMsg.region.points),
					timeNanoSecs=Msgs.toNanoSecs(region.stamp),
				)
			case SensingPolygon.type:
				poly = SensingPolygon(
					polygonId=polyMsg.id,
					regionId=region.id,
					envelope=Msgs.toCoordsList(polyMsg.region.points),
					timeNanoSecs=Msgs.toNanoSecs(region.stamp),
					centerOfRotation=Msgs.toCoords(polyMsg.center_of_rotation)
				)
			case TargetPolygon.type:
				poly = TargetPolygon(
					polygonId=polyMsg.id,
					regionId=region.id,
					envelope=Msgs.toCoordsList(polyMsg.region.points),
					timeNanoSecs=Msgs.toNanoSecs(region.stamp),
					centerOfRotation=Msgs.toCoords(polyMsg.center_of_rotation)
				)
			case _:
				raise RuntimeError(f"Unexpected region type: {rType}")
		return poly

	@final
	def _parseRegularSpaceArray(self, rType: DynamicPolygon.Types, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		for region in regions.spaces:
			for polyMsg in region.polygons:
				poly = self.__createPolygon(rType, region, polyMsg)
				self.__appendPolygonToRegion(region.id, poly)
		return

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

	@final
	def render(self) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.log(f"{self.get_fully_qualified_name()} skipping render... RViz is not ready yet to receive messages.")
			return
		message = MarkerArray()
		for regionId in self.mapRegions:
			region = self.mapRegions[regionId]
			Ros.ConcatMessageArray(message.markers, region.render(durationNs=-1))
		for regionId in self.sensors:
			region = self.sensors[regionId]
			Ros.ConcatMessageArray(message.markers, region.render())
		for regionId in self.targets:
			region = self.targets[regionId]
			Ros.ConcatMessageArray(message.markers, region.render())
		self.__rvizPublisher.publish(message)
		return

class MapSubscriber(RegionsSubscriberBase):
	"""This object subscribes to the relevant map topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToMapColdStart(self, self.__parseMap)

	def __parseMap(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._parseRegularSpaceArray(DynamicPolygon.type, regions)
		self.onMapUpdated()
		return

	@abstractmethod
	def onMapUpdated(self) -> None: ...

class TargetSubscriber(RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant target topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToSymbols(self, self.__parseTarget)

	def __parseTarget(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._parseRegularSpaceArray(TargetPolygon.type, regions)
		self.onTargetsUpdated()
		return

	@abstractmethod
	def onTargetsUpdated(self) -> None: ...


class SensorSubscriber(RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant sensor topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToSensors(self, self.__parseSensor)

	def __parseSensor(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._parseRegularSpaceArray(SensingPolygon.type, regions)
		self.onSensorsUpdated()
		return

	@abstractmethod
	def onSensorsUpdated(self) -> None: ...
