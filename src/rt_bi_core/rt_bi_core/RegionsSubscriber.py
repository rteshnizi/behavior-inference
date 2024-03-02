from abc import ABC, abstractmethod
from typing import NamedTuple, cast, final

from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.MinQueue import MinQueue
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.SpatialRegion import SpatialRegion
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_core.Spatial.TargetPolygon import TargetPolygon


class _queueElement(NamedTuple):
	type: MovingPolygon.Types
	spaces: list[Msgs.RtBi.RegularSpace]

class __RegionsSubscriberBase(RtBiNode, ABC):
	"""
	This Node provides an API to listen to all the messages about polygonal regions.
	It also creates a publisher for RViz and a :meth:`~RegionsSubscriberBase.render` method.

	**NOTICE**
	* Subclasses of this class must subscribe to the relevant topics.
	* Subclasses do not need to create a publisher to RViz. Just call :meth:`~RegionsSubscriberBase.render`
	"""
	def __init__(self, **kwArgs):
		newKw = { "node_name": "map_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.__eventPQueue: MinQueue[_queueElement] = MinQueue(key=self.__updatePqKey)
		self.__processingTimer: Ros.Timer | None = None
		self.mapRegions: dict[str, SpatialRegion[MovingPolygon | StaticPolygon]] = {}
		self.sensors: dict[str, SpatialRegion[SensingPolygon]] = {}
		self.targets: dict[str, SpatialRegion[TargetPolygon]] = {}
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName("map"))

	def __updatePqKey(self, val: _queueElement) -> float:
		(_, region) = val
		return float(Msgs.toNanoSecs(region[0].stamp))

	def __storePolygon(self, regionId: str, poly: MovingPolygon | StaticPolygon | SensingPolygon | TargetPolygon) -> None:
		match poly.type:
			case StaticPolygon.type | MovingPolygon.type:
				poly = cast(StaticPolygon | MovingPolygon, poly)
				if regionId not in self.mapRegions:
					regRegion = SpatialRegion[MovingPolygon | StaticPolygon]()
					self.mapRegions[regionId] = regRegion
				else:
					regRegion = self.mapRegions[regionId]
				regRegion.addConnectedComponent(poly)
			case SensingPolygon.type:
				poly = cast(SensingPolygon, poly)
				if regionId not in self.sensors:
					regRegion = SpatialRegion[SensingPolygon]()
					self.sensors[regionId] = regRegion
				else:
					regRegion = self.sensors[regionId]
				regRegion.addConnectedComponent(poly)
			case TargetPolygon.type:
				poly = cast(TargetPolygon, poly)
				if regionId not in self.sensors:
					regRegion = SpatialRegion[TargetPolygon]()
					self.targets[regionId] = regRegion
				else:
					regRegion = self.targets[regionId]
				regRegion.addConnectedComponent(poly)
			case _:
				raise RuntimeError(f"Unexpected region type: {poly.type}")
		return

	def __createPolygon(self, rType: MovingPolygon.Types, region: Msgs.RtBi.RegularSpace, polyMsg: Msgs.RtBi.Polygon) -> MovingPolygon | StaticPolygon | SensingPolygon | TargetPolygon:
		match rType:
			case StaticPolygon.type:
				poly = StaticPolygon(
					polygonId=polyMsg.id,
					regionId=region.id,
					timeNanoSecs=Msgs.toNanoSecs(region.stamp),
					envelope=Msgs.toCoordsList(polyMsg.region.points),
				)
			case MovingPolygon.type:
				poly = MovingPolygon(
					polygonId=polyMsg.id,
					regionId=region.id,
					envelope=Msgs.toCoordsList(polyMsg.region.points),
					timeNanoSecs=Msgs.toNanoSecs(region.stamp),
					centerOfRotation=Msgs.toCoords(polyMsg.center_of_rotation),
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

	def __processEnqueuedUpdates(self) -> None:
		if self.__processingTimer is not None: self.__processingTimer.destroy()
		self.log(f"Processing enqueued updates.")
		while not self.__eventPQueue.isEmpty:
			(rType, regions) = self.__eventPQueue.dequeue()
			for region in regions:
				for polyMsg in region.polygons:
					poly = self.__createPolygon(rType, region, polyMsg)
					self.__storePolygon(region.id, poly)
				self.onRegionsUpdated(rType, regions)
		self.__processingTimer = Ros.CreateTimer(self, self.__processEnqueuedUpdates)
		return

	@final
	def _enqueueUpdate(self, rType: MovingPolygon.Types, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		"""Enqueues the update. Subclasses must call this function upon subscription message."""
		if len(regions.spaces) == 0: return

		self.log(f"Record {len(regions.spaces)} updated regions in event pQ.")
		self.__eventPQueue.enqueue(_queueElement(type=rType, spaces=list(regions.spaces)))
		self.__processEnqueuedUpdates()
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
		message = RViz.Msgs.MarkerArray()
		for regionId in self.mapRegions:
			region = self.mapRegions[regionId]
			Ros.ConcatMessageArray(message.markers, region.render(durationNs=-1))
		for regionId in self.sensors:
			region = self.sensors[regionId]
			Ros.ConcatMessageArray(message.markers, region.render(durationNs=-1))
		for regionId in self.targets:
			region = self.targets[regionId]
			Ros.ConcatMessageArray(message.markers, region.render(durationNs=-1))
		self.__rvizPublisher.publish(message)
		return

	@abstractmethod
	def onRegionsUpdated(self, rType: MovingPolygon.Types, regions: list[Msgs.RtBi.RegularSpace]) -> None:
		"""Override to customize processing of updates.

		:param DynamicPolygon.Types rType: The type of the region updated.
		:param list regions: The list of regular spaces updated.
		:type regions: list[Msgs.RtBi.RegularSpace]
		"""
		...

class MapSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant map topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToMapColdStart(self, self.__parseMap)
		RtBiInterfaces.subscribeToKnownRegions(self, self.__parseKnownRegion)

	def __parseMap(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(StaticPolygon.type, regions)
		return

	def __parseKnownRegion(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(MovingPolygon.type, regions)
		return

class TargetSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant target topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToSymbols(self, self.__parseTarget)

	def __parseTarget(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(TargetPolygon.type, regions)
		return

class SensorSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant sensor topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToSensors(self, self.__parseSensor)

	def __parseSensor(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(SensingPolygon.type, regions)
		return
