from abc import ABC, abstractmethod
from typing import Any, NamedTuple, TypeAlias, cast, final

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.MinQueue import MinQueue, numeric
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import MapPolygon, MovingPolygon, PolygonFactory, SensingPolygon, StaticPolygon, TargetPolygon
from rt_bi_core.Spatial.Polygon import PolygonFactoryKeys

SubscriberPolygon: TypeAlias = MapPolygon | SensingPolygon | TargetPolygon


class _RegSpaceUpdate(NamedTuple):
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
	def __init__(self, pauseQueuingMsgs: bool, **kwArgs):
		newKw = { "node_name": "map_base", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.pauseQueuingMsgs = pauseQueuingMsgs
		self.__msgPq: MinQueue[_RegSpaceUpdate] = MinQueue(key=self.__eventPqKey)
		self.__processingTimer: Ros.Timer | None = None
		self.mapRegions: dict[str, list[MapPolygon]] = {}
		self.mapPolyIdMap: dict[MovingPolygon.Id | StaticPolygon.Id, tuple[str, int]] = {}
		self.sensorRegions: dict[str, list[SensingPolygon]] = {}
		self.sensorPolyIdMap: dict[SensingPolygon.Id, tuple[str, int]] = {}
		self.targetRegions: dict[str, list[TargetPolygon]] = {}
		self.targetPolyIdMap: dict[TargetPolygon.Id, tuple[str, int]] = {}
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName("map"))

	def __minPqKey(self, poly: SubscriberPolygon) -> int:
		return poly.timeNanoSecs

	def __eventPqKey(self, val: _RegSpaceUpdate) -> numeric:
		(_, region) = val
		if len(region) == 0: raise AssertionError("Empty RegularSpaceArray message has been recorded.")
		return float(Msgs.toNanoSecs(region[0].stamp))

	def __storePolygon(self, regionId: str, poly: SubscriberPolygon) -> None:
		match poly.type:
			case StaticPolygon.type | MovingPolygon.type:
				poly = cast(MapPolygon, poly)
				if regionId not in self.mapRegions:
					self.mapRegions[regionId] = []
				self.mapPolyIdMap[poly.id] = (regionId, len(self.mapRegions[regionId]))
				self.mapRegions[regionId].append(poly)
			case SensingPolygon.type:
				poly = cast(SensingPolygon, poly)
				if regionId not in self.sensorRegions:
					self.sensorRegions[regionId] = []
				self.sensorPolyIdMap[poly.id] = (regionId, len(self.sensorRegions[regionId]))
				self.sensorRegions[regionId].append(poly)
			case TargetPolygon.type:
				poly = cast(TargetPolygon, poly)
				if regionId not in self.sensorRegions:
					self.targetRegions[regionId] = []
				self.targetPolyIdMap[poly.id] = (regionId, len(self.targetRegions[regionId]))
				self.targetRegions[regionId].append(poly)
			case _:
				raise RuntimeError(f"Unexpected region type: {poly.type}")
		return

	def __createPolygon(self, rType: MovingPolygon.Types, regularSpace: Msgs.RtBi.RegularSpace, polyMsg: Msgs.RtBi.Polygon) -> SubscriberPolygon:
		kwArgs: dict[PolygonFactoryKeys, Any] = {
			"polygonId": polyMsg.id,
			"regionId": regularSpace.id,
			"envelope": Msgs.toCoordsList(polyMsg.region.points),
			"timeNanoSecs": Msgs.toNanoSecs(regularSpace.stamp),
			"predicates": regularSpace.predicates if isinstance(regularSpace.predicates, list) else [],
			"hIndex": -1,
			"centerOfRotation": Msgs.toCoords(polyMsg.center_of_rotation),
		}
		match rType:
			case StaticPolygon.type:
				PolyCls = StaticPolygon
			case MovingPolygon.type:
				PolyCls = MovingPolygon
			case SensingPolygon.type:
				PolyCls = SensingPolygon
			case TargetPolygon.type:
				PolyCls = TargetPolygon
			case _:
				raise RuntimeError(f"Unexpected region type: {rType}")
		poly = PolygonFactory(PolyCls, kwArgs)
		return poly

	def __processEnqueuedUpdatesSlow(self) -> None:
		if self.__processingTimer is not None: self.__processingTimer.destroy()
		if self.__msgPq.isEmpty: self.log("No updates to process.. see you next time!")
		else:
			self.log(f"[SLOW] Processing enqueued updates.")
			(rType, msgs) = self.__msgPq.peek
			if len(msgs) == 0:
				self.__msgPq.dequeue()
				self.__processingTimer = Ros.CreateTimer(self, self.__processEnqueuedUpdatesSlow, intervalNs=50)
				return
			regularSpace = msgs[-1]
			if len(regularSpace.polygons) == 0:
				msgs.pop()
				self.__processingTimer = Ros.CreateTimer(self, self.__processEnqueuedUpdatesSlow, intervalNs=50)
				return
			polyMsg = Ros.PopMessage(regularSpace.polygons, 0, Msgs.RtBi.Polygon)
			poly = self.__createPolygon(rType, regularSpace, polyMsg)
			self.__storePolygon(regularSpace.id, poly)
			self.onPolygonUpdated(rType, poly)
		self.__processingTimer = Ros.CreateTimer(self, self.__processEnqueuedUpdatesSlow, intervalNs=1000)
		return

	def __processEnqueuedUpdates(self) -> None:
		if self.__processingTimer is not None: self.__processingTimer.destroy()
		if not self.__msgPq.isEmpty: self.log("Processing enqueued updates.")
		else: self.log("No updates to process.. see you next time!")
		while not self.__msgPq.isEmpty:
			(rType, msgs) = self.__msgPq.dequeue()
			for regularSpace in msgs:
				for polyMsg in regularSpace.polygons:
					poly = self.__createPolygon(rType, regularSpace, polyMsg)
					self.__storePolygon(regularSpace.id, poly)
					self.onPolygonUpdated(rType, poly)
		self.__processingTimer = Ros.CreateTimer(self, self.__processEnqueuedUpdates)
		return

	@final
	def _enqueueUpdate(self, rType: MovingPolygon.Types, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		"""Enqueues the update. Subclasses must call this function upon subscription message."""
		if len(regions.spaces) == 0: return
		if self.pauseQueuingMsgs: return
		self.log(f"Record {len(regions.spaces)} updated regions in event pQ.")
		self.__msgPq.enqueue(_RegSpaceUpdate(type=rType, spaces=list(regions.spaces)))
		self.__processEnqueuedUpdates()
		return

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

	@abstractmethod
	def createMarkers(self) -> list[RViz.Msgs.Marker]: ...

	@final
	def render(self) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.log(f"{self.get_fully_qualified_name()} skipping render... RViz is not ready yet to receive messages.")
			return
		msg = RViz.Msgs.MarkerArray()
		markers = self.createMarkers()
		if len(markers) == 0: return
		msg.markers = markers
		self.__rvizPublisher.publish(msg)
		return

	@abstractmethod
	def onPolygonUpdated(self, rType: MovingPolygon.Types, polygon: SubscriberPolygon) -> None:
		"""Override to customize processing of updates.

		:param MovingPolygon.Types rType: The type of the region updated.
		:param regions: The list of regular spaces updated.
		:type regions: list[Msgs.RtBi.RegularSpace]
		"""
		...

class MapSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant map topics."""
	def __init__(self, pauseQueuingMsgs: bool, **kwArgs):
		super().__init__(pauseQueuingMsgs=pauseQueuingMsgs, **kwArgs)
		RtBiInterfaces.subscribeToMapColdStart(self, self.__parseMap)
		RtBiInterfaces.subscribeToKnownRegions(self, self.__parseKnownRegion)

	def __parseMap(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		self.pauseQueuingMsgs = False
		super()._enqueueUpdate(StaticPolygon.type, regions)
		return

	def __parseKnownRegion(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(MovingPolygon.type, regions)
		return

	@abstractmethod
	def onPolygonUpdated(self, rType: MovingPolygon.Type | StaticPolygon.Type, polygon: MapPolygon) -> None: ...

class TargetSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant target topics."""
	def __init__(self, pauseQueuingMsgs: bool, **kwArgs):
		super().__init__(pauseQueuingMsgs=pauseQueuingMsgs, **kwArgs)
		self.pauseQueuingMsgs = False
		RtBiInterfaces.subscribeToSymbols(self, self.__parseTarget)

	def __parseTarget(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(TargetPolygon.type, regions)
		return

	@abstractmethod
	def onPolygonUpdated(self, rType: TargetPolygon.Type, polygon: TargetPolygon) -> None: ...

class SensorSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant sensor topics."""
	def __init__(self, pauseQueuingMsgs: bool, **kwArgs):
		super().__init__(pauseQueuingMsgs=pauseQueuingMsgs, **kwArgs)
		RtBiInterfaces.subscribeToSensors(self, self.__parseSensor)

	def __parseSensor(self, regions: Msgs.RtBi.RegularSpaceArray) -> None:
		super()._enqueueUpdate(SensingPolygon.type, regions)
		return

	@abstractmethod
	def onPolygonUpdated(self, rType: SensingPolygon.Type, polygon: SensingPolygon) -> None: ...
