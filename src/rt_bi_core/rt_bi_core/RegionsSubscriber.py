from abc import ABC, abstractmethod
from typing import Any, TypeAlias, cast, final

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.MinQueue import MinQueue
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import AffinePolygon, DynamicPolygon, MapPolygon, PolygonFactory, SensingPolygon, StaticPolygon, TargetPolygon
from rt_bi_core.Spatial.Polygon import PolygonFactoryKeys

SubscriberPolygon: TypeAlias = MapPolygon | SensingPolygon | TargetPolygon

class __RegionsSubscriberBase(RtBiNode, ABC):
	"""
	This Node provides an API to listen to all the messages about polygonal regions.
	It also creates a publisher for RViz and a :meth:`~RegionsSubscriberBase.render` method.

	**NOTICE**
	* Subclasses of this class must subscribe to the relevant topics.
	* Subclasses do not need to create a publisher to RViz. Just call :meth:`~RegionsSubscriberBase.render`
	"""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		self.__msgPq: MinQueue[Msgs.RtBi.RegularSet] = MinQueue(key=self.__eventPqKey)
		self.__processingTimer: Ros.Timer | None = None
		self.mapRegions: dict[str, list[MapPolygon]] = {}
		self.sensorRegions: dict[str, list[SensingPolygon]] = {}
		self.targetRegions: dict[str, list[TargetPolygon]] = {}
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName("map"))

	def __eventPqKey(self, val: Msgs.RtBi.RegularSet) -> int:
		nanoSecs = Msgs.toNanoSecs(val.stamp)
		if nanoSecs == 0: raise AssertionError("Update with no timestamp: ")
		return nanoSecs

	def __storeGeometry(self, setId: str, poly: SubscriberPolygon) -> None:
		match poly.type:
			case StaticPolygon.type | AffinePolygon.type | DynamicPolygon.type:
				poly = cast(MapPolygon, poly)
				if setId not in self.mapRegions:
					self.mapRegions[setId] = []
				self.mapRegions[setId].append(poly)
			case SensingPolygon.type:
				poly = cast(SensingPolygon, poly)
				if setId not in self.sensorRegions:
					self.sensorRegions[setId] = []
				self.sensorRegions[setId].append(poly)
			case TargetPolygon.type:
				poly = cast(TargetPolygon, poly)
				if setId not in self.targetRegions:
					self.targetRegions[setId] = []
				self.targetRegions[setId].append(poly)
			case _:
				raise RuntimeError(f"Unexpected region type: {poly.type}")
		return

	def __useLatestGeometry(self, regularSet: Msgs.RtBi.RegularSet) -> None:
		poly: SubscriberPolygon | None = None
		match regularSet.space_type:
			case StaticPolygon.type.value:
				PolyCls = StaticPolygon
				if regularSet.id in self.mapRegions:
					poly = self.mapRegions[regularSet.id][-1]
			case DynamicPolygon.type.value:
				PolyCls = DynamicPolygon
				if regularSet.id in self.mapRegions:
					poly = self.mapRegions[regularSet.id][-1]
			case AffinePolygon.type.value:
				PolyCls = AffinePolygon
				if regularSet.id in self.mapRegions:
					poly = self.mapRegions[regularSet.id][-1]
			case SensingPolygon.type.value:
				PolyCls = SensingPolygon
				if regularSet.id in self.mapRegions:
					poly = self.sensorRegions[regularSet.id][-1]
			case TargetPolygon.type.value:
				PolyCls = TargetPolygon
				if regularSet.id in self.mapRegions:
					poly = self.targetRegions[regularSet.id][-1]
			case _:
				raise RuntimeError(f"Unexpected region type: {regularSet.space_type}")
		if poly is None: raise RuntimeError(f"No polygon stored for id: {regularSet.id}")

		predicates = Predicates(regularSet.predicates) if isinstance(regularSet.predicates, list) else Predicates([])
		kwArgs: dict[PolygonFactoryKeys, Any] = {
			"polygonId": poly.id.polygonId,
			"regionId": regularSet.id,
			"subPartId": "",
			"envelope": poly.envelope,
			"timeNanoSecs": Msgs.toNanoSecs(regularSet.stamp),
			"predicates": predicates,
			"hIndex": -1,
			"centerOfRotation": poly.centerOfRotation,
		}
		poly = PolygonFactory(PolyCls, kwArgs)

		self.__storeGeometry(regularSet.id, poly)
		self.onPolygonUpdated(poly)
		return

	def __createGeometry(self, regularSet: Msgs.RtBi.RegularSet, polyMsg: Msgs.RtBi.Polygon) -> None:
		predicates = Predicates(regularSet.predicates) if isinstance(regularSet.predicates, list) else Predicates([])
		kwArgs: dict[PolygonFactoryKeys, Any] = {
			"polygonId": polyMsg.id,
			"regionId": regularSet.id,
			"subPartId": "",
			"envelope": Msgs.toCoordsList(polyMsg.region.points),
			"timeNanoSecs": Msgs.toNanoSecs(regularSet.stamp),
			"predicates": predicates,
			"hIndex": -1,
			"centerOfRotation": Msgs.toCoords(polyMsg.center_of_rotation),
		}
		match regularSet.space_type:
			case Msgs.RtBi.RegularSet.STATIC:
				PolyCls = StaticPolygon
			case Msgs.RtBi.RegularSet.DYNAMIC:
				PolyCls = DynamicPolygon
			case Msgs.RtBi.RegularSet.AFFINE:
				PolyCls = AffinePolygon
			case Msgs.RtBi.RegularSet.SENSING:
				PolyCls = SensingPolygon
			case Msgs.RtBi.RegularSet.TARGET:
				PolyCls = TargetPolygon
			case _:
				raise RuntimeError(f"Unexpected space type event queue: {regularSet.space_type}\n\tMSG = {repr(regularSet)}")
		poly = PolygonFactory(PolyCls, kwArgs)

		self.__storeGeometry(regularSet.id, poly)
		self.onPolygonUpdated(poly)
		return

	def __processEnqueuedUpdates(self) -> None:
		if self.__processingTimer is not None: self.__processingTimer.destroy()
		if not self.__msgPq.isEmpty: self.log(f"** Processing enqueued updates. Queue Size = {len(self.__msgPq)}")
		nowNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		timerInterval = 1000
		while not self.__msgPq.isEmpty:
			nextTimeStamp = Msgs.toNanoSecs(self.__msgPq.peek.stamp)
			if nowNanoSecs < nextTimeStamp:
				delta = nextTimeStamp - nowNanoSecs
				timerInterval = delta if delta < timerInterval else timerInterval
				break
			regularSet = self.__msgPq.dequeue()
			if len(regularSet.polygons) == 0:
				self.__useLatestGeometry(regularSet)
				continue
			for polyMsg in regularSet.polygons:
				self.__createGeometry(regularSet, polyMsg)
		if self.__msgPq.isEmpty:
			self.log(f"** Processing finished -- EXHAUSTED the event queue.")
		else:
			self.log(f"** Processing finished. Queue Size = {len(self.__msgPq)}")
			self.log(f"Next event is in the future @ {repr(nextTimeStamp)} -- timer started for {timerInterval}ns.")
		self.__processingTimer = Ros.CreateTimer(self, self.__processEnqueuedUpdates, timerInterval)
		return

	@final
	def _enqueueUpdates(self, setArr: Msgs.RtBi.RegularSetArray) -> None:
		"""Enqueues the update. Subclasses must call this function upon subscription message."""
		if len(setArr.sets) == 0: return
		setArr.sets = Ros.AsList(setArr.sets, Msgs.RtBi.RegularSet)
		self.log(f"{len(setArr.sets)} updates arrived.")
		for match in setArr.sets:
			self.log(f"Recording update of type {match.space_type} in event pQ @{Msgs.toNanoSecs(match.stamp)}.")
			self.__msgPq.enqueue(match)
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
	def onPolygonUpdated(self, polygon: SubscriberPolygon) -> None:
		"""Override to customize processing of updates.

		:param regions: The list of regular spaces updated.
		:type regions: list[Msgs.RtBi.RegularSet]
		"""
		...

class MapSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant map topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		self.mapInitPhasesRemaining = 2
		# Hold off from taking affine region updates until map is initialized
		RtBiInterfaces.subscribeToMap(self, self.__parseMap)
		RtBiInterfaces.subscribeToKnownRegions(self, self.__parseKnownRegion)

	def __parseMap(self, setArr: Msgs.RtBi.RegularSetArray) -> None:
		if self.mapInitPhasesRemaining > 0: self.mapInitPhasesRemaining -= 1
		super()._enqueueUpdates(setArr)
		return

	def __parseKnownRegion(self, setArr: Msgs.RtBi.RegularSetArray) -> None:
		super()._enqueueUpdates(setArr)
		return

	@abstractmethod
	def onPolygonUpdated(self, polygon: MapPolygon) -> None: ...

class TargetSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant target topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToTargets(self, self.__parseTarget)

	def __parseTarget(self, setArr: Msgs.RtBi.RegularSetArray) -> None:
		super()._enqueueUpdates(setArr)
		return

	@abstractmethod
	def onPolygonUpdated(self, polygon: TargetPolygon) -> None: ...

class SensorSubscriber(__RegionsSubscriberBase, ABC):
	"""This object subscribes to the relevant sensor topics."""
	def __init__(self, **kwArgs):
		super().__init__(**kwArgs)
		RtBiInterfaces.subscribeToSensors(self, self.__parseSensor)

	def __parseSensor(self, setArr: Msgs.RtBi.RegularSetArray) -> None:
		super()._enqueueUpdates(setArr)
		return

	@abstractmethod
	def onPolygonUpdated(self, polygon: SensingPolygon) -> None: ...
