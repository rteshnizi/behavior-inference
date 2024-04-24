from abc import ABC, abstractmethod
from typing import Any, TypeAlias, cast, final

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.MinQueue import MinQueue
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import AffinePolygon, DynamicPolygon, MapPolygon, PolygonFactory, SensingPolygon, StaticPolygon, TargetPolygon
from rt_bi_core.Spatial.Polygon import PolygonFactoryKeys
from rt_bi_core.Spatial.Tracklet import Tracklet

SubscriberPolygon: TypeAlias = MapPolygon | SensingPolygon | TargetPolygon

# TODO: Optimization -- History keeps all of the updates.
# Fix it if memory usage is high?
# Maybe remove a polygon (except for the last) once it is processed.
class RegionsSubscriber(RtBiNode, ABC):
	"""
	This Node provides an API to listen to all the messages about polygonal regions.
	It also creates a publisher for RViz and a :meth:`~RegionsSubscriber.render` method.

	**NOTICE**
	* Subclasses of this class must subscribe to the relevant topics and
		assign :meth:`~RegionSubscriber.subscriptionListener` to handle the messages.
	* Subclasses do not need to create a publisher to RViz. Just call :meth:`~RegionsSubscriber.render`
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

	def __notifySubclasses(self, poly: SubscriberPolygon) -> None:
		if poly.type == StaticPolygon.type or poly.type == DynamicPolygon.type or poly.type == AffinePolygon.type:
			self.onMapUpdated(poly)
		elif poly.type == SensingPolygon.type:
			self.onSensorUpdated(poly)
		elif poly.type == TargetPolygon.type:
			self.onTargetUpdated(poly)
		else: raise RuntimeError("This should never happen") # No update goes missing
		return

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
		self.__notifySubclasses(poly)
		return

	def __useLatestGeometry(self, regularSet: Msgs.RtBi.RegularSet) -> None:
		polys: list[SubscriberPolygon] | None = None # For majority of the cases this is a list with a single poly, except for when initializing temporal events
		match regularSet.set_type:
			case StaticPolygon.type.value:
				if regularSet.id in self.mapRegions:
					polys = [self.mapRegions[regularSet.id][-1]]
			case DynamicPolygon.type.value:
				if regularSet.id in self.mapRegions:
					polys = [self.mapRegions[regularSet.id][-1]]
			case AffinePolygon.type.value:
				if regularSet.id in self.mapRegions:
					polys = [self.mapRegions[regularSet.id][-1]]
			case SensingPolygon.type.value:
				if regularSet.id in self.sensorRegions:
					polys = [self.sensorRegions[regularSet.id][-1]]
			case TargetPolygon.type.value:
				if regularSet.id in self.targetRegions:
					polys = [self.targetRegions[regularSet.id][-1]]
			case _:
				self.log(f"Unexpected region type: {regularSet.set_type}")
		if polys is None and regularSet.set_type == Msgs.RtBi.RegularSet.TEMPORAL:
			# When it's a purely temporal set, we assign its predicates to all map regions and send updates
			polys = [self.mapRegions[polyId][-1] for polyId in self.mapRegions]
		if polys is None:
			raise RuntimeError(f"No polygon stored for id: {regularSet.id}, set type: {regularSet.set_type}.")

		for poly in polys:
			predicates = Predicates.fromMsgArray(regularSet.predicates) if isinstance(regularSet.predicates, list) else Predicates()
			kwArgs: dict[PolygonFactoryKeys, Any] = {
				"polygonId": poly.id.polygonId,
				"regionId": poly.id.regionId,
				"subPartId": "",
				"envelope": poly.envelope,
				"timeNanoSecs": Msgs.toNanoSecs(regularSet.stamp),
				"predicates": predicates,
				"hIndex": -1,
				"centerOfRotation": poly.centerOfRotation,
			}
			if poly.type == SensingPolygon.type: kwArgs["tracklets"] = poly.tracklets
			poly = PolygonFactory(type(poly), kwArgs)
			self.__storeGeometry(poly.id.regionId, poly)
		return

	def __createTracklets(self, regularSet: Msgs.RtBi.RegularSet) -> dict[str, Tracklet]:
		tracklets = {}
		for i in range(len(regularSet.estimations)):
			trackletMsg = Ros.GetMessage(regularSet.estimations, i, Msgs.RtBi.Tracklet)
			tracklet = Tracklet(
				idStr=trackletMsg.id,
				timeNanoSecs=Msgs.toNanoSecs(regularSet.stamp),
				hIndex=-1,
				x=trackletMsg.pose.position.x,
				y=trackletMsg.pose.position.y,
				angleFromX=Msgs.toAngle(trackletMsg.pose.orientation),
				entered=trackletMsg.entered,
				exited=trackletMsg.exited,
			)
			tracklets[trackletMsg.id] = tracklet
		return tracklets

	def __createGeometry(self, regularSet: Msgs.RtBi.RegularSet, polyMsg: Msgs.RtBi.Polygon) -> None:
		predicates = Predicates.fromMsgArray(regularSet.predicates) if isinstance(regularSet.predicates, list) else Predicates()
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
		match regularSet.set_type:
			case Msgs.RtBi.RegularSet.STATIC:
				PolyCls = StaticPolygon
			case Msgs.RtBi.RegularSet.DYNAMIC:
				PolyCls = DynamicPolygon
			case Msgs.RtBi.RegularSet.AFFINE:
				PolyCls = AffinePolygon
			case Msgs.RtBi.RegularSet.SENSING:
				PolyCls = SensingPolygon
				kwArgs["tracklets"] = self.__createTracklets(regularSet)
			case Msgs.RtBi.RegularSet.TARGET:
				PolyCls = TargetPolygon
			case _:
				raise RuntimeError(f"Unexpected space type event queue: {regularSet.set_type}\n\tMSG = {repr(regularSet)}")
		poly = PolygonFactory(PolyCls, kwArgs)
		self.__storeGeometry(regularSet.id, poly)
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
	def enqueueUpdate(self, setArr: Msgs.RtBi.RegularSetArray) -> None:
		"""Enqueues the update. Subclasses must call this function upon subscription message."""
		if len(setArr.sets) == 0: return
		setArr.sets = Ros.AsList(setArr.sets, Msgs.RtBi.RegularSet)
		self.log(f"{len(setArr.sets)} updates arrived.")
		for match in setArr.sets:
			self.log(f"Recording update of type {match.set_type} in event pQ @{Msgs.toNanoSecs(match.stamp)}.")
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
	def onSensorUpdated(self, polygon: SensingPolygon) -> None: ...

	@abstractmethod
	def onTargetUpdated(self, polygon: TargetPolygon) -> None: ...

	@abstractmethod
	def onMapUpdated(self, polygon: MapPolygon) -> None: ...
