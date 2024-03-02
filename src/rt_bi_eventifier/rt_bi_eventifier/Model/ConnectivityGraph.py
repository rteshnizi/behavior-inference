from typing import Callable, Literal, Sequence, TypeAlias, TypeVar, cast, overload

import networkx as nx

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.Ros import AppendMessage, ConcatMessageArray, Log, Logger, Publisher
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.ShadowPolygon import ShadowPolygon
from rt_bi_core.Spatial.SpatialRegion import SpatialRegion
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon

_PolygonAlias: TypeAlias = SensingPolygon | ShadowPolygon | MovingPolygon | StaticPolygon
_T_Polygon = TypeVar("_T_Polygon", bound=_PolygonAlias)

class ConnectivityGraph(nx.DiGraph):
	"""
		The implementation of a Connectivity Graph in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	ContentKey: TypeAlias = Literal["polygon", "centroid", "fromTime"]
	ContentValue: TypeAlias = _PolygonAlias | Shapely.Point | int
	NodeContent: TypeAlias = dict[ContentKey, ContentValue]
	""" Data contents of a `ConnectivityGraph` node. """
	Map: TypeAlias = SpatialRegion[MovingPolygon | StaticPolygon]
	Sensors: TypeAlias = SpatialRegion[SensingPolygon]
	Shadows: TypeAlias = SpatialRegion[ShadowPolygon]

	def __init__(
			self,
			timeNanoSecs: int,
			mapRegions: list[MovingPolygon | StaticPolygon] | Map,
			sensors: list[SensingPolygon] | Sensors,
			rvizPublisher: Publisher | None = None,
	) -> None:
		super().__init__()
		self.timeNanoSecs = timeNanoSecs
		Log(f"Constructing Connectivity Graph @ {self.timeNanoSecs}")
		"""Geometric description of all the regions represented in this graph."""
		self.__sensors = ConnectivityGraph.Sensors()
		"""The SpatialRegion representing the field-of-view."""
		self.__shadows = ConnectivityGraph.Shadows()
		"""The SpatialRegion representing the shadows."""
		self.__map = ConnectivityGraph.Map()
		"""The SpatialRegion representing the map's perimeter."""
		self.__rvizPublisher = rvizPublisher
		self.__constructRegularRegion(regionList=mapRegions, constructionCallback=self.__constructMap, logStr="Map")
		self.__constructRegularRegion(regionList=sensors, constructionCallback=self.__constructFov, logStr="Sensors")
		Log("Constructing Shadows.")
		self.__constructShadows()

	def __repr__(self):
		return "CGr-%d" % self.timeNanoSecs

	@overload
	def getContent(self, node: NxUtils.Id, contentKey: Literal[""]) -> NodeContent: ...
	@overload
	def getContent(self, node: NxUtils.Id, contentKey: Literal["polygon"]) -> _PolygonAlias: ...
	@overload
	def getContent(self, node: NxUtils.Id, contentKey: Literal["centroid"]) -> Shapely.Point: ...
	@overload
	def getContent(self, node: NxUtils.Id, contentKey: Literal["fromTime"]) -> int: ...

	def getContent(self, node: NxUtils.Id, contentKey: ContentKey | Literal[""]) -> NodeContent | ContentValue:
		if contentKey == "": return self.nodes[node]
		else: return self.nodes[node][contentKey]

	def __addNode(self, polygon: _PolygonAlias) -> None:
		if (
			polygon.type != SensingPolygon.type and
			polygon.type != ShadowPolygon.type and
			polygon.type != MovingPolygon.type
		):
			raise TypeError(f"Unknown node type {polygon.type}")

		if polygon.type == SensingPolygon.type:
			self.sensors.addConnectedComponent(cast(SensingPolygon, polygon))
		if polygon.type == ShadowPolygon.type:
			self.shadows.addConnectedComponent(cast(ShadowPolygon, polygon))
		if polygon.type == MovingPolygon.type:
			self.map.addConnectedComponent(cast(MovingPolygon, polygon))

		self.add_node(polygon.id)
		self.nodes[polygon.id]["polygon"] = polygon
		self.nodes[polygon.id]["centroid"] = polygon.interior.centroid
		self.nodes[polygon.id]["fromTime"] = self.timeNanoSecs
		return

	def __addEdges(self, fromRegion: _PolygonAlias, toRegion: _PolygonAlias, directed=False):
		self.add_edge(fromRegion.shortName, toRegion.shortName)
		if directed: return
		self.add_edge(toRegion.shortName, fromRegion.shortName)

	def __constructRegularRegion(self, regionList: Sequence[_T_Polygon] | Map | Sensors, constructionCallback: Callable[[Sequence[_T_Polygon]], None], logStr: str) -> None:
		if isinstance(regionList, Sequence):
			Log(f"Constructing {logStr}.")
			constructionCallback(regionList)
		else:
			Log(f"Making a shallow copy of {logStr} from previous graph.")
			l = [regionList[n] for n in regionList]
			constructionCallback(l) # pyright: ignore[reportArgumentType]
		return

	def __constructMap(self, regions: Sequence[MovingPolygon | StaticPolygon]) -> None:
		for polygon in regions:
			self.map.addConnectedComponent(polygon)
			if polygon.timeNanoSecs > self.map.timeNanoSec: self.map.timeNanoSec = polygon.timeNanoSecs
		return

	def __constructFov(self, regions: Sequence[SensingPolygon]) -> None:
		for region1 in regions:
			self.__addNode(region1)
			for region2 in regions:
				if region1.shortName == region2.shortName: continue
				if GeometryLib.intersects(region1.interior, region2.interior): self.__addEdges(region1, region2, directed=True)
		return

	def __addShadowNode(self, shadow: Shapely.Polygon) -> None:
		__SHADOW_ID_PREFIX = "https://rezateshnizi.com/rt-bi/tower_bridge/defintion/"
		region = ShadowPolygon(polygonId="", regionId="", envelope=[], timeNanoSecs=self.timeNanoSecs, interior=shadow)
		self.__addNode(region)
		for fovName in self.sensors:
			fovComponent = self.sensors[fovName]
			if not fovComponent.hasTrack: continue
			if GeometryLib.haveOverlappingEdge(fovComponent.interior, region.interior):
				for i in fovComponent.tracklets:
					tracklet = fovComponent.tracklets[i]
					if tracklet.spawned:
						self.__addEdges(region, fovComponent, directed=True)
					if tracklet.vanished:
						self.__addEdges(fovComponent, region, directed=True)
		return

	def __constructShadows(self) -> None:
		shadows = []
		if (not self.sensors.isEmpty) and GeometryLib.intersects(self.map.interior, self.sensors.interior):
			shadows = GeometryLib.difference(self.map.interior, self.sensors.interior)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, Shapely.LineString), shadows))
			shadows = GeometryLib.union(shadows)
			shadows = GeometryLib.toGeometryList(shadows)
		else:
			mapPolys = GeometryLib.toGeometryList(self.map.interior)
			shadows = shadows + mapPolys
		if len(shadows) == 0:
			Logger().warn("No Shadows.")
			return # Avoid dealing with empty unions
		for shadow in shadows:
			self.__addShadowNode(shadow)
		return

	def __constructMovingPolygonPartitions(self, polygon: MovingPolygon, regularRegion: Sensors | Shadows) -> None:
		# envelopePoly = Shapely.Polygon(polygon.envelope) if polygon.overlappingRegionType != MovingPolygon.type else polygon.interior
		envelopePoly = Shapely.Polygon(polygon.envelope)
		intersectionPolys = GeometryLib.intersection(envelopePoly, regularRegion.interior)
		intersectionPolys = GeometryLib.toGeometryList(intersectionPolys)
		nonEmptyIntersections = [p for p in intersectionPolys if p.length > 0]
		insidePolys: list[Shapely.Polygon] = list(filter(lambda p: not isinstance(p, Shapely.LineString), nonEmptyIntersections))
		for i in range(len(insidePolys)):
			poly = insidePolys[i]
			for subRegionId in regularRegion:
				subRegion = regularRegion[subRegionId]
				intersection = GeometryLib.intersection(subRegion.interior, poly)
				if not intersection.is_empty:
					parts = MovingPolygon(
						centerOfRotation=polygon.centerOfRotation,
						polygonId=polygon.id.polygonId,
						regionId=polygon.id.regionId,
						envelope=polygon.envelope,
						timeNanoSecs=polygon.timeNanoSecs,
						overlappingRegionType=subRegion.type,
						overlappingPolygonId=subRegion.id,
						interior=poly,
					)
					Log(f"Adding partition {parts.shortName}")
					self.__addNode(parts)
					self.__addEdges(subRegion, parts)
					break
		return

	def __constructMovingPolygons(self, polygons: Sequence[MovingPolygon]) -> None:
		for polygon in polygons:
			if polygon.type != MovingPolygon.type: continue
			self.__constructMovingPolygonPartitions(polygon, self.sensors)
			self.__constructMovingPolygonPartitions(polygon, self.shadows)
		return

	def __getShadowAreaMarkers(self, markerArray: RViz.Msgs.MarkerArray) -> RViz.Msgs.MarkerArray:
		for shadowName in self.shadows:
			shadow = self.shadows[shadowName]
			textCoords = shadow.interior.centroid
			textCoords = (textCoords.x, textCoords.y)
			timerText = "area(%s) = %.3f" % (shadowName, shadow.interior.area)
			timerMarker = RViz.createText(shadow.id, textCoords, timerText, ColorNames.RED, fontSize=7.5, idSuffix="area")
			AppendMessage(markerArray.markers, timerMarker)
		return markerArray

	@property
	def map(self) -> Map:
		"""The geometric description of the map."""
		return self.__map

	@property
	def sensors(self) -> Sensors:
		return self.__sensors

	@property
	def shadows(self) -> Shadows:
		return self.__shadows

	@property
	def allIds(self) -> set[NxUtils.Id]:
		"""Returns all of the regions in this connectivity graph."""
		ids = set(self.shadows.polygonIds) & set(self.sensors.polygonIds)
		return ids

	def getRegion(self, region: _T_Polygon) -> _T_Polygon | None:
		if region.id in self.sensors: return self.sensors[region.id]
		if region.id in self.shadows: return self.shadows[region.id]
		if region.id in self.map: return self.map[region.id] # pyright: ignore[reportReturnType]
		return None

	def render(self, rvizPublisher: Publisher | None = None) -> None:
		if rvizPublisher is not None: self.__rvizPublisher = rvizPublisher
		if self.__rvizPublisher is None: return
		markers: list[RViz.Msgs.Marker] = []
		ConcatMessageArray(markers, self.shadows.render())
		ConcatMessageArray(markers, self.map.render())
		# ConcatMessageArray(markers, self.sensors.render())
		message = RViz.Msgs.MarkerArray()
		for marker in markers: AppendMessage(message.markers, marker)
		message = self.__getShadowAreaMarkers(message)
		self.__rvizPublisher.publish(message)
		return

	def clearRender(self, rvizPublisher: Publisher | None = None) -> None:
		if rvizPublisher is not None: self.__rvizPublisher = rvizPublisher
		if self.__rvizPublisher is None: return
		markers: list[RViz.Msgs.Marker] = []
		ConcatMessageArray(markers, self.shadows.clearRender())
		ConcatMessageArray(markers, self.map.clearRender())
		# ConcatMessageArray(markers, self.sensors.clearRender())
		message = RViz.Msgs.MarkerArray()
		for marker in markers: AppendMessage(message.markers, marker)
		self.__rvizPublisher.publish(message)
		return
