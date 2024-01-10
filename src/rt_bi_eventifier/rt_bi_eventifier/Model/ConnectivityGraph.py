from typing import Callable, Dict, List, Literal, Sequence, Set, TypeVar, Union

import networkx as nx
from visualization_msgs.msg import Marker, MarkerArray

from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_eventifier.Model.FieldOfView import FieldOfView
from rt_bi_eventifier.Model.MapRegions import MapRegions
from rt_bi_eventifier.Model.Shadows import Shadows
from rt_bi_eventifier.Model.SymbolRegions import SymbolRegions
from rt_bi_utils.Geometry import Geometry, LineString, Point, Polygon
from rt_bi_utils.Ros import AppendMessage, ConcatMessageArray, Log, Logger
from rt_bi_utils.RViz import ColorNames, Publisher, RViz

RegionType = TypeVar("RegionType", bound=Union[SensorRegion, SymbolRegion, ShadowRegion, MapRegion])

class ConnectivityGraph(nx.DiGraph):
	"""
		The implementation of a Connectivity Graph in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	NodeContent = Dict[Literal["region", "centroid", "fromTime"], Union[RegionType, Point, int]]
	"""### NodeContent
		Data contents of a ConnectivityGraph node.
	"""

	def __init__(
			self,
			timeNanoSecs: int,
			mapRegions: Union[List[MapRegion], MapRegions],
			fovRegions: Union[List[SensorRegion], FieldOfView],
			symbols: Union[List[SymbolRegion], SymbolRegions] = [],
			rvizPublisher: Union[Publisher, None] = None,
	) -> None:
		super().__init__()
		self.timeNanoSecs = timeNanoSecs
		Log("Constructing Connectivity Graph @ %d" % self.timeNanoSecs)
		"""Geometric description of all the regions represented in this graph."""
		self.__fieldOfView: FieldOfView = FieldOfView()
		"""The RegularSpatialRegion representing the field-of-view."""
		self.__shadows: Shadows = Shadows()
		"""The RegularSpatialRegion representing the shadows."""
		self.__symbols: SymbolRegions = SymbolRegions()
		"""The RegularSpatialRegion representing the shadows."""
		self.__mapPerimeter: MapRegions = MapRegions()
		"""The RegularSpatialRegion representing the map's perimeter."""
		self.__rvizPublisher = rvizPublisher
		self.__constructRegularRegion(regionList=mapRegions, constructionCallback=self.__constructPerimeter, loggerString="Map Perimeter")
		self.__constructRegularRegion(regionList=fovRegions, constructionCallback=self.__constructFov, loggerString="Field-of-View")
		Log("Constructing Shadows.")
		self.__constructShadows()
		self.__constructRegularRegion(regionList=symbols, constructionCallback=self.__constructSymbols, loggerString="Symbols") # Must come after constructing shadows

	def __repr__(self):
		return "CGr-%d" % self.timeNanoSecs

	def __addNode(self, region: RegionType) -> RegionType:
		if (
			region.regionType != SensorRegion.RegionType.SENSING and
			region.regionType != ShadowRegion.RegionType.SHADOW and
			region.regionType != SymbolRegion.RegionType.SYMBOL
		):
			raise TypeError("Unknown node type %s" % region.regionType)

		if region.regionType == SensorRegion.RegionType.SENSING: self.fieldOfView.addConnectedComponent(region) # type: ignore
		if region.regionType == ShadowRegion.RegionType.SHADOW: self.shadows.addConnectedComponent(region) # type: ignore
		if region.regionType == SymbolRegion.RegionType.SYMBOL: self.symbols.addConnectedComponent(region) # type: ignore

		nodeName = region.name
		self.add_node(nodeName)
		self.nodes[nodeName]["region"] = region
		self.nodes[nodeName]["centroid"] = region.interior.centroid
		self.nodes[nodeName]["fromTime"] = self.timeNanoSecs
		return region

	def __addEdges(self, fromRegion: RegionType, toRegion: RegionType, directed=False):
		self.add_edge(fromRegion.name, toRegion.name)
		if directed: return
		self.add_edge(toRegion.name, fromRegion.name)

	def __constructRegularRegion(self, regionList: Union[Sequence[RegionType], MapRegions, FieldOfView, SymbolRegions], constructionCallback: Callable[[Sequence[RegionType]], None], loggerString: str) -> None:
		if isinstance(regionList, Sequence):
			Log("Constructing %s." % loggerString)
			constructionCallback(regionList)
		else:
			Log("Making a shallow copy of %s from previous graph." % loggerString)
			l = [regionList[n] for n in regionList]
			constructionCallback(l) # type: ignore
		return

	def __constructPerimeter(self, regions: Sequence[MapRegion]) -> None:
		for region in regions:
			self.mapRegions.addConnectedComponent(region)
			if region.timeNanoSecs > self.mapRegions.timeNanoSec: self.mapRegions.timeNanoSec = region.timeNanoSecs
		return

	def __constructFov(self, regions: Sequence[SensorRegion]) -> None:
		for region1 in regions:
			self.__addNode(region1)
			for region2 in regions:
				if region1.shortName == region2.shortName: continue
				if Geometry.intersects(region1.interior, region2.interior): self.__addEdges(region1, region2, directed=True)
		return

	def __addShadowNode(self, shadow: Polygon) -> None:
		region = ShadowRegion(idNum=len(self.__shadows), envelope=[], timeNanoSecs=self.timeNanoSecs, interior=shadow)
		self.__addNode(region)
		for fovName in self.fieldOfView:
			fovComponent = self.fieldOfView[fovName]
			if not fovComponent.hasTrack: continue
			if Geometry.haveOverlappingEdge(fovComponent.interior, region.interior):
				for i in fovComponent.tracklets:
					tracklet = fovComponent.tracklets[i]
					if tracklet.spawned:
						self.__addEdges(region, fovComponent, directed=True)
					if tracklet.vanished:
						self.__addEdges(fovComponent, region, directed=True)
		return

	def __constructShadows(self) -> None:
		shadows = []
		if (not self.fieldOfView.isEmpty) and Geometry.intersects(self.mapRegions.interior, self.fieldOfView.interior):
			shadows = Geometry.difference(self.mapRegions.interior, self.fieldOfView.interior)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
			shadows = Geometry.union(shadows)
			shadows = Geometry.toGeometryList(shadows)
		else:
			shadows.append(self.mapRegions.interior)
		if len(shadows) == 0:
			Logger().warn("No Shadows.")
			return # Avoid dealing with empty unions
		for shadow in shadows:
			self.__addShadowNode(shadow)
		return

	def __constructSymbolsPartitions(self, symbol: SymbolRegion, regularRegion: Union[FieldOfView, Shadows]) -> None:
		envelopePoly = Polygon(symbol.envelope) if symbol.overlappingRegionType != SymbolRegion.RegionType.BASE else symbol.interior
		intersectionPolys = Geometry.intersection(envelopePoly, regularRegion.interior)
		intersectionPolys = Geometry.toGeometryList(intersectionPolys)
		nonEmptyIntersections = [p for p in intersectionPolys if p.length > 0]
		insidePolys: List[Polygon] = list(filter(lambda p: not isinstance(p, LineString), nonEmptyIntersections))
		for i in range(len(insidePolys)):
			poly = insidePolys[i]
			for subRegionId in regularRegion:
				subRegion = regularRegion[subRegionId]
				intersection = Geometry.intersection(subRegion.interior, poly)
				if not intersection.is_empty:
					splitSymbol = SymbolRegion(
						centerOfRotation=symbol.centerOfRotation,
						idNum=symbol.idNum,
						envelope=symbol.envelope,
						timeNanoSecs=symbol.timeNanoSecs,
						overlappingRegionType=regularRegion.regionType,
						overlappingRegionId=subRegion.idNum,
						interior=poly,
					)
					Log(f"Adding partition {splitSymbol.name}")
					self.__addNode(splitSymbol)
					self.__addEdges(subRegion, splitSymbol)
					break
		return

	def __constructSymbols(self, symbols: Sequence[SymbolRegion]) -> None:
		for symbol in symbols:
			if symbol.regionType != SensorRegion.RegionType.SYMBOL: continue
			self.__constructSymbolsPartitions(symbol, self.fieldOfView)
			self.__constructSymbolsPartitions(symbol, self.shadows)
		return

	def __getShadowAreaMarkers(self, markerArray: MarkerArray) -> MarkerArray:
		for shadowName in self.shadows:
			shadow = self.shadows[shadowName]
			textCoords = shadow.interior.centroid
			textCoords = (textCoords.x, textCoords.y)
			timerText = "area(%s) = %.3f" % (shadowName, shadow.interior.area)
			timerMarker = RViz.createText("rt_st_%s" % shadowName, textCoords, timerText, ColorNames.RED, fontSize=7.5)
			AppendMessage(markerArray.markers, timerMarker)
		return markerArray

	@property
	def mapRegions(self) -> MapRegions:
		"""The geometric description of the map."""
		return self.__mapPerimeter

	@property
	def fieldOfView(self) -> FieldOfView:
		return self.__fieldOfView

	@property
	def shadows(self) -> Shadows:
		return self.__shadows

	@property
	def symbols(self) -> SymbolRegions:
		return self.__symbols

	@property
	def allRegionNames(self) -> Set[str]:
		"""Returns all of the regions in this connectivity graph."""
		listOfRegions = set(self.shadows.regionNames) & set(self.fieldOfView.regionNames) & set(self.symbols.regionNames)
		return listOfRegions

	def getRegion(self, region: RegionType) -> Union[RegionType, None]:
		rName = region.shortName
		if rName in self.fieldOfView: return self.fieldOfView[rName] # type: ignore
		if rName in self.shadows: return self.shadows[rName] # type: ignore
		if rName in self.symbols: return self.symbols[rName] # type: ignore
		return None

	def render(self, rvizPublisher: Publisher | None = None) -> None:
		if rvizPublisher is not None: self.__rvizPublisher = rvizPublisher
		if self.__rvizPublisher is None: return
		markers: List[Marker] = []
		ConcatMessageArray(markers, self.shadows.render())
		ConcatMessageArray(markers, self.symbols.render())
		# ConcatMessageArray(markers, self.fieldOfView.render())
		message = MarkerArray()
		for marker in markers: AppendMessage(message.markers, marker)
		message = self.__getShadowAreaMarkers(message)
		self.__rvizPublisher.publish(message)
		return

	def clearRender(self, rvizPublisher: Publisher | None = None) -> None:
		if rvizPublisher is not None: self.__rvizPublisher = rvizPublisher
		if self.__rvizPublisher is None: return
		markers: List[Marker] = []
		ConcatMessageArray(markers, self.shadows.clearRender())
		ConcatMessageArray(markers, self.symbols.clearRender())
		message = MarkerArray()
		for marker in markers: AppendMessage(message.markers, marker)
		self.__rvizPublisher.publish(message)
		return
