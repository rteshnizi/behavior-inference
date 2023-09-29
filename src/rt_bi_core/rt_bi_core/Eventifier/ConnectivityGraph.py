from typing import Callable, Dict, List, Literal, Set, Type, Union

import networkx as nx
from visualization_msgs.msg import Marker, MarkerArray

from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Eventifier.MapPerimeter import MapPerimeter
from rt_bi_core.Eventifier.RegularSymbol import RegularSymbol
from rt_bi_core.Eventifier.Shadows import Shadows
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_utils.Geometry import Geometry, LineString, Point, Polygon
from rt_bi_utils.Ros import Logger
from rt_bi_utils.RViz import KnownColors, Publisher, RViz


class ConnectivityGraph(nx.DiGraph):
	"""
		The implementation of a Connectivity Graph in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	NodeContent = Dict[Literal["region", "centroid", "fromTime"], Union[DynamicRegion, Point, int]]
	"""### NodeContent
		Data contents of a ConnectivityGraph node.
	"""

	def __init__(
			self,
			timeNanoSecs: int,
			mapRegions: Union[List[MapRegion], MapPerimeter],
			fovRegions: Union[List[SensorRegion], FieldOfView],
			symbols: Dict[str, Symbol] = {},
			rvizPublisher: Union[Publisher, None] = None,
	) -> None:
		"""
		Creates a connectivity graph using the initial information.

		Parameters
		----------
		timeNanoSecs : float
			The timestamp of this connectivity graph.
		regions : List[PolygonalRegion]
			Geometric description of all the regions represented in this graph.
		validators : Dict[str, Validator], optional
			The dictionary of validators used in the specifications, if they are known, otherwise by default {}.
		tracks : Dict[Tuple[float, int], Track], optional
			Information about any observed tracks at that time instant. Tracks is a dictionary of (time, trackId) to Track, by default {}.
		"""
		super().__init__()
		self.timeNanoSecs = timeNanoSecs
		Logger().info("Constructing Connectivity Graph @ %d" % self.timeNanoSecs)
		"""Geometric description of all the regions represented in this graph."""
		self.__fieldOfView: FieldOfView = FieldOfView()
		"""The RegularSpatialRegion representing the field-of-view."""
		self.__shadows: Shadows = Shadows()
		"""The RegularSpatialRegion representing the shadows."""
		self.__symbols: RegularSymbol = RegularSymbol()
		"""The RegularSpatialRegion representing the shadows."""
		self.__mapPerimeter: MapPerimeter = MapPerimeter()
		"""The RegularSpatialRegion representing the map's perimeter."""
		self.__rvizPublisher = rvizPublisher
		self.__constructRegularRegion(regionList=mapRegions, constructionCallback=self.__constructPerimeter, loggerString="Map Perimeter")
		self.__constructRegularRegion(regionList=fovRegions, constructionCallback=self.__constructFov, loggerString="Field-of-View")
		Logger().info("Constructing Shadows.")
		self.__constructShadows()
		# Logger().info("Constructing Symbols.")
		# self.__constructSymbols(symbols)

	def __repr__(self):
		return "cGraph-%.2f" % self.timeNanoSecs

	def __addNode(self, region: Type[PolygonalRegion]) -> None:
		if (
			region.regionType != PolygonalRegion.RegionType.SENSING and
			region.regionType != PolygonalRegion.RegionType.SHADOW and
			region.regionType != PolygonalRegion.RegionType.SYMBOL
		):
			raise TypeError("Unknown node type %s" % region.regionType)
		nodeName = region.name
		self.add_node(nodeName)
		self.nodes[nodeName]["region"] = region
		self.nodes[nodeName]["centroid"] = region.interior.centroid
		self.nodes[nodeName]["fromTime"] = self.timeNanoSecs
		return

	def __addEdges(self, fromStr: Type[PolygonalRegion], toStr: Type[PolygonalRegion], directed=False):
		self.add_edge(fromStr.name, toStr.name)
		if directed: return
		self.add_edge(toStr.name, fromStr.name)

	def __constructRegularRegion(self, regionList: List[Union[MapRegion, SensorRegion]], constructionCallback: Union[Callable[[List[MapRegion]], None], Callable[[List[SensorRegion]], None]], loggerString: str) -> None:
		if isinstance(regionList, list):
			Logger().info("Constructing %s." % loggerString)
			constructionCallback(regionList)
		else:
			Logger().info("Making a shallow copy of %s from previous graph." % loggerString)
			constructionCallback([regionList[n] for n in regionList])
		return

	def __constructPerimeter(self, regions: List[MapRegion]) -> None:
		for region in regions:
			self.__mapPerimeter.addConnectedComponent(region)
		return

	def __constructFov(self, regions: List[SensorRegion]) -> None:
		for region in regions:
			if region.regionType != PolygonalRegion.RegionType.SENSING: continue
			self.fieldOfView.addConnectedComponent(region)
			self.__addNode(region)
		return

	def __addShadowNode(self, shadow: Polygon) -> None:
		region = ShadowRegion(idNum=len(self.__shadows), envelope=[], timeNanoSecs=self.timeNanoSecs, interior=shadow)
		self.__addNode(region)
		self.__shadows.addConnectedComponent(region)
		for fovName in self.fieldOfView:
			fovComponent = self.fieldOfView[fovName]
			if not fovComponent.hasTrack: continue
			if Geometry.haveOverlappingEdge(fovComponent.interior, region.interior):
				for trackId in fovComponent.__tracks:
					track = fovComponent.__tracks[trackId]
					if track.pose.spawn:
						self.__addEdges(region.name, fovName, directed=True)
					if track.pose.vanished:
						self.__addEdges(fovName, region.name, directed=True)
		return

	def __constructShadows(self) -> None:
		shadows = []
		if (not self.fieldOfView.isEmpty) and Geometry.intersects(self.mapPerimeter.interior, self.fieldOfView.interior):
			shadows = Geometry.difference(self.mapPerimeter.interior, self.fieldOfView.interior)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
			shadows = Geometry.union(shadows)
			shadows = Geometry.toGeometryList(shadows)
		else:
			shadows.append(self.mapPerimeter.interior)
		if len(shadows) == 0:
			Logger().warn("No Shadows.")
			return # Avoid dealing with empty unions
		for shadow in shadows:
			self.__addShadowNode(shadow)
		return

	def __constructSymbols(self, symbols: List[SymbolRegion]) -> None:
		return
		for symbol in symbols:
			insidePolys = Geometry.intersection(symbol.interior, self.fieldOfView.interior)
			insidePolys = [p for p in insidePolys if p.length > 0]
			insidePolys = list(filter(lambda p: not isinstance(p, LineString), insidePolys))
			for i in range(len(insidePolys)):
				poly = insidePolys[i]
				name = "%s-%d" % (symbol.name, i)
				region = SymbolRegion(name, Geometry.getGeometryCoords(poly), inFov=True)
				self.__addNode(region)
				broken = False
				for fovNode in self.__sensorNodes:
					fovRegion: SensorRegion = self.nodes[fovNode]["region"]
					if Geometry.intersects(fovRegion.interior, poly):
						self.__addEdges(fovNode, name)
						broken = True
						break
				if broken: continue
			shadows = Geometry.difference(symbol.interior, self.fieldOfView.interior)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
			for i in range(len(shadows)):
				poly = shadows[i]
				name = "SYM-%s-%d" % (symbolName, (i + len(insidePolys)))
				region = SymbolRegion(name, Geometry.getGeometryCoords(poly), inFov=False)
				self.__addNode(region)
				broken = False
				for shadowNode in self.__shadowNodes:
					shadowRegion: SensorRegion = self.nodes[shadowNode]["region"]
					if Geometry.intersects(shadowRegion.interior, poly):
						self.__addEdges(shadowNode, name)
						break
				if broken: continue
		return

	@property
	def mapPerimeter(self) -> MapPerimeter:
		"""The geometric description of the map."""
		return self.__mapPerimeter

	@property
	def fieldOfView(self) -> FieldOfView:
		return self.__fieldOfView

	@property
	def shadows(self) -> Shadows:
		return self.__shadows

	@property
	def symbols(self) -> RegularSymbol:
		return self.__symbols

	@property
	def allRegions(self) -> Set[str]:
		"""Returns all of the regions in this connectivity graph."""
		listOfRegions = set(self.shadows.regionNames) & set(self.fieldOfView.regionNames) & set(self.symbols.regionNames)
		return listOfRegions

	def render(self) -> None:
		if self.__rvizPublisher is None: return

		markers: List[Marker] = []
		markers += self.shadows.render()
		markers += self.symbols.render()
		markers += self.fieldOfView.render()
		message = MarkerArray()
		for marker in markers:
			message.markers.append(marker)
		Logger().info("Rendering %d CGraph markers." % len(message.markers))
		self.__rvizPublisher.publish(message)
		return
