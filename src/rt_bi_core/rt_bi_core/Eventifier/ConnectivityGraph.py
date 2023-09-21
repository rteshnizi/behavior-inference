from typing import Callable, Dict, List, Set, Tuple, Type, Union

import networkx as nx
from matplotlib.pyplot import Figure, close, figure, pause
from mpl_toolkits.mplot3d import Axes3D  # cspell: disable-line, leave this here, otherwise fig.gca(projection="3d") won't work

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Eventifier.MapPerimeter import MapPerimeter
from rt_bi_core.Eventifier.RegularSymbol import RegularSymbol
from rt_bi_core.Eventifier.Shadows import Shadows
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_utils.Geometry import Geometry, LineString, Polygon

___a = Axes3D # This is here to avoid a warning for unused import

class ConnectivityGraph(nx.DiGraph):
	"""The implementation of a Connectivity Graph in python as described in the dissertation of Reza Teshnizi."""
	def __init__(
			self,
			timeNanoSecs: int,
			mapRegions: Union[List[MapRegion], MapPerimeter],
			fovRegions: Union[List[SensorRegion], FieldOfView],
			symbols: Dict[str, Symbol] = {},
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
		RosUtils.Logger().info("Constructing Connectivity Graph @ %d" % self.timeNanoSecs)
		"""Geometric description of all the regions represented in this graph."""
		self.__fieldOfView: FieldOfView = FieldOfView()
		"""The RegularSpatialRegion representing the field-of-view."""
		self.__shadows: Shadows = Shadows()
		"""The RegularSpatialRegion representing the shadows."""
		self.__symbols: RegularSymbol = RegularSymbol()
		"""The RegularSpatialRegion representing the shadows."""
		self.__mapPerimeter: MapPerimeter = MapPerimeter()
		"""The RegularSpatialRegion representing the map's perimeter."""
		self.__fig: Union[Figure, None] = None
		self.__constructRegularRegion(regionList=mapRegions, constructionCallback=self.__constructPerimeter, loggerString="Map Perimeter")
		self.__constructRegularRegion(regionList=fovRegions, constructionCallback=self.__constructFov, loggerString="Field-of-View")
		RosUtils.Logger().info("Constructing Shadows.")
		self.__constructShadows()
		# RosUtils.Logger().info("Constructing Symbols.")
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
			RosUtils.Logger().info("Constructing %s." % loggerString)
			constructionCallback(regionList)
		else:
			RosUtils.Logger().info("Making a shallow copy of %s from previous graph." % loggerString)
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
		region = ShadowRegion(idNum=len(self.__shadows), envelope=[], interior=shadow)
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
		if (not self.fieldOfView.isEmpty) and Geometry.polygonAndPolygonIntersect(self.mapPerimeter.interior, self.fieldOfView.interior):
			shadows = Geometry.difference(self.mapPerimeter.interior, self.fieldOfView.interior)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
			shadows = Geometry.union(shadows)
			shadows = Geometry.convertToPolygonList(shadows)
		else:
			shadows.append(self.mapPerimeter.interior)
		if len(shadows) == 0:
			RosUtils.Logger().warn("No Shadows.")
			return # Avoid dealing with empty unions
		for shadow in shadows:
			self.__addShadowNode(shadow)
		return

	def __constructSymbols(self, symbols: List[SymbolRegion]) -> None:
		return
		for symbol in symbols:
			insidePolys = Geometry.intersect(symbol.interior, self.fieldOfView.interior)
			insidePolys = [p for p in insidePolys if p.length > 0]
			insidePolys = list(filter(lambda p: not isinstance(p, LineString), insidePolys))
			for i in range(len(insidePolys)):
				poly = insidePolys[i]
				name = "%s-%d" % (symbol.name, i)
				region = SymbolRegion(name, Geometry.getPolygonCoords(poly), inFov=True)
				self.__addNode(region)
				broken = False
				for fovNode in self.__sensorNodes:
					fovRegion: SensorRegion = self.nodes[fovNode]["region"]
					if fovRegion.interior.intersects(poly):
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
				region = SymbolRegion(name, Geometry.getPolygonCoords(poly), inFov=False)
				self.__addNode(region)
				broken = False
				for shadowNode in self.__shadowNodes:
					shadowRegion: SensorRegion = self.nodes[shadowNode]["region"]
					if shadowRegion.interior.intersects(poly):
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

	# def construct(self, regions)

	def render(self):
		if self.__fig is None:
			fig = figure(self.name)
			ax = fig.add_subplot(projection="3d")
		else:
			fig = self.__fig
		xFov = []
		xpFov = None
		yFov = []
		ypFov = None
		zFov = []
		zpFov = None
		xSymbol = []
		ySymbol = []
		zSymbol = []
		xShadow = []
		yShadow = []
		zShadow = []
		for node in self.nodes:
			if "type" not in self.nodes[node]:
				raise "Graph node needs a type"
			elif self.nodes[node]["type"] == "sensor":
				(xFov, yFov) = self.nodes[node]["region"].interior.exterior.coords.xy
				zFov = [self.nodes[node]["fromTime"]] * len(xFov)
				ax.plot(xFov, yFov, zFov, "g-")
				# ax.text(self.nodes[node]["centroid"].x, self.nodes[node]["centroid"].y, self.nodes[node]["fromTime"], node, color="darkgreen")
				if xpFov is not None:
					for i in range(len(xFov) - 1):
						ax.plot([xpFov[i], xFov[i]], [ypFov[i], yFov[i]], [zpFov[i], zFov[i]], "g-")
				(xpFov, ypFov, zpFov) = (xFov, yFov, zFov)
			elif self.nodes[node]["type"] == "symbol":
				(xSymbol, ySymbol) = self.nodes[node]["region"].interior.exterior.coords.xy
				zSymbol = [self.nodes[node]["fromTime"]] * len(xSymbol)
				ax.plot(xSymbol, ySymbol, zSymbol, "b-")
				# ax.text(self.nodes[node]["centroid"].x, self.nodes[node]["centroid"].y, self.nodes[node]["fromTime"], node, color="darkblue")
			elif self.nodes[node]["type"] == "shadow":
				(xShadow, yShadow) = self.nodes[node]["region"].interior.exterior.coords.xy
				zShadow = [self.nodes[node]["fromTime"]] * len(xShadow)
				ax.plot(xShadow, yShadow, zShadow, "k-")
				# ax.text(self.nodes[node]["centroid"].x, self.nodes[node]["centroid"].y, self.nodes[node]["fromTime"], node, color="maroon")
			else:
				raise "Unknown node type: %s" % self.nodes[node]["type"]
		ax.autoscale() # cspell: disable-line
		fig.set_facecolor("grey") # cspell: disable-line
		if self.__fig is None:
			fig.show()
			self.__fig = fig
		else:
			pause(0.05)
		return

	def killDisplayedGraph(self) -> None:
		close(self.__fig)
		self.fig = None
		return
