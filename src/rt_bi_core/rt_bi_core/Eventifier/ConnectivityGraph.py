from typing import Dict, List, Set, Type, Union

import networkx as nx
from matplotlib.pyplot import Figure, axis, close, figure, pause
from mpl_toolkits.mplot3d import Axes3D  # cspell: disable-line, leave this here, otherwise fig.gca(projection="3d") won't work

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_utils.Geometry import Geometry, LineString, MultiPolygon, Polygon

___a = Axes3D # This is here to avoid a warning for unused import

class ConnectivityGraph(nx.DiGraph):
	"""The implementation of a Connectivity Graph in python as described in the dissertation of Reza Teshnizi."""
	def __init__(
			self,
			timeNanoSecs: int,
			regions: List[Type[PolygonalRegion]],
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
		self.__symbols = symbols
		self.__regions: List[PolygonalRegion] = regions
		"""Geometric description of all the regions represented in this graph."""
		self.__mapPerimeter: Union[Polygon, MultiPolygon]
		self.__sensorNodes: Set[str] = set()
		"""Name of all the sensor nodes."""
		self.__shadowNodes: Set[str] = set()
		"""Name of all the shadow nodes."""
		self.__symbolNodes: Set[str] = set()
		"""Name of all the named symbol (regions defined in the alphabet) nodes."""
		self.__fieldOfView: FieldOfView = FieldOfView()
		self.__fig: Union[Figure, None] = None
		RosUtils.Logger().info("Constructing Perimeter.")
		self.__constructPerimeter()
		RosUtils.Logger().info("Constructing FOV.")
		self.__constructFov()
		RosUtils.Logger().info("Constructing Shadows.")
		self.__constructShadows()
		RosUtils.Logger().warn("SKIPPING construction of Symbols.")
		self.__constructSymbols()

	def __repr__(self):
		return "cGraph-%.2f" % self.timeNanoSecs

	def __addNode(self, region: Type[PolygonalRegion]) -> None:
		nodeName = region.name
		self.add_node(nodeName)
		self.nodes[nodeName]["region"] = region
		self.nodes[nodeName]["centroid"] = region.interior.centroid
		self.nodes[nodeName]["fromTime"] = self.timeNanoSecs
		if region.regionType == PolygonalRegion.RegionType.SENSING:
			self.__sensorNodes.add(nodeName)
		elif region.regionType == PolygonalRegion.RegionType.SHADOW:
			self.__shadowNodes.add(nodeName)
		elif region.regionType == PolygonalRegion.RegionType.SYMBOL:
			self.__symbolNodes.add(nodeName)
		else:
			raise TypeError("Unknown node type %s" % region.regionType)
		return

	def __addEdges(self, fromStr: Type[PolygonalRegion], toStr: Type[PolygonalRegion], directed=False):
		self.add_edge(fromStr.name, toStr.name)
		if directed: return
		self.add_edge(toStr.name, fromStr.name)

	def __constructPerimeter(self) -> None:
		polygons = [r.interior for r in self.__regions]
		self.__mapPerimeter = Geometry.union(polygons)
		return

	def __constructFov(self) -> None:
		for region in self.__regions:
			if region.regionType != PolygonalRegion.RegionType.SENSING: continue
			self.fieldOfView.addSensor(region)
			self.__addNode(region)
		return

	def __addShadowNode(self, shadow: Polygon) -> None:
		region = ShadowRegion(idNum=len(self.__shadowNodes), envelope=Geometry.getPolygonCoords(shadow))
		self.__addNode(region)
		for sensorName in self.__sensorNodes:
			fovComponent: SensorRegion = self.nodes[sensorName]["region"]
			if not fovComponent.hasTrack: continue
			if Geometry.haveOverlappingEdge(fovComponent.interior, region.interior):
				for trackId in fovComponent.__tracks:
					track = fovComponent.__tracks[trackId]
					if track.pose.spawn:
						self.__addEdges(region.name, sensorName, directed=True)
					if track.pose.vanished:
						self.__addEdges(sensorName, region.name, directed=True)
		return

	def __constructShadows(self) -> None:
		allShadowPolygons = []
		for region in self.__regions:
			if region.regionType == PolygonalRegion.RegionType.SENSING: continue
			if Geometry.polygonAndPolygonIntersect(region.interior, self.fieldOfView.fov):
				shadows = Geometry.shadows(region.interior, self.fieldOfView.fov)
				shadows = [p for p in shadows if p.length > 0]
				shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
				allShadowPolygons = allShadowPolygons + shadows
			else:
				allShadowPolygons.append(region.interior)
		if len(allShadowPolygons) == 0: return # Avoid dealing with empty unions
		mergedPolys = Geometry.union(allShadowPolygons)
		mergedPolys = Geometry.convertToPolygonList(mergedPolys)
		for shadow in mergedPolys:
			self.__addShadowNode(shadow)
		return

	def __constructSymbols(self):
		for symbolName in self.__symbolNodes:
			symbol: SymbolRegion = self.nodes[symbolName]["region"]
			insidePolys = Geometry.intersect(symbol.interior, self.fieldOfView.fov)
			insidePolys = [p for p in insidePolys if p.length > 0]
			insidePolys = list(filter(lambda p: not isinstance(p, LineString), insidePolys))
			for i in range(len(insidePolys)):
				poly = insidePolys[i]
				name = "%s-%d" % (symbolName, i)
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
			shadows = Geometry.shadows(symbol.interior, self.fieldOfView.fov)
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
	def mapPerimeter(self) -> MapRegion:
		"""The geometric description of the map."""
		return self.__mapPerimeter

	@property
	def fieldOfView(self) -> FieldOfView:
		return self.__fieldOfView

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
