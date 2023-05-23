import networkx as nx
from shapely.geometry import LineString, Polygon, MultiPolygon
from typing import Dict, List, Tuple, Union

from sa_bil.core.model.polygonalRegion import PolygonalRegion
from sa_bil.core.model.sensingRegion import SensingRegion
from sa_bil.core.model.shadowRegion import ShadowRegion
from sa_bil.core.model.map import Map
from sa_bil.core.model.validatorRegion import ValidatorRegion
from sa_bil.core.observation.track import Tracks
from sa_bil.core.spec.validator import Validator
from sa_bil.core.utils.geometry import Geometry
from sa_bil.core.utils.graph import GraphAlgorithms

class ConnectivityGraph(nx.DiGraph):
	def __init__(self, envMap: Map, fovUnion: Union[Polygon, MultiPolygon], tracks: Tracks, timestamp: float, validators: Dict[str, Validator]):
		super().__init__()
		self.validators = validators
		self.fovNodes: List[str] = []
		self.shadowNodes: List[str] = []
		self.symbolNodes: List[str] = []
		self.map: Map = envMap
		self.time = timestamp
		print("Building graph for %s" % self.time)
		self._build(fovUnion, tracks)
		self._fig1 = None
		# DEBUG MEMBERS
		self.polygonsToDraw = []

	def __repr__(self):
		return "cGraph-%.2f" % self.time

	def _addNode(self, nodeName: str, region: PolygonalRegion, type: str):
		self.add_node(nodeName)
		self.nodes[nodeName]["region"] = region
		self.nodes[nodeName]["centroid"] = region.polygon.centroid
		self.nodes[nodeName]["type"] = type
		self.nodes[nodeName]["fromTime"] = self.time
		if type == "sensor":
			self.fovNodes.append(nodeName)
		elif type == "shadow":
			self.shadowNodes.append(nodeName)
		elif type == "symbol":
			self.symbolNodes.append(nodeName)
		else:
			raise "Unknown node type %s" % type
		return

	def _addEdges(self, fromStr, toStr, directed=False):
		self.add_edge(fromStr, toStr)
		if directed: return
		self.add_edge(toStr, fromStr)

	def _addSingleSensorRegionConnectedComponent(self, connectedComponent, index: int, tracks: Tracks):
		# if len(tracks) == 0: return
		name = "FOV-%d" % index
		region = SensingRegion(name, [], self.time, index, polygon=connectedComponent, tracks=tracks)
		self._addNode(name, region, "sensor")
		return

	def _addSensorRegionConnectedComponents(self, fovUnion: Union[Polygon, MultiPolygon], tracks: Tracks):
		fovUnion = Geometry.convertToPolygonList(fovUnion)
		i = 0
		for connectedComponent in fovUnion:
			self._addSingleSensorRegionConnectedComponent(connectedComponent, i, tracks)
			i += 1
		return

	def _createNewShadow(self, shadow: Polygon):
		name = "S-%d" % len(self.shadowNodes)
		region = ShadowRegion(name, Geometry.getPolygonCoords(shadow))
		self._addNode(name, region, "shadow")
		for fovNode in self.fovNodes:
			fovComponent: SensingRegion = self.nodes[fovNode]["region"]
			if not fovComponent.containsTracks: continue
			if Geometry.haveOverlappingEdge(fovComponent.polygon, region.polygon):
				for trackId in fovComponent.tracks:
					track = fovComponent.tracks[trackId]
					if track.pose.spawn:
						self._addEdges(name, fovNode, directed=True)
					if track.pose.vanished:
						self._addEdges(fovNode, name, directed=True)
		return

	def _mergeShadow(self, existingShadow: ShadowRegion, shadow: Polygon):
		p = Geometry.union([existingShadow.polygon, shadow])
		region = ShadowRegion(existingShadow.name, Geometry.getPolygonCoords(p))
		self.nodes[existingShadow.name]["region"] = region
		self.nodes[existingShadow.name]["centroid"] = region.polygon.centroid
		return

	def _constructShadows(self, fovUnion: Union[Polygon, MultiPolygon]):
		allShadowPolygons = []
		for regionName in self.map.regions:
			mapR = self.map.regions[regionName]
			if Geometry.polygonAndPolygonIntersect(mapR.polygon, fovUnion):
				shadows = Geometry.shadows(mapR.polygon, fovUnion)
				shadows = [p for p in shadows if p.length > 0]
				shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
				allShadowPolygons = allShadowPolygons + shadows
			else:
				allShadowPolygons.append(mapR.polygon)
		mergedPolys = Geometry.union(allShadowPolygons)
		mergedPolys = Geometry.convertToPolygonList(mergedPolys)
		for shadow in mergedPolys:
			self._createNewShadow(shadow)
		return

	def _addSymbols(self, fovUnion: Union[Polygon, MultiPolygon]):
		for validatorName in self.validators:
			validator = self.validators[validatorName]
			if not validator.isRegion: continue
			validatorRegion: PolygonalRegion = validator.value
			insidePolys = Geometry.intersect(validatorRegion.polygon, fovUnion)
			insidePolys = [p for p in insidePolys if p.length > 0]
			insidePolys = list(filter(lambda p: not isinstance(p, LineString), insidePolys))
			for i in range(len(insidePolys)):
				poly = insidePolys[i]
				name = "%s-%d" % (validatorName, i)
				region = ValidatorRegion(name, Geometry.getPolygonCoords(poly), inFov=True)
				self._addNode(name, region, "symbol")
				broken = False
				for fovNode in self.fovNodes:
					fovRegion: SensingRegion = self.nodes[fovNode]["region"]
					if fovRegion.polygon.intersects(poly):
						self._addEdges(fovNode, name)
						broken = True
						break
				if broken: continue
			shadows = Geometry.shadows(validatorRegion.polygon, fovUnion)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
			for i in range(len(shadows)):
				poly = shadows[i]
				name = "SYM-%s-%d" % (validatorName, (i + len(insidePolys)))
				region = ValidatorRegion(name, Geometry.getPolygonCoords(poly), inFov=False)
				self._addNode(name, region, "symbol")
				broken = False
				for shadowNode in self.shadowNodes:
					shadowRegion: SensingRegion = self.nodes[shadowNode]["region"]
					if shadowRegion.polygon.intersects(poly):
						self._addEdges(shadowNode, name)
						break
				if broken: continue
		return

	def _build(self, fovUnion: Union[Polygon, MultiPolygon], tracks: Tracks):
		self._addSensorRegionConnectedComponents(fovUnion, tracks)
		self._constructShadows(fovUnion)
		self._addSymbols(fovUnion)
		return

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		if displayGeomGraph:
			self._figGeom = GraphAlgorithms.displayGeometricGraph(self)
		if displaySpringGraph:
			self._figGraph = GraphAlgorithms.displaySpringGraph(self, self.fovNodes, self.shadowNodes, self.symbolNodes)
		return

	def killDisplayedGraph(self):
		if self._figGeom:
			GraphAlgorithms.killDisplayedGraph(self._figGeom)
			self._figGeom = None
		if self._figGraph:
			GraphAlgorithms.killDisplayedGraph(self._figGraph)
			self._figGraph = None
