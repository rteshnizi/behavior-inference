from typing import Dict, List, Union

import networkx as nx
from shapely.geometry import LineString, MultiPolygon, Polygon
from rt_bi_core.Model.Fov import Fov

from rt_bi_core.Model.MapRegion import PolygonalRegion
from rt_bi_core.Model.SensingRegion import SensingRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.Track import Tracks
from rt_bi_core.Model.ValidatorRegion import ValidatorRegion
from rt_bi_core.Specs.Validator import Validator
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Graph import GraphAlgorithms


class ConnectivityGraph(nx.DiGraph):
	def __init__(self, timeNanoSecs: float, regions: List[PolygonalRegion], fovUnion: Union[Polygon, MultiPolygon], tracks: Tracks, validators: Dict[str, Validator] = {}) -> None:
		"""
		The implementation of a Connectivity Graph in python on ROS2 (Humble).

		Parameters
		----------
		timeNanoSecs : float
			The timestamp of this connectivity graph.
		regions : List[PolygonalRegion]
			All the regions represented in this graph.
		validators : Dict[str, Validator], optional
			The dictionary of validators used in the specifications, if they are known, otherwise by default {}.
		"""
		super().__init__()
		self.validators = validators
		self.fovNodes: List[str] = []
		self.shadowNodes: List[str] = []
		self.symbolNodes: List[str] = []
		self.regions: List[PolygonalRegion] = regions
		self.__fov: Fov
		self.timeNanoSecs = timeNanoSecs
		print("Building graph for %s" % self.timeNanoSecs)

	def __repr__(self):
		return "cGraph-%.2f" % self.timeNanoSecs

	def __addNode(self, nodeName: str, region: PolygonalRegion, type: str):
		self.add_node(nodeName)
		self.nodes[nodeName]["region"] = region
		self.nodes[nodeName]["centroid"] = region.polygon.centroid
		self.nodes[nodeName]["type"] = type
		self.nodes[nodeName]["fromTime"] = self.timeNanoSecs
		if type == "sensor":
			self.fovNodes.append(nodeName)
		elif type == "shadow":
			self.shadowNodes.append(nodeName)
		elif type == "symbol":
			self.symbolNodes.append(nodeName)
		else:
			raise "Unknown node type %s" % type
		return

	def __addEdges(self, fromStr, toStr, directed=False):
		self.add_edge(fromStr, toStr)
		if directed: return
		self.add_edge(toStr, fromStr)

	def __addSingleSensorRegionConnectedComponent(self, connectedComponent, index: int, tracks: Tracks):
		# if len(tracks) == 0: return
		name = "FOV-%d" % index
		region = SensingRegion(name, [], self.timeNanoSecs, index, polygon=connectedComponent, tracks=tracks)
		self.__addNode(name, region, "sensor")
		return

	def __addSensorRegionConnectedComponents(self, fovUnion: Union[Polygon, MultiPolygon], tracks: Tracks):
		fovUnion = Geometry.convertToPolygonList(fovUnion)
		i = 0
		for connectedComponent in fovUnion:
			self.__addSingleSensorRegionConnectedComponent(connectedComponent, i, tracks)
			i += 1
		return

	def __createNewShadow(self, shadow: Polygon):
		name = "S-%d" % len(self.shadowNodes)
		region = ShadowRegion(name, Geometry.getPolygonCoords(shadow))
		self.__addNode(name, region, "shadow")
		for fovNode in self.fovNodes:
			fovComponent: SensingRegion = self.nodes[fovNode]["region"]
			if not fovComponent.containsTracks: continue
			if Geometry.haveOverlappingEdge(fovComponent.polygon, region.polygon):
				for trackId in fovComponent.tracks:
					track = fovComponent.tracks[trackId]
					if track.pose.spawn:
						self.__addEdges(name, fovNode, directed=True)
					if track.pose.vanished:
						self.__addEdges(fovNode, name, directed=True)
		return

	def __mergeShadow(self, existingShadow: ShadowRegion, shadow: Polygon):
		p = Geometry.union([existingShadow.polygon, shadow])
		region = ShadowRegion(existingShadow.name, Geometry.getPolygonCoords(p))
		self.nodes[existingShadow.name]["region"] = region
		self.nodes[existingShadow.name]["centroid"] = region.polygon.centroid
		return

	def __constructShadows(self, fovUnion: Union[Polygon, MultiPolygon]):
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
			self.__createNewShadow(shadow)
		return

	def __addSymbols(self, fovUnion: Union[Polygon, MultiPolygon]):
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
				self.__addNode(name, region, "symbol")
				broken = False
				for fovNode in self.fovNodes:
					fovRegion: SensingRegion = self.nodes[fovNode]["region"]
					if fovRegion.polygon.intersects(poly):
						self.__addEdges(fovNode, name)
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
				self.__addNode(name, region, "symbol")
				broken = False
				for shadowNode in self.shadowNodes:
					shadowRegion: SensingRegion = self.nodes[shadowNode]["region"]
					if shadowRegion.polygon.intersects(poly):
						self.__addEdges(shadowNode, name)
						break
				if broken: continue
		return

	def __build(self, fovUnion: Union[Polygon, MultiPolygon], tracks: Tracks):
		self.__addSensorRegionConnectedComponents(fovUnion, tracks)
		self.__constructShadows(fovUnion)
		self.__addSymbols(fovUnion)
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
