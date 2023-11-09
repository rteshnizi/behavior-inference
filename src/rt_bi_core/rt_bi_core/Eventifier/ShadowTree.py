from math import inf
from queue import Queue
from typing import Dict, List, Literal, Sequence, Set, Tuple, TypeVar, Union

import networkx as nx
from visualization_msgs.msg import Marker, MarkerArray

from rt_bi_core.BehaviorAutomaton.Lambda import NfaLambda
from rt_bi_core.BehaviorAutomaton.SpaceTime import ProjectiveSpaceTimeSet
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import CollisionInterval, ContinuousTimeCollisionDetection as CtCd
from rt_bi_core.Eventifier.EventAggregator import EventAggregator
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.RegularDynamicRegion import RegularDynamicRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_core.Model.TimeRegion import TimeRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.Ros import AppendMessage, Logger, Publisher
from rt_bi_utils.RViz import KnownColors, RViz

RegionTypeX = TypeVar("RegionTypeX", bound=DynamicRegion)
RegionTypeY = TypeVar("RegionTypeY", bound=DynamicRegion)

class ShadowTree(nx.DiGraph):
	"""
		The implementation of a Shadow Tree in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	SUBMODULE_TYPES = Literal["connectivity_graph", "continuous_time_collision_detection"]
	SUBMODULES: Tuple[SUBMODULE_TYPES, ...] = ("connectivity_graph", "continuous_time_collision_detection")

	def __init__(self, rvizPublishers: Dict[SUBMODULE_TYPES, Union[Publisher, None]]):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(name="ShTr-V3")
		self.__history: List[ConnectivityGraph] = []
		self.componentEvents: List[List[Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """
		self.__rvizPublishers = rvizPublishers

	@property
	def history(self) -> Sequence[ConnectivityGraph]:
		"""The list of the graphs."""
		return self.__history

	@property
	def length(self) -> int:
		"""The length of the shadow tree history stack."""
		return len(self.__history)

	def __repr__(self) -> str:
		timeRangeStr = "∅"
		if self.length == 1:
			timeRangeStr = "%d" % self.history[0].timeNanoSecs
		if self.length > 1:
			timeRangeStr = "%d-%d" % (self.history[0].timeNanoSecs, self.history[-1].timeNanoSecs)
		return "%s[%s]{d=%d}" % (self.name, timeRangeStr, self.length)

	def __getLowerAndUpperNode(self, n1: str, n2: str) -> Tuple[str, str]:
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def __addNode(self, n: str, timeNanoSecs: int) -> None:
		self.add_node(n)
		self.nodes[n]["fromTime"] = timeNanoSecs
		# The va"toTime" will be updated accordingly when adding temporal edges.
		# inf here represents "a large value".
		self.nodes[n]["toTime"] = inf
		return

	def __addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime = None, toTime = None) -> None:
		(lower, upper) = self.__getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)
		return

	def __shadowsAreConnectedTemporally(self, pastGraph: ConnectivityGraph, nowGraph: ConnectivityGraph, pastShadow: ConnectivityGraph.NodeContent, nowShadow: ConnectivityGraph.NodeContent) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the transformation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		pastRegion = pastShadow["region"]
		nowRegion = nowShadow["region"]
		if isinstance(pastRegion, DynamicRegion) and isinstance(nowRegion, DynamicRegion):
			intersectionOfShadows = Geometry.intersection(pastRegion.interior, nowRegion.interior)
			for fov in pastGraph.fieldOfView:
				if fov not in pastGraph.nodes: continue
				pastSensorRegion: SensorRegion = pastGraph.nodes[fov]["region"]
				if fov not in nowGraph.nodes: continue
				nowSensorRegion: SensorRegion = nowGraph.nodes[fov]["region"]
				pastCoords = Geometry.getGeometryCoords(pastSensorRegion.interior)
				nowCoords = Geometry.getGeometryCoords(nowSensorRegion.interior)
				transformation: AffineTransform = Geometry.getAffineTransformation(pastCoords, nowCoords)
				ps = []
				for e in nowSensorRegion.edges:
					edgeC = nowSensorRegion.edges[e]
					edgeP = pastSensorRegion.getEquivalentEdge(edgeC, transformation)
					if edgeP is None:
						raise AssertionError("No equivalent edge found based on transformation.")
					polygon = Polygon([edgeP.coords[0], edgeP.coords[1], edgeC.coords[1], edgeC.coords[0]])
					ps.append(polygon)
				u = Geometry.union(ps)
				polys = Geometry.difference(intersectionOfShadows, u)
				for p in polys:
					if p.is_valid and (not p.is_empty) and p.area > Geometry.EPSILON:
						return True
		return False

	def __connectGraphsTemporally(self, newGraph: ConnectivityGraph) -> None:
		if self.length == 0: return
		lastGraph = self.history[-1]
		# Add temporal edges between FOVs
		for sensor in lastGraph.fieldOfView:
			fovNodeInShadowTree = self.generateTemporalName(sensor, lastGraph.timeNanoSecs)
			if fovNodeInShadowTree not in self.nodes: continue
			fovNodeInCurrentGraph = self.generateTemporalName(sensor, newGraph.timeNanoSecs)
			if fovNodeInCurrentGraph not in self.nodes: continue
			self.__addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
		# Add temporal edges between symbols
		for symNode in lastGraph.symbols:
			symNodeInShadowTree = self.generateTemporalName(symNode, lastGraph.timeNanoSecs)
			symNodeInCurrentGraph = self.generateTemporalName(symNode, newGraph.timeNanoSecs)
			self.__addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
		# Add temporal edges between shadows
		for shadowNodeInPastGraph in lastGraph.shadows:
			for shadowNodeInNowGraph in newGraph.shadows:
				previousShadowNodeRegion: ShadowRegion = lastGraph.nodes[shadowNodeInPastGraph]["region"]
				currentShadowNodeRegion: ShadowRegion = newGraph.nodes[shadowNodeInNowGraph]["region"]
				if Geometry.intersects(previousShadowNodeRegion.interior, currentShadowNodeRegion.interior):
					if self.__shadowsAreConnectedTemporally(lastGraph, newGraph, lastGraph.nodes[shadowNodeInPastGraph], newGraph.nodes[shadowNodeInNowGraph]):
						shadowNodeInShadowTree = self.generateTemporalName(shadowNodeInPastGraph, lastGraph.timeNanoSecs)
						shadowNodeInNowGraph = self.generateTemporalName(shadowNodeInNowGraph, newGraph.timeNanoSecs)
						self.nodes[shadowNodeInShadowTree]["toTime"] = newGraph.timeNanoSecs
						self.__addEdge(shadowNodeInShadowTree, shadowNodeInNowGraph, isTemporal=True)
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> bool:
		if self.length > 0 and graph.timeNanoSecs < self.__history[-1].timeNanoSecs:
				Logger().info("OUT OF ORDER graph --> %.3f vs %.3f" % ((graph.timeNanoSecs / SensorRegion.NANO_CONSTANT, self.__history[0].timeNanoSecs / SensorRegion.NANO_CONSTANT)))
				return False

		for node in graph.nodes:
			temporalName = self.generateTemporalName(node, graph.timeNanoSecs)
			self.__addNode(temporalName, graph.timeNanoSecs)
			self.shallowCopyNode(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self.generateTemporalName(edge[0], graph.timeNanoSecs)
			to = self.generateTemporalName(edge[1], graph.timeNanoSecs)
			self.__addEdge(frm, to, False)

		graph.render(self.__rvizPublishers["connectivity_graph"])
		if self.length > 0 and EventAggregator.isIsomorphic(self.__history[-1], graph):
			Logger().info("Updating last CGraph with the most recent isomorphic version --> %s" % repr(graph))
			self.__history[-1] = graph
		else:
			Logger().info("Appending graph to history --> %s" % repr(graph))
			self.__history.append(graph)
		self.render()
		return True

	def __iterateRegularRegion(self, lastCGraph: ConnectivityGraph, regularRegion: RegularDynamicRegion[RegionTypeY], r1Past: RegionTypeX, r1Now: RegionTypeX, checked: Set[Tuple[str, str]]) -> List[CollisionInterval[RegionTypeX, RegionTypeY]]:
		intervals: List[CollisionInterval[RegionTypeX, RegionTypeY]] = []
		for rName in regularRegion:
			if rName == r1Now.name: continue
			r2Now = regularRegion[rName]
			if (r1Now.name, r2Now.name) in checked: continue
			checked.add((r1Now.name, r2Now.name))
			checked.add((r2Now.name, r1Now.name))
			r2Past = lastCGraph.getRegion(r2Now)
			r2Past = r2Now if r2Past is None else r2Past
			intervals += CtCd.estimateCollisionsIntervals((r1Past, r1Now), (r2Past, r2Now), self.__rvizPublishers["continuous_time_collision_detection"])
		return intervals

	def multiPartiteLayout(self) -> Dict[str, Tuple[float, float]]:
		y = 0
		pos: Dict[str, Tuple[float, float]] = {}
		for subG in self.history:
			keys = sorted(subG.nodes, key=lambda n: subG.nodes[n]["region"].interior.centroid.x)
			i = 0
			for n in keys:
				nn = self.generateTemporalName(n , subG.timeNanoSecs)
				pos[nn] = (i / len(subG.nodes), y)
				i += 1
			y += 2
		return pos

	def bfs(self, start: str, goalFunc: NfaLambda.NfaLambdaFunc) -> Union[List[str], None]:
		q = Queue()
		visited = set()
		q.put([start])
		while not q.empty():
			path = q.get()
			n = path[-1]
			visited.add(n)
			nodeData = self.nodes[n]
			spaceTimeSetOfNode = ProjectiveSpaceTimeSet(nodeData["region"].interior, TimeRegion(nodeData["fromTime"], nodeData["toTime"], True, True))
			# if nodeData["type"] == "sensor": continue # FIXME: check if there is a track head here
			if goalFunc(spaceTimeSetOfNode):
				return path
			for child in self.adj[n]:
				if child in visited: continue
				newPath = list(path)
				newPath.append(child)
				q.put(newPath)
		return None

	def updateNamedRegions(self, timeNanoSecs: int, regions: List[SymbolRegion]) -> None:
		return

	def updateSensors(self, timeNanoSecs: int, regions: List[SensorRegion], symbols: Dict[str, Symbol] = {}) -> None:
		if self.length == 0:
			Logger().error("Initial CGraph cannot be created from from sensors.")
			return

		lastCGraph = self.history[-1]
		# There are there possibilities for sensor names. We have decided to require specific messages for S1.
		# 1. S1 is in pastSensors but not in nowSensors ----> S1 has turned off -> the shadows around S1 have merged.
		# 2. S2 is not in pastSensors but is in nowSensors -> S2 has turned on --> the shadows around S2 have splitted.
		# 3. S3 is in pastSensors but not in nowSensors ----> S3 has moved ------> the shadows around S3 have evolved.

		if len(lastCGraph.fieldOfView) > 0:
			fovRegions = FieldOfView(regions)
			for rName in lastCGraph.fieldOfView:
				fovRegions.addConnectedComponent(lastCGraph.fieldOfView[rName])
		else:
			fovRegions = regions

		nowCGraph = ConnectivityGraph(
			timeNanoSecs=timeNanoSecs,
			mapRegions=lastCGraph.mapPerimeter,
			fovRegions=fovRegions,
			rvizPublisher=self.__rvizPublishers["connectivity_graph"]
		)
		Logger().info("last = %s, now = %s" % (repr(lastCGraph), repr(nowCGraph)))
		checked: Set[Tuple[str, str]] = set()
		intervals: List[CollisionInterval[SensorRegion, MapRegion]] = []
		for r1Now in regions:
			r1Past = lastCGraph.getRegion(r1Now)
			r1Past = r1Now if r1Past is None else r1Past


			r2Past = MapRegion(lastCGraph.timeNanoSecs, Geometry.getGeometryCoords(nowCGraph.mapPerimeter.interior), timeNanoSecs=lastCGraph.timeNanoSecs)
			r2Now = MapRegion(nowCGraph.timeNanoSecs, Geometry.getGeometryCoords(nowCGraph.mapPerimeter.interior), timeNanoSecs=nowCGraph.timeNanoSecs)
			intervals += CtCd.estimateCollisionsIntervals((r1Past, r1Now), (r2Past, r2Now), self.__rvizPublishers["continuous_time_collision_detection"])
			intervals += self.__iterateRegularRegion(lastCGraph, nowCGraph.fieldOfView, r1Past, r1Now, checked)
			intervals += self.__iterateRegularRegion(lastCGraph, nowCGraph.symbols, r1Past, r1Now, checked)

		intervals = CtCd.refineCollisionIntervals(intervals)
		eventGraphs = EventAggregator.aggregateCollisionEvents(intervals, lastCGraph, nowCGraph, symbols)
		Logger().info("Intervals = %d, Graphs = %d" % (len(intervals), len(eventGraphs)))
		for graph in eventGraphs:
			if self.__appendToHistory(graph): self.__connectGraphsTemporally(graph)
		return

	def updateMap(self, timeNanoSecs: int, regions: List[MapRegion]) -> None:
		if self.length == 0:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, mapRegions=regions, fovRegions=[], rvizPublisher=self.__rvizPublishers["connectivity_graph"])
			self.__appendToHistory(currentConnectivityG)
		return

	def getShadowAreaMarkers(self) -> Sequence[Marker]:
		markers = []
		if self.length == 0: return []
		lastCGraph = self.history[-1]
		for shadowName in lastCGraph.shadows:
			shadow = lastCGraph.shadows[shadowName]
			textCoords = shadow.interior.centroid
			textCoords = (textCoords.x, textCoords.y)
			timerText = "area(%s) = %.3f" % (shadowName, shadow.interior.area)
			timerMarker = RViz.createText("rt_st_%s" % shadowName, textCoords, timerText, KnownColors.RED, fontSize=7.5)
			markers.append(timerMarker)
		return markers

	def render(self) -> None:
		publisher = self.__rvizPublishers["connectivity_graph"]
		if publisher is None: return
		if self.length == 0: return
		markerArray = MarkerArray()
		lastCGraph = self.history[-1]
		timerCoords = Geometry.findBottomLeft(lastCGraph.mapPerimeter.envelopePolygon)
		timerCoords = Geometry.addCoords(timerCoords, (20, 20))
		timerText = "T = %.3f" % (float(lastCGraph.timeNanoSecs) / float(SensorRegion.NANO_CONSTANT))
		timerMarker = RViz.createText("rt_st_time", timerCoords, timerText, KnownColors.RED, fontSize=7.5)
		AppendMessage(markerArray.markers, timerMarker)

		timerCoords = Geometry.findBottomLeft(lastCGraph.mapPerimeter.envelopePolygon)
		timerCoords = Geometry.addCoords(timerCoords, (20, 10))
		timerText = "D = %d" % self.length
		timerMarker = RViz.createText("rt_st_depth", timerCoords, timerText, KnownColors.RED, fontSize=7.5)
		AppendMessage(markerArray.markers, timerMarker)

		shadowAreaMarkers = self.getShadowAreaMarkers()
		for marker in shadowAreaMarkers: AppendMessage(markerArray.markers, marker)

		Logger().info("Rendering %d ShadowTree markers." % len(markerArray.markers))
		publisher.publish(markerArray)
		return

	@staticmethod
	def shallowCopyNode(frm: dict, to: dict) -> None:
		for key in frm: to[key] = frm[key]
		return

	@staticmethod
	def generateTemporalName(name: str, time: int) -> str:
		""" Use this function to safely generate (typo-free and in uniform format) the names for the temporal edges. """
		return "%s-%d" % (name, time)
