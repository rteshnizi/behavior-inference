from math import inf
from queue import Queue
from typing import Dict, List, Literal, Tuple, Union

import networkx as nx
from visualization_msgs.msg import MarkerArray

from rt_bi_core.BehaviorAutomaton.Lambda import NfaLambda
from rt_bi_core.BehaviorAutomaton.SpaceTime import ProjectiveSpaceTimeSet
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.BehaviorAutomaton.TimeRegion import TimeInterval
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_core.Eventifier.EventAggregator import EventAggregator
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.Ros import Logger, Publisher
from rt_bi_utils.RViz import KnownColors, RViz

Queue.__repr__ = lambda q: repr(q.queue)

class ShadowTree(nx.DiGraph):
	"""
		The implementation of a Shadow Tree in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	SUBMODULE_TYPES = Literal["connectivity_graph", "continuous_time_collision_detection"]
	SUBMODULES: Tuple[SUBMODULE_TYPES] = ("connectivity_graph", "continuous_time_collision_detection")

	def __init__(self, rvizPublishers: Dict[SUBMODULE_TYPES, Union[Publisher, None]]):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(name="ShadowTree V3")
		self.__history: List[ConnectivityGraph] = []
		self.componentEvents: List[List[Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """
		self.__rvizPublishers = rvizPublishers

	@property
	def history(self) -> Tuple[ConnectivityGraph, ...]:
		"""The list of the graphs."""
		return tuple(self.__history)

	@property
	def length(self) -> int:
		"""The length of the shadow tree history stack."""
		return len(self.__history)

	def __repr__(self) -> str:
		return self.name

	def __getLowerAndUpperNode(self, n1: str, n2: str) -> Tuple[str, str]:
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def __addNode(self, n: str, timeNanoSecs: int) -> None:
		self.add_node(n)
		self.nodes[n]["fromTime"] = timeNanoSecs
		self.nodes[n]["toTime"] = inf # This will be updated accordingly when adding temporal edges
		return

	def __addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime = None, toTime = None) -> None:
		(lower, upper) = self.__getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)
		return

	def __shadowsAreConnectedTemporally(self, pastGraph: ConnectivityGraph, nowGraph: ConnectivityGraph, pastShadow: ConnectivityGraph.NodeContent, nowShadow: ConnectivityGraph.NodeContent) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the translation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		pastRegion = pastShadow["region"]
		nowRegion = nowShadow["region"]
		intersectionOfShadows = Geometry.intersection(pastRegion.interior, nowRegion.interior)
		for fov in pastGraph.fieldOfView:
			if fov not in pastGraph.nodes: continue
			pastSensorRegion: SensorRegion = pastGraph.nodes[fov]["region"]
			if fov not in nowGraph.nodes: continue
			nowSensorRegion: SensorRegion = nowGraph.nodes[fov]["region"]
			pastCoords = Geometry.getGeometryCoords(pastSensorRegion.interior)
			nowCoords = Geometry.getGeometryCoords(nowSensorRegion.interior)
			transformation: AffineTransform = Geometry.getAffineTransformation(pastCoords, nowCoords, nowSensorRegion.centerOfRotation)
			ps = []
			for e in nowSensorRegion.edges:
				edgeC = nowSensorRegion.edges[e]
				edgeP = pastSensorRegion.getEquivalentEdge(edgeC, transformation, nowSensorRegion.centerOfRotation)
				if edgeP is None:
					raise AssertionError("No equivalent edge found based on transformation.")
				polygon = Polygon([edgeP.coords[0], edgeP.coords[1], edgeC.coords[1], edgeC.coords[0]])
				ps.append(polygon)
			u = Geometry.union(ps)
			p = intersectionOfShadows.difference(u)
			if p.is_valid and (not p.is_empty) and p.area > Geometry.EPSILON:
				return True
		return False

	def __connectGraphsTemporally(self, pastGraph: ConnectivityGraph, nowGraph: ConnectivityGraph) -> None:
		# Add temporal edges between FOVs
		for sensor in pastGraph.fieldOfView:
			fovNodeInShadowTree = self.generateTemporalName(sensor, pastGraph.timeNanoSecs)
			if fovNodeInShadowTree not in self.nodes: continue
			fovNodeInCurrentGraph = self.generateTemporalName(sensor, nowGraph.timeNanoSecs)
			if fovNodeInCurrentGraph not in self.nodes: continue
			self.__addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
		# Add temporal edges between symbols
		for symNode in pastGraph.symbols:
			symNodeInShadowTree = self.generateTemporalName(symNode, pastGraph.timeNanoSecs)
			symNodeInCurrentGraph = self.generateTemporalName(symNode, nowGraph.timeNanoSecs)
			self.__addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
		# Add temporal edges between shadows
		for shadowNodeInPastGraph in pastGraph.shadows:
			for shadowNodeInNowGraph in nowGraph.shadows:
				previousShadowNodeRegion: ShadowRegion = pastGraph.nodes[shadowNodeInPastGraph]["region"]
				currentShadowNodeRegion: ShadowRegion = nowGraph.nodes[shadowNodeInNowGraph]["region"]
				if Geometry.intersects(previousShadowNodeRegion.interior, currentShadowNodeRegion.interior):
					if self.__shadowsAreConnectedTemporally(pastGraph, nowGraph, pastGraph.nodes[shadowNodeInPastGraph], nowGraph.nodes[shadowNodeInNowGraph]):
						shadowNodeInShadowTree = self.generateTemporalName(shadowNodeInPastGraph, pastGraph.timeNanoSecs)
						shadowNodeInNowGraph = self.generateTemporalName(shadowNodeInNowGraph, nowGraph.timeNanoSecs)
						self.nodes[shadowNodeInShadowTree]["toTime"] = nowGraph.timeNanoSecs
						self.__addEdge(shadowNodeInShadowTree, shadowNodeInNowGraph, isTemporal=True)
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		for node in graph.nodes:
			temporalName = self.generateTemporalName(node, graph.timeNanoSecs)
			self.__addNode(temporalName, graph.timeNanoSecs)
			self.shallowCopyNode(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self.generateTemporalName(edge[0], graph.timeNanoSecs)
			to = self.generateTemporalName(edge[1], graph.timeNanoSecs)
			self.__addEdge(frm, to, False)
		self.__history.append(graph)
		graph.render()
		self.render()
		return

	def multiPartiteLayout(self) -> Dict[str, Tuple[float, float]]:
		y = 0
		pos: Dict[str, Tuple[float, float]] = {}
		for subG in self.history:
			keys = sorted(subG.nodes, key=lambda n: subG.nodes[n]["region"].interior.centroid.x)
			i = 0
			for n in keys:
				nn = self.generateTemporalName(n , subG.timeNanoSecs)
				pos[nn] = [0, 0]
				pos[nn][0] = i / len(subG.nodes)
				pos[nn][1] = y
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
			spaceTimeSetOfNode = ProjectiveSpaceTimeSet(nodeData["region"].interior, TimeInterval(nodeData["fromTime"], nodeData["toTime"], True, True))
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
		if len(self.history) == 0:
			Logger().error("Initial CGraph cannot be created from from sensors.")
			return

		pastCGraph = self.history[-1]

		# There are there possibilities for sensor names. We have decided to require specific messages for S1.
		# 1. S1 is in pastSensors but not in nowSensors ----> S1 has turned off -> the shadows around S1 have merged.
		# 2. S2 is not in pastSensors but is in nowSensors -> S2 has turned on --> the shadows around S2 have splitted.
		# 3. S3 is in pastSensors but not in nowSensors ----> S3 has moved ------> the shadows around S3 have evolved.

		if len(pastCGraph.fieldOfView) > 0:
			fovRegions = FieldOfView(regions)
			for rName in pastCGraph.fieldOfView:
				fovRegions.addConnectedComponent(pastCGraph.fieldOfView[rName])
		else:
			fovRegions = regions

		nowCGraph = ConnectivityGraph(timeNanoSecs=timeNanoSecs, mapRegions=pastCGraph.mapPerimeter, fovRegions=fovRegions, rvizPublisher=self.__rvizPublishers["connectivity_graph"])
		# 1. Sensors that have turned off -> the shadows around S1 have merged.
		turnedOffSensors = pastCGraph.fieldOfView - nowCGraph.fieldOfView
		if len(turnedOffSensors) > 0:
			Logger().info("S1.1.1 -> Sensors turned off: %s" % repr(turnedOffSensors))
		else:
			Logger().info("S1.1.2 -> No recently turned off sensor.")

		# 2. Sensors that have turned on --> the shadows around S2 have splitted. No real algorithm needed, the code does it, on its own.
		turnedOnSensors = nowCGraph.fieldOfView - pastCGraph.fieldOfView
		if len(turnedOnSensors) > 0:
			Logger().info("S2.1.1 -> Sensors turned on: %s" % repr(turnedOnSensors))
			self.__appendToHistory(nowCGraph)
		else:
			Logger().info("S2.1.2 -> No newly turned on sensor.")

		# 3. Sensors that have moved ------> the shadows around S3 have evolved.
		evolvedSensors = pastCGraph.fieldOfView & nowCGraph.fieldOfView
		if len(evolvedSensors) > 0:
			Logger().info("S3.1.1 -> Sensors evolved: %s" % repr(evolvedSensors))

			events = CtCd.estimateIntermediateCollisionsWithPolygon(pastCGraph.fieldOfView, nowCGraph.fieldOfView, pastCGraph.mapPerimeter.interior, self.__rvizPublishers["continuous_time_collision_detection"])
			eventGraphs = EventAggregator.obtainCollisionCGraphs(events, pastCGraph, nowCGraph, symbols)
			for graph in eventGraphs:
				self.__appendToHistory(graph)
				eventGraphs = self.__connectGraphsTemporally(pastCGraph, graph)
				pastCGraph = graph

		return

	def updateMap(self, timeNanoSecs: int, regions: List[MapRegion]) -> None:
		if self.length == 0:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, mapRegions=regions, fovRegions=[], rvizPublisher=self.__rvizPublishers["connectivity_graph"])
			self.__appendToHistory(currentConnectivityG)
		return

	def render(self) -> None:
		publisher = self.__rvizPublishers["connectivity_graph"]
		if publisher is None: return
		message = MarkerArray()
		lastCGraph = self.history[-1]
		timerCoords = Geometry.findBottomLeft(lastCGraph.mapPerimeter.envelopePolygon)
		timerCoords = Geometry.addCoords(timerCoords, (20, 20))
		timerText = "T = %.3f" % (float(lastCGraph.timeNanoSecs) / float(SensorRegion.NANO_CONSTANT))
		timerMarker = RViz.CreateText("rt_st_time", timerCoords, timerText, KnownColors.RED, fontSize=7.5)
		message.markers.append(timerMarker)

		timerCoords = Geometry.findBottomLeft(lastCGraph.mapPerimeter.envelopePolygon)
		timerCoords = Geometry.addCoords(timerCoords, (20, 10))
		timerText = "D = %d" % self.length
		timerMarker = RViz.CreateText("rt_st_depth", timerCoords, timerText, KnownColors.RED, fontSize=7.5)
		message.markers.append(timerMarker)

		Logger().info("Rendering %d ShadowTree markers." % len(message.markers))
		publisher.publish(message)
		return

	@staticmethod
	def shallowCopyNode(frm: dict, to: dict) -> None:
		for key in frm: to[key] = frm[key]
		return

	@staticmethod
	def generateTemporalName(name: str, time: int) -> str:
		""" Use this function to safely generate (typo-free and in uniform format) the names for the temporal edges. """
		return "%s-%d" % (name, time)
