from typing import Dict, List, Literal, Sequence, Tuple, TypeVar, Union

import networkx as nx
from networkx.classes.reportviews import OutEdgeView
from visualization_msgs.msg import MarkerArray

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import CollisionInterval, ContinuousTimeCollisionDetection as CtCd
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion
from rt_bi_eventifier.Model.EventAggregator import EventAggregator
from rt_bi_eventifier.Model.FieldOfView import FieldOfView
from rt_bi_eventifier.Model.SymbolRegions import SymbolRegions
from rt_bi_interfaces.msg import Adjacency, ComponentEvent, Events, Graph
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.NetworkX import NxUtils
from rt_bi_utils.Ros import AppendMessage, Log, Logger, Publisher
from rt_bi_utils.RViz import ColorNames, RViz

# CSpell: ignore reportviews

RegionTypeX = TypeVar("RegionTypeX", SensorRegion, SymbolRegion, MapRegion)
RegionTypeY = TypeVar("RegionTypeY", SensorRegion, SymbolRegion, MapRegion)

class ShadowTree(nx.DiGraph):
	"""
		The implementation of a Shadow Tree in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	SUBMODULE_TYPES = Literal["connectivity_graph", "continuous_time_collision_detection", "shadow_tree"]
	SUBMODULES: Tuple[SUBMODULE_TYPES, ...] = ("connectivity_graph", "continuous_time_collision_detection", "shadow_tree")
	TOPICS = Literal["eventifier_graph", "eventifier_event"]
	RENDER_DELTA_X = 150
	RENDER_DELTA_Y = 50
	RENDER_RADIUS = 10
	RENDER_FONT_SIZE = 7.5

	def __init__(self, eventPublisher: Dict[TOPICS, Publisher], rvizPublishers: Dict[SUBMODULE_TYPES, Union[Publisher, None]]):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(name="ShTr-V3")
		self.__history: List[ConnectivityGraph] = []
		self.__timeToCGraph: Dict[int, int] = {}
		""" A map from time of the graph to its index in the history list. """
		self.componentEvents: List[List[Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """
		self.__eventPublisher = eventPublisher
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

	@staticmethod
	def __generateTemporalName(name: str, time: int) -> str:
		""" Use this function to safely generate (typo-free and in uniform format) the names for the temporal edges. """
		return "%s-%d" % (name, time)

	def __getCGraph(self, timeNanoSecs: int) -> ConnectivityGraph:
		ind = self.__timeToCGraph[timeNanoSecs]
		return self.history[ind]

	def __addNode(self, node: str, timeNanoSecs: int) -> str:
		temporalName = self.__generateTemporalName(node, timeNanoSecs)
		# Logger().warn(f"Adding to ST {temporalName}")
		self.add_node(temporalName, regionName=node, timeNanoSecs=timeNanoSecs)
		return temporalName

	def __addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime: Union[int, None] = None, toTime: Union[int, None] = None) -> None:
		if isTemporal and (fromTime is None or toTime is None):
			raise ValueError("Temporal edge requires from and to times.")
		if n1 == n2: Logger().warn("Self loop edge added to ShadowTree for %s" % n1)
		if n1 not in self.nodes: Logger().error(f"Adding edge to none-existing node-1 {n1}")
		if n2 not in self.nodes: Logger().error(f"Adding edge to none-existing node-2 {n2}")
		self.add_edge(n1, n2, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)
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
		if isinstance(pastRegion, AffineRegion) and isinstance(nowRegion, AffineRegion):
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

	def __connectGraphsTemporally(self, fromGraph: ConnectivityGraph, toGraph: ConnectivityGraph) -> None:
		# Add temporal edges between FOVs
		for sensor in fromGraph.fieldOfView:
			fovNodeInShadowTree = self.__generateTemporalName(sensor, fromGraph.timeNanoSecs)
			if fovNodeInShadowTree not in self.nodes: continue
			fovNodeInCurrentGraph = self.__generateTemporalName(sensor, toGraph.timeNanoSecs)
			if fovNodeInCurrentGraph not in self.nodes: continue
			self.__addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True, fromTime=fromGraph.timeNanoSecs, toTime=toGraph.timeNanoSecs)
		# Add temporal edges between symbols
		for symNode in fromGraph.symbols:
			fromTemporalName = self.__generateTemporalName(symNode, fromGraph.timeNanoSecs)
			splittedNodes = toGraph.symbols.getRegionsByShortName(fromGraph.symbols[symNode].shortName)
			for n in splittedNodes:
				toTemporalName = self.__generateTemporalName(n.name, toGraph.timeNanoSecs)
				self.__addEdge(fromTemporalName, toTemporalName, isTemporal=True, fromTime=fromGraph.timeNanoSecs, toTime=toGraph.timeNanoSecs)
		# Add temporal edges between shadows
		for shadowNodeInPastGraph in fromGraph.shadows:
			for shadowNodeInNowGraph in toGraph.shadows:
				previousShadowNodeRegion: ShadowRegion = fromGraph.nodes[shadowNodeInPastGraph]["region"]
				currentShadowNodeRegion: ShadowRegion = toGraph.nodes[shadowNodeInNowGraph]["region"]
				if Geometry.intersects(previousShadowNodeRegion.interior, currentShadowNodeRegion.interior):
					if self.__shadowsAreConnectedTemporally(fromGraph, toGraph, fromGraph.nodes[shadowNodeInPastGraph], toGraph.nodes[shadowNodeInNowGraph]):
						shadowNodeInShadowTree = self.__generateTemporalName(shadowNodeInPastGraph, fromGraph.timeNanoSecs)
						shadowNodeInNowGraph = self.__generateTemporalName(shadowNodeInNowGraph, toGraph.timeNanoSecs)
						self.__addEdge(shadowNodeInShadowTree, shadowNodeInNowGraph, isTemporal=True, fromTime=fromGraph.timeNanoSecs, toTime=toGraph.timeNanoSecs)
		return

	def __replaceInHistory(self, indexToReplace: int, graph: ConnectivityGraph) -> None:
		for n in self.__history[indexToReplace].nodes:
			node = self.__generateTemporalName(n, self.__history[indexToReplace].timeNanoSecs)
			self.remove_node(node)
		self.__timeToCGraph.pop(self.__history[indexToReplace].timeNanoSecs)
		self.__history[indexToReplace] = graph
		self.__timeToCGraph[graph.timeNanoSecs] = indexToReplace
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		if self.length > 0 and graph.timeNanoSecs < self.__history[-1].timeNanoSecs:
				Log("Older graph than latest in history --> %d vs %d" % ((graph.timeNanoSecs, self.__history[-1].timeNanoSecs)))
				return

		for node in graph.nodes:
			self.__addNode(node, graph.timeNanoSecs)
		for edge in graph.edges:
			frm = self.__generateTemporalName(edge[0], graph.timeNanoSecs)
			to = self.__generateTemporalName(edge[1], graph.timeNanoSecs)
			self.__addEdge(frm, to, isTemporal=False)

		graph.render(self.__rvizPublishers["connectivity_graph"])
		if self.length > 0 and EventAggregator.isIsomorphic(self.__history[-1], graph):
			# if graph.timeNanoSecs == self.__history[-1].timeNanoSecs:
			# 	return # Identical graph
			Log("Updating last CGraph with the most recent isomorphic version --> %s" % repr(graph))
			self.__replaceInHistory(self.length - 1, graph)
		else:
			Log("Appending graph to history --> %s" % repr(list(graph.nodes)))
			self.__history.append(graph)
			self.__timeToCGraph[graph.timeNanoSecs] = self.length - 1
			# self.__publishGraphEvent()
		if self.length > 1: self.__connectGraphsTemporally(self.__history[-2], self.__history[-1])
		self.render()

	def __publishGraphEvent(self) -> None:
		if self.length == 1:
			adjacency: List[Tuple[str, List[str]]] = [(n, list(adjDict.keys())) for (n, adjDict) in self.history[0].adjacency()]
			graphMsg = Graph()
			for (node, neighbors) in adjacency:
				AppendMessage(graphMsg.vertices, node)
				adjacencyMsg = Adjacency()
				for neighbor in neighbors:
					AppendMessage(adjacencyMsg.neighbors, neighbor)
				AppendMessage(graphMsg.adjacency, adjacencyMsg)
			self.__eventPublisher["eventifier_graph"].publish(graphMsg)
			return
		lastCGraph = self.history[-1]
		prevCGraph = self.history[-2]
		lastCGraphNodes = [self.__generateTemporalName(n, lastCGraph.timeNanoSecs) for n in lastCGraph.nodes]
		prevCGraphNodes = [self.__generateTemporalName(n, prevCGraph.timeNanoSecs) for n in prevCGraph.nodes]
		appearedNodes: List[str] = []
		disappearedNodes: List[str] = []
		mergedNodes: List[Tuple[str, List[str]]] = []
		splittedNodes: List[Tuple[str, List[str]]] = []
		for n in lastCGraphNodes:
			if self.in_degree(n) == 0: appearedNodes.append(self.nodes[n]["regionName"])
			if self.in_degree(n) > 1:
				inNeighbors = [edge[1] for edge in self.in_edges(n)]
				mergedNodes.append((self.nodes[n]["regionName"], inNeighbors))
		for n in prevCGraphNodes:
			if self.out_degree(n) == 0: disappearedNodes.append(self.nodes[n]["regionName"])
			if self.out_degree(n) > 1:
				outNeighbors = [edge[1] for edge in self.out_edges(n)]
				splittedNodes.append((self.nodes[n]["regionName"], outNeighbors))
		eventsMsg = Events()
		event = ComponentEvent()
		event.ros_time = lastCGraph.timeNanoSecs
		event.type = "A"
		for n in appearedNodes: AppendMessage(event.after, n)
		AppendMessage(eventsMsg.component_events, event)

		event = ComponentEvent()
		event.ros_time = lastCGraph.timeNanoSecs
		event.type = "D"
		for n in disappearedNodes: AppendMessage(event.before, n)
		AppendMessage(eventsMsg.component_events, event)

		event = ComponentEvent()
		event.ros_time = lastCGraph.timeNanoSecs
		event.type = "M"
		for n in mergedNodes:
			AppendMessage(event.before, n[0])
			for inNode in n[1]: AppendMessage(event.after, inNode)
		AppendMessage(eventsMsg.component_events, event)

		event = ComponentEvent()
		event.ros_time = lastCGraph.timeNanoSecs
		event.type = "D"
		for n in splittedNodes:
			AppendMessage(event.before, n[0])
			for inNode in n[1]: AppendMessage(event.after, inNode)
		AppendMessage(eventsMsg.component_events, event)

		self.__eventPublisher["eventifier_event"].publish(eventsMsg)
		return

	def __nodeSortKey(self, n: str) -> Tuple[int, float]:
		try:
			timeNanoSecs = self.nodes[n]["timeNanoSecs"]
			regionName: str = self.nodes[n]["regionName"]
			cGraph: ConnectivityGraph = self.__getCGraph(timeNanoSecs)
			region: AffineRegion = cGraph.nodes[regionName]["region"]
			x: float = region.interior.centroid.x
			return (timeNanoSecs, x)
		except Exception as e:
			Logger().error(n)
			raise e

	def __multiPartiteLayout(self) -> NxUtils.GraphLayout:
		keys = sorted(self.nodes, key=self.__nodeSortKey)
		pos: NxUtils.GraphLayout = {}
		y = 0
		x = 0
		timeNanoSecs: int = 0
		for n in keys:
			if self.nodes[n]["timeNanoSecs"] > timeNanoSecs:
				timeNanoSecs = self.nodes[n]["timeNanoSecs"]
				x = 0
				y += self.RENDER_DELTA_Y
			pos[n] = (x, y)
			x += self.RENDER_DELTA_X
		return pos

	def __getTextMarkers(self, markerArray: MarkerArray) -> MarkerArray:
		lastCGraph = self.history[-1]
		timerCoords = Geometry.findBottomLeft(lastCGraph.mapRegions.envelopePolygon)
		timerCoords = Geometry.addCoords(timerCoords, (20, 20))
		timerText = "T = %d" % (lastCGraph.timeNanoSecs)
		timerMarker = RViz.createText("rt_st_time", timerCoords, timerText, ColorNames.RED, fontSize=7.5)
		AppendMessage(markerArray.markers, timerMarker)

		timerCoords = Geometry.findBottomLeft(lastCGraph.mapRegions.envelopePolygon)
		timerCoords = Geometry.addCoords(timerCoords, (20, 10))
		timerText = "D = %d" % self.length
		timerMarker = RViz.createText("rt_st_depth", timerCoords, timerText, ColorNames.RED, fontSize=7.5)
		AppendMessage(markerArray.markers, timerMarker)
		return markerArray

	def __getNodeMarkers(self, markerArray: MarkerArray) -> MarkerArray:
		if len(self.nodes) == 0: return markerArray
		nodePositions = self.__multiPartiteLayout()
		for n in nodePositions:
			cGraph: ConnectivityGraph = self.__getCGraph(self.nodes[n]["timeNanoSecs"])
			regionName: str = self.nodes[n]["regionName"]
			region: AffineRegion = cGraph.nodes[regionName]["region"]
			if region.regionType == AffineRegion.RegionType.SHADOW:
				outlineColor = ColorNames.LIGHT_GREY
			elif region.regionType == AffineRegion.RegionType.SENSING:
				outlineColor = ColorNames.GREEN
			elif region.regionType == AffineRegion.RegionType.SYMBOL:
				outlineColor = ColorNames.PURPLE
			else:
				outlineColor = ColorNames.RED
			coords = nodePositions[n]
			marker = RViz.createCircle("rt_st_%s" % n, centerX=coords[0], centerY=coords[1], radius=self.RENDER_RADIUS, outline=outlineColor)
			AppendMessage(markerArray.markers, marker)
			marker = RViz.createText("rt_st_%s_txt" % n, coords, regionName, ColorNames.RED, fontSize=self.RENDER_FONT_SIZE)
			AppendMessage(markerArray.markers, marker)
		return markerArray

	def __getEdgeMarkers(self, markerArray: MarkerArray) -> MarkerArray:
		if len(self.nodes) == 0: return markerArray
		nodePositions = self.__multiPartiteLayout()
		outEdgeView: OutEdgeView = self.out_edges()
		for (frm, to) in outEdgeView:
			edgeData = outEdgeView[frm, to]
			strId = repr((frm, to))
			color = ColorNames.DARK_CYAN if ("isTemporal" in edgeData and edgeData["isTemporal"]) else ColorNames.DARK_MAGENTA
			(dx, dy) = Geometry.subtractCoords(nodePositions[to], nodePositions[frm])
			dVect = Geometry.getUnitVector(dx, dy)
			dVect = Geometry.scaleCoords(dVect, self.RENDER_RADIUS)
			fromCoords = Geometry.addCoords(nodePositions[frm], dVect)
			toCoords = Geometry.subtractCoords(nodePositions[to], dVect)
			marker = RViz.createLine(strId, coords=[fromCoords, toCoords], outline=color, width=2)
			AppendMessage(markerArray.markers, marker)
		return markerArray

	def __mergeWithRegularRegion(self, regions: List[RegionTypeX], regularRegion: RegularAffineRegion[RegionTypeX]) -> Tuple[Dict[str, List[RegionTypeX]], int]:
		regionsDict: Dict[str, List[RegionTypeX]] = {}
		maxT = 0
		for r1 in regions:
			if r1.shortName not in regionsDict: regionsDict[r1.shortName] = []
			regionsDict[r1.shortName].append(r1)
			maxT = maxT if maxT > r1.timeNanoSecs else r1.timeNanoSecs
		if len(regularRegion) > 0:
			for r2ExtendedName in regularRegion:
				r2 = regularRegion[r2ExtendedName]
				if r2.shortName not in regionsDict: regionsDict[r2.shortName] = []
				if r2.regionType == SymbolRegion.RegionType.SYMBOL:
					r2 = SymbolRegion(centerOfRotation=r2.centerOfRotation, idNum=r2.idNum, envelope=r2.envelope, timeNanoSecs=r2.timeNanoSecs, overlappingRegionId=0, overlappingRegionType=SymbolRegion.RegionType.BASE)
				regionsDict[r2.shortName].append(r2) # type: ignore
				maxT = maxT if maxT > regularRegion[r2ExtendedName].timeNanoSecs else regularRegion[r2ExtendedName].timeNanoSecs
		for r in regionsDict:
			regionsDict[r] = sorted(regionsDict[r], key=lambda r: r.timeNanoSecs)
		return (regionsDict, maxT)

	def updateAffineRegions(self, sensors: List[SensorRegion], symbols: List[SymbolRegion]) -> None:
		if self.length == 0:
			Logger().error("Initial CGraph must be created from map.")
			return

		Log("=================================================================================================")
		lastCGraph = self.history[-1]
		# There are there possibilities for sensor names. We have decided to require specific messages for S1.
		# 1. S1 is in pastSensors but not in nowSensors ----> S1 has turned off -> the shadows around S1 have merged.
		# 2. S2 is not in pastSensors but is in nowSensors -> S2 has turned on --> the shadows around S2 have splitted.
		# 3. S3 is in pastSensors but not in nowSensors ----> S3 has moved ------> the shadows around S3 have evolved.

		(sensorRegions, maxT1) = self.__mergeWithRegularRegion(sensors, lastCGraph.fieldOfView)
		(symbolRegions, maxT2) = self.__mergeWithRegularRegion(symbols, lastCGraph.symbols)
		maxTimeNanoSecs = max(maxT1, maxT2)
		intervals = CtCd.estimateCollisionIntervals(sensorRegions, symbolRegions, lastCGraph.mapRegions, maxTimeNanoSecs, self.__rvizPublishers["continuous_time_collision_detection"])
		intervals = CtCd.refineCollisionIntervals(intervals)
		Log(f"Intervals = {len(intervals)}")
		eventGraphs = EventAggregator.aggregateCollisionEvents(intervals, lastCGraph, lastCGraph.mapRegions)
		if len(eventGraphs) == 0:
			nowCGraph = ConnectivityGraph(maxTimeNanoSecs, lastCGraph.mapRegions, [sensorRegions[r][-1] for r in sensorRegions], [symbolRegions[r][-1] for r in symbolRegions])
			eventGraphs = [nowCGraph]
		Log(f"Aggregated = {repr(eventGraphs)}")
		for graph in eventGraphs: self.__appendToHistory(graph)
		return

	def updateMap(self, timeNanoSecs: int, regions: List[MapRegion]) -> None:
		if self.length == 0:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, mapRegions=regions, fovRegions=[], rvizPublisher=self.__rvizPublishers["connectivity_graph"])
			self.__appendToHistory(currentConnectivityG)
		return

	def render(self) -> None:
		publisher = self.__rvizPublishers["shadow_tree"]
		if publisher is None: return
		if self.length == 0: return
		markerArray = MarkerArray()
		AppendMessage(markerArray.markers, RViz.removeAllMarkers())
		markerArray = self.__getTextMarkers(markerArray)
		markerArray = self.__getNodeMarkers(markerArray)
		markerArray = self.__getEdgeMarkers(markerArray)
		publisher.publish(markerArray)
		return
