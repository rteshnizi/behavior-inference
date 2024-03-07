from dataclasses import dataclass
from typing import Any, Literal, Sequence, TypeAlias, cast

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RViz import ColorNames, RViz
from rt_bi_core.Spatial import GraphInputPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.ShadowPolygon import ShadowPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_eventifier.Model.EventAggregator import EventAggregator

_PolygonAlias: TypeAlias = SensingPolygon | ShadowPolygon

class ShadowTree(NxUtils.Graph[_PolygonAlias]):
	"""
		The implementation of a Shadow Tree in python as described in the dissertation.
	"""

	SUBMODULES = ("connectivity_graph", "continuous_time_collision_detection", "shadow_tree")
	SUBMODULE = Literal["connectivity_graph", "continuous_time_collision_detection", "shadow_tree"]
	"""The name of a ShadowTree sub-module publisher."""
	ST_TOPIC = Literal["eventifier_graph", "eventifier_event"]
	"""The name of a ShadowTree topic."""
	__RENDER_RADIUS = 10

	@dataclass(frozen=True)
	class NodeData(NxUtils.NodeData[_PolygonAlias]): ...

	def __init__(self, eventPublisher: dict[ST_TOPIC, Ros.Publisher], rvizPublishers: dict[SUBMODULE, Ros.Publisher | None]):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(rVizPublisher=rvizPublishers.pop("shadow_tree", None))
		# super().__init__(rVizPublisher=None)
		self.__history: list[ConnectivityGraph] = []
		self.componentEvents: list[list[Shapely.Polygon]] = []
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

	def getNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._multiPartiteLayout()
		for id in nodePositions:
			poly = self.getContent(id, "polygon")
			if poly.type == ShadowPolygon.type:
				outlineColor = ColorNames.GREY_LIGHT
			elif poly.type == SensingPolygon.type:
				outlineColor = ColorNames.GREEN
			elif poly.type == MovingPolygon.type:
				outlineColor = ColorNames.PURPLE
			else:
				outlineColor = ColorNames.RED
			coords = nodePositions[id]
			marker = RViz.createCircle(id, centerX=coords[0], centerY=coords[1], radius=self.__RENDER_RADIUS, outline=outlineColor)
			Ros.AppendMessage(markers, marker)
		return markers

	def getEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._multiPartiteLayout()
		outEdgeView: NxUtils.OutEdgeView = self.out_edges()
		for (frm, to) in outEdgeView:
			frm = cast(NxUtils.Id, frm)
			to = cast(NxUtils.Id, to)
			if frm == to: continue
			edgeData = outEdgeView[frm, to]
			color = ColorNames.CYAN_DARK if ("isTemporal" in edgeData and edgeData["isTemporal"]) else ColorNames.MAGENTA_DARK
			(dx, dy) = GeometryLib.subtractCoords(nodePositions[to], nodePositions[frm])
			dVect = GeometryLib.getUnitVector(dx, dy)
			dVect = GeometryLib.scaleCoords(dVect, self.__RENDER_RADIUS)
			fromCoords = GeometryLib.addCoords(nodePositions[frm], dVect)
			toCoords = GeometryLib.subtractCoords(nodePositions[to], dVect)
			marker = RViz.createLine(frm, coords=[fromCoords, toCoords], outline=color, width=2, idSuffix=repr(to))
			Ros.AppendMessage(markers, marker)
		return markers

	def __shadowsAreConnectedTemporally(self, pastGraph: ConnectivityGraph, nowGraph: ConnectivityGraph, pastPoly: ShadowPolygon, nowPoly: ShadowPolygon) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the transformation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		intersectionOfShadows = GeometryLib.intersection(pastPoly.interior, nowPoly.interior)
		for pastSensorPoly in pastGraph.sensors:
			if pastSensorPoly.id not in nowGraph: continue
			nowSensorPoly = nowGraph.getContent(pastSensorPoly.id, "polygon")
			pastCoords = GeometryLib.getGeometryCoords(pastSensorPoly.interior)
			nowCoords = GeometryLib.getGeometryCoords(nowSensorPoly.interior)
			transformation = GeometryLib.getAffineTransformation(pastCoords, nowCoords)
			ps = []
			for nowEdge in nowSensorPoly.edges:
				pastEdge = pastSensorPoly.getEquivalentEdge(nowEdge, transformation)
				if pastEdge is None:
					raise AssertionError("No equivalent edge found based on transformation.")
				polygon = Shapely.Polygon([pastEdge.coords[0], pastEdge.coords[1], nowEdge.coords[1], nowEdge.coords[0]])
				ps.append(polygon)
			u = GeometryLib.union(ps)
			polys = GeometryLib.difference(intersectionOfShadows, u)
			for p in polys:
				if p.is_valid and (not p.is_empty) and p.area > GeometryLib.EPSILON:
					return True
		return False

	def __connectGraphsTemporally(self, fromGraph: ConnectivityGraph, toGraph: ConnectivityGraph) -> None:
		# # Add temporal edges between FOVs
		# for prevSensorId in fromGraph.sensors:
		# 	nodeIdInPrevGraph = NxUtils.Id(
		# 		regionId=prevSensorId.regionId,
		# 		polygonId=prevSensorId.polygonId,
		# 		overlappingRegionId=prevSensorId.overlappingRegionId,
		# 		overlappingPolygonId=prevSensorId.overlappingPolygonId,
		# 		timeNanoSecs=fromGraph.timeNanoSecs,
		# 	)
		# 	if nodeIdInPrevGraph not in self.nodes: continue
		# 	nodeIdInNextGraph = NxUtils.Id(
		# 		regionId=prevSensorId.regionId,
		# 		polygonId=prevSensorId.polygonId,
		# 		overlappingRegionId=prevSensorId.overlappingRegionId,
		# 		overlappingPolygonId=prevSensorId.overlappingPolygonId,
		# 		timeNanoSecs=fromGraph.timeNanoSecs,
		# 	)
		# 	if nodeIdInNextGraph not in self.nodes: continue
		# 	self.addEdge(nodeIdInPrevGraph, nodeIdInNextGraph, addReverseEdge=False, content=NxUtils.EdgeData(isTemporal=True))
		# # Add temporal edges between moving polygons
		# for prevMapPolyId in fromGraph.map:
		# 	nodeIdInPrevGraph = prevMapPolyId.updateId(fromGraph.timeNanoSecs)
		# 	if nodeIdInPrevGraph not in self.nodes: continue
		# 	if fromGraph.map[prevMapPolyId].type == StaticPolygon.type: continue
		# 	parts = toGraph.map.getAllParts(prevMapPolyId)
		# 	for poly in parts:
		# 		nodeIdInNextGraph = poly.id.updateId(toGraph.timeNanoSecs)
		# 		self.addEdge(nodeIdInPrevGraph, nodeIdInNextGraph, addReverseEdge=False, content=NxUtils.EdgeData(isTemporal=True))
		# # Add temporal edges between shadows
		for prevShadowPoly in fromGraph.shadows:
			for nextShadowPoly in toGraph.shadows:
				if GeometryLib.intersects(prevShadowPoly.interior, nextShadowPoly.interior):
					if self.__shadowsAreConnectedTemporally(fromGraph, toGraph, prevShadowPoly, nextShadowPoly):
						nodeIdInPrevGraph = prevShadowPoly.id.updateTime(fromGraph.timeNanoSecs)
						nodeIdInNextGraph = nextShadowPoly.id.updateTime(toGraph.timeNanoSecs)
						self.addEdge(nodeIdInPrevGraph, nodeIdInNextGraph, addReverseEdge=False, content=NxUtils.EdgeData(isTemporal=True))
		return

	def __replaceInHistory(self, index: int, graph: ConnectivityGraph) -> None:
		assert index < len(self.__history), f"Index out of history length: index = {index}, Len = {len(self.__history)}"
		Ros.Log(f"Graph being replaced: {repr(self.__history[index])} with {len(self.__history[index].nodes)} nodes")
		Ros.Log(f"Graph replacing: {repr(graph)} with {len(graph.nodes)} nodes")
		for polyId in self.__history[index].nodes:
			polyId = cast(NxUtils.Id, polyId)
			self.remove_node(polyId)
		self.__history[index] = graph
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		if self.length > 0 and graph.timeNanoSecs < self.__history[-1].timeNanoSecs:
				Ros.Logger().error(f"Older graph than latest in history --> {graph.timeNanoSecs} vs {self.__history[-1].timeNanoSecs}")
				return

		graph.render()
		if self.length > 0 and self.__history[-1].timeNanoSecs == graph.timeNanoSecs:
			Ros.Log("Modifying shadow tree: REPLACE timestamp.")
			self.__replaceInHistory(self.length - 1, graph)
		elif self.length > 0 and EventAggregator.isIsomorphic(self.__history[-1], graph):
			Ros.Log("Modifying shadow tree: REPLACE isomorphic.")
			self.__replaceInHistory(self.length - 1, graph)
		else:
			Ros.Log(f"Modifying shadow tree: APPENDING {len(graph.nodes)} nodes.")
			self.__history.append(graph)
			self.__publishGraphEvent()

		for id in graph.nodes:
			id = cast(NxUtils.Id, id)
			poly = graph.getContent(id, "polygon")
			if not isinstance(poly, _PolygonAlias): raise AssertionError(f"Invalid polygon type {poly.type} in ShadowTree.")
			self.addNode(id, self.NodeData(polygon=poly))
		for edge in graph.edges:
			frmPolyId: NxUtils.Id = edge[0]
			toPolyId: NxUtils.Id = edge[1]
			self.addEdge(frmPolyId, toPolyId, addReverseEdge=False, content=NxUtils.EdgeData(isTemporal=False))

		if self.length > 1: self.__connectGraphsTemporally(self.__history[-2], self.__history[-1])
		self.render()

	def __publishGraphEvent(self) -> None:
		# if self.length == 1:
		# 	adjacency: list[tuple[str, list[str]]] = [(n, list(adjDict.keys())) for (n, adjDict) in self.history[0].adjacency()]
		# 	graphMsg = Graph()
		# 	for (node, neighbors) in adjacency:
		# 		Ros.AppendMessage(graphMsg.vertices, node)
		# 		adjacencyMsg = Adjacency()
		# 		for neighbor in neighbors:
		# 			Ros.AppendMessage(adjacencyMsg.neighbors, neighbor)
		# 		Ros.AppendMessage(graphMsg.adjacency, adjacencyMsg)
		# 	self.__eventPublisher["eventifier_graph"].publish(graphMsg)
		# 	return
		# lastCGraph = self.history[-1]
		# prevCGraph = self.history[-2]
		# lastCGraphNodes = [polyId.updateId(lastCGraph.timeNanoSecs) for polyId in lastCGraph.nodes]
		# prevCGraphNodes = [polyId.updateId(prevCGraph.timeNanoSecs) for polyId in prevCGraph.nodes]
		# appearedNodes: list[NxUtils.Id] = []
		# disappearedNodes: list[NxUtils.Id] = []
		# mergedNodes: list[tuple[NxUtils.Id, list[NxUtils.Id]]] = []
		# splittedNodes: list[tuple[NxUtils.Id, list[NxUtils.Id]]] = []
		# for n in lastCGraphNodes:
		# 	if self.in_degree(n) == 0: appearedNodes.append(n)
		# 	if self.in_degree(n) > 1:
		# 		inNeighbors: list[NxUtils.Id] = [edge[1] for edge in self.in_edges(n)]
		# 		mergedNodes.append((n, inNeighbors))
		# for n in prevCGraphNodes:
		# 	if self.out_degree(n) == 0: disappearedNodes.append(n)
		# 	if self.out_degree(n) > 1:
		# 		outNeighbors: list[NxUtils.Id] = [edge[1] for edge in self.out_edges(n)]
		# 		splittedNodes.append((n, outNeighbors))
		# eventsMsg = Events()
		# event = ComponentEvent()
		# event.ros_time = lastCGraph.timeNanoSecs
		# event.type = "A"
		# for n in appearedNodes: Ros.AppendMessage(event.after, n)
		# Ros.AppendMessage(eventsMsg.component_events, event)

		# event = ComponentEvent()
		# event.ros_time = lastCGraph.timeNanoSecs
		# event.type = "D"
		# for n in disappearedNodes: Ros.AppendMessage(event.before, n)
		# Ros.AppendMessage(eventsMsg.component_events, event)

		# event = ComponentEvent()
		# event.ros_time = lastCGraph.timeNanoSecs
		# event.type = "M"
		# for n in mergedNodes:
		# 	Ros.AppendMessage(event.before, n[0])
		# 	for inNode in n[1]: Ros.AppendMessage(event.after, inNode)
		# Ros.AppendMessage(eventsMsg.component_events, event)

		# event = ComponentEvent()
		# event.ros_time = lastCGraph.timeNanoSecs
		# event.type = "D"
		# for n in splittedNodes:
		# 	Ros.AppendMessage(event.before, n[0])
		# 	for inNode in n[1]: Ros.AppendMessage(event.after, inNode)
		# Ros.AppendMessage(eventsMsg.component_events, event)

		# self.__eventPublisher["eventifier_event"].publish(eventsMsg)
		return

	def updatePolygon(self, polygon: GraphInputPolygon) -> None:
		Ros.Log(f"--------------------------------{polygon.timeNanoSecs:20}--------------------------------")
		if self.length == 0:
			if polygon.type != StaticPolygon.type:
				Ros.Log("Initial CGraph must be created from a static map.")
				return
			cGraph = ConnectivityGraph(polygon.timeNanoSecs, [polygon], [], self.__rvizPublishers["connectivity_graph"])
			self.__appendToHistory(cGraph)
			return

		lastCGraph = self.history[-1]
		if polygon.type != SensingPolygon.type:
			updatedSensors = lastCGraph.sensors
			updatedMap = [polygon] + lastCGraph.map
		else:
			updatedSensors = [polygon] + lastCGraph.sensors
			updatedMap = lastCGraph.map

		# There are there possibilities for dynamic polygon.
		# 1. P1 is in pastSensors but not in nowSensors ----> P1 has turned off -> the shadows around P1 have merged.
		# 2. P2 is not in pastSensors but is in nowSensors -> P2 has turned on --> the shadows around P2 have splitted.
		# 3. P3 is in pastSensors but not in nowSensors ----> S3 has moved ------> the shadows around P3 have evolved.

		intervals = CtCd.estimateCollisionIntervals(updatedSensors, updatedMap, self.__rvizPublishers["continuous_time_collision_detection"])
		intervals = CtCd.refineCollisionIntervals(intervals)
		Ros.Log(f"Intervals = {len(intervals)}")
		eventGraphs = EventAggregator.aggregateCollisionEvents(intervals, lastCGraph)
		if len(eventGraphs) == 0:
			nowCGraph = ConnectivityGraph(polygon.timeNanoSecs, updatedMap, updatedSensors, self.__rvizPublishers["connectivity_graph"])
			eventGraphs = [nowCGraph]
		Ros.Log(f"Aggregated = {repr(eventGraphs)}")
		for graph in eventGraphs: self.__appendToHistory(graph)
		return
