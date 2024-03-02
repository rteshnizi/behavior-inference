from dataclasses import dataclass
from typing import Literal, Sequence, TypeAlias, cast, overload

import networkx as nx
from networkx.classes.reportviews import OutEdgeView

from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.Ros import AppendMessage, Log, Logger, Publisher
from rt_bi_commons.Utils.RViz import ColorNames, RViz
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.ShadowPolygon import ShadowPolygon
from rt_bi_core.Spatial.SpatialRegion import SpatialRegion
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_eventifier.Model.EventAggregator import EventAggregator
from rt_bi_interfaces.msg import Adjacency, ComponentEvent, Events, Graph

_InputPolyTypes: TypeAlias = MovingPolygon | StaticPolygon | SensingPolygon
# CSpell: ignore reportviews
class ShadowTree(nx.DiGraph):
	"""
		The implementation of a Shadow Tree in python as described in the dissertation.
		© Reza Teshnizi 2018-2023
	"""

	SUBMODULE_TYPES = Literal["connectivity_graph", "continuous_time_collision_detection", "shadow_tree"]
	SUBMODULES: tuple[SUBMODULE_TYPES, ...] = ("connectivity_graph", "continuous_time_collision_detection", "shadow_tree")
	TOPICS = Literal["eventifier_graph", "eventifier_event"]
	RENDER_DELTA_X = 150
	RENDER_DELTA_Y = 50
	RENDER_RADIUS = 10
	RENDER_FONT_SIZE = 7.5

	@dataclass(frozen=True)
	class Id(NxUtils.Id):
		"""
		An identifier `dataclass` for every ShadowTree nodes.
		:param str regionId: id of the region owning this polygon.
		:param str polygonId: id of the polygon.
		:param str overlappingRegionId: id of the region owning the overlapping polygon, defaults to ``""``.
		:param str overlappingPolygonId: id of the polygon this polygon lies upon, defaults to ``""``.
		"""
		timeNanoSecs: int

		@staticmethod
		def fromPolyId(id: NxUtils.Id, timeNanoSecs: int) -> "ShadowTree.Id":
			return ShadowTree.Id(id.regionId, id.polygonId, id.overlappingRegionId, id.overlappingPolygonId, timeNanoSecs=timeNanoSecs)

	ContentKey: TypeAlias = Literal["polygon"]
	ContentValue: TypeAlias = SensingPolygon | ShadowPolygon | MovingPolygon | StaticPolygon
	NodeContent: TypeAlias = dict[ContentKey, ContentValue]

	def __init__(self, eventPublisher: dict[TOPICS, Publisher], rvizPublishers: dict[SUBMODULE_TYPES, Publisher | None]):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(name="ShTr-V3")
		self.__history: list[ConnectivityGraph] = []
		self.__timeToCGraph: dict[int, int] = {}
		""" A map from time of the graph to its index in the history list. """
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

	@overload
	def getContent(self, node: Id, contentKey: Literal[""]) -> NodeContent: ...
	@overload
	def getContent(self, node: Id, contentKey: Literal["polygon"]) -> SensingPolygon | ShadowPolygon | MovingPolygon | StaticPolygon: ...

	def getContent(self, node: Id, contentKey: ContentKey | Literal[""]) -> NodeContent | ContentValue:
		if contentKey == "": return self.nodes[node]
		else: return self.nodes[node][contentKey]

	def __getCGraph(self, timeNanoSecs: int) -> ConnectivityGraph:
		ind = self.__timeToCGraph[timeNanoSecs]
		return self.history[ind]

	def __addNode(self, id: MovingPolygon.Id, polygon: SensingPolygon | ShadowPolygon | MovingPolygon | StaticPolygon) -> Id:
		nodeId = ShadowTree.Id.fromPolyId(id, timeNanoSecs=polygon.timeNanoSecs)
		self.add_node(nodeId, polygon=polygon)
		return nodeId

	def __addEdge(self, n1: Id, n2: Id, isTemporal: bool, fromTime: int | None = None, toTime: int | None = None) -> None:
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
		pastRegion = pastShadow["polygon"]
		nowRegion = nowShadow["polygon"]
		if isinstance(pastRegion, AffinePolygon) and isinstance(nowRegion, AffinePolygon):
			intersectionOfShadows = GeometryLib.intersection(pastRegion.interior, nowRegion.interior)
			for fov in pastGraph.sensors:
				if fov not in pastGraph.nodes: continue
				pastSensorRegion: SensingPolygon = pastGraph.nodes[fov]["polygon"]
				if fov not in nowGraph.nodes: continue
				nowSensorRegion: SensingPolygon = nowGraph.nodes[fov]["polygon"]
				pastCoords = GeometryLib.getGeometryCoords(pastSensorRegion.interior)
				nowCoords = GeometryLib.getGeometryCoords(nowSensorRegion.interior)
				transformation: AffineTransform = GeometryLib.getAffineTransformation(pastCoords, nowCoords)
				ps = []
				for e in nowSensorRegion.edges:
					edgeC = nowSensorRegion.edges[e]
					edgeP = pastSensorRegion.getEquivalentEdge(edgeC, transformation)
					if edgeP is None:
						raise AssertionError("No equivalent edge found based on transformation.")
					polygon = Shapely.Polygon([edgeP.coords[0], edgeP.coords[1], edgeC.coords[1], edgeC.coords[0]])
					ps.append(polygon)
				u = GeometryLib.union(ps)
				polys = GeometryLib.difference(intersectionOfShadows, u)
				for p in polys:
					if p.is_valid and (not p.is_empty) and p.area > GeometryLib.EPSILON:
						return True
		return False

	def __connectGraphsTemporally(self, fromGraph: ConnectivityGraph, toGraph: ConnectivityGraph) -> None:
		# Add temporal edges between FOVs
		for prevSensorId in fromGraph.sensors:
			nodeIdInPrevGraph = ShadowTree.Id.fromPolyId(prevSensorId, fromGraph.timeNanoSecs)
			if nodeIdInPrevGraph not in self.nodes: continue
			nodeIdInNextGraph = ShadowTree.Id.fromPolyId(prevSensorId, toGraph.timeNanoSecs)
			if nodeIdInNextGraph not in self.nodes: continue
			self.__addEdge(nodeIdInPrevGraph, nodeIdInNextGraph, isTemporal=True, fromTime=fromGraph.timeNanoSecs, toTime=toGraph.timeNanoSecs)
		# Add temporal edges between moving polygons
		for prevMapPolyId in fromGraph.map:
			nodeIdInPrevGraph = ShadowTree.Id.fromPolyId(prevMapPolyId, fromGraph.timeNanoSecs)
			if nodeIdInPrevGraph not in self.nodes: continue
			if fromGraph.map[prevMapPolyId].type == StaticPolygon.type: continue
			parts = toGraph.map.getAllParts(prevMapPolyId)
			for poly in parts:
				nodeIdInNextGraph = ShadowTree.Id.fromPolyId(poly.id, toGraph.timeNanoSecs)
				self.__addEdge(nodeIdInPrevGraph, nodeIdInNextGraph, isTemporal=True, fromTime=fromGraph.timeNanoSecs, toTime=toGraph.timeNanoSecs)
		# Add temporal edges between shadows
		for prevShadowPolyId in fromGraph.shadows:
			for nextShadowPolyId in toGraph.shadows:
				previousShadowPoly = cast(ShadowPolygon, fromGraph.getContent(prevShadowPolyId, "polygon"))
				currentShadowPoly = cast(ShadowPolygon, toGraph.getContent(nextShadowPolyId, "polygon"))
				if GeometryLib.intersects(previousShadowPoly.interior, currentShadowPoly.interior):
					if self.__shadowsAreConnectedTemporally(fromGraph, toGraph, fromGraph.nodes[prevShadowPolyId], toGraph.nodes[nextShadowPolyId]):
						nodeIdInPrevGraph = ShadowTree.Id.fromPolyId(prevShadowPolyId, fromGraph.timeNanoSecs)
						nodeIdInNextGraph = ShadowTree.Id.fromPolyId(nextShadowPolyId, toGraph.timeNanoSecs)
						self.__addEdge(nodeIdInPrevGraph, nodeIdInNextGraph, isTemporal=True, fromTime=fromGraph.timeNanoSecs, toTime=toGraph.timeNanoSecs)
		return

	def __replaceInHistory(self, indexToReplace: int, graph: ConnectivityGraph) -> None:
		for polyId in self.__history[indexToReplace].nodes:
			polyId = cast(MovingPolygon.Id, polyId)
			node = ShadowTree.Id.fromPolyId(polyId, self.__history[indexToReplace].timeNanoSecs)
			self.remove_node(node)
		self.__timeToCGraph.pop(self.__history[indexToReplace].timeNanoSecs)
		self.__history[indexToReplace] = graph
		self.__timeToCGraph[graph.timeNanoSecs] = indexToReplace
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		if self.length > 0 and graph.timeNanoSecs < self.__history[-1].timeNanoSecs:
				Log("Older graph than latest in history --> %d vs %d" % ((graph.timeNanoSecs, self.__history[-1].timeNanoSecs)))
				return

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

		for id in graph.nodes:
			id = cast(NxUtils.Id, id)
			self.__addNode(id, graph.getContent(id, "polygon"))
		for edge in graph.edges:
			frmPolyId = edge[0]
			toPolyId = edge[1]
			self.__addEdge(ShadowTree.Id.fromPolyId(frmPolyId, graph.timeNanoSecs), ShadowTree.Id.fromPolyId(toPolyId, graph.timeNanoSecs), isTemporal=False)

		if self.length > 1: self.__connectGraphsTemporally(self.__history[-2], self.__history[-1])
		self.render()

	def __publishGraphEvent(self) -> None:
		if self.length == 1:
			adjacency: list[tuple[str, list[str]]] = [(n, list(adjDict.keys())) for (n, adjDict) in self.history[0].adjacency()]
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
		lastCGraphNodes = [ShadowTree.Id.fromPolyId(polyId, lastCGraph.timeNanoSecs) for polyId in lastCGraph.nodes]
		prevCGraphNodes = [ShadowTree.Id.fromPolyId(polyId, prevCGraph.timeNanoSecs) for polyId in prevCGraph.nodes]
		appearedNodes: list[ShadowTree.Id] = []
		disappearedNodes: list[ShadowTree.Id] = []
		mergedNodes: list[tuple[ShadowTree.Id, list[ShadowTree.Id]]] = []
		splittedNodes: list[tuple[ShadowTree.Id, list[ShadowTree.Id]]] = []
		for n in lastCGraphNodes:
			if self.in_degree(n) == 0: appearedNodes.append(n)
			if self.in_degree(n) > 1:
				inNeighbors: list[ShadowTree.Id] = [edge[1] for edge in self.in_edges(n)]
				mergedNodes.append((n, inNeighbors))
		for n in prevCGraphNodes:
			if self.out_degree(n) == 0: disappearedNodes.append(n)
			if self.out_degree(n) > 1:
				outNeighbors: list[ShadowTree.Id] = [edge[1] for edge in self.out_edges(n)]
				splittedNodes.append((n, outNeighbors))
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

	def __nodeSortKey(self, id: "ShadowTree.Id") -> tuple:
		((minX, minY), (maxX, maxY)) = self.getContent(id, "polygon").bounds
		return (id.timeNanoSecs, minX, maxX, minY, maxY)

	def __multiPartiteLayout(self) -> "NxUtils.GraphLayout[ShadowTree.Id]":
		keys: list[ShadowTree.Id] = sorted(self.nodes, key=self.__nodeSortKey)
		pos: NxUtils.GraphLayout = {}
		y = 0
		x = 0
		timeNanoSecs: int = 0
		for n in keys:
			if n.timeNanoSecs > timeNanoSecs:
				timeNanoSecs = n.timeNanoSecs
				x = 0
				y += self.RENDER_DELTA_Y
			pos[n] = (x, y)
			x += self.RENDER_DELTA_X
		return pos

	def __getTextMarkers(self, markerArray: RViz.Msgs.MarkerArray) -> RViz.Msgs.MarkerArray:
		lastCGraph = self.history[-1]
		timerCoords = GeometryLib.findBottomLeft(lastCGraph.map.envelopePolygon)
		timerCoords = GeometryLib.addCoords(timerCoords, (20, 20))
		id = NxUtils.Id("shadow-tree", "time", "", "")
		timerMarker = RViz.createText(id, timerCoords, f"T = {lastCGraph.timeNanoSecs}", ColorNames.RED, fontSize=7.5)
		AppendMessage(markerArray.markers, timerMarker)

		timerCoords = GeometryLib.findBottomLeft(lastCGraph.map.envelopePolygon)
		timerCoords = GeometryLib.addCoords(timerCoords, (20, 10))
		id = NxUtils.Id("shadow-tree", "depth", "", "")
		timerMarker = RViz.createText(id, timerCoords, f"D = {self.length}", ColorNames.RED, fontSize=7.5)
		AppendMessage(markerArray.markers, timerMarker)
		return markerArray

	def __getNodeMarkers(self, markerArray: RViz.Msgs.MarkerArray) -> RViz.Msgs.MarkerArray:
		if len(self.nodes) == 0: return markerArray
		nodePositions = self.__multiPartiteLayout()
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
			marker = RViz.createCircle(id, centerX=coords[0], centerY=coords[1], radius=self.RENDER_RADIUS, outline=outlineColor)
			AppendMessage(markerArray.markers, marker)
			marker = RViz.createText(id, coords, poly.shortName, ColorNames.RED, fontSize=self.RENDER_FONT_SIZE)
			AppendMessage(markerArray.markers, marker)
		return markerArray

	def __getEdgeMarkers(self, markerArray: RViz.Msgs.MarkerArray) -> RViz.Msgs.MarkerArray:
		if len(self.nodes) == 0: return markerArray
		nodePositions = self.__multiPartiteLayout()
		outEdgeView: OutEdgeView = self.out_edges()
		for (frm, to) in outEdgeView:
			frm = cast(ShadowTree.Id, frm)
			to = cast(ShadowTree.Id, to)
			edgeData = outEdgeView[frm, to]
			color = ColorNames.CYAN_DARK if ("isTemporal" in edgeData and edgeData["isTemporal"]) else ColorNames.MAGENTA_DARK
			(dx, dy) = GeometryLib.subtractCoords(nodePositions[to], nodePositions[frm])
			dVect = GeometryLib.getUnitVector(dx, dy)
			dVect = GeometryLib.scaleCoords(dVect, self.RENDER_RADIUS)
			fromCoords = GeometryLib.addCoords(nodePositions[frm], dVect)
			toCoords = GeometryLib.subtractCoords(nodePositions[to], dVect)
			marker = RViz.createLine(frm, coords=[fromCoords, toCoords], outline=color, width=2, idSuffix=repr(to))
			AppendMessage(markerArray.markers, marker)
		return markerArray

	def __mergeWithRegularRegion(self, updatedRegion: SpatialRegion[_InputPolyTypes], outdatedRegion: SpatialRegion[_InputPolyTypes] | None = None) -> tuple[list[_InputPolyTypes], int]:
		"""Get a list of all polygons at different times.
		CtCd uses this to construct :class:`ContinuousTimeRegions`.

		:param updatedRegion: The regions coming from the update stream.
		:type updatedRegion: `SpatialRegion[_InputPolyTypes]`
		:param outdatedRegion: The existing region from the previous ShadowTree layer, defaults to `None`.
		:type outdateRegion: `SpatialRegion[_InputPolyTypes]` or `None`
		:return: A tuple: a list of polygons, and the maximum time value among them.
		:rtype: `tuple[list[_InputPolyTypes], int]`
		"""
		polys: list[_InputPolyTypes] = updatedRegion.polygons
		if outdatedRegion is not None:
			for id in outdatedRegion:
				if id not in updatedRegion: polys.append(outdatedRegion[id])
		maxT = max((p.timeNanoSecs for p in polys))
		return (polys, maxT)

	def updateRegion(self, region: SpatialRegion[MovingPolygon | StaticPolygon] | SpatialRegion[SensingPolygon]) -> None:
		if self.length == 0 and (region.type != StaticPolygon.type):
			Log("Initial CGraph must be created from map.")
			return
		if self.length == 0 and (region.type == StaticPolygon.type):
			updatedMapRegions = cast(SpatialRegion[MovingPolygon | StaticPolygon], region)
			currentConnectivityG = ConnectivityGraph(region.timeNanoSec, updatedMapRegions, [], self.__rvizPublishers["connectivity_graph"])
			self.__appendToHistory(currentConnectivityG)
			return

		Log("=================================================================================================")
		lastCGraph = self.history[-1]
		if region.type == SensingPolygon.type:
			(updatedSensorPolys, maxT1) = self.__mergeWithRegularRegion(cast(SpatialRegion[_InputPolyTypes], region), cast(SpatialRegion[_InputPolyTypes], lastCGraph.sensors))
			(updatedMapRegions, maxT2) = self.__mergeWithRegularRegion(cast(SpatialRegion[_InputPolyTypes], lastCGraph.map))
		else:
			(updatedSensorPolys, maxT1) = self.__mergeWithRegularRegion(cast(SpatialRegion[_InputPolyTypes], lastCGraph.sensors))
			(updatedMapRegions, maxT2) = self.__mergeWithRegularRegion(cast(SpatialRegion[_InputPolyTypes], region), cast(SpatialRegion[_InputPolyTypes], lastCGraph.map))
		updatedSensorPolys = cast(list[SensingPolygon], updatedSensorPolys)
		updatedMapRegions = cast(list[MovingPolygon | StaticPolygon], updatedMapRegions)
		# There are there possibilities for sensor names. We have decided to require specific messages for S1.
		# 1. S1 is in pastSensors but not in nowSensors ----> S1 has turned off -> the shadows around S1 have merged.
		# 2. S2 is not in pastSensors but is in nowSensors -> S2 has turned on --> the shadows around S2 have splitted.
		# 3. S3 is in pastSensors but not in nowSensors ----> S3 has moved ------> the shadows around S3 have evolved.

		intervals = CtCd.estimateCollisionIntervals(updatedSensorPolys, updatedMapRegions, self.__rvizPublishers["continuous_time_collision_detection"])
		intervals = CtCd.refineCollisionIntervals(intervals)
		Log(f"Intervals = {len(intervals)}")
		eventGraphs = EventAggregator.aggregateCollisionEvents(intervals, lastCGraph)
		if len(eventGraphs) == 0:
			maxTimeNanoSecs = max(maxT1, maxT2)
			nowCGraph = ConnectivityGraph(maxTimeNanoSecs, updatedMapRegions, updatedSensorPolys, self.__rvizPublishers["connectivity_graph"])
			eventGraphs = [nowCGraph]
		Log(f"Aggregated = {repr(eventGraphs)}")
		for graph in eventGraphs: self.__appendToHistory(graph)
		return

	def render(self) -> None:
		publisher = self.__rvizPublishers["shadow_tree"]
		if publisher is None: return
		if self.length == 0: return
		markerArray = RViz.Msgs.MarkerArray()
		AppendMessage(markerArray.markers, RViz.removeAllMarkers())
		markerArray = self.__getTextMarkers(markerArray)
		markerArray = self.__getNodeMarkers(markerArray)
		markerArray = self.__getEdgeMarkers(markerArray)
		publisher.publish(markerArray)
		return
