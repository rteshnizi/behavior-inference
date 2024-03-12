from dataclasses import dataclass
from typing import Literal, Sequence, TypeVar, cast

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RViz import ColorNames, RViz
from rt_bi_core.Spatial import GraphPolygon, MapPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion
from rt_bi_eventifier.Model.EventAggregator import EventAggregator

_T_Poly = TypeVar("_T_Poly", bound=GraphPolygon)

class ShadowTree(NxUtils.Graph[GraphPolygon]):
	"""
		The implementation of a Shadow Tree in python as described in the dissertation.
	"""

	SUBMODULES = ("connectivity_graph", "continuous_time_collision_detection", "shadow_tree")
	SUBMODULE = Literal["connectivity_graph", "continuous_time_collision_detection", "shadow_tree"]
	"""The name of a ShadowTree sub-module publisher."""
	ST_TOPIC = Literal["eventifier_graph", "eventifier_event"]
	"""The name of a ShadowTree topic."""
	__RENDER_RADIUS = 10
	__MAX_HISTORY = 2

	@dataclass(frozen=True, order=True)
	class NodeData(NxUtils.NodeData[GraphPolygon]):
		subset: int = -1
		"""Used in multipartite layout.
		https://networkx.org/documentation/stable/reference/generated/networkx.drawing.layout.multipartite_layout.html
		"""

		@staticmethod
		def extend(data: NxUtils.NodeData[GraphPolygon], subset: int) -> "ShadowTree.NodeData":
			return ShadowTree.NodeData(
				polygon=data.polygon,
				subset=subset
			)

	def __init__(self, eventPublisher: dict[ST_TOPIC, Ros.Publisher], rvizPublishers: dict[SUBMODULE, Ros.Publisher | None]):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(rVizPublisher=rvizPublishers.pop("shadow_tree", None))
		# super().__init__(rVizPublisher=None)
		self.__history: list[ConnectivityGraph] = []
		self.hIndex = 0

		self.componentEvents: list[list[Shapely.Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """
		self.__eventPublisher = eventPublisher
		self.__rvizPublishers = rvizPublishers
		self.__ctrs: dict[MovingPolygon.Id, ContinuousTimeRegion[GraphPolygon]] = {}

	@property
	def length(self) -> int:
		"""The length of the shadow tree history stack."""
		return len(self.__history)

	@property
	def processedTime(self) -> int:
		if self.length == 0: return -1
		return self.__history[-1].timeNanoSecs

	def __repr__(self) -> str:
		timeRangeStr = "∅"
		if self.length == 1:
			timeRangeStr = "%d" % self.__history[0].timeNanoSecs
		if self.length > 1:
			timeRangeStr = "%d-%d" % (self.__history[0].timeNanoSecs, self.__history[-1].timeNanoSecs)
		return "%s[%s]{d=%d}" % (self.name, timeRangeStr, self.length)

	def addNode(self, id: NxUtils.Id, cGraph: ConnectivityGraph) -> NxUtils.Id:
		assert cGraph.hIndex is not None and cGraph.hIndex > -1, f"Unset hIndex is not allowed in ShadowTree: cGraph = {repr(cGraph)}, hIndex = {cGraph.hIndex}"
		content = cGraph.getContent(id)
		id = id.copy(hIndex=cGraph.hIndex)
		content = ShadowTree.NodeData.extend(data=content, subset=cGraph.hIndex)
		return super().addNode(id=id, content=content)

	def addEdge(self, fromId: NxUtils.Id, toId: NxUtils.Id, fromCGraph: ConnectivityGraph, toCGraph: ConnectivityGraph) -> None:
		fromId = fromId.copy(hIndex=fromCGraph.hIndex)
		toId = toId.copy(hIndex=toCGraph.hIndex)
		content = NxUtils.EdgeData(isTemporal=fromCGraph.timeNanoSecs != toCGraph.timeNanoSecs)
		return super().addEdge(fromId, toId, addReverseEdge=False, content=content)

	def getNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._3dLayout()
		for id in nodePositions:
			poly = self.getContent(id, "polygon")
			if poly.type == StaticPolygon.type:
				outlineColor = ColorNames.ORANGE
			elif poly.type == SensingPolygon.type:
				outlineColor = ColorNames.GREEN
			elif poly.type == MovingPolygon.type:
				outlineColor = ColorNames.PURPLE
			else:
				outlineColor = ColorNames.RED
			coords = nodePositions[id]
			marker = RViz.createCircle(id, center=coords, radius=self.__RENDER_RADIUS, outline=outlineColor)
			Ros.AppendMessage(markers, marker)
			marker = RViz.createText(id, coords=coords, text=poly.shortName, outline=ColorNames.WHITE, idSuffix="txt")
			Ros.AppendMessage(markers, marker)
		return markers

	def getEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._3dLayout()
		outEdgeView: NxUtils.OutEdgeView = self.out_edges()
		for (frm, to) in outEdgeView:
			frm = cast(NxUtils.Id, frm)
			to = cast(NxUtils.Id, to)
			if frm == to: continue
			edgeData = outEdgeView[frm, to]
			color = ColorNames.CYAN_DARK if ("isTemporal" in edgeData and edgeData["isTemporal"]) else ColorNames.MAGENTA_DARK
			# (dx, dy, dz) = GeometryLib.subtractCoords(nodePositions[to], nodePositions[frm])
			# dVect = GeometryLib.getUnitVector((dx, dy, dz))
			# dVect = GeometryLib.scaleCoords(dVect, self.__RENDER_RADIUS)
			# fromCoords = GeometryLib.addCoords(nodePositions[frm], dVect)
			# toCoords = GeometryLib.subtractCoords(nodePositions[to], dVect)
			# marker = RViz.createLine(frm, coordsList=[fromCoords, toCoords], outline=color, width=2, idSuffix=repr(to))
			marker = RViz.createLine(frm, coordsList=[nodePositions[frm], nodePositions[to]], outline=color, width=2, idSuffix=repr(to))
			Ros.AppendMessage(markers, marker)
		return markers

	def at(self, timeNanoSecs: int) -> ConnectivityGraph:
		map_ = []
		sensors = []
		for ctr in self.__ctrs.values():
			if timeNanoSecs not in ctr: continue
			if ctr.type == SensingPolygon.type:
				sensors.append(ctr[timeNanoSecs])
			else:
				map_.append(ctr[timeNanoSecs])
		cGraph = ConnectivityGraph(timeNanoSecs, map_, sensors, self.__rvizPublishers["connectivity_graph"])
		return cGraph

	def renderLatestCGraph(self) -> None:
		Ros.Log("Rendering latest CGraph.")
		self.__history[-1].render()
		return

	def __shadowsAreConnectedTemporally(self, pastGraph: ConnectivityGraph, nowGraph: ConnectivityGraph, pastPoly: MapPolygon, nowPoly: MapPolygon) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the transformation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		intersectionOfShadows = GeometryLib.intersection(pastPoly.interior, nowPoly.interior)
		intersectionOfShadows = GeometryLib.filterPolygons(intersectionOfShadows)
		intersectionOfShadows = GeometryLib.union(intersectionOfShadows)
		for pastSensorPoly in pastGraph.sensors:
			nowSensorId = pastSensorPoly.id.copy(timeNanoSecs=nowGraph.timeNanoSecs, hIndex=-1)
			if nowSensorId not in nowGraph: continue
			nowSensorPoly = nowGraph.getContent(nowSensorId, "polygon")
			pastCoords = GeometryLib.getGeometryCoords(pastSensorPoly.interior)
			nowCoords = GeometryLib.getGeometryCoords(nowSensorPoly.interior)
			transformation = GeometryLib.getAffineTransformation(pastCoords, nowCoords)
			objs = []
			for nowEdge in nowSensorPoly.edges:
				pastEdge = pastSensorPoly.getEquivalentEdge(nowEdge, transformation)
				if pastEdge is None:
					raise AssertionError("No equivalent edge found based on transformation.")
				if pastEdge == nowEdge: continue
				obj = Shapely.Polygon([
					pastEdge.coords[0], pastEdge.coords[1],
					nowEdge.coords[1], nowEdge.coords[0],
				])
				obj = Shapely.make_valid(obj)
				obj = Shapely.set_precision(obj, GeometryLib.EPSILON)
				subParts = GeometryLib.toGeometryList(obj)
				for p in subParts: objs.append(p)

			sweptBySensors = Shapely.GeometryCollection(geoms=objs)
			remainingShadows = GeometryLib.difference(intersectionOfShadows, sweptBySensors)
			if len(remainingShadows) > 0: return True
		return False

	def __connectTopLayerTemporally(self) -> None:
		fromGraph = self.__history[self.length - 2]
		toGraph = self.__history[self.length - 1]
		assert toGraph.hIndex is not None, f"Cannot connect graph with unset hIndex in shadow tree. {repr(toGraph)}"
		# Add temporal edges between FOVs
		for fromAntiShadow in fromGraph.antiShadows:
			for toAntiShadow in toGraph.antiShadows:
				if fromAntiShadow.id.copy(timeNanoSecs=-1, hIndex=-1, subPartId="") == toAntiShadow.id.copy(timeNanoSecs=-1, hIndex=-1, subPartId=""):
					self.addEdge(fromAntiShadow.id, toAntiShadow.id, fromGraph, toGraph)
		# Add temporal edges between shadows
		for fromShadow in fromGraph.shadows:
			for toShadow in toGraph.shadows:
				if GeometryLib.intersects(fromShadow.interior, toShadow.interior):
					if self.__shadowsAreConnectedTemporally(fromGraph, toGraph, fromShadow, toShadow):
						self.addEdge(fromShadow.id, toShadow.id, fromGraph, toGraph)
		return

	def __removeFromHistory(self, index: int, delete: bool) -> None:
		assert index >= 0 and index < len(self.__history), f"Index out of history bounds: index = {index}, Len = {len(self.__history)}"
		hIndex = self.__history[index].hIndex
		assert hIndex is not None, f"Graph with unset hIndex found in shadow tree. {repr(self.__history[index])}"

		Ros.Log(f"Removing Graph: {repr(self.__history[index])} with {len(self.__history[index].nodes)} nodes.")
		polyId: NxUtils.Id
		for polyId in self.__history[index].nodes:
			id = NxUtils.Id(hIndex=hIndex, timeNanoSecs=polyId.timeNanoSecs, regionId=polyId.regionId, polygonId=polyId.polygonId, subPartId=polyId.subPartId)
			assert id in self, f"Remove failed: {id} is not a node in the graph."
			self.remove_node(id)
		if delete: self.__history.pop(index)

	def __replaceInHistory(self, index: int, graph: ConnectivityGraph) -> None:
		""" **Does not add nodes.** This just manipulates `self.__history` variable. """
		self.__removeFromHistory(index, False)
		Ros.Log(f"REPLACE graph with {len(graph.antiShadows)} anti-shadows and {len(graph.shadows)} shadows.")
		# graph.logGraphNodes()
		hIndex = cast(int, self.__history[index].hIndex)
		graph.hIndex = hIndex
		self.__history[index] = graph
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		if self.length > 0 and graph.timeNanoSecs < self.__history[-1].timeNanoSecs:
				Ros.Logger().error(f"Older graph than latest in history --> {graph.timeNanoSecs} vs {self.__history[-1].timeNanoSecs}")
				return

		if self.length > 0 and EventAggregator.isIsomorphic(self.__history[-1], graph):
			Ros.Log("Isomorphic graph detected.")
			self.__replaceInHistory(self.length - 1, graph)
		else:
			Ros.Log(f"APPENDING graph with {len(graph.antiShadows)} anti-shadows and {len(graph.shadows)} shadows.")
			graph.hIndex = self.hIndex
			self.__history.append(graph)
			self.hIndex += 1
			self.__publishGraphEvent()

		for id in graph.nodes:
			id = cast(NxUtils.Id, id)
			self.addNode(id, graph)
		for edge in graph.edges:
			self.addEdge(edge[0], edge[1], graph, graph)

		if self.length > 1: self.__connectTopLayerTemporally()
		if self.length > self.__MAX_HISTORY:
			Ros.Log(f"History depth is more than MAX={self.__MAX_HISTORY} graphs.")
			self.__removeFromHistory(0, True)
		self.render()

	def __publishGraphEvent(self) -> None:
		# if self.length == 1:
		# 	adjacency: list[tuple[str, list[str]]] = [(n, list(adjDict.keys())) for (n, adjDict) in self.__history[0].adjacency()]
		# 	graphMsg = Graph()
		# 	for (node, neighbors) in adjacency:
		# 		Ros.AppendMessage(graphMsg.vertices, node)
		# 		adjacencyMsg = Adjacency()
		# 		for neighbor in neighbors:
		# 			Ros.AppendMessage(adjacencyMsg.neighbors, neighbor)
		# 		Ros.AppendMessage(graphMsg.adjacency, adjacencyMsg)
		# 	self.__eventPublisher["eventifier_graph"].publish(graphMsg)
		# 	return
		# lastCGraph = self.__history[-1]
		# prevCGraph = self.__history[-2]
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

	def __updateCTRs(self, poly: GraphPolygon) -> None:
		if poly.timeNanoSecs < self.processedTime:
			Ros.Log(f"Out of sync update: {poly.timeNanoSecs} < {self.processedTime} processed already.")
			return
		idSansTime = poly.id.sansTime()
		Ros.Log(f"Updating CTR: {idSansTime.shortNames()}.")
		if idSansTime not in self.__ctrs:
			self.__ctrs[idSansTime] = ContinuousTimeRegion(polyConfigs=[poly])
			return

		self.__ctrs[idSansTime].addPolygon(poly, self.processedTime)
		return

	def __latestNanoSecsBounds(self) -> tuple[int, int]:
		minNs = ContinuousTimeRegion.INF_NS + 1
		maxNs = -1
		for ctr in self.__ctrs.values():
			if ctr.latestNanoSecs < minNs:
				minNs = ctr.latestNanoSecs
			if ctr.latestNanoSecs != ContinuousTimeRegion.INF_NS and ctr.latestNanoSecs > maxNs:
				maxNs = ctr.latestNanoSecs
		return (minNs, maxNs)

	def updatePolygon(self, polygon: GraphPolygon) -> None:
		Ros.Log(f"<=============================== {polygon.timeNanoSecs:20} =====================================>")
		Ros.Log(f"New Poly = {polygon}")
		if self.length == 0 and polygon.type != StaticPolygon.type:
			Ros.Log("Initial CGraph must be created from a static map.")
			return
		self.__updateCTRs(polygon)
		(minLatestNs, maxLatestNs) = self.__latestNanoSecsBounds()
		Ros.Log("Post Update", [
			f"Processed up to {self.processedTime}",
			f"Min LatestNs =  {minLatestNs}",
			f"Max LatestNs =  {maxLatestNs}",
		])
		if self.processedTime > 0 and minLatestNs <= self.processedTime:
			Ros.Log(f"Not ready to process yet: Min LatestNs <= processedTime")
			return
		minLatestNs = polygon.timeNanoSecs if polygon.type == StaticPolygon.type else minLatestNs
		Ros.Log(f"Processing {minLatestNs:20}.")

		if self.length == 0:
			cGraph = self.at(minLatestNs)
			self.__appendToHistory(cGraph)
			return

		ctrs = list(self.__ctrs.values())
		intervals = CtCd.estimateCollisionIntervals(ctrs, minLatestNs, self.__rvizPublishers["continuous_time_collision_detection"])
		intervals = CtCd.refineCollisionIntervals(intervals)
		Ros.Log(f"After refinement {len(intervals)} intervals remained.")
		intervals.sort(key=lambda e: (e[-1], e[-2])) # Sort events by their end time, then start-time
		Ros.Log("Aggregating intervals.")
		eventGraphs: list[ConnectivityGraph] = []
		for interval in intervals:
			# Pick an arbitrary point in the interval. We pick the end.
			(_, _, _, _, _, eventTimeNs) = interval
			eventGraphs.append(self.at(eventTimeNs))
		if len(eventGraphs) == 0: eventGraphs = [self.at(polygon.timeNanoSecs)] # If no events, just update the locations of polygons.
		Ros.Log("Aggregated CGraphs", eventGraphs)
		for graph in eventGraphs: self.__appendToHistory(graph)
		return
