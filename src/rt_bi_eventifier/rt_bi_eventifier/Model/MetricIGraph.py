from dataclasses import dataclass
from typing import Callable, Literal, cast

from rt_bi_commons.Shared.Color import RGBA, ColorUtils
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RViz import ColorNames, RViz
from rt_bi_core.Spatial import GraphPolygon, MapPolygon
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.ContinuousTimePolygon import ContinuousTimePolygon
from rt_bi_core.Spatial.DynamicPolygon import DynamicPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_eventifier.Model.EventAggregator import EventAggregator


class MetricIGraph(NxUtils.Graph[GraphPolygon]):
	"""
		The implementation of a Region IGraph in python as described in the dissertation.
	"""

	SUBMODULES = ("c_graph", "ctcd", "i_graph")
	SUBMODULE = Literal["c_graph", "ctcd", "i_graph"]
	"""The name of a ShadowTree sub-module publisher."""
	__RENDER_RADIUS = 10
	__MAX_HISTORY = 4

	@dataclass(frozen=True, order=True)
	class NodeData(NxUtils.NodeData[GraphPolygon]):
		subset: int = -1
		"""Used in multipartite layout.
		https://networkx.org/documentation/stable/reference/generated/networkx.drawing.layout.multipartite_layout.html
		"""

		@staticmethod
		def extend(data: NxUtils.NodeData[GraphPolygon], subset: int) -> "MetricIGraph.NodeData":
			return MetricIGraph.NodeData(
				polygon=data.polygon,
				subset=subset
			)

	def __init__(self, rvizPublishers: dict[SUBMODULE, Ros.Publisher | None] | None = None):
		"""Initialize the I-graph."""
		rvizPublisher = None if rvizPublishers is None else rvizPublishers.pop("i_graph", None)
		super().__init__(rVizPublisher=rvizPublisher)
		# super().__init__(rVizPublisher=None)
		self.__history: list[ConnectivityGraph] = []
		self.hIndex = 0

		self.componentEvents: list[list[Shapely.Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """
		self.__rvizPublishers = rvizPublishers if rvizPublishers is not None else {}
		self.__ctrs: dict[AffinePolygon.Id, ContinuousTimePolygon[GraphPolygon]] = {}

	@property
	def history(self) -> list[ConnectivityGraph]:
		return self.__history

	@property
	def depth(self) -> int:
		"""The depth of the I-graph history stack."""
		return len(self.history)

	@property
	def processedTime(self) -> int:
		if self.depth == 0: return -1
		return self.history[-1].timeNanoSecs

	def __repr__(self) -> str:
		if self.depth == 0:
			timeRangeStr = "0"
		if self.depth == 1:
			timeRangeStr = "%d" % self.history[0].timeNanoSecs
		if self.depth > 1:
			timeRangeStr = "%d , %d" % (self.history[0].timeNanoSecs, self.history[-1].timeNanoSecs)
		return f"IGr-[{timeRangeStr})(D={self.depth}, N={len(self.nodes)}, E={len(self.edges)})"

	def asStr(self, depth = 2) -> str:
		# self.hIndex - 1 is because the value of hIndex is one more than the last assigned hIndex.
		filterFn = lambda n: cast(NxUtils.Id, n).hIndex > (self.hIndex - 1) - depth
		return super().asStr(filterFn)

	def addNode(self, id: NxUtils.Id, cGraph: ConnectivityGraph) -> NxUtils.Id:
		assert cGraph.hIndex is not None and cGraph.hIndex > -1, f"Unset hIndex is not allowed in ShadowTree: cGraph = {repr(cGraph)}, hIndex = {cGraph.hIndex}"
		content = cGraph.getContent(id)
		id = id.copy(hIndex=cGraph.hIndex)
		content = MetricIGraph.NodeData.extend(data=content, subset=cGraph.hIndex)
		return super().addNode(id=id, content=content)

	def addEdge(self, fromId: NxUtils.Id, toId: NxUtils.Id, fromCGraph: ConnectivityGraph, toCGraph: ConnectivityGraph) -> None:
		fromId = fromId.copy(hIndex=fromCGraph.hIndex)
		toId = toId.copy(hIndex=toCGraph.hIndex)
		content = NxUtils.EdgeData(isTemporal=fromCGraph.timeNanoSecs != toCGraph.timeNanoSecs)
		return super().addEdge(fromId, toId, addReverseEdge=False, content=content)

	def __getNodeRenderZOffset(self, type_: StaticPolygon.Type | SensingPolygon.Type | AffinePolygon.Type | DynamicPolygon.Type) -> float:
		if type_ == SensingPolygon.type: return 2 * self.__RENDER_RADIUS
		elif type_ == AffinePolygon.type: return -2 * self.__RENDER_RADIUS
		else: return 0

	def createNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._3dLayout()
		for id in nodePositions:
			poly = self.getContent(id, "polygon")
			color = poly.envelopeColor
			zOffset = self.__getNodeRenderZOffset(poly.type)
			coords = (nodePositions[id][0], nodePositions[id][1], nodePositions[id][2] + zOffset)
			marker = RViz.createSphere(id, center=coords, radius=self.__RENDER_RADIUS, color=color)
			Ros.AppendMessage(markers, marker)
			coords = (coords[0], coords[1], coords[2] + self.__RENDER_RADIUS)
			marker = RViz.createText(id, coords=coords, text=poly.shortName, outline=ColorNames.WHITE, fontSize=8.0, idSuffix="txt")
			Ros.AppendMessage(markers, marker)
		id = RViz.Id(hIndex=-1, timeNanoSecs=-1, regionId="IGraph", polygonId="Name", subPartId="")
		marker = RViz.createText(id, coords=(250, -50), text=f"{repr(self)}", outline=ColorNames.WHITE, idSuffix="txt")
		Ros.AppendMessage(markers, marker)
		return markers

	def __getEdgeRenderColor(self, frmPoly: GraphPolygon, toPoly: GraphPolygon, isTemporal: bool) -> RGBA:
		if isTemporal: return ColorNames.CYAN_DARK
		return ColorUtils.avgColor(frmPoly.envelopeColor, toPoly.envelopeColor)

	def createEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._3dLayout()
		outEdgeView: NxUtils.OutEdgeView = self.out_edges()
		for (frm, to) in outEdgeView:
			frm = cast(NxUtils.Id, frm)
			to = cast(NxUtils.Id, to)
			if frm == to: continue
			frmPoly = self.getContent(frm, "polygon")
			zOffset = self.__getNodeRenderZOffset(frmPoly.type)
			frmCoords = (nodePositions[frm][0], nodePositions[frm][1], nodePositions[frm][2] + zOffset)
			toPoly = self.getContent(to, "polygon")
			zOffset = self.__getNodeRenderZOffset(toPoly.type)
			toCoords = (nodePositions[to][0], nodePositions[to][1], nodePositions[to][2] + zOffset)
			edgeData = outEdgeView[frm, to]
			color = self.__getEdgeRenderColor(frmPoly, toPoly, "isTemporal" in edgeData and edgeData["isTemporal"])
			marker = RViz.createLine(frm, coordsList=[frmCoords, toCoords], outline=color, width=1, idSuffix=repr(to))
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
		cGraph = ConnectivityGraph(timeNanoSecs, map_, sensors, self.__rvizPublishers.get("c_graph", None))
		return cGraph

	def renderLatestCGraph(self) -> None:
		if self.depth == 0: return
		Ros.Log(f"{repr(self)} Rendering latest CGraph.")
		self.history[-1].render()
		return

	def __shadowsAreConnectedTemporally(self, pastGraph: ConnectivityGraph, nowGraph: ConnectivityGraph, pastPoly: MapPolygon, nowPoly: MapPolygon) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the transformation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		try:
			intersectionOfShadows = GeometryLib.intersection(pastPoly.interior, nowPoly.interior)
			intersectionOfShadows = GeometryLib.filterPolygons(intersectionOfShadows)
			if len(intersectionOfShadows) == 0: return False
			intersectionOfShadows = GeometryLib.union(intersectionOfShadows)
			objs = []
			for nowSensor in nowGraph.sensors:
				if pastGraph.hasSensor(nowSensor.id):
					pastSensor = pastGraph.getSensor(nowSensor.id)
					pastCoords = GeometryLib.getGeometryCoords(pastSensor.interior)
					nowCoords = GeometryLib.getGeometryCoords(nowSensor.interior)
					transformation = GeometryLib.getAffineTransformation(pastCoords, nowCoords)
					for nowEdge in nowSensor.edges:
						pastEdge = pastSensor.getEquivalentEdge(nowEdge, transformation)
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
				else:
					objs.append(nowSensor.interior)
			objs = objs if len(objs) > 0 else [Shapely.Polygon()]
			sweptBySensors = Shapely.GeometryCollection(geoms=objs)
			remainingShadows = GeometryLib.difference(intersectionOfShadows, sweptBySensors)
			if len(remainingShadows) > 0: return True
		except Exception as e:
			from traceback import format_exc
			Ros.Log(f"Error in X-Connection Test -- {e}\n{format_exc()}")
		return False

	def __connectTopLayerTemporally(self) -> None:
		Ros.Log(" ------------------------------- CONNECT-TEMPORALLY - START -----------------------------")
		fromGraph = self.history[self.depth - 2]
		toGraph = self.history[self.depth - 1]
		assert toGraph.hIndex is not None, f"Cannot connect graph with unset hIndex in I-graph. {repr(toGraph)}"
		# Add temporal edges between FOVs
		if (
			(toGraph.hasTrack and not toGraph.fovEvent) or
			(toGraph.hasTrack and toGraph.track.exited)
		):
			Ros.Log(" ------------------------------- CONNECT-Z -------------------------------")
			for fromNodeId in fromGraph.nodes:
				fromPoly = fromGraph.getContent(fromNodeId, "polygon")
				if fromPoly.type != SensingPolygon.type: continue
				if not fromPoly.hasTrack: continue
				for toNodeId in toGraph.nodes:
					if fromNodeId == toNodeId: continue
					toPoly = toGraph.getContent(toNodeId, "polygon")
					if toPoly.type != SensingPolygon.type: continue
					if not toPoly.hasTrack: continue
					if GeometryLib.intersects(fromPoly.interior, toPoly.interior):
						Ros.Log("AntiShadows are connected", (fromPoly.id, toPoly.id))
						self.addEdge(fromPoly.id, toPoly.id, fromGraph, toGraph)
		elif (
			(not toGraph.hasTrack) or
			(toGraph.fovEvent and toGraph.track.entered)
		):
			Ros.Log(" ------------------------------- CONNECT-X -----------------------------")
			for fromNodeId in fromGraph.nodes:
				fromPoly = fromGraph.getContent(fromNodeId, "polygon")
				if not fromPoly.isAccessible: continue
				if fromPoly.type == SensingPolygon.type: continue
				for toNodeId in toGraph.nodes:
					if fromNodeId == toNodeId: continue
					toPoly = toGraph.getContent(toNodeId, "polygon")
					if not toPoly.isAccessible: continue
					if toPoly.type == SensingPolygon.type: continue
					if GeometryLib.intersects(fromPoly.interior, toPoly.interior):
						if self.__shadowsAreConnectedTemporally(fromGraph, toGraph, fromPoly, toPoly):
							Ros.Log("Shadows are connected", (fromPoly.id, toPoly.id))
							self.addEdge(fromPoly.id, toPoly.id, fromGraph, toGraph)
		Ros.Log(" ------------------------------- CONNECT-TEMPORALLY - END -------------------------------")
		return

	def __removeFromHistory(self, index: int, delete: bool) -> None:
		assert index >= 0 and index < len(self.history), f"Index out of history bounds: index = {index}, Len = {len(self.history)}"
		hIndex = self.history[index].hIndex
		assert hIndex is not None, f"Graph with unset hIndex found in I-graph. {repr(self.history[index])}"

		Ros.Log(f"Removing Graph: {repr(self.history[index])} with {len(self.history[index].nodes)} nodes.")
		polyId: NxUtils.Id
		for polyId in self.history[index].nodes:
			id_ = NxUtils.Id(hIndex=hIndex, timeNanoSecs=polyId.timeNanoSecs, regionId=polyId.regionId, polygonId=polyId.polygonId, subPartId=polyId.subPartId)
			self.removeNode(id_)
		if delete: self.history.pop(index)

	def __replaceInHistory(self, index: int, graph: ConnectivityGraph) -> None:
		""" **Does not add nodes.** This just manipulates `self.history` variable. """
		self.__removeFromHistory(index, False)
		Ros.Log(f"REPLACE graph with {len(graph.shadows)} shadows and {len(graph.antiShadows)} anti-shadows.")
		# graph.logGraphNodes()
		hIndex = cast(int, self.history[index].hIndex)
		graph.hIndex = hIndex
		self.history[index] = graph
		return

	def __appendToHistory(self, graph: ConnectivityGraph, eventHandler: Callable[["MetricIGraph", bool], None]) -> None:
		shouldBroadcastEvent = False
		isomorphicUpdate = False
		if self.depth > 0 and graph.timeNanoSecs < self.history[-1].timeNanoSecs:
				Ros.Logger().error(f"Older graph than latest in history --> {graph.timeNanoSecs} vs {self.history[-1].timeNanoSecs}")
				return

		shouldBroadcastEvent = True
		if self.depth > 0 and EventAggregator.isIsomorphic(self.history[-1], graph):
			Ros.Log("Isomorphic graph detected.")
			self.__replaceInHistory(self.depth - 1, graph)
			isomorphicUpdate = True
		else:
			Ros.Log(f"APPENDING graph with {len(graph.shadows)} shadows and {len(graph.antiShadows)} anti-shadows.")
			graph.hIndex = self.hIndex
			self.history.append(graph)
			self.hIndex += 1

		for id_ in graph.nodes:
			id_ = cast(NxUtils.Id, id_)
			self.addNode(id_, graph)
		for edge in graph.edges:
			self.addEdge(edge[0], edge[1], graph, graph)

		if self.depth > 1: self.__connectTopLayerTemporally()
		if self.depth > self.__MAX_HISTORY:
			Ros.Log(f"History depth is more than MAX={self.__MAX_HISTORY} graphs.")
			self.__removeFromHistory(0, True)
		if shouldBroadcastEvent: eventHandler(self, isomorphicUpdate)
		self.render()
		return

	def __updateCTRs(self, poly: GraphPolygon) -> ContinuousTimePolygon[GraphPolygon] | None:
		if poly.timeNanoSecs < self.processedTime:
			Ros.Log(f"Out of sync update: {poly.timeNanoSecs} < {self.processedTime} processed already.")
			return None
		idSansTime = poly.id.sansTime()
		if idSansTime not in self.__ctrs:
			Ros.Log("Creating CTR...")
			self.__ctrs[idSansTime] = ContinuousTimePolygon(polyConfigs=[poly])
			Ros.Log(f"Created {repr(self.__ctrs[idSansTime])}")
			return self.__ctrs[idSansTime]
		Ros.Log(f"Updating -> {repr(self.__ctrs[idSansTime])}")
		self.__ctrs[idSansTime].addPolygon(poly, self.processedTime)
		Ros.Log(f"Updated  -> {repr(self.__ctrs[idSansTime])}")
		return self.__ctrs[idSansTime]

	def __latestNanoSecsBounds(self) -> tuple[int, int]:
		minNs = ContinuousTimePolygon.INF_NS + 1
		maxNs = -1
		for ctr in self.__ctrs.values():
			if ctr.latestNanoSecs < minNs:
				minNs = ctr.latestNanoSecs
			if ctr.latestNanoSecs != ContinuousTimePolygon.INF_NS and ctr.latestNanoSecs > maxNs:
				maxNs = ctr.latestNanoSecs
		return (minNs, maxNs)

	def updatePolygon(self, polygon: GraphPolygon, eventHandler: Callable[["MetricIGraph", bool], None]) -> None:
		Ros.Log(128 * "↓") # A separator in the logs
		Ros.Log(f"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX {repr(self)} XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
		Ros.Log("Updated Poly ->", [ f"{polygon}", f"T={polygon.timeNanoSecs}", f"{polygon.predicates}"])
		if self.depth == 0 and polygon.type != StaticPolygon.type:
			Ros.Log("Initial CGraph must be created from a static map.")
			return
		updatedCTR = self.__updateCTRs(polygon)
		if updatedCTR is None: return

		(minLatestNs, maxLatestNs) = self.__latestNanoSecsBounds()
		Ros.Log("Post Update", [
			f"Processed up to {self.processedTime}",
			f"Min LatestNs =  {minLatestNs}",
			f"Max LatestNs =  {maxLatestNs}",
		])
		if self.processedTime > 0 and minLatestNs <= self.processedTime:
			Ros.Log(f"Not ready to process yet: Min LatestNs <= processedTime")
			return
		minLatestNs = polygon.timeNanoSecs if updatedCTR.isProjective else minLatestNs
		Ros.Log(f"Processing {minLatestNs:20}.")

		if self.depth == 0:
			cGraph = self.at(minLatestNs)
			self.__appendToHistory(cGraph, eventHandler)
			return

		ctrs = list(self.__ctrs.values())
		intervals = CtCd.estimateCollisionIntervals(ctrs, minLatestNs, self.__rvizPublishers.get("ctcd", None))
		intervals = CtCd.refineCollisionIntervals(intervals)
		Ros.Log(f"After refinement {len(intervals)} intervals remained.")
		intervals.sort(key=lambda e: (e[-1], e[-2])) # Sort events by their end time, then start-time
		eventGraphs: list[ConnectivityGraph] = []
		for interval in intervals:
			# Pick an arbitrary point in the interval. We pick the end.
			(_, _, _, _, _, eventTimeNs) = interval
			eventGraphs.append(self.at(eventTimeNs))
		if len(eventGraphs) == 0: eventGraphs = [self.at(polygon.timeNanoSecs)] # If no events, just update the locations of polygons.
		Ros.Log("Aggregated CGraphs", eventGraphs)
		for graph in eventGraphs: self.__appendToHistory(graph, eventHandler)
		return
