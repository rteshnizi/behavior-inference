from typing_extensions import Self, cast

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RViz import RViz


class TopologicalGraph(NxUtils.Graph):
	__RENDER_RADIUS = 10
	def __init__(self, vertices: list[Msgs.RtBi.Id], adjacency: list[Msgs.RtBi.Adjacency], rVizPublisher: Ros.Publisher | None):
		super().__init__(rVizPublisher)

		for i in range(len(vertices)):
			id_ = Msgs.toId(vertices[i])
			self.addNode(id_)
		for i in range(len(vertices)):
			fromId = Msgs.toId(vertices[i])
			adjMsg = adjacency[i]
			for toIdMsg in adjMsg.neighbors:
				toId = Msgs.toId(toIdMsg)
				self.addEdge(fromId, toId)

	def createNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		if len(self.nodes) == 0: return markers
		nodePositions = self._3dLayout()
		for id in nodePositions:
			coords = nodePositions[id]
			marker = RViz.createCircle(id, center=coords, radius=self.__RENDER_RADIUS, outline=ColorNames.ORANGE)
			Ros.AppendMessage(markers, marker)
			marker = RViz.createText(id, coords=coords, text=f"{id.shortNames}", outline=ColorNames.WHITE, idSuffix="txt")
			Ros.AppendMessage(markers, marker)
		return markers

	def createEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
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

	def apply(self, event: Msgs.RtBi.Event) -> Self:
		Ros.Log(f"Type: {event.type} @ {event.time_nano_secs}")
		assert isinstance(event.before, list)
		assert isinstance(event.after, list)
		edges: list[tuple[NxUtils.Id, NxUtils.Id]] = []
		if event.type == "A":
			Ros.Log(f"APPEARED", event.after)
			for idMsg in event.after:
				toId = Msgs.toId(idMsg)
				self.addNode(toId)
			for i in range(len(event.spatial_edge_from)):
				fromId = Ros.GetMessage(event.spatial_edge_from, i, Msgs.RtBi.Id)
				toId = Ros.GetMessage(event.spatial_edge_to, i, Msgs.RtBi.Id)
				edges.append((Msgs.toId(fromId), Msgs.toId(toId)))
		elif event.type == "D":
			Ros.Log(f"DISAPPEARED", event.before)
			for idMsg in event.before:
				toId = Msgs.toId(idMsg)
				self.removeNode(toId)
		elif event.type == "S":
			assert len(event.before) == 1
			fromId = Msgs.toId(event.before[0])
			Ros.Log(f"SPLITTED {fromId}", event.after)
			for idMsg in event.after:
				toId = Msgs.toId(idMsg)
				self.addNode(toId)
				# self.addEdge(fromId, toId, content=NxUtils.EdgeData(isTemporal=True))
		elif event.type == "M":
			Ros.Logger().error("We don't want merge events...")
			assert len(event.after) == 1
			Ros.Log("MERGED", event.before)
		else:
			raise AssertionError(f"Unexpected event type: {event.type}.")
		# for (fromId, toId) in edges: self.addEdge(fromId, toId, addReverseEdge=True)
		Ros.Log(f"NEW EDGES", edges)
		return self
