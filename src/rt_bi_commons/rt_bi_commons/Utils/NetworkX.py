from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from json import dumps
from typing import Callable

import networkx as nx
from typing_extensions import Any, Generic, Literal, LiteralString, Optional, Protocol, Sequence, TypeAlias, TypeVar, cast, final, overload

from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Pose import Coords
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RViz


class _PolygonLike(Protocol):
	@property
	def id(self) -> NodeId: ...
	@property
	def centroid(self) -> Coords: ...
	@property
	def bounds(self) -> tuple[Coords, Coords]: ...
	@property
	def timeNanoSecs(self) -> int: ...
	@property
	def predicates(self) -> Predicates: ...


_Polygon = TypeVar("_Polygon", bound=_PolygonLike)

@dataclass(frozen=True)
class NodeData(Generic[_Polygon]):
	"""The variables of this Class will be stored node attribute key-values."""
	polygon: Optional[_Polygon] = None
	predicates: Optional[Predicates] = None


@dataclass(frozen=True)
class EdgeData:
	isTemporal: bool

class NxUtils:
	from networkx.classes.reportviews import OutEdgeView  # CSpell: ignore reportviews

	GraphLayout2D: TypeAlias = dict[NodeId, tuple[float, float]]
	"""A dictionary from node id to an (X, Y) coordinate."""
	GraphLayout3D: TypeAlias = dict[NodeId, tuple[float, float, float]]
	"""A dictionary from node id to an (X, Y) coordinate."""
	Id = NodeId
	NodeData = NodeData
	EdgeData = EdgeData

	class Graph(Generic[_Polygon], nx.DiGraph, ABC):
		__RENDER_DELTA_X = 75
		__RENDER_DELTA_Y = 90
		__RENDER_DELTA_Z = 75
		def __init__(self, rVizPublisher: Ros.Publisher | None):
			nx.DiGraph.__init__(self)
			self.rVizPublisher = rVizPublisher

		def __contains__(self, id_: NodeId | Sequence[NodeId]) -> bool:
			if not isinstance(id_, NodeId) and not isinstance(id_, Sequence):
				Ros.Log(f"NodeId is of invalid type: {type(id_)}.")
				return False
			return super().__contains__(id_)

		def removeNode(self, id_: NodeId) -> None:
			assert isinstance(id_, NodeId), f"Unexpected Id type: {type(id_)}, repr = {repr(id_)}"
			assert id_ in self, f"Remove failed: {id_} is not a node in the graph."
			self.remove_node(id_)
			return

		def addNode(self, id: NodeId, content: NodeData[_Polygon] | None = None) -> NodeId:
			assert isinstance(id, NodeId), f"Unexpected Id type: {type(id)}, repr = {repr(id)}"
			if content is not None: self.add_node(id, **asdict(content))
			else: self.add_node(id)
			return id

		def addEdge(self, frmId: NodeId, toId: NodeId, addReverseEdge=False, content: EdgeData | None = None) -> None:
			if frmId not in self: raise AssertionError(f"{frmId} is not a node in the graph.")
			if toId not in self: raise AssertionError(f"{toId} is not a node in the graph.")
			if frmId == toId: raise AssertionError(f"No loop-back edge! {frmId}")
			if content is not None: self.add_edge(frmId, toId, **asdict(content))
			else: self.add_edge(frmId, toId)
			if not addReverseEdge: return
			if content is not None: self.add_edge(toId, frmId, **asdict(content))
			else: self.add_edge(toId, frmId)
			return

		@overload
		def getContent(self, node: NodeId) -> NodeData[_Polygon]: ...
		@overload
		def getContent(self, node: NodeId, contentKey: None) -> NodeData[_Polygon]: ...
		@overload
		def getContent(self, node: NodeId, contentKey: Literal["polygon"]) -> _Polygon: ...
		@overload
		def getContent(self, node: NodeId, contentKey: Literal["predicates"]) -> Predicates: ...

		def getContent(self, node: NodeId, contentKey: LiteralString | None = None) -> NodeData[_Polygon] | _Polygon | Predicates:
			assert isinstance(node, NodeId), f"Unexpected Id type: {type(node)}, repr = {repr(node)}"
			if contentKey is None: return NodeData(**self.nodes[node])
			elif contentKey == "predicates":
				d = self.getContent(node)
				if d.predicates is not None: predicates = d.predicates
				else: predicates = self.getContent(node, "polygon").predicates
				return predicates
			else: return self.nodes[node][contentKey]

		def _3dLayout(self) -> "NxUtils.GraphLayout3D":
			pos2d = self._multiPartiteLayout2()
			pos: NxUtils.GraphLayout3D = {}
			minHIndex = 0
			for id in pos2d:
				if (minHIndex == 0 or id.hIndex < minHIndex) and id.hIndex > 0: minHIndex = id.hIndex
			for id in pos2d:
				dz = id.hIndex - minHIndex
				pos[id] = (pos2d[id][0], pos2d[id][1], dz * self.__RENDER_DELTA_Z)
			return pos

		def __nodeSortKey(self, id: NodeId) -> tuple:
			((minX, minY), (maxX, maxY)) = self.getContent(id, "polygon").bounds
			return (id.timeNanoSecs, minX, maxX, minY, maxY)

		def _multiPartiteLayout2(self) -> "NxUtils.GraphLayout2D":
			pos: NxUtils.GraphLayout2D = {}
			id_: NxUtils.Id
			for id_ in self.nodes:
				poly = self.getContent(id_, "polygon")
				pos[id_] = (poly.centroid[0], poly.centroid[1])
			return pos

		def _multiPartiteLayout1(self) -> "NxUtils.GraphLayout2D":
			ids: list[NxUtils.Id] = sorted(self.nodes, key=self.__nodeSortKey)
			pos: NxUtils.GraphLayout2D = {}
			y = 0
			x = 0
			for id in ids:
				pos[id] = (x, y)
				if y == 0: y = self.__RENDER_DELTA_Y
				else: y = 0
				x += self.__RENDER_DELTA_X
			return pos

		@abstractmethod
		def createNodeMarkers(self) -> list[RViz.Msgs.Marker]: ...

		@abstractmethod
		def createEdgeMarkers(self) -> list[RViz.Msgs.Marker]: ...

		def asStr(self, nodeFilter: Callable[[NodeId], bool] | None = None, nodeMapping: dict = {}) -> str:
			if nodeFilter is None: g = self
			else: g = nx.subgraph_view(self, filter_node=nodeFilter)
			jsonDict = nx.adjacency_data(g)
			for node in jsonDict["nodes"]:
				node = cast(dict[str, Any], node)
				node["predicates"] = cast(_Polygon, node["polygon"]).predicates
				node.pop("polygon")
				node.pop("subset")
			return dumps(jsonDict, default=vars)

		@final
		def __createMarkers(self) -> list[RViz.Msgs.Marker]:
			markers = []
			Ros.ConcatMessageArray(markers, self.createNodeMarkers())
			Ros.ConcatMessageArray(markers, self.createEdgeMarkers())
			return markers

		@final
		def render(self) -> None:
			if Ros.IsProfiling(): return
			if self.rVizPublisher is None: return
			markerArray = RViz.Msgs.MarkerArray()
			Ros.AppendMessage(markerArray.markers, RViz.removeAllMarkers())
			Ros.ConcatMessageArray(markerArray.markers, self.__createMarkers())
			Ros.Publish(self.rVizPublisher, markerArray)
			return
