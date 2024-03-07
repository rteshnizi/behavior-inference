from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from typing import Protocol, TypeAlias

import networkx as nx
from typing_extensions import Any, Generic, Literal, LiteralString, Mapping, TypeVar, final, overload

from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Pose import Coords
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RViz


class _PolygonLike(Protocol):
	@property
	def id(self) -> NodeId: ...
	@property
	def centroid(self) -> Coords: ...
	@property
	def bounds(self) -> tuple[Coords, Coords]: ...

_Polygon = TypeVar("_Polygon", bound=_PolygonLike)

@dataclass(frozen=True)
class NodeData(Generic[_Polygon]):
	"""The variables of this Class will be stored node attribute key-values."""
	polygon: _Polygon


@dataclass(frozen=True)
class EdgeData:
	isTemporal: bool

class NxUtils:
	from networkx.classes.reportviews import OutEdgeView  # CSpell: ignore reportviews

	NxDefaultLayout: TypeAlias = Mapping[Any, Any]
	GraphLayout: TypeAlias = dict[NodeId, tuple[float, float]]
	"""A dictionary from node id to an (X, Y) coordinate."""
	Id = NodeId
	NodeData = NodeData
	EdgeData = EdgeData

	class Graph(Generic[_Polygon], nx.DiGraph, ABC):
		__RENDER_DELTA_X = 200
		__RENDER_DELTA_Y = 100
		def __init__(self, rVizPublisher: Ros.Publisher | None):
			super().__init__()
			self.__rVizPublisher = rVizPublisher

		def __contains__(self, id: NodeId) -> bool:
			if not isinstance(id, NodeId):
				Ros.Log(f"NodeId is of invalid type: {type(id)}.")
				return False
			return super().__contains__(id)

		def addNode(self, id: NodeId, content: NodeData[_Polygon]) -> NodeId:
			data = asdict(content) if content is not None else {}
			self.add_node(id, **data)
			return id

		def addEdge(self, frmId: NodeId, toId: NodeId, addReverseEdge=False, content: EdgeData | None = None) -> None:
			if frmId not in self: raise AssertionError(f"{frmId} is not a node in the graph.")
			if toId not in self: raise AssertionError(f"{toId} is not a node in the graph.")
			if frmId == toId: raise AssertionError(f"No loop-back edge! {frmId}")
			data = asdict(content) if content is not None else {}
			self.add_edge(frmId, toId, **data)
			if not addReverseEdge: return
			self.add_edge(toId, frmId, **data)
			return

		@overload
		def getContent(self, node: NodeId, contentKey: Literal[""]) -> NodeData[_Polygon]: ...
		@overload
		def getContent(self, node: NodeId, contentKey: Literal["polygon"]) -> _Polygon: ...

		def getContent(self, node: NodeId, contentKey: LiteralString | Literal[""]) -> NodeData[_Polygon] | Any:
			if contentKey == "": return self.nodes[node]
			else: return self.nodes[node][contentKey]

		def __nodeSortKey(self, id: NodeId) -> tuple:
			((minX, minY), (maxX, maxY)) = self.getContent(id, "polygon").bounds
			return (id.timeNanoSecs, minX, maxX, minY, maxY)

		def _multiPartiteLayout(self) -> "NxUtils.GraphLayout":
			ids: list[NxUtils.Id] = sorted(self.nodes, key=self.__nodeSortKey)
			pos: NxUtils.GraphLayout = {}
			y = 0
			x = 0
			timeNanoSecs = 0
			zag = -1
			zig = (self.__RENDER_DELTA_Y / 10)
			for id in ids:
				if id.timeNanoSecs > timeNanoSecs:
					timeNanoSecs = id.timeNanoSecs
					x = 0
					y += self.__RENDER_DELTA_Y
				pos[id] = (x, (y + (zig * zag)))
				zag *= -1
				x += self.__RENDER_DELTA_X
			return pos

		@abstractmethod
		def getNodeMarkers(self) -> list[RViz.Msgs.Marker]: ...

		@abstractmethod
		def getEdgeMarkers(self) -> list[RViz.Msgs.Marker]: ...

		@final
		def __createMarkers(self) -> list[RViz.Msgs.Marker]:
			markers = []
			Ros.ConcatMessageArray(markers, self.getNodeMarkers())
			Ros.ConcatMessageArray(markers, self.getEdgeMarkers())
			return markers

		@final
		def render(self) -> None:
			if self.__rVizPublisher is None: return
			markerArray = RViz.Msgs.MarkerArray()
			Ros.AppendMessage(markerArray.markers, RViz.removeAllMarkers())
			Ros.ConcatMessageArray(markerArray.markers, self.__createMarkers())
			self.__rVizPublisher.publish(markerArray)
			return
