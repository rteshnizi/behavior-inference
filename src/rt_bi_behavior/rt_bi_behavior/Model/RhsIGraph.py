from json import loads
from typing import Any, cast

import networkx as nx

from rt_bi_behavior.Model.Transition import TransitionStatement
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils


class RhsIGraph(NxUtils.Graph):
	def __init__(self, g: nx.DiGraph | None = None):
		NxUtils.Graph.__init__(self, None)
		nx.DiGraph.__init__(self, g)

	def createNodeMarkers(self) -> list:
		return []

	def createEdgeMarkers(self) -> list:
		return []

	def satisfies(self, node: NodeId, criterion: TransitionStatement) -> bool:
		predicates = self.getContent(node, "predicates")
		return criterion.evaluate(predicates)

	def propagate(self, source: NodeId) -> dict[NodeId, list[NodeId]]:
		if source not in self.nodes: return {}
		destinations = cast(dict[NodeId, list[NodeId]], nx.shortest_path(self, source))
		return cast(dict[NodeId, list[NodeId]], destinations)

	@classmethod
	def fromMsg(cls, msg: Msgs.RtBi.IGraph) -> "RhsIGraph":
		d: dict[str, Any] = loads(msg.adjacency_json)
		for node in d["nodes"]:
			node["id"] = NodeId.fromDict(node["id"])
			node["predicates"] = Predicates(node["predicates"])
		for adj in d["adjacency"]:
			for edge in adj:
				edge["id"] = NodeId.fromDict(edge["id"])
		g = nx.adjacency_graph(d)
		return RhsIGraph(g)
