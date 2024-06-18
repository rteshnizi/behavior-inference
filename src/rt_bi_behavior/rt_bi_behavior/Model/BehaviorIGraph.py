from json import loads
from typing import Any, cast

import networkx as nx

from rt_bi_behavior.Model.Transition import TransitionStatement
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils


class BehaviorIGraph(NxUtils.Graph):
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

	def propagate(self, source: NodeId, visited: set[NodeId]) -> dict[NodeId, list[NodeId]]:
		"""Returns a dictionary from target Id to path"""
		if source not in self.nodes: return {}
		weightFn = lambda u, v, d: 1000000 if v in visited else 1
		# The return value is a tuple of two dictionaries keyed by target nodes. The first dictionary stores distance to each target node. The second stores the path to each target node.
		paths: tuple[dict[NodeId, float], dict[NodeId, list[NodeId]]] = nx.single_source_dijkstra(self, source, cutoff=1000001, weight=weightFn) # pyright: ignore[reportArgumentType]
		destinations = paths[1]
		# Ros.Log("DIJKSTRA", paths, severity=Ros.LoggingSeverity.ERROR)
		# Ros.Log("DIJKSTRA", [(d, destinations[d]) for d in destinations], severity=Ros.LoggingSeverity.ERROR)
		return destinations

	def propagateOneStep(self, source: NodeId, visited: set[NodeId]) -> dict[NodeId, list[NodeId]]:
		if source not in self.nodes: return {}
		paths: dict[NodeId, list[NodeId]] = {}
		for destination in cast(list[NodeId], self[source]):
			if destination in visited: continue
			paths[destination] = [source, destination]
		return paths

	@classmethod
	def fromMsg(cls, msg: Msgs.RtBi.IGraph) -> "BehaviorIGraph":
		d: dict[str, Any] = loads(msg.adjacency_json)
		for node in d["nodes"]:
			node["id"] = NodeId.fromDict(node["id"])
			node["predicates"] = Predicates(node["predicates"])
		for adj in d["adjacency"]:
			for edge in adj:
				edge["id"] = NodeId.fromDict(edge["id"])
		# Ros.Log("Graph Nodes", d["nodes"], severity=Ros.LoggingSeverity.ERROR)
		# Ros.Log("Graph Adjacency", d["adjacency"], severity=Ros.LoggingSeverity.ERROR)
		g = nx.adjacency_graph(d)
		return BehaviorIGraph(g)
