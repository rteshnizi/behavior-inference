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

	def propagate(self, source: NodeId) -> dict[NodeId, list[NodeId]]:
		"""Returns a dictionary from target Id to path"""
		if source not in self.nodes: return {}
		destinations = cast(dict[NodeId, list[NodeId]], nx.shortest_path(self, source))
		return destinations

	def propagateOneStep(self, source: NodeId, visited: set[NodeId]) -> dict[NodeId, list[NodeId]]:
		if source not in self.nodes: return {}
		paths: dict[NodeId, list[NodeId]] = {}
		if "https://rezateshnizi.com/tower_bridge/defintion/av1" in repr(source):
			Ros.Log(f"Neighbors of {source}", self[source])
			Ros.Log("Visited", visited)
		for destination in cast(list[NodeId], self[source]):
			if destination in visited: continue
			paths[destination] = [source, destination]
		return paths

	@classmethod
	def fromMsg(cls, msg: Msgs.RtBi.IGraph) -> "BehaviorIGraph":
		d: dict[str, Any] = loads(msg.adjacency_json)
		# found = False
		for node in d["nodes"]:
			node["id"] = NodeId.fromDict(node["id"])
			# if "[9]@0/https://rezateshnizi.com/tower_bridge/defintion/av1/0/9-sets#114-12-0" == repr(node["id"]):
			# if node["id"].hIndex == 15 or node["id"].hIndex == 14:
			# 	found = True
			node["predicates"] = Predicates(node["predicates"])
		for adj in d["adjacency"]:
			for edge in adj:
				edge["id"] = NodeId.fromDict(edge["id"])
		# Ros.Log("Graph Nodes", d["nodes"])
		# Ros.Log("Graph Adjacency", d["adjacency"])
		g = nx.adjacency_graph(d)
		return BehaviorIGraph(g)
