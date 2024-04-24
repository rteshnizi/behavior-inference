from json import loads
from typing import Any, cast

import networkx as nx

from rt_bi_behavior.Model.BehaviorAutomaton import StateToken
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils


class RhsIGraph(NxUtils.Graph):
	def evaluate(self, token: StateToken) -> StateToken:
		return token

	def createNodeMarkers(self) -> list:
		return []

	def createEdgeMarkers(self) -> list:
		return []

	@classmethod
	def fromMsg(cls, msg: Msgs.RtBi.IGraph) -> "RhsIGraph":
		d: dict[str, Any] = loads(msg.adjacency_json)
		for node in d["nodes"]:
			node["id"] = NodeId.fromDict(node["id"])
			node["predicates"] = Predicates(node["predicates"])
		for adj in d["adjacency"]:
			for edge in adj:
				edge["id"] = NodeId.fromDict(edge["id"])
		return nx.adjacency_graph(d)
