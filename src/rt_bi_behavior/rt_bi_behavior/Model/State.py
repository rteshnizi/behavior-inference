from typing import TypedDict

from rt_bi_commons.Shared.NodeId import NodeId


class StateTokenWithHistory(TypedDict):
	id: str
	path: list[NodeId]

class StateToken(TypedDict):
	id: str
	iGraphNode: NodeId

class State(TypedDict):
	label: str
	tokens: list[StateToken]
	style: str
	fillcolor: str

class State2(TypedDict):
	label: str
	tokens: list[StateTokenWithHistory]
	style: str
	fillcolor: str
