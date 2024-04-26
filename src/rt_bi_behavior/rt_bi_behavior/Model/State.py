from typing import TypedDict

from rt_bi_commons.Shared.NodeId import NodeId


class StateToken(TypedDict):
	id: str
	iGraphNode: NodeId

class State(TypedDict):
	label: str
	tokens: list[StateToken]
