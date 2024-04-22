from dataclasses import asdict, dataclass
from typing import Any

from rt_bi_commons.Shared.NodeId import NodeId


class State:
	def __init__(self, name: str, starting: bool, accepting: bool) -> None:
		self.name = name
		self.start = starting
		self.accepting = accepting

	def __str__(self) -> str:
		return self.name

	def __repr__(self) -> str:
		parts = [
			"START" if self.start else "",
			"FINAL" if self.accepting else "",
			self.name,
		]
		partsStr = "-".join(filter(None, parts))
		return f"({partsStr})"

	def __hash__(self) -> int:
		return hash(repr(self))

	def __eq__(self, other: "State") -> bool:
		return self.name == other.name

@dataclass(frozen=True)
class StateToken:
	stateName: str
	graphNode: NodeId

	@staticmethod
	def fromNodeIdJson(stateName: str, nodeIdJson: str) -> "StateToken":
		return StateToken(
			stateName=stateName,
			graphNode=NodeId.fromJson(nodeIdJson),
		)

	def asDict(self) -> dict[str, Any]:
		return {
			"stateName": self.stateName,
			"graphNode": asdict(self.graphNode),
		}
