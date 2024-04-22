from dataclasses import asdict, dataclass
from typing import Any

from rt_bi_commons.Shared.NodeId import NodeId

_privateCounter = 0

@dataclass(frozen=True)
class StateToken:
	name: str
	graphNode: NodeId

	@staticmethod
	def fromNodeIdJson(nodeIdJson: str) -> "StateToken":
		global _privateCounter
		name = f"t{_privateCounter}"
		_privateCounter += 1
		return StateToken(
			name=name,
			graphNode=NodeId.fromJson(nodeIdJson),
		)

	def asDict(self) -> dict[str, Any]:
		return {
			"name": self.name,
			"graphNode": asdict(self.graphNode),
		}
