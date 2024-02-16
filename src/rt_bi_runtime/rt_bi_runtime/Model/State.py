class State:
	def __init__(self, name: str, starting: bool, accepting: bool) -> None:
		self.name = name
		self.start = starting
		self.accepting = accepting

	def __repr__(self) -> str:
		return "(%s-%s-%s)" % (self.name, "T" if self.start else "F", "T" if self.accepting else "F")

	def __hash__(self) -> int:
		return hash(repr(self))

	def __eq__(self, other: "State") -> bool:
		return self.name == other.name
