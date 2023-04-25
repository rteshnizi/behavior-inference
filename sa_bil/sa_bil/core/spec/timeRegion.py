class TimeInterval:
	def __init__(self, start: float, end: float, startIncluded: bool, endIncluded: bool):
		self.start = start
		self.end = end
		self.startIncluded = startIncluded
		self.endIncluded = endIncluded

	def __contains__(self, time: float) -> bool:
		if self.startIncluded and time == self.start: return True
		if self.start < time and time < self.end: return True
		if self.endIncluded and time == self.endIncluded: return True
		return False

	def __repr__(self) -> str:
		startSym = "[" if self.startIncluded else "("
		endSym = "]" if self.endIncluded else ")"
		return "T%s%.2f, %.2f%s" % (startSym, self.start, self.end, endSym)

	def intersect(self, other: "TimeInterval") -> bool:
		if self.startIncluded and other.startIncluded and self.start == other.start: return True
		if self.endIncluded and other.endIncluded and self.end == other.end: return True
		if self.endIncluded and other.startIncluded and self.end == other.start: return True
		if self.startIncluded and other.endIncluded and self.start == other.end: return True
		if other.start < self.start and self.start < other.end: return True
		if other.start < self.end and self.end < other.end: return True
		if self.start < other.start and other.start < self.end: return True
		if self.start < other.end and other.end < self.end: return True
		return False

class TemporalRegion:
	"""
		Temporal Region
	"""
	def __init__(self, endPoints: list, intervalFlags: list, endPointFlags: list):
		self.endPoints = endPoints
		self.intervalFlags = intervalFlags
		self.endPointFlags = endPointFlags

	def __contains__(self, time: float):
		raise RuntimeError("Not implemented")
