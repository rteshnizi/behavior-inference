from rt_bi_commons.Utils.Msgs import Msgs


class TimeInterval:
	def __init__(self, minNanoSecs: int, maxNanoSecs: int, includeMin: bool, includeMax: bool) -> None:
		self.minNanoSecs: int = minNanoSecs
		self.maxNanoSecs: int = maxNanoSecs
		self.includeMin: bool = includeMin
		self.includeMax: bool = includeMax
		return

	def __contains__(self, timeNanoSecs: int) -> bool:
		if self.includeMin and timeNanoSecs == self.minNanoSecs: return True
		if self.minNanoSecs < timeNanoSecs and timeNanoSecs < self.maxNanoSecs: return True
		if self.includeMax and timeNanoSecs == self.includeMax: return True
		return False

	def __repr__(self) -> str:
		startSym = "[" if self.includeMin else "("
		endSym = "]" if self.includeMax else ")"
		return "T%s%d, %d%s" % (startSym, self.minNanoSecs, self.maxNanoSecs, endSym)

	def intersects(self, other: "TimeInterval") -> bool:
		if self.includeMin and other.includeMin and self.minNanoSecs == other.minNanoSecs: return True
		if self.includeMax and other.includeMax and self.maxNanoSecs == other.maxNanoSecs: return True
		if self.includeMax and other.includeMin and self.maxNanoSecs == other.minNanoSecs: return True
		if self.includeMin and other.includeMax and self.minNanoSecs == other.maxNanoSecs: return True
		if other.minNanoSecs < self.minNanoSecs and self.minNanoSecs < other.maxNanoSecs: return True
		if other.minNanoSecs < self.maxNanoSecs and self.maxNanoSecs < other.maxNanoSecs: return True
		if self.minNanoSecs < other.minNanoSecs and other.minNanoSecs < self.maxNanoSecs: return True
		if self.minNanoSecs < other.maxNanoSecs and other.maxNanoSecs < self.maxNanoSecs: return True
		return False

	def toMsg(self) -> Msgs.RtBi.TimeInterval:
		msg = Msgs.RtBi.TimeInterval()
		msg.min = Msgs.toTimeMsg(self.minNanoSecs)
		msg.max = Msgs.toTimeMsg(self.maxNanoSecs)
		msg.include_min = self.includeMin
		msg.include_max = self.includeMax
		return msg

	@classmethod
	def fromMsg(cls, msg: Msgs.RtBi.TimeInterval) -> "TimeInterval":
		return TimeInterval(
			minNanoSecs=Msgs.toNanoSecs(msg.min),
			maxNanoSecs=Msgs.toNanoSecs(msg.max),
			includeMin=msg.include_min,
			includeMax=msg.include_max,
		)
