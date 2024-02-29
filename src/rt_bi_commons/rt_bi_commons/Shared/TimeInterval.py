import rt_bi_interfaces.msg as RtBiMsgs
from rt_bi_commons.Utils.Msgs import Msgs


class TimeInterval:
	def __init__(self ,startNanoSecs: int, endNanoSecs: int, includeLeft: bool, includeRight: bool) -> None:
		self.startNanoSecs: int = startNanoSecs
		self.endNanoSecs: int = endNanoSecs
		self.includeLeft: bool = includeLeft
		self.includeRight: bool = includeRight
		return

	def __contains__(self, time: float) -> bool:
		if self.includeLeft and time == self.startNanoSecs: return True
		if self.startNanoSecs < time and time < self.endNanoSecs: return True
		if self.includeRight and time == self.includeRight: return True
		return False

	def __repr__(self) -> str:
		startSym = "[" if self.includeLeft else "("
		endSym = "]" if self.includeRight else ")"
		return "T%s%d, %d%s" % (startSym, self.startNanoSecs, self.endNanoSecs, endSym)

	def intersects(self, other: "TimeInterval") -> bool:
		if self.includeLeft and other.includeLeft and self.startNanoSecs == other.startNanoSecs: return True
		if self.includeRight and other.includeRight and self.endNanoSecs == other.endNanoSecs: return True
		if self.includeRight and other.includeLeft and self.endNanoSecs == other.startNanoSecs: return True
		if self.includeLeft and other.includeRight and self.startNanoSecs == other.endNanoSecs: return True
		if other.startNanoSecs < self.startNanoSecs and self.startNanoSecs < other.endNanoSecs: return True
		if other.startNanoSecs < self.endNanoSecs and self.endNanoSecs < other.endNanoSecs: return True
		if self.startNanoSecs < other.startNanoSecs and other.startNanoSecs < self.endNanoSecs: return True
		if self.startNanoSecs < other.endNanoSecs and other.endNanoSecs < self.endNanoSecs: return True
		return False

	def toMsg(self) -> RtBiMsgs.TimeInterval:
		msg = RtBiMsgs.TimeInterval()
		msg.start = Msgs.toTimeMsg(self.startNanoSecs)
		msg.end = Msgs.toTimeMsg(self.endNanoSecs)
		msg.include_left = msg.include_left
		msg.include_right = msg.include_right
		return msg

	@classmethod
	def fromMsg(cls, msg: RtBiMsgs.TimeInterval) -> "TimeInterval":
		return TimeInterval(
			startNanoSecs=Msgs.toNanoSecs(msg.start),
			endNanoSecs=Msgs.toNanoSecs(msg.end),
			includeLeft=msg.include_left,
			includeRight=msg.include_right,
		)
