from typing import Sequence

from visualization_msgs.msg import Marker

from rt_bi_commons.Shared.Pose import Pose
from rt_bi_commons.Utils.RViz import ColorNames, RViz


class Tracklet(Pose):
	def __init__(self, id: int, timeNanoSecs: int, x: float, y: float, angleFromX: float, spawned = False, vanished = False) -> None:
		super().__init__(timeNanoSecs, x, y, angleFromX)
		self.id = id
		self.spawned = spawned
		self.vanished = vanished

	def __repr__(self) -> str:
		s = "TK#%d" % self.id
		if self.spawned and self.vanished: return "%s%s" % ("[+/-]", s)
		if self.spawned: return "%s%s" % ("[+]", s)
		if self.vanished: return "%s%s" % ("[-]", s)
		return s

	def render(self, durationNs: int = -1) -> Sequence[Marker]:
		msgs = []
		if self.spawned: msgs.append(RViz.createCircle("TK#%d-circle" % self.id, self.x, self.y, radius=13, outline=ColorNames.GREEN, width=3.0, durationNs=durationNs))
		elif self.vanished: msgs.append(RViz.createCircle("TK#%d-circle" % self.id, self.x, self.y, radius=13, outline=ColorNames.RED, width=3.0, durationNs=durationNs))
		else: msgs.append(RViz.createCircle("TK#%d-circle" % self.id, self.x, self.y, radius=10, outline=ColorNames.PURPLE, width=3.0, durationNs=durationNs))
		msgs.append(RViz.createText("TK#%d-txt" % self.id, (self.x, self.y), repr(self), ColorNames.WHITE, durationNs=durationNs))
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = []
		msgs.append(RViz.removeMarker("TK#%d-circle" % self.id))
		msgs.append(RViz.removeMarker("TK#%d-txt" % self.id))
		return msgs
