from typing import Sequence

from visualization_msgs.msg import Marker

from rt_bi_utils.Pose import Pose
from rt_bi_utils.Ros import Logger
from rt_bi_utils.RViz import KnownColors, RViz


class Tracklet(Pose):
	def __init__(self, id: int, timeNanoSecs: int, x: float, y: float, angleFromX: float) -> None:
		super().__init__(timeNanoSecs, x, y, angleFromX)
		self.id = id

	def __repr__(self) -> str:
		return "Trk-%d" % self.id

	def render(self) -> Sequence[Marker]:
		msgs = []
		if self.spawned: msgs.append(RViz.createCircle("%s-circle" % repr(self), self.x, self.y, radius=5, outline=KnownColors.LIGHT_GREEN))
		elif self.vanished: msgs.append(RViz.createCircle("%s-circle" % repr(self), self.x, self.y, radius=5, outline=KnownColors.DARK_RED))
		else: msgs.append(RViz.createCircle("%s-circle" % repr(self), self.x, self.y, radius=10, outline=KnownColors.PURPLE, width=3.0))
		msgs.append(RViz.createText("%s-txt" % repr(self), (self.x, self.y), repr(self), KnownColors.WHITE))
		Logger().info("%d TRACKLET MARKERS MADE @ %s" % (len(msgs), repr((self.x, self.y))))
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = []
		msgs.append(RViz.removeMarker("%s-circle" % repr(self)))
		msgs.append(RViz.removeMarker("%s-txt" % repr(self)))
		return msgs
