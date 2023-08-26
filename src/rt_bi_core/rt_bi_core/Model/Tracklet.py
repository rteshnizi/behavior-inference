# from bil.model.trajectory import Trajectory
from typing import Dict, List, Tuple

from visualization_msgs.msg import Marker

from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import KnownColors, RViz


class Tracklet:
	def __init__(self, idNum: int, timeNanoSecs: int, x: float, y: float, psi: float, isInterpolated=False):
		self.id = idNum
		self.isInterpolated = isInterpolated
		self.pose = Pose(timeNanoSecs, x, y, psi)
		self.canvasId = None

	def __repr__(self):
		return "Trk-%d: %s" % (self.id, repr(self.pose))

	def render(self) -> List[Marker]:
		msgs = []
		color = KnownColors.GREEN
		if self.pose.spawn:
			color = KnownColors.GREEN
		elif self.pose.vanished:
			color = KnownColors.MAROON
		tag = "track%d-%.1f" % (self.id, self.pose.timeNanoSecs)
		color = color if self.isInterpolated else KnownColors.PURPLE
		width = 1 if self.isInterpolated else 3
		msgs.append(RViz.CreateCircle(tag, self.pose.x, self.pose.y, color, width))

	def clearRender(self):
		return

Tracklets = Dict[Tuple[float, int], Tracklet]
"""
Tracklets is a dictionary of (time, trackId) to Tracklet.
"""
