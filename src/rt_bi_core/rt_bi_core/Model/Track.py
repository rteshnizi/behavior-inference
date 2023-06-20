# from bil.model.trajectory import Trajectory
from typing import Dict, Tuple
from bil.gui.drawing import Drawing
from bil.observation.pose import Pose

class Track:
	def __init__(self, idNum: int, time: float, x: float, y: float, psi: float, isInterpolated=False):
		self.id = idNum
		self.isInterpolated = isInterpolated
		self.pose = Pose(time, x, y, psi)
		self.canvasId = None

	def __repr__(self):
		return "Trk-%d: %s" % (self.id, repr(self.pose))

	def render(self, canvas):
		if self.canvasId is not None: self.clearRender(canvas)
		fill = ""
		if self.pose.spawn:
			fill = "GREEN"
		elif self.pose.vanished:
			fill = "MAROON"
		tag = "track%d-%.1f" % (self.id, self.pose.time)
		color = "GREEN" if self.isInterpolated else "PURPLE"
		width = 1 if self.isInterpolated else 3
		self.canvasId = Drawing.CreateCircle(canvas, self.pose.x, self.pose.y, radius=5, fill=fill, outline=color, tag=tag, width=width)


	def clearRender(self, canvas):
		if self.canvasId is not None:
			Drawing.RemoveShape(canvas, self.canvasId)
			self.canvasId = None

Tracks = Dict[Tuple[float, int], Track]
"""
Tracks is a dictionary of (time, trackId) to Track.
"""
