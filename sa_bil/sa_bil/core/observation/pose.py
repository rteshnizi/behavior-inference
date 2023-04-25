from shapely.geometry import Point

class Pose:
	def __init__(self, time, x, y, angleFromX):
		self.time: float = time
		self.x: float = x
		self.y: float = y
		self.angleFromX: float = angleFromX
		self.pt = Point(x, y)
		# FIXME: Set when deletedTracks is set and when new ID is detected in tracks dict
		self.spawn = False
		self.vanished = False

	@property
	def psi(self):
		"""The same as `angleFromX`"""
		return self.angleFromX

	def _bareRepr(self):
		return "(%.2f, %.2f, %.2f)" % (self.time, self.x, self.y)

	def __repr__(self):
		s = self._bareRepr()
		if self.spawn and self.vanished: return "%s%s" % ("+/- ", s)
		if self.spawn: return "%s%s" % ("+ ", s)
		if self.vanished: return "%s%s" % ("- ", s)
		return s
