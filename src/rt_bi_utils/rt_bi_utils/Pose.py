from shapely.geometry import Point


class Pose:
	def __init__(self, timeNanoSecs: int, x: float, y: float, angleFromX: float):
		"""
		Representation of a pose in RTBI implementation.

		Parameters
		----------
		timeNanoSecs : `float`
			The time in nanoseconds.
		x : `float`

		y : `float`

		angleFromX : `float`

		"""
		self.timeNanoSecs: int = timeNanoSecs
		self.x: float = float(x)
		self.y: float = float(y)
		self.angleFromX: float = float(angleFromX)
		self.pt = Point(x, y)
		# FIXME: Set when deletedTracks is set and when new ID is detected in tracks dict
		self.spawn = False
		self.vanished = False

	@property
	def psi(self) -> float:
		"""The same as `angleFromX`"""
		return self.angleFromX

	def _bareRepr(self) -> str:
		return "(%.2f, %.2f, %.2f)" % (self.timeNanoSecs, self.x, self.y)

	def __repr__(self) -> str:
		s = self._bareRepr()
		if self.spawn and self.vanished: return "%s%s" % ("+/- ", s)
		if self.spawn: return "%s%s" % ("+ ", s)
		if self.vanished: return "%s%s" % ("- ", s)
		return s
