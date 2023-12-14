class Pose:
	def __init__(self, timeNanoSecs: int, x: float, y: float, angleFromX: float):
		"""
		Representation of a pose.

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
		self.spawned = False
		self.vanished = False

	@property
	def psi(self) -> float:
		"""The same as `angleFromX`"""
		return self.angleFromX

	def __bareRepr(self) -> str:
		return "(%d, %.2f, %.2f)" % (self.timeNanoSecs, self.x, self.y)

	def __repr__(self) -> str:
		s = self.__bareRepr()
		if self.spawned and self.vanished: return "%s%s" % ("+/- ", s)
		if self.spawned: return "%s%s" % ("+ ", s)
		if self.vanished: return "%s%s" % ("- ", s)
		return s
