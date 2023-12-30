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

	@property
	def psi(self) -> float:
		"""The same as `angleFromX`"""
		return self.angleFromX

	def __repr__(self) -> str:
		return "(T=%d ns, X=%.2f, Y=%.2f)" % (self.timeNanoSecs, self.x, self.y)
