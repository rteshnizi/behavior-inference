# NOTICE: This object must only rely on external dependencies. Otherwise there will be many circular-dependency issues.
from math import isnan, nan
from typing import Sequence

from scipy.spatial.transform import Rotation

Coords = tuple[float, float]
"""``(x, y)``"""
Coords2d = Coords
"""``(x, y)``"""
Coords3d = tuple[float, float, float]
"""``(x, y, z)``
Only used in RViz.
"""
CoordsList = list[Coords]
"""``[(x0, y0), (x1, y1), (x2, y2), ...]``"""
Quaternion = tuple[float, float, float, float]
"""``(x, y, z, w)``"""

class Pose:
	""" Representation of a pose. """
	def __init__(self, timeNanoSecs: int, x: float, y: float, angleFromX: float = nan, quat: Quaternion | None = None):
		"""
		:param int timeNanoSecs: The time in nanoseconds.
		:param float x: The X coordinate.
		:param float y: The Y coordinate.
		:param float angleFromX: Angle with respect to the X-axis in radians. To obtain with scipy ``rotation.as_euler("xyz")[2]``.
		:param Quaternion quat: Angle represented a `Quaternion`. To obtain with scipy ``Rotation.from_euler("z", angle).as_quat()``.
		"""
		self.timeNanoSecs: int = timeNanoSecs
		self.x: float = float(x)
		self.y: float = float(y)
		self.angleFromX: float = float(angleFromX)
		if not isnan(angleFromX) and quat is not None: raise AssertionError("Only one of 'angleFromX' and 'quat' must be set.")
		if isnan(angleFromX) and quat is not None: self.angleFromX: float = quatToAngle(quat)

	@property
	def psi(self) -> float:
		"""The same as `angleFromX` in radians."""
		return self.angleFromX

	def angleAsQuaternion(self) -> Quaternion:
		return angleToQuat(self.angleFromX)

	def asCoords(self) -> Coords:
		return (self.x, self.y)

	def __repr__(self) -> str:
		return "(T=%d ns, X=%.2f, Y=%.2f)" % (self.timeNanoSecs, self.x, self.y)

def angleToQuat(angleRad: float) -> Quaternion:
	quat = Rotation.from_euler("z", angleRad, degrees=False).as_quat()
	return(quat[0], quat[1], quat[2], quat[3])

def quatToAngle(quat: Quaternion | Sequence[float]) -> float:
	return Rotation.from_quat(list(quat)).as_euler("xyz")[2]
