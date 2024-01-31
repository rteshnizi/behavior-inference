from math import isnan, nan
from typing import List, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation

Coords = Tuple[float, float]
CoordsList = List[Coords]
Quaternion = Tuple[float, float, float, float]
"""`(x, y, z, w)`"""

class Pose:
	def __init__(self, timeNanoSecs: int, x: float, y: float, angleFromX: float = nan, quat: Union[Quaternion, None] = None):
		"""
		Representation of a pose.

		Parameters
		----------
		timeNanoSecs : `int`
			The time in nanoseconds.
		x : `float`

		y : `float`

		angleFromX : `float`
			In radians.
		"""
		self.timeNanoSecs: int = timeNanoSecs
		self.x: float = float(x)
		self.y: float = float(y)
		if isnan(angleFromX) and quat is None: raise ValueError("At least one of \"angleFromX\" or \"quat\" must be set.")
		if not isnan(angleFromX): self.angleFromX: float = float(angleFromX)
		else: self.angleFromX: float = float(angleFromX)

	@property
	def psi(self) -> float:
		"""The same as `angleFromX` in radians."""
		return self.angleFromX

	def angleAsQuaternion(self) -> Quaternion:
		quat = angleToQuat(self.angleFromX)
		return(quat[0], quat[1], quat[2], quat[3])

	def asCoords(self) -> Coords:
		return (self.x, self.y)

	def __repr__(self) -> str:
		return "(T=%d ns, X=%.2f, Y=%.2f)" % (self.timeNanoSecs, self.x, self.y)

def angleToQuat(angleRad: float) -> np.ndarray: # CSpell: ignore ndarray
	return Rotation.from_euler("z", angleRad, degrees=False).as_quat()
