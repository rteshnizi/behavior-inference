from collections.abc import Sequence
from typing import AbstractSet, cast, overload

from rclpy.clock import Time

from rt_bi_commons.Shared.Pose import Coords, CoordsList, Pose, quatToAngle
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely

NANO_CONVERSION_CONSTANT = 10 ** 9

class Msgs:
	import builtin_interfaces.msg as Std
	import geometry_msgs.msg as Geometry

	import rt_bi_interfaces.msg as RtBi
	import rt_bi_interfaces.srv as RtBiSrv
	__ = RtBiSrv # RtBiSrv is being re-exported. This is here to get rid of unused import warning.

	@classmethod
	def DurationMsg(cls, sec: int, nanoSec: int) -> Std.Duration:
		d = cls.Std.Duration()
		d.sec = sec
		d.nanosec = nanoSec
		return d

	@classmethod
	def toTimeMsg(cls, time: float | int) -> Std.Time:
		"""
			* To process as secs, `time` must be a `float`.
			* To process as nanoSecs, `time` must be a `int`.
		"""
		msg = cls.Std.Time()
		if isinstance(time, float):
			sec = time
			msg.sec = int(sec)
			msg.nanosec = int((sec % 1) * NANO_CONVERSION_CONSTANT)
		elif isinstance(time, int):
			nanoSec = time
			msg.sec = int(nanoSec / NANO_CONVERSION_CONSTANT)
			msg.nanosec = nanoSec % NANO_CONVERSION_CONSTANT
		return msg

	@classmethod
	def toNanoSecs(cls, t: Std.Time | Time) -> int:
		if isinstance(t, cls.Std.Time):
			(sec, nanoSecs) = (t.sec, t.nanosec) # CSpell: ignore nanosec
		elif isinstance(t, Time):
			(sec, nanoSecs) = cast(tuple[int, int], t.seconds_nanoseconds())
		else:
			raise ValueError(f"Value is of unknown type: {repr(t)}")
		return ((sec * NANO_CONVERSION_CONSTANT) + nanoSecs)

	@classmethod
	def toCoords(cls, p: Geometry.Point32) -> Coords:
		return (p.x, p.y)

	@classmethod
	def toCoordsList(cls, pts: Sequence[Geometry.Point32] | AbstractSet[Geometry.Point32] | list[Geometry.Point32]) -> CoordsList:
		return [(p.x, p.y) for p in pts]

	@classmethod
	def toStdPoint(cls, pose: Pose) -> Geometry.Point32:
		return cls.Geometry.Point32(x=pose.x, y=pose.y, z=0.0)

	@classmethod
	def toStdPose(cls, pose: Pose) -> Geometry.Pose:
		pointMsg = cls.Geometry.Point32(x=pose.x, y=pose.y, z=0.0)
		quat = pose.angleAsQuaternion()
		quadMsg = cls.Geometry.Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
		return cls.Geometry.Pose(position=pointMsg, orientation=quadMsg)

	@classmethod
	def toAngle(cls, q: Geometry.Quaternion) -> float:
		return quatToAngle((q.x, q.y, q.z, q.w))

	@overload
	@classmethod
	def toStdPolygon(cls, poly: Shapely.Polygon) -> Geometry.Polygon: ...

	@overload
	@classmethod
	def toStdPolygon(cls, poly: RtBi.Polygon) -> Geometry.Polygon: ...

	@classmethod
	def toStdPolygon(cls, poly: Shapely.Polygon | RtBi.Polygon) -> Geometry.Polygon:
		if isinstance(poly, cls.RtBi.Polygon):
			return poly.region
		msg = cls.Geometry.Polygon(points=[cls.Geometry.Point32(x=p[0], y=p[1], z=0.0) for p in GeometryLib.getGeometryCoords(poly)])
		return msg
