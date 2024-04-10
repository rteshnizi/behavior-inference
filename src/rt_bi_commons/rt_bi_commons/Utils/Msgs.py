from collections.abc import Sequence
from typing import AbstractSet, Final, cast

from rclpy.clock import Time

from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Pose import Coords, CoordsList, Pose, quatToAngle
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely

NANO_CONVERSION_CONSTANT: Final[int] = 10 ** 9
"""``10 ** 9``"""

class Msgs:
	import builtin_interfaces.msg as Std
	import geometry_msgs.msg as Geometry

	import rt_bi_interfaces.msg as RtBi
	import rt_bi_interfaces.srv as RtBiSrv
	__ = RtBiSrv # RtBiSrv is being re-exported. This is here to get rid of unused import warning.

	@classmethod
	def toSecsNanoSecsPair(cls, nanoSecs: int) -> tuple[int, int]:
		secs = int(nanoSecs / NANO_CONVERSION_CONSTANT)
		nanoSecs = nanoSecs % NANO_CONVERSION_CONSTANT
		return (secs, nanoSecs)

	@classmethod
	def DurationMsg(cls, nanoSecs: int) -> Std.Duration:
		d = cls.Std.Duration()
		(secs, nanoSecs) = cls.toSecsNanoSecsPair(nanoSecs)
		d.sec = secs
		d.nanosec = nanoSecs
		return d

	@classmethod
	def toTimeMsg(cls, time: float | int | Time) -> Std.Time:
		"""
			* To process as secs, `time` must be a `float`.
			* To process as nanoSecs, `time` must be a `int`.
		"""
		msg = cls.Std.Time()
		if isinstance(time, Time):
			msg = time.to_msg()
		elif isinstance(time, float):
			nanoSecs = cls.toNanoSecs(time)
			(secs, nanoSecs) = cls.toSecsNanoSecsPair(nanoSecs)
			msg.sec = int(secs)
			msg.nanosec = int((secs % 1) * NANO_CONVERSION_CONSTANT)
		elif isinstance(time, int):
			nanoSecs = time
			(secs, nanoSecs) = cls.toSecsNanoSecsPair(nanoSecs)
			msg.sec = secs
			msg.nanosec = nanoSecs
		else:
			raise ValueError(f"Value is of unknown type: {repr(time)}")
		return msg

	@classmethod
	def toNanoSecs(cls, time: Std.Time | Time | float) -> int:
		"""Returns a given time as a single `int` in nano-seconds."""
		if isinstance(time, cls.Std.Time):
			(secs, nanoSecs) = (time.sec, time.nanosec) # CSpell: ignore nanosec
		elif isinstance(time, Time):
			(secs, nanoSecs) = cast(tuple[int, int], time.seconds_nanoseconds())
		elif isinstance(time, float):
			secs = time
			secs = int(secs)
			nanoSecs = int((secs % 1) * NANO_CONVERSION_CONSTANT)
		else:
			raise ValueError(f"Value is of unknown type: {repr(time)}")
		return ((secs * NANO_CONVERSION_CONSTANT) + nanoSecs)

	@classmethod
	def toCoords(cls, p: Geometry.Point32) -> Coords:
		return (p.x, p.y)

	@classmethod
	def toCoordsList(cls, pts: Sequence[Geometry.Point32] | AbstractSet[Geometry.Point32] | list[Geometry.Point32]) -> CoordsList:
		return [(p.x, p.y) for p in pts]

	@classmethod
	def toStdPoint(cls, p: Pose | Coords) -> Geometry.Point32:
		if isinstance(p, Pose): return cls.Geometry.Point32(x=p.x, y=p.y, z=0.0)
		return cls.Geometry.Point32(x=p[0], y=p[1], z=0.0)

	@classmethod
	def toStdPose(cls, pose: Pose) -> Geometry.Pose:
		pointMsg = cls.Geometry.Point32(x=pose.x, y=pose.y, z=0.0)
		quat = pose.angleAsQuaternion()
		quadMsg = cls.Geometry.Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
		return cls.Geometry.Pose(position=pointMsg, orientation=quadMsg)

	@classmethod
	def toAngle(cls, q: Geometry.Quaternion) -> float:
		return quatToAngle((q.x, q.y, q.z, q.w))

	@classmethod
	def toStdPolygon(cls, poly: Shapely.Polygon | RtBi.Polygon) -> Geometry.Polygon:
		if isinstance(poly, cls.RtBi.Polygon): return poly.region
		points = [cls.Geometry.Point32(x=p[0], y=p[1], z=0.0) for p in GeometryLib.getGeometryCoords(poly)]
		msg = cls.Geometry.Polygon(points=points)
		return msg

	@classmethod
	def toIdMsg(cls, id_: NodeId) -> RtBi.Id:
		return Msgs.RtBi.Id(
			h_index=id_.hIndex,
			time_nano_secs=id_.timeNanoSecs,
			region_id=id_.regionId,
			polygon_id=id_.polygonId,
			sub_part_id=id_.subPartId,
		)

	@classmethod
	def toId(cls, id_: RtBi.Id) -> NodeId:
		return NodeId(
			hIndex=id_.h_index,
			timeNanoSecs=id_.time_nano_secs,
			regionId=id_.region_id,
			polygonId=id_.polygon_id,
			subPartId=id_.sub_part_id,
		)
