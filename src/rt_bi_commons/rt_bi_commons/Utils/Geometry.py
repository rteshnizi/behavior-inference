import warnings
from math import cos, inf, nan, sin, sqrt

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from skimage.transform import AffineTransform as AffineTransform, matrix_transform
from typing_extensions import Callable, Final, Sequence, TypeAlias, TypeVar, cast

from rt_bi_commons.Shared.Pose import Coords, Coords2d, Coords3d, CoordsList, Pose, Quaternion, angleToQuat, quatToAngle
from rt_bi_commons.Utils import Ros


class Shapely:
	"""This class sets up a group of functions and type aliases that help use shapely objects easier."""
	from shapely import buffer, convex_hull, get_rings, make_valid, set_precision, union_all
	from shapely.geometry import GeometryCollection, LinearRing, LineString, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon

	ConnectedComponent: TypeAlias = Polygon | LineString | Point
	MultiComponent: TypeAlias = MultiPolygon | MultiLineString | MultiPoint
	AnyObj: TypeAlias = Polygon | MultiPolygon | LineString | MultiLineString | Point | MultiPoint

	@staticmethod
	def __pointNeg(self: Point) -> Point: # pyright: ignore[reportSelfClsParameterName]
		""" `-1 * Point` """
		return Shapely.Point(-1 * self.x, -1 * self.y)

	@staticmethod
	def __pointAdd(self: Point, other: Point) -> Point: # pyright: ignore[reportSelfClsParameterName]
		""" `Point + Point` """
		return Shapely.Point(self.x + other.x, self.y + other.y)

	@staticmethod
	def __pointSub(self: Point, other: Point) -> Point: # pyright: ignore[reportSelfClsParameterName]
		""" `Point - Point` """
		return Shapely.Point(self.x - other.x, self.y - other.y)

	@staticmethod
	def __pointDivScalar(self: Point, num: float) -> Point: # pyright: ignore[reportSelfClsParameterName]
		""" `Point / num` """
		return Shapely.Point(self.x / num, self.y / num)

	@staticmethod
	def __pointMulScalar(self: Point, num: float) -> Point: # pyright: ignore[reportSelfClsParameterName]
		""" `Point * num` """
		return Shapely.Point(self.x * num, self.y * num)

	@staticmethod
	def __connectedGeometryRepr(self: ConnectedComponent) -> str: # pyright: ignore[reportSelfClsParameterName]
		coordsList = GeometryLib.getGeometryCoords(self)
		numCoords = f"#{len(coordsList)}"
		valid = f"v:{'Y' if self.is_valid else 'N'}"
		empty = f"e:{'Y' if self.is_empty else 'N'}"
		return ", ".join([numCoords, valid, empty])

	@staticmethod
	def __multiGeometryRepr(self: MultiComponent) -> str: # pyright: ignore[reportSelfClsParameterName]
		return "{%s}" % ", ".join([repr(o) for o in self.geoms])

	Point.__neg__ = __pointNeg
	Point.__add__ = __pointAdd
	Point.__sub__ = __pointSub
	Point.__mul__ = __pointMulScalar
	Point.__truediv__ = __pointDivScalar

	Point.__repr__ = lambda self: f"{self.x:7.2f}, {self.y:7.2f}"
	LinearRing.__repr__ = lambda self: f"R[{Shapely.__connectedGeometryRepr(self)}, l:{self.length:7.2f}]"
	LineString.__repr__ = lambda self: f"L[{Shapely.__connectedGeometryRepr(self)}, l:{self.length:7.2f}]"
	Polygon.__repr__ = lambda self: f"P[{Shapely.__connectedGeometryRepr(self)}, a:{self.area:7.2f}]"

	MultiPoint.__repr__ = lambda self: Shapely.__multiGeometryRepr(self)
	MultiLineString.__repr__ = lambda self: Shapely.__multiGeometryRepr(self)
	MultiPolygon.__repr__ = lambda self: Shapely.__multiGeometryRepr(self)
	GeometryCollection.__repr__ = lambda self: Shapely.__multiGeometryRepr(self)

class GeometryLib:
	"""
	Custom Computational Geometry implementation.
	© Reza Teshnizi 2018-2024
	"""
	EPSILON: Final = 1e-6
	Vector = Coords
	Vector3d = Coords3d
	Coords = Coords
	CoordsList = CoordsList
	Quaternion = Quaternion
	CoordsMap = dict[Coords, Coords]
	angleToQuat: Callable[[float], Quaternion] = angleToQuat
	quatToAngle: Callable[[Quaternion | Sequence[float]], float] = quatToAngle

	__Coords = TypeVar("__Coords", bound=Coords | Coords3d)
	__Vector = TypeVar("__Vector", bound=Vector | Vector3d)

	# region: Logging functions
	@staticmethod
	def __reportShapelyException(functionName: str, exc: Exception, objs: Sequence[Shapely.AnyObj]) -> None:
		from traceback import format_exc
		Ros.Logger().error(f"1. Shapely exception.. in {functionName}(): {exc}")
		Ros.Log("2. Shapely exception.. args", objs)
		i = 3
		for o in objs:
			if hasattr(o, "geoms"):
				verts = [GeometryLib.getGeometryCoords(g) for g in o.geoms]
			else:
				verts = GeometryLib.getGeometryCoords(o)
			Ros.Log(f"{i}. Shapely exception.. Verts of {repr(o)}", verts)
			i += 1
		Ros.Log(f"{i}. Shapely exception.. Stack trace:\n{format_exc()}")
		return

	@staticmethod
	def __reportSkImageException(functionName: str, exc: Exception, objs: Sequence[CoordsList]) -> None:
		from traceback import format_exc
		Ros.Logger().error("1. SkImage exception.. in %s(): %s" % (functionName, repr(exc)))
		Ros.Log("2. SkImage exception.. args", objs)
		Ros.Log(f"3. SkImage exception.. Stack trace:\n{format_exc()}")
		return
	# endregion: Logging functions

	# region: Basic operations
	@staticmethod
	def toCoords3d(coords: Coords | Coords3d) -> Coords3d:
		x = coords[0]
		y = coords[1]
		z = coords[2] if len(coords) == 3 else 0
		return (x, y, z)

	@staticmethod
	def toCoords(pt: Shapely.Point) -> Coords:
		return (pt.x, pt.y)

	@staticmethod
	def addCoords(c1: __Coords, c2: __Coords) -> __Coords:
		(x, y) = (c1[0] + c2[0], c1[1] + c2[1])
		if len(c1) == 2: return (x, y) # pyright: ignore[reportReturnType]
		z = c1[2] + c2[2]
		return (x, y, z) # pyright: ignore[reportReturnType]

	@staticmethod
	def subtractCoords(c1: __Coords, c2: __Coords) -> __Coords:
		(x, y) = (c1[0] - c2[0], c1[1] - c2[1])
		if len(c1) == 2: return (x, y) # pyright: ignore[reportReturnType]
		z = c1[2] - c2[2]
		return (x, y, z) # pyright: ignore[reportReturnType]

	@staticmethod
	def scaleCoords(c1: __Coords, s: float) -> __Coords:
		(x, y) = (c1[0] * s, c1[1] * s)
		if len(c1) == 2: return (x, y) # pyright: ignore[reportReturnType]
		z = c1[2] * s
		return (x, y, z) # pyright: ignore[reportReturnType]

	@staticmethod
	def coordsDistance(coords1: Coords | Coords3d, coords2: Coords | Coords3d) -> float:
		coords1 = GeometryLib.toCoords3d(coords1)
		coords2 = GeometryLib.toCoords3d(coords2)
		dx = coords1[0] - coords2[0]
		dy = coords1[1] - coords2[1]
		dz = coords1[2] - coords2[2]
		return GeometryLib.vectLength(dx, dy, dz)

	@staticmethod
	def coordsAreEqual(coords1: Coords | Coords3d, coords2: Coords | Coords3d, withinEpsilon = False) -> bool:
		d1 = GeometryLib.coordsDistance(coords1, coords2)
		margin = GeometryLib.EPSILON if withinEpsilon else 0
		return d1 <= margin

	@staticmethod
	def coordsAreAlmostEqual(coords1: Coords, coords2: Coords) -> bool:
		d1 = GeometryLib.coordsDistance(coords1, coords2)
		return d1 <= GeometryLib.EPSILON * 10 # EPSILON is too small unfortunately

	@staticmethod
	def vectLength(x: float, y: float, z = 0.0) -> float:
		return sqrt((x * x) + (y * y) + (z * z))

	@staticmethod
	def orthogonal(p1: Coords, p2: Coords) -> Coords:
		return (p2[1] - p1[1], -1 * (p2[0] - p1[0]))

	@staticmethod
	def midpoint(p1: Coords, p2: Coords) -> Coords:
		return GeometryLib.midpointXy(p1[0], p1[1], p2[0], p2[1])

	@staticmethod
	def midpointXy(x1: float, y1: float, x2: float, y2: float) -> Coords:
		return ((x1 + x2) / 2, (y1 + y2) / 2)

	@staticmethod
	def slope(x1: float, y1: float, x2: float, y2: float) -> float:
		return (y2 - y1) / (x2 - x1)

	@staticmethod
	def getUnitVector(vect: __Vector) -> __Vector:
		"""
		Returns a tuple (x, y) of a vector.
		"""
		if len(vect) == 2:
			(x, y) = cast(GeometryLib.Vector, vect)
			distance = GeometryLib.vectLength(x, y)
			return (x / distance, y / distance) # pyright: ignore[reportReturnType]
		(x, y, z) = cast(GeometryLib.Vector3d, vect)
		distance = GeometryLib.vectLength(x, y, z)
		return (x / distance, y / distance, z / distance) # pyright: ignore[reportReturnType]

	@staticmethod
	def getUnitVectorFromAngle(theta: float) -> Vector:
		"""
		theta is the angle w.r.t X axis
		Returns a tuple (x, y) of a vector.
		"""
		x = cos(theta)
		y = sin(theta)
		return GeometryLib.getUnitVector((x, y))
	# endregion: Basic operations

	# region: Shapely wrappers
	@staticmethod
	def toPoint(p: Pose | Coords2d) -> Shapely.Point:
		x = 0
		y = 0
		if isinstance(p, Pose):
			x = p.x
			y = p.y
		else:
			x = p[0]
			y = p[1]
		return Shapely.Point(x, y)

	@staticmethod
	def filterPolygons(geom: Shapely.AnyObj) -> list[Shapely.Polygon]:
		geomList = GeometryLib.toGeometryList(geom)
		geomList = [Shapely.make_valid(Shapely.set_precision(p, GeometryLib.EPSILON)) for p in geomList]
		geomList = [p for p in geomList if (not p.is_empty) and p.area > 0]
		return geomList

	@staticmethod
	def getGeometryCoords(geom: Shapely.ConnectedComponent) -> CoordsList:
		if not hasattr(geom, "exterior"):
			if not hasattr(geom, "coords"):
				raise ValueError(f"Unknown geometry: {repr(geom)}")
			return list(geom.coords)
		return list(geom.exterior.coords)

	@staticmethod
	def lineSegmentsAreAlmostEqual(l1: Shapely.LineString, l2: Shapely.LineString) -> bool:
		l1Coords = GeometryLib.getGeometryCoords(l1)
		l2Coords = GeometryLib.getGeometryCoords(l2)
		if GeometryLib.coordsAreAlmostEqual(l1Coords[0], l2Coords[0]) and GeometryLib.coordsAreAlmostEqual(l1Coords[1], l2Coords[1]): return True
		if GeometryLib.coordsAreAlmostEqual(l1Coords[0], l2Coords[1]) and GeometryLib.coordsAreAlmostEqual(l1Coords[1], l2Coords[0]): return True
		return False

	@staticmethod
	def toGeometryList(polys: Shapely.AnyObj) -> list[Shapely.ConnectedComponent]:
		"""If you are unsure if a shapely object is multi-part or a single object,
		this function returns either a list of the sub-parts of the multi-part object,
		or a list containing the given connected object.
		"""
		try:
			if len(polys.geoms) > 0:
				return list(polys.geoms)
			raise RuntimeError("`geoms` should never be empty in a multi-geometry.")
		except:
			# Assumption here is that if it throws because Shapely.Polygon does not have a `geoms` property.
			return [polys]

	@staticmethod
	def lineSegmentSlope(segment: Shapely.LineString) -> float:
		x11, y11 = segment.coords[0]
		x12, y12 = segment.coords[1]
		if x12 - x11 == 0: return inf
		slope = (y12 - y11) / (x12 - x11)
		return slope

	@staticmethod
	def intersects(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> bool:
		"""## Intersects
		A function to safely test if two geometries intersect while handling shapely exceptions and warnings.

		Parameters
		----------
		o1 : `Shapely.AnyObj`
			A known Shapely geometry type.
		o2 : `Shapely.AnyObj`
			A known Shapely geometry type.

		Returns
		-------
		bool
			The result of the test.
		"""
		if (not o1.is_valid) or (not o2.is_valid): return False
		if o1.is_empty or o2.is_empty: return False
		try:
			o1 = Shapely.set_precision(o1, GeometryLib.EPSILON)
			o2 = Shapely.set_precision(o2, GeometryLib.EPSILON)
			return o1.intersects(o2)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.intersects.__name__, e, [o1, o2])
			return False

	@staticmethod
	def intersection(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> Shapely.AnyObj:
		if (not o1.is_valid) or (not o2.is_valid):
			return type(o1)()
		if o1.is_empty or o2.is_empty:
			return type(o1)()
		try:
			o1 = Shapely.set_precision(o1, GeometryLib.EPSILON)
			o2 = Shapely.set_precision(o2, GeometryLib.EPSILON)
			return o1.intersection(o2, grid_size=GeometryLib.EPSILON)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.intersection.__name__, e, [o1, o2])
			return type(o1)()

	@staticmethod
	def hausdorff(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> float:
		if (not o1.is_valid) or (not o2.is_valid):
			return inf
		if o1.is_empty or o2.is_empty:
			return inf
		try:
			o1 = Shapely.set_precision(o1, GeometryLib.EPSILON)
			o2 = Shapely.set_precision(o2, GeometryLib.EPSILON)
			return o1.hausdorff_distance(o2)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.distance.__name__, e, [o1, o2])
			return inf

	@staticmethod
	def distance(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> float:
		if (not o1.is_valid) or (not o2.is_valid):
			return inf
		if o1.is_empty or o2.is_empty:
			return inf
		try:
			o1 = Shapely.set_precision(o1, GeometryLib.EPSILON)
			o2 = Shapely.set_precision(o2, GeometryLib.EPSILON)
			return o1.distance(o2)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.distance.__name__, e, [o1, o2])
			return inf

	@staticmethod
	def union(polys: Sequence[Shapely.Polygon]) -> Shapely.AnyObj:
		try:
			polys = [Shapely.make_valid(Shapely.set_precision(p, GeometryLib.EPSILON)) for p in polys]
			polys = [p for p in polys if (not p.is_empty) and p.area > 0]
			return Shapely.union_all(polys, grid_size=GeometryLib.EPSILON)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.union.__name__, e, polys)
			return type(polys[0])()

	@staticmethod
	def __difference(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> list[Shapely.Polygon]:
		objs2 = GeometryLib.toGeometryList(o2)
		objs2 = [Shapely.make_valid(Shapely.set_precision(p, GeometryLib.EPSILON)) for p in objs2]
		objs2 = [p for p in objs2 if (not p.is_empty) and p.area > 0]
		for o2 in objs2:
			o1 = o1.difference(o2, grid_size=GeometryLib.EPSILON)
		diff = GeometryLib.toGeometryList(o1)
		diff = [Shapely.make_valid(Shapely.set_precision(p, GeometryLib.EPSILON)) for p in diff]
		diff = [p for p in diff if (not p.is_empty) and p.area > 0]
		return diff

	@staticmethod
	def difference(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> list[Shapely.Polygon]:
		"""Returns the parts of `obj1` that does not intersect with `obj2`."""
		try:
			objs2 = GeometryLib.filterPolygons(o2)
			for o2 in objs2:
				o1 = o1.difference(o2, grid_size=GeometryLib.EPSILON)
			diff = GeometryLib.filterPolygons(o1)
			return diff
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.difference.__name__, e, [o1, o2])
			return [type(o1)()]

	@staticmethod
	def __haveOverlappingEdge(p1: Shapely.Polygon, p2: Shapely.Polygon) -> bool:
		"""
		FIXME: https://github.com/shapely/shapely/issues/1101#issuecomment-1336198843
		"""
		r = GeometryLib.intersection(p1, p2)
		if isinstance(r, Shapely.LineString) or isinstance(r, Shapely.MultiLineString):
			return True if r.is_valid and r.length > 0 else False
		return False

	@staticmethod
	def haveOverlappingEdge(p1: Shapely.Polygon, p2: Shapely.Polygon) -> bool:
		"""
		Previous implementation used intersection operator. But it turn out to be VERY buggy (see __haveOverlappingEdge)
		This one will iterate over all of the boundary edges check if lines are parallel and sees if the distance is minimal.
		"""
		p1 = Shapely.set_precision(p1, GeometryLib.EPSILON)
		p2 = Shapely.set_precision(p2, GeometryLib.EPSILON)
		shapelyIntersectionCheck = GeometryLib.__haveOverlappingEdge(p1, p2)
		if shapelyIntersectionCheck: return True
		# if GeometryLib.distance(p1, p2) > GeometryLib.EPSILON: return False # This is an important optimization. The process below is time consuming
		lineSegments1 = []
		lineSegments2 = []
		for p1Rings in Shapely.get_rings(p1):
			lineSegments1 += list(map(Shapely.LineString, zip(p1Rings.coords[:-1], p1Rings.coords[1:])))
		for p2Rings in Shapely.get_rings(p2):
			lineSegments2 += list(map(Shapely.LineString, zip(p2Rings.coords[:-1], p2Rings.coords[1:])))
		for lineSeg1 in lineSegments1:
			# lineSeg1 = Shapely.set_precision(lineSeg1, GeometryLib.EPSILON)
			if lineSeg1.length < GeometryLib.EPSILON: continue
			slope1 = GeometryLib.lineSegmentSlope(lineSeg1)
			for lineSeg2 in lineSegments2:
				# lineSeg1 = Shapely.set_precision(lineSeg2, GeometryLib.EPSILON)
				if lineSeg2.length < GeometryLib.EPSILON: continue
				slope2 = GeometryLib.lineSegmentSlope(lineSeg2)
				if abs(slope1 - slope2) > GeometryLib.EPSILON: continue
				d = GeometryLib.distance(lineSeg1, lineSeg2)
				if d < GeometryLib.EPSILON:
					return True
		return False
	# endregion: Shapely wrappers

	# region: SciKit Matrix operations
	@staticmethod
	def getCenterOfRotation(transformation: AffineTransform) -> Coords:
		"""
		Estimate the center of rotation.

		Parameters
		----------
		transformation : `AffineTransform`
			The transformation.

		Returns
		-------
		`Coords`
			The center of rotation if there exists one, `(nan, nan)` otherwise.
		"""
		matrix = transformation.params # pyright: ignore[reportAttributeAccessIssue]
		# [[a0  a1  a2]
		#  [b0  b1  b2]
		#  [0   0    1]]
		# If (almost) identity transformation, then there is no center of rotation.
		if GeometryLib.isIdentityTransform(transformation): return (nan, nan)
		(a0, a1, a2) = (matrix[0][0], matrix[0][1], matrix[0][2])
		(b0, b1, b2) = (matrix[1][0], matrix[1][1], matrix[1][2])

		x: float = (a2 + ((a1 * b2) / (1 - b1))) / (1 - a0 - ((a1 * b0) / (1 - b1)))
		y: float = (b0 * x + b2) / (1 - b1)
		return (x, y)

	@staticmethod
	def isIdentityTransform(transformation: AffineTransform) -> bool:
		"""### Is Identity Transformation
		Test whether the given transformation is (almost) identity transformation.


		Parameters
		----------
		transformation : AffineTransform

		Returns
		-------
		bool
			True if the sum of element-wise diff between the matrix and identity matrix is almost zero.
		"""
		matrix = transformation.params # pyright: ignore[reportAttributeAccessIssue]
		if ((matrix - np.identity(3)).sum()) < GeometryLib.EPSILON: return True
		return False

	@staticmethod
	def getAffineTransformation(start: CoordsList | None, end: CoordsList | None) -> AffineTransform:
		"""
		Estimate the affine transformation matrix that would transform the given polygon from the start state to end state.
		This method assumes that the vertices are ordered (labeled).

		Parameters
		----------
		start : Shapely.Polygon
			The starting configuration of the polygon.
		end : Shapely.Polygon
			The starting configuration of the polygon.
		centerOfRotation : Pose
			The center of rotation of the rotation motion.

		Returns
		-------
		AffineTransform
			An affine transformation matrix object.
		"""
		matrix = AffineTransform()
		if start is None or end is None: return matrix # identity transform

		startCoords = np.array(start)
		endCoords = np.array(end)
		weights = np.ones(len(start))
		try:
			if matrix.estimate(startCoords, endCoords, weights=weights): return matrix
			raise RuntimeError("SciKit failed to estimate a transform.")
		except Exception as e:
			GeometryLib.__reportSkImageException(GeometryLib.getAffineTransformation.__name__, e, [start, end])
			raise e

	@staticmethod
	def getParameterizedAffineTransformation(transformation: AffineTransform, param: float) -> AffineTransform:
		"""

			Given an affine transformation and a parameter between 0 and 1, this method returns a linear interpolation transformation.

			### Remarks
			This method returns an affine transformation that provides a linear interpolation of the given transformation, on the following assumptions:
			* `param` is in [0, 1] interval,
			* The affine transformation at `param == 0` is Identity Matrix,
			* The affine transformation at `param == 1` is the given transformation,
			* A slerp method is used to obtain the rotation interpolation.
		"""
		if param > 1 or param < 0: raise ValueError("Parameter should be in range [0, 1]. Given param = %f" % param)
		# Easy cases that do not need calculation
		if param == 0: return AffineTransform()
		if param == 1: return AffineTransform(transformation.params) # pyright: ignore[reportAttributeAccessIssue]
		scale = [((transformation.scale[0] - 1) * param) + 1, ((transformation.scale[1] - 1) * param) + 1]
		rotations = Rotation.from_matrix((
			[
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1]
			],
			[
				[transformation.params[0][0], transformation.params[0][1], 0], # pyright: ignore[reportAttributeAccessIssue]
				[transformation.params[1][0], transformation.params[1][1], 0], # pyright: ignore[reportAttributeAccessIssue]
				[0, 0, 1]
			]))
		slerp = Slerp([0, 1], rotations)
		# https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html#scipy-spatial-transform-rotation-as-euler
		# Any orientation can be expressed as a composition of 3 elementary rotations.
		# Once the axis sequence has been chosen, Euler angles define the angle of rotation around each respective axis
		rotation = slerp([0, param, 1])[1].as_euler("xyz")[2]
		shear = transformation.shear * param
		# shear = 0
		translation = [transformation.translation[0] * param, transformation.translation[1] * param]
		# translation = [0, 0]
		parameterizedMatrix = AffineTransform(matrix=None, scale=scale, rotation=rotation, shear=shear, translation=translation)
		return parameterizedMatrix

	@staticmethod
	def applyMatrixTransformToCoordsList(transformation: AffineTransform, coordsList: CoordsList) -> CoordsList:
		transformedCoords = matrix_transform(coordsList, transformation.params) # pyright: ignore[reportAttributeAccessIssue]
		return transformedCoords

	@staticmethod
	def applyMatrixTransformToCoords(transformation: AffineTransform, pose: Coords) -> Coords:
		transformedCoords = GeometryLib.applyMatrixTransformToCoordsList(transformation, [(pose[0], pose[1])])
		return transformedCoords[0]

	@staticmethod
	def applyMatrixTransformToPose(transformation: AffineTransform, pose: Pose) -> Pose:
		transformedCoords = GeometryLib.applyMatrixTransformToCoordsList(transformation, [(pose.x, pose.y)])
		transformedPose = Pose(0, transformedCoords[0][0], transformedCoords[0][1], 0)
		return transformedPose

	@staticmethod
	def applyMatrixTransformToPolygon(transformation: AffineTransform, polygon: Shapely.Polygon) -> Shapely.Polygon:
		pCoords = GeometryLib.getGeometryCoords(polygon)
		transformedCoords = GeometryLib.applyMatrixTransformToCoordsList(transformation, pCoords)
		transformedPolygon = Shapely.Polygon(transformedCoords)
		transformedPolygon = Shapely.set_precision(transformedPolygon, GeometryLib.EPSILON)
		return transformedPolygon

	@staticmethod
	def applyMatrixTransformToLineString(transformation: AffineTransform, line: Shapely.LineString) -> Shapely.LineString:
		pCoords = GeometryLib.getGeometryCoords(line)
		transformedCoords = GeometryLib.applyMatrixTransformToCoordsList(transformation, pCoords)
		transformedLineString = Shapely.LineString(transformedCoords)
		transformedLineString = Shapely.set_precision(transformedLineString, GeometryLib.EPSILON)
		return transformedLineString

	@staticmethod
	def inverseTransformation(transformation: AffineTransform) -> AffineTransform:
		inverted = AffineTransform(matrix=transformation._inv_matrix)
		return inverted

	@staticmethod
	def expandVertObbWithAngularVelocity(coords: Coords, angle: float, centerOfRotation: Coords, expandAway = True) -> Coords:
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	@staticmethod
	def getLineSegmentExpandedBb(transformation: AffineTransform, lineSeg: Shapely.LineString, centerOfRotation: Coords) -> Shapely.Polygon:
		""" Gets a tight bounding box for a line segment that is moving with a constant angular velocity. """
		angle: float = abs(transformation.rotation)
		originalCoords = GeometryLib.getGeometryCoords(lineSeg)
		if len(originalCoords) > 2: raise ValueError(f"A line segment must have two vertices. Input: {repr(lineSeg)}")
		if GeometryLib.isIdentityTransform(transformation): return Shapely.buffer(lineSeg, GeometryLib.EPSILON)

		finalConfig = GeometryLib.applyMatrixTransformToLineString(transformation, lineSeg)
		finalCoords = GeometryLib.getGeometryCoords(finalConfig)
		verts: GeometryLib.CoordsList = []
		for j in range(16):
			v1 = GeometryLib.expandVertObbWithAngularVelocity(originalCoords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = GeometryLib.expandVertObbWithAngularVelocity(finalCoords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = GeometryLib.expandVertObbWithAngularVelocity(finalCoords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = GeometryLib.expandVertObbWithAngularVelocity(originalCoords[1], angle, centerOfRotation, j & 8 != 0)
			verts += [v1, v2, v3, v4]
		pts = Shapely.MultiPoint(verts)
		expandedObb = Shapely.set_precision(Shapely.convex_hull(pts), GeometryLib.EPSILON)
		if isinstance(expandedObb, Shapely.Polygon): return expandedObb
		if isinstance(expandedObb, Shapely.LineString): return Shapely.buffer(expandedObb, GeometryLib.EPSILON)
		raise ValueError(f"Expanded OBB is neither a polygon nor a line segment: {repr(expandedObb)}")
	# endregion: SciKit Matrix operations

warnings.filterwarnings("error") # Turn shapely C++ errors into exceptions for better debugging.
# FIXME: https://docs.python.org/3/library/warnings.html#warning-filter, add regex for shapely or add module.
