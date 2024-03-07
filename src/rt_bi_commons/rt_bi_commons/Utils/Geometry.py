import traceback
import warnings
from math import cos, inf, nan, sin, sqrt
from typing import Callable, Final, Sequence, TypeAlias

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from skimage.transform import AffineTransform as AffineTransform, matrix_transform

from rt_bi_commons.Shared.Pose import Coords, CoordsList, Pose, Quaternion, angleToQuat, quatToAngle
from rt_bi_commons.Utils.Ros import Logger


class Shapely:
	"""This class sets up a group of functions and type aliases that help use shapely objects easier."""
	from shapely.constructive import convex_hull
	from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point
	from shapely.geometry.multipoint import MultiPoint
	from shapely.geometry.polygon import LinearRing, Polygon
	from shapely.ops import unary_union
	from shapely.validation import make_valid

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
		if len(coordsList) > 0 and len(coordsList) < 4:
			return "%s" % repr(coordsList)
		return "[#%d, v:%s, e:%s]" % (len(coordsList), "Y" if self.is_valid else "N", "Y" if self.is_empty else "N")

	@staticmethod
	def __multiGeometryRepr(self: MultiComponent) -> str: # pyright: ignore[reportSelfClsParameterName]
		return "{%s}" % ", ".join([repr(o) for o in self.geoms])

	Point.__neg__ = __pointNeg
	Point.__add__ = __pointAdd
	Point.__sub__ = __pointSub
	Point.__mul__ = __pointMulScalar
	Point.__truediv__ = __pointDivScalar

	Point.__repr__ = lambda self: f"{repr((self.x, self.y))}"
	LinearRing.__repr__ = lambda self: f"R{Shapely.__connectedGeometryRepr(self)}"
	LineString.__repr__ = lambda self: f"L{Shapely.__connectedGeometryRepr(self)}"
	Polygon.__repr__ = lambda self: f"P{Shapely.__connectedGeometryRepr(self)}"

	MultiPoint.__repr__ = __multiGeometryRepr
	MultiLineString.__repr__ = __multiGeometryRepr
	MultiPolygon.__repr__ = __multiGeometryRepr


class GeometryLib:
	"""
	Custom Computational Geometry implementation.
	© Reza Teshnizi 2018-2023
	TODO: Make sure to use shapely.prepared.prep() to optimize
	"""
	EPSILON: Final = 0.001
	Vector = Coords
	Coords = Coords
	CoordsList = CoordsList
	Quaternion = Quaternion
	CoordsMap = dict[Coords, Coords]
	angleToQuat: Callable[[float], Quaternion] = angleToQuat
	quatToAngle: Callable[[Quaternion | Sequence[float]], float] = quatToAngle
	@staticmethod
	def __reportShapelyException(functionName: str, exc: Exception, objs: Sequence[Shapely.AnyObj]) -> None:
		Logger().error("1. Shapely exception.. in %s(): %s" % (functionName, repr(exc)))
		Logger().error("2. Shapely exception.. args: %s" % (", ".join([repr(o) for o in objs])))
		i = 3
		for o in objs:
			if hasattr(o, "geoms"):
				verts = [GeometryLib.getGeometryCoords(g) for g in o.geoms]
			else:
				verts = GeometryLib.getGeometryCoords(o)
			Logger().error("%d. Shapely exception.. Verts of %s: %s" % (i, repr(o), repr(verts)))
			i += 1
		Logger().info("%d. Shapely exception.. Stack trace:\n%s" % (i, traceback.format_exc()))
		return

	@staticmethod
	def __reportSkImageException(functionName: str, exc: Exception, objs: Sequence[CoordsList]) -> None:
		Logger().error("1. SkImage exception.. in %s(): %s" % (functionName, repr(exc)))
		Logger().error("2. SkImage exception.. args: %s" % (", ".join([repr(o) for o in objs])))
		i = 3
		for o in objs:
			Logger().error("%d. SkImage exception.. Verts: %s" % (i, repr(o)))
			i += 1
		Logger().info("%d. SkImage exception.. Stack trace:\n%s" % (i, traceback.format_exc()))
		return

	@staticmethod
	def toCoords(pt: Shapely.Point) -> Coords:
		return (pt.x, pt.y)

	@staticmethod
	def toPose(pt: Shapely.Point, timeNanoSecs: int = 0, angle: float = 0.0) -> Pose:
		return Pose(timeNanoSecs, pt.x, pt.y, float(angle))

	@staticmethod
	def addCoords(c1: Coords, c2: Coords) -> Coords:
		return(c1[0] + c2[0], c1[1] + c2[1])

	@staticmethod
	def subtractCoords(c1: Coords, c2: Coords) -> Coords:
		"""c1 - c2

		Parameters
		----------
		c1 : Coords
		c2 : Coords

		Returns
		-------
		Coords
		"""
		return(c1[0] - c2[0], c1[1] - c2[1])

	@staticmethod
	def addScalarToCoords(c1: Coords, s: float) -> Coords:
		"""c1 + (s * I)

		Parameters
		----------
		c1 : Coords
		s : float

		Returns
		-------
		Coords
		"""
		return(c1[0] + s, c1[1] + s)

	@staticmethod
	def scaleCoords(c1: Coords, s: float) -> Coords:
		return(c1[0] * s, c1[1] * s)

	@staticmethod
	def getGeometryCoords(geom: Shapely.ConnectedComponent) -> CoordsList:
		if not hasattr(geom, "exterior"):
			if not hasattr(geom, "coords"):
				Logger().error("Unknown geometry %s." % repr(geom))
				return []
			return list(geom.coords)
		return list(geom.exterior.coords)

	@staticmethod
	def pointStringId(x: float, y: float) -> str:
		return "%.5f,%.5f" % (x, y)

	@staticmethod
	def lineStringId(line: Shapely.LineString) -> tuple[str, ...]:
		coordsList = GeometryLib.getGeometryCoords(line)
		return tuple(GeometryLib.pointStringId(coords[0], coords[1]) for coords in coordsList)

	@staticmethod
	def coordsDistance(coords1: Coords, coords2: Coords) -> float:
		return GeometryLib.distance(coords1[0], coords1[1], coords2[0], coords2[1])

	@staticmethod
	def vectorsAreEqual(vec1: Vector, vec2: Vector, withinEpsilon = True) -> bool:
		d1 = GeometryLib.coordsDistance(vec1, vec2)
		margin = GeometryLib.EPSILON if withinEpsilon else 0
		return d1 <= margin

	@staticmethod
	def coordsAreEqual(coords1: Coords, coords2: Coords) -> bool:
		return GeometryLib.vectorsAreEqual(coords1, coords2, False)

	@staticmethod
	def coordsAreAlmostEqual(coords1: Coords, coords2: Coords) -> bool:
		d1 = GeometryLib.coordsDistance(coords1, coords2)
		return d1 <= 0.35 # FIXME: EPSILON is too small unfortunately

	@staticmethod
	def lineSegmentsAreAlmostEqual(l1: Shapely.LineString, l2: Shapely.LineString) -> bool:
		l1Coords = GeometryLib.getGeometryCoords(l1)
		l2Coords = GeometryLib.getGeometryCoords(l2)
		if GeometryLib.coordsAreAlmostEqual(l1Coords[0], l2Coords[0]) and GeometryLib.coordsAreAlmostEqual(l1Coords[1], l2Coords[1]): return True
		if GeometryLib.coordsAreAlmostEqual(l1Coords[0], l2Coords[1]) and GeometryLib.coordsAreAlmostEqual(l1Coords[1], l2Coords[0]): return True
		return False

	@staticmethod
	def vectLength(x: float, y: float) -> float:
		return sqrt((y * y) + (x * x))

	@staticmethod
	def isLineSegment(l: Shapely.LineString) -> bool:
		return isinstance(l, Shapely.LineString) and len(l.coords) == 2

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
	def distance(x1: float, y1: float, x2: float, y2: float) -> float:
		dx = x2 - x1
		dy = y2 - y1
		length = GeometryLib.vectLength(dx, dy)
		return length

	@staticmethod
	def getUnitVector(x: float, y: float) -> Vector:
		"""
		Returns a tuple (x, y) of a vector.
		"""
		distance = GeometryLib.vectLength(x, y)
		return (x / distance, y / distance)

	@staticmethod
	def getUnitVectorFromAngle(theta: float) -> Vector:
		"""
		theta is the angle w.r.t X axis
		Returns a tuple (x, y) of a vector.
		"""
		x = cos(theta)
		y = sin(theta)
		return GeometryLib.getUnitVector(x, y)

	@staticmethod
	def pushPointEpsilon(x: float, y: float, pt: Shapely.Point) -> Shapely.Point:
		"""
		Push point epsilon in the direction of the given x, y.
		"""
		(x, y) = GeometryLib.getUnitVector(x, y)
		y = y * GeometryLib.EPSILON
		x = x * GeometryLib.EPSILON
		return Shapely.Point(pt.x + x, pt.y + y)

	@staticmethod
	def getDirectionXyFromLineSeg(l: Shapely.LineString) -> Vector:
		"""
		Returns (x, y) tuple
		"""
		if not GeometryLib.isLineSegment(l):
			raise RuntimeError("This method is only tested for line segments")
		coords = GeometryLib.getGeometryCoords(l)
		x = coords[1][0] - coords[0][0]
		y = coords[1][1] - coords[0][1]
		return (x, y)

	@staticmethod
	def isPointOnLine(p: Shapely.Point, l: Shapely.LineString) -> bool:
		return l.distance(p) <= GeometryLib.EPSILON

	@staticmethod
	def convexHull(coordsList: CoordsList) -> Shapely.Polygon:
		pts = Shapely.MultiPoint(coordsList)
		try:
			return Shapely.convex_hull(pts)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.intersects.__name__, e, [pts])
			return Shapely.Polygon()

	@staticmethod
	def toGeometryList(polys: Shapely.MultiComponent) -> list[Shapely.ConnectedComponent]:
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
	def isXyInsidePolygon(ptX: float, ptY: float, polygon: Shapely.Polygon) -> bool:
		return GeometryLib.isPointInsidePolygon(Shapely.Point(ptX, ptY), polygon)

	@staticmethod
	def isPointInsidePolygon(pt: Shapely.Point, polygon: Shapely.Polygon) -> bool:
		# FIXME: Because of numerical errors, we can rely on contains
		# So if the point is contained and the distance from exterior is minimal
		# return polygon.contains(pt) and polygon.boundary.distance(pt) > Geometry.EPSILON
		return polygon.contains(pt)

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
			return o1.intersects(o2)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.intersects.__name__, e, [o1, o2])
			return False

	@staticmethod
	def isEmptyObjList(objs: Sequence[Shapely.AnyObj]) -> bool:
		return all(obj.is_empty for obj in objs)

	@staticmethod
	def nonEmptyPolys(objs: Sequence[Shapely.AnyObj]) -> list[Shapely.Polygon]:
		return [Shapely.make_valid(p) for p in objs if p.is_valid and not p.is_empty and p.area > 0]

	@staticmethod
	def intersection(o1: Shapely.AnyObj, o2: Shapely.AnyObj) -> Sequence[Shapely.AnyObj]:
		if (not o1.is_valid) or (not o2.is_valid): return type(o1)()
		if o1.is_empty or o2.is_empty: return type(o1)()
		try:
			if GeometryLib.intersects(o1, o2):
				intersection = o1.intersection(o2)
				intersection = GeometryLib.toGeometryList(intersection)
				return intersection
			else:
				return [type(o1)()]
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.intersection.__name__, e, [o1, o2])
			return [type(o1)()]

	@staticmethod
	def union(polys: Sequence[Shapely.Polygon]) -> Sequence[Shapely.Polygon]:
		polys = GeometryLib.nonEmptyPolys(polys)
		try:
			result = Shapely.unary_union(polys)
			return GeometryLib.toGeometryList(result)
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.union.__name__, e, polys)
			return type(polys[0])()

	@staticmethod
	def difference(obj1: Shapely.Polygon | Sequence[Shapely.Polygon], obj2: Shapely.Polygon | Sequence[Shapely.Polygon]) -> list[Shapely.Polygon]:
		"""Returns the parts of `obj1` that does not intersect with `obj2`."""
		from shapely.geometry import GeometryCollection
		try:
			if isinstance(obj1, Shapely.Polygon): p1 = obj1
			else: p1 = GeometryCollection(geoms=obj1)
			if isinstance(obj2, Shapely.Polygon): p2 = obj2
			else: p2 = GeometryCollection(geoms=obj2)

			if GeometryLib.intersects(p1, p2):
				diff = p1.difference(p2)
				diff = GeometryLib.toGeometryList(diff)
				diff = GeometryLib.nonEmptyPolys(diff)
				return diff
			else:
				return [p1]
		except Exception as e:
			GeometryLib.__reportShapelyException(GeometryLib.difference.__name__, e, [p1, p2])
			return [type(p1)()]

	@staticmethod
	def __haveOverlappingEdge(p1: Shapely.Polygon, p2: Shapely.Polygon) -> bool:
		"""
		DEPRECATED: https://github.com/Toblerity/Shapely/issues/1101
		"""
		# if p1.touches(p2):
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
		shapelyIntersectionCheck = GeometryLib.__haveOverlappingEdge(p1, p2)
		if shapelyIntersectionCheck: return True
		if p1.distance(p2) > GeometryLib.EPSILON: return False # This is an important optimization. The process below is time consuming
		lineSegments1 = list(map(Shapely.LineString, zip(p1.exterior.coords[:-1], p1.exterior.coords[1:])))
		lineSegments2 = list(map(Shapely.LineString, zip(p2.exterior.coords[:-1], p2.exterior.coords[1:])))
		for lineSeg1 in lineSegments1:
			slope1 = GeometryLib.lineSegmentSlope(lineSeg1)
			for lineSeg2 in lineSegments2:
				slope2 = GeometryLib.lineSegmentSlope(lineSeg2)
				if abs(slope1 - slope2) > GeometryLib.EPSILON: continue
				d = lineSeg1.distance(lineSeg2)
				if d < GeometryLib.EPSILON:
					return True
		return False

	@staticmethod
	def getIntersectingEdges(geom: Shapely.LineString | Shapely.Polygon, polygon: Shapely.Polygon | Shapely.MultiPolygon) -> Sequence[Shapely.LineString]:
		if isinstance(polygon, Shapely.MultiPolygon): raise NotImplementedError("I haven't checked the API to see how to work with this yet. %s" % GeometryLib.getIntersectingEdges.__name__)
		edges = []
		hashes = set()

		verts = list(polygon.exterior.coords)
		geom = geom if isinstance(geom, Shapely.LineString) else geom.exterior
		points = GeometryLib.intersection(polygon.exterior, geom)
		isEmpty = True
		for pt in points:
			if not pt.is_empty:
				isEmpty = False
				break
		if isEmpty: return edges
		for point in points:
			for v1, v2 in zip(verts, verts[1:]):
				edge = Shapely.LineString([v1, v2])
				if edge.wkb_hex in hashes: continue
				if GeometryLib.isPointOnLine(point, edge):
					edges.append(edge)
					hashes.add(edge.wkb_hex)
					break
		return edges

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
	def getAffineTransformation(start: CoordsList, end: CoordsList) -> AffineTransform:
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
		startCoords = np.array(start)
		endCoords = np.array(end)
		weights = np.ones(len(start))
		matrix = AffineTransform()
		try:
			if matrix.estimate(startCoords, endCoords, weights=weights): return matrix
			raise RuntimeError("SciKit failed to estimate a transform")
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
		if param == 0: return AffineTransform(np.identity(3))
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
		return transformedPolygon

	@staticmethod
	def applyMatrixTransformToLineString(transformation: AffineTransform, line: Shapely.LineString) -> Shapely.LineString:
		pCoords = GeometryLib.getGeometryCoords(line)
		transformedCoords = GeometryLib.applyMatrixTransformToCoordsList(transformation, pCoords)
		transformedLineString = Shapely.LineString(transformedCoords)
		return transformedLineString

	@staticmethod
	def inverseTransformation(transformation: AffineTransform) -> AffineTransform:
		inverted  = AffineTransform(matrix=transformation._inv_matrix)
		return inverted

	@staticmethod
	def expandVertObbWithAngularVelocity(coords: Coords, angle: float, centerOfRotation: Coords, expandAway = True) -> Coords:
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	@staticmethod
	def getLineSegmentExpandedBb(transformation: AffineTransform, lineSeg: Shapely.LineString, centerOfRotation: Coords) -> Shapely.Polygon | Shapely.LineString:
		""" Gets a tight bounding box for a line segment that is moving with a constant angular velocity. """
		angle: float = abs(transformation.rotation)
		originalCoords = GeometryLib.getGeometryCoords(lineSeg)
		if len(originalCoords) > 2: raise ValueError(f"A line segment must have two vertices. Input: {repr(lineSeg)}")
		if GeometryLib.isIdentityTransform(transformation):
			return lineSeg
		finalConfig = GeometryLib.applyMatrixTransformToLineString(transformation, lineSeg)
		finalCoords = GeometryLib.getGeometryCoords(finalConfig)
		verts: GeometryLib.CoordsList = []
		for j in range(16):
			v1 = GeometryLib.expandVertObbWithAngularVelocity(originalCoords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = GeometryLib.expandVertObbWithAngularVelocity(finalCoords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = GeometryLib.expandVertObbWithAngularVelocity(finalCoords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = GeometryLib.expandVertObbWithAngularVelocity(originalCoords[1], angle, centerOfRotation, j & 8 != 0)
			verts += [v1, v2, v3, v4]
		expandedObb = GeometryLib.convexHull(verts)
		if isinstance(expandedObb, Shapely.Polygon): return expandedObb
		raise ValueError(f"Expanded OBB is not a polygon: {repr(expandedObb)}")

warnings.filterwarnings("error") # Turn shapely C++ errors into exceptions for better debugging.
