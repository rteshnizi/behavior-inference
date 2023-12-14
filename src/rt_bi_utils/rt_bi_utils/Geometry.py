import traceback
import warnings
from copy import copy
from math import cos, inf, isnan, nan, sin, sqrt
from typing import Dict, List, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from shapely.constructive import convex_hull
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point
from shapely.geometry.multipoint import MultiPoint
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid
from skimage.transform import AffineTransform as AffineTransform, matrix_transform

from rt_bi_utils.Pose import Pose
from rt_bi_utils.Ros import Logger


class Geometry:
	"""
	Custom Computational Geometry implementation.
	© Reza Teshnizi 2018-2023
	TODO: Make sure to use shapely.prepared.prep() to optimize
	"""
	EPSILON = 0.001
	Vector = Tuple[float, float]
	Coords = Tuple[float, float]
	CoordsList = List[Coords]
	CoordsMap = Dict[Coords, Coords]
	ConnectedGeometry = Union[Polygon, LineString, Point]
	MultiGeometry = Union[MultiPolygon, MultiLineString, MultiPoint]
	Geometry = Union[Polygon, MultiPolygon, LineString, MultiLineString, Point, MultiPoint]

	@staticmethod
	def __reportShapelyException(functionName: str, exc: Exception, objs: List[Geometry]) -> None:
		Logger().error("1. Shapely exception.. in %s(): %s" % (functionName, repr(exc)))
		Logger().error("2. Shapely exception.. args: %s" % (", ".join([repr(o) for o in objs])))
		i = 3
		for o in objs:
			if hasattr(o, "geoms"):
				verts = [Geometry.getGeometryCoords(g) for g in o.geoms]
			else:
				verts = Geometry.getGeometryCoords(o)
			Logger().error("%d. Shapely exception.. Verts of %s: %s" % (i, repr(o), repr(verts)))
			i += 1
		Logger().info("%d. Shapely exception.. Stack trace:\n%s" % (i, traceback.format_exc()))
		return

	@staticmethod
	def toPose(pt: Point, timeNanoSecs: int = 0, angle: float = 0.0) -> Pose:
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
	def getGeometryCoords(geom: ConnectedGeometry) -> CoordsList:
		if not hasattr(geom, "exterior"):
			if not hasattr(geom, "coords"):
				Logger().error("Unknown geometry %s." % repr(geom))
				return []
			return list(geom.coords) # type: ignore

		return list(geom.exterior.coords)

	@staticmethod
	def pointStringId(x: float, y: float) -> str:
		return "%.5f,%.5f" % (x, y)

	@staticmethod
	def lineStringId(line: LineString) -> Tuple[str, ...]:
		coordsList = Geometry.getGeometryCoords(line)
		return tuple(Geometry.pointStringId(coords[0], coords[1]) for coords in coordsList)

	@staticmethod
	def coordsDistance(coords1: Coords, coords2: Coords) -> float:
		return Geometry.distance(coords1[0], coords1[1], coords2[0], coords2[1])

	@staticmethod
	def vectorsAreEqual(vec1: Vector, vec2: Vector, withinEpsilon = True) -> bool:
		d1 = Geometry.coordsDistance(vec1, vec2)
		margin = Geometry.EPSILON if withinEpsilon else 0
		return d1 <= margin

	@staticmethod
	def coordsAreEqual(coords1: Coords, coords2: Coords) -> bool:
		return Geometry.vectorsAreEqual(coords1, coords2)

	@staticmethod
	def coordsAreAlmostEqual(coords1: Coords, coords2: Coords) -> bool:
		d1 = Geometry.coordsDistance(coords1, coords2)
		return d1 <= 0.35 # FIXME: EPSILON is too small unfortunately

	@staticmethod
	def lineSegmentsAreAlmostEqual(l1: LineString, l2: LineString) -> bool:
		l1Coords = Geometry.getGeometryCoords(l1)
		l2Coords = Geometry.getGeometryCoords(l2)
		if Geometry.coordsAreAlmostEqual(l1Coords[0], l2Coords[0]) and Geometry.coordsAreAlmostEqual(l1Coords[1], l2Coords[1]): return True
		if Geometry.coordsAreAlmostEqual(l1Coords[0], l2Coords[1]) and Geometry.coordsAreAlmostEqual(l1Coords[1], l2Coords[0]): return True
		return False

	@staticmethod
	def vectLength(x: float, y: float) -> float:
		return sqrt((y * y) + (x * x))

	@staticmethod
	def isLineSegment(l: LineString) -> bool:
		return isinstance(l, LineString) and len(l.coords) == 2

	@staticmethod
	def orthogonal(p1: Coords, p2: Coords) -> Coords:
		return (p2[1] - p1[1], -1 * (p2[0] - p1[0]))

	@staticmethod
	def midpoint(p1: Coords, p2: Coords) -> Coords:
		return Geometry.midpointXy(p1[0], p1[1], p2[0], p2[1])

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
		length = Geometry.vectLength(dx, dy)
		return length

	@staticmethod
	def getUnitVector(x: float, y: float) -> Vector:
		"""
		Returns a tuple (x, y) of a vector.
		"""
		distance = Geometry.vectLength(x, y)
		return (x / distance, y / distance)

	@staticmethod
	def getUnitVectorFromAngle(theta: float) -> Vector:
		"""
		theta is the angle w.r.t X axis
		Returns a tuple (x, y) of a vector.
		"""
		x = cos(theta)
		y = sin(theta)
		return Geometry.getUnitVector(x, y)

	@staticmethod
	def pushPointEpsilon(x: float, y: float, pt: Point) -> Point:
		"""
		Push point epsilon in the direction of the given x, y.
		"""
		(x, y) = Geometry.getUnitVector(x, y)
		y = y * Geometry.EPSILON
		x = x * Geometry.EPSILON
		return Point(pt.x + x, pt.y + y)

	@staticmethod
	def getDirectionXyFromLineSeg(l: LineString) -> Vector:
		"""
		Returns (x, y) tuple
		"""
		if not Geometry.isLineSegment(l):
			raise RuntimeError("This method is only tested for line segments")
		coords = Geometry.getGeometryCoords(l)
		x = coords[1][0] - coords[0][0]
		y = coords[1][1] - coords[0][1]
		return (x, y)

	@staticmethod
	def isPointOnLine(p: Point, l: LineString) -> bool:
		return l.distance(p) <= Geometry.EPSILON

	@staticmethod
	def convexHull(coordsList: CoordsList) -> Polygon:
		pts = MultiPoint(coordsList)
		try:
			return convex_hull(pts)
		except Exception as e:
			Geometry.__reportShapelyException(Geometry.intersects.__name__, e, [pts])
			return Polygon()


	@staticmethod
	def toGeometryList(polys: MultiGeometry) -> List[ConnectedGeometry]:
		try:
			if len(polys.geoms) > 0:
				return list(polys.geoms)
			raise RuntimeError("`geoms` should never be empty in a multi-geometry.")
		except:
			# Assumption here is that if it throws because Polygon does not have a `geoms` property.
			return [polys]

	@staticmethod
	def isXyInsidePolygon(ptX: float, ptY: float, polygon: Polygon) -> bool:
		return Geometry.isPointInsidePolygon(Point(ptX, ptY), polygon)

	@staticmethod
	def isPointInsidePolygon(pt: Point, polygon: Polygon) -> bool:
		# FIXME: Because of numerical errors, we can rely on contains
		# So if the point is contained and the distance from exterior is minimal
		# return polygon.contains(pt) and polygon.boundary.distance(pt) > Geometry.EPSILON
		return polygon.contains(pt)

	@staticmethod
	def lineSegmentSlope(segment: LineString) -> float:
		x11, y11 = segment.coords[0]
		x12, y12 = segment.coords[1]
		if x12 - x11 == 0: return inf
		slope = (y12 - y11) / (x12 - x11)
		return slope

	@staticmethod
	def intersects(o1: Geometry, o2: Geometry) -> bool:
		"""## Intersects
		A function to safely test if two geometries intersect while handling shapely exceptions and warnings.

		Parameters
		----------
		o1 : GeometricObject
			A known Shapely geometry type.
		o2 : GeometricObject
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
			Geometry.__reportShapelyException(Geometry.intersects.__name__, e, [o1, o2])
			return False

	@staticmethod
	def intersection(o1: Geometry, o2: Geometry) -> Geometry:
		"""## Intersects
		A function to safely test obtain the intersection of two geometries while handling shapely exceptions and warnings.

		Parameters
		----------
		o1 : GeometricObject
			A known Shapely geometry type.
		o2 : GeometricObject
			A known Shapely geometry type.

		Returns
		-------
		bool
			A known Shapely geometry type of lower dimension.
		"""
		if (not o1.is_valid) or (not o2.is_valid): return type(o1)()
		if o1.is_empty or o2.is_empty: return type(o1)()
		try:
			if Geometry.intersects(o1, o2):
				return o1.intersection(o2)
			else:
				return type(o1)()
		except Exception as e:
			Geometry.__reportShapelyException(Geometry.intersection.__name__, e, [o1, o2])
			return type(o1)()

	@staticmethod
	def union(polyList: List[Polygon]) -> Union[Polygon, MultiPolygon]:
		validPolyList = []
		for p in polyList:
			parts = make_valid(p)
			validPolyList.append(parts)
		try:
			return unary_union(validPolyList)
		except Exception as e:
			Geometry.__reportShapelyException(Geometry.union.__name__, e, validPolyList)
			return type(polyList[0])()

	@staticmethod
	def difference(p1: Polygon, p2: Polygon) -> List[Polygon]:
		"""
		Given p1 and p2, produce the subtraction polygons.

		Parameters
		----------
		p1 : `Polygon`
			The polygon representing a base polygon.
		p2 : `Polygon`
			The polygon to be subtracted.

		Returns
		-------
		`Union[List[Polygon], MultiPolygon]`
			The a polygon or multi-polygon of the subtraction result.
		"""
		try:
			if Geometry.intersects(p1, p2):
				subtraction = p1.difference(p2) # FIXME: difference() is wrong because it excludes the boundaries of fov from the shadows
				# subtraction = mapRegionPoly.difference(mapRegionPoly.intersection(fovPoly))
				# subtraction = mapRegionPoly.symmetric_difference(fovPoly).difference(fovPoly)
				return Geometry.toGeometryList(subtraction)
			else:
				return [p1]
		except Exception as e:
			Geometry.__reportShapelyException(Geometry.difference.__name__, e, [p1, p2])
			return [type(p1)()]

	@staticmethod
	def __haveOverlappingEdge(p1: Polygon, p2: Polygon) -> bool:
		"""
		DEPRECATED: https://github.com/Toblerity/Shapely/issues/1101
		"""
		# if p1.touches(p2):
		r = Geometry.intersection(p1, p2)
		if isinstance(r, LineString) or isinstance(r, MultiLineString):
			return True if r.is_valid and r.length > 0 else False
		return False

	@staticmethod
	def haveOverlappingEdge(p1: Polygon, p2: Polygon) -> bool:
		"""
		Previous implementation used intersection operator. But it turn out to be VERY buggy (see __haveOverlappingEdge)
		This one will iterate over all of the boundary edges check if lines are parallel and sees if the distance is minimal.
		"""
		shapelyIntersectionCheck = Geometry.__haveOverlappingEdge(p1, p2)
		if shapelyIntersectionCheck: return True
		if p1.distance(p2) > Geometry.EPSILON: return False # This is an important optimization. The process below is time consuming
		lineSegments1 = list(map(LineString, zip(p1.exterior.coords[:-1], p1.exterior.coords[1:])))
		lineSegments2 = list(map(LineString, zip(p2.exterior.coords[:-1], p2.exterior.coords[1:])))
		for lineSeg1 in lineSegments1:
			slope1 = Geometry.lineSegmentSlope(lineSeg1)
			for lineSeg2 in lineSegments2:
				slope2 = Geometry.lineSegmentSlope(lineSeg2)
				if abs(slope1 - slope2) > Geometry.EPSILON: continue
				d = lineSeg1.distance(lineSeg2)
				if d < Geometry.EPSILON:
					return True
		return False

	@staticmethod
	def getIntersectingEdges(geom: Union[LineString, Polygon], polygon: Union[Polygon, MultiPolygon]) -> List[LineString]:
		if isinstance(polygon, MultiPolygon): raise NotImplementedError("I haven't checked the API to see how to work with this yet. %s" % Geometry.getIntersectingEdges.__name__)
		edges = []
		hashes = set()

		verts = list(polygon.exterior.coords)
		geom = geom if isinstance(geom, LineString) else geom.exterior
		points = Geometry.intersection(polygon.exterior, geom)
		if points.is_empty: return edges
		points = Geometry.toGeometryList(points)
		for point in points:
			for v1, v2 in zip(verts, verts[1:]):
				edge = LineString([v1, v2])
				if edge.wkb_hex in hashes: continue
				if Geometry.isPointOnLine(point, edge):
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
		matrix = transformation.params
		# [[a0  a1  a2]
		#  [b0  b1  b2]
		#  [0   0    1]]
		# If (almost) identity transformation, then there is no center of rotation.
		if Geometry.isIdentityTransform(transformation): return (nan, nan)
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
		matrix = transformation.params
		if ((matrix - np.identity(3)).sum()) < Geometry.EPSILON: return True
		return False

	@staticmethod
	def getAffineTransformation(start: CoordsList, end: CoordsList) -> AffineTransform:
		"""
		Estimate the affine transformation matrix that would transform the given polygon from the start state to end state.
		This method assumes that the vertices are ordered (labeled).

		Parameters
		----------
		start : Polygon
			The starting configuration of the polygon.
		end : Polygon
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
		if matrix.estimate(startCoords, endCoords, weights=weights): return matrix
		raise RuntimeError("SciKit failed to estimate a transform")

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
		if param == 1: return AffineTransform(transformation.params)
		scale = [((transformation.scale[0] - 1) * param) + 1, ((transformation.scale[1] - 1) * param) + 1]
		rotations = Rotation.from_matrix((
			[
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1]
			],
			[
				[transformation.params[0][0], transformation.params[0][1], 0],
				[transformation.params[1][0], transformation.params[1][1], 0],
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
		transformedCoords = matrix_transform(coordsList, transformation.params)
		return transformedCoords

	@staticmethod
	def applyMatrixTransformToPose(transformation: AffineTransform, pose: Pose) -> Pose:
		transformedCoords = Geometry.applyMatrixTransformToCoordsList(transformation, [(pose.x, pose.y)])
		transformedPose = copy(pose)
		pose.x = transformedCoords[0][0]
		pose.y = transformedCoords[0][1]
		return transformedPose

	@staticmethod
	def applyMatrixTransformToPolygon(transformation: AffineTransform, polygon: Polygon) -> Polygon:
		pCoords = Geometry.getGeometryCoords(polygon)
		transformedCoords = Geometry.applyMatrixTransformToCoordsList(transformation, pCoords)
		transformedPolygon = Polygon(transformedCoords)
		return transformedPolygon

	@staticmethod
	def applyMatrixTransformToLineString(transformation: AffineTransform, line: LineString) -> LineString:
		pCoords = Geometry.getGeometryCoords(line)
		transformedCoords = Geometry.applyMatrixTransformToCoordsList(transformation, pCoords)
		transformedLineString = LineString(transformedCoords)
		return transformedLineString

	@staticmethod
	def inverseTransformation(transformation: AffineTransform) -> AffineTransform:
		inverted  = AffineTransform(matrix=transformation._inv_matrix)
		return inverted

	@staticmethod
	def findBottomLeft(poly: Polygon) -> Coords:
		(minX, minY, maxX, maxY) = poly.bounds
		return (minX, minY)

def __pointNeg(self: Point) -> Point:
	"""
		#### No need to import this class to your files.

		This class sets up a group of functions that help use shapely objects easier.

		`-1 * Point`
	"""
	return Point(-1 * self.x, -1 * self.y)

def __pointAdd(self: Point, other: Point) -> Point:
	"""
		#### No need to import this class to your files.

		This class sets up a group of functions that help use shapely objects easier.

		`Point + Point`
	"""
	return Point(self.x + other.x, self.y + other.y)

def __pointSub(self: Point, other: Point) -> Point:
	"""
		#### No need to import this class to your files.

		This class sets up a group of functions that help use shapely objects easier.

		`Point - Point`
	"""
	return Point(self.x - other.x, self.y - other.y)

def __pointDivScalar(self: Point, num: float) -> Point:
	"""
		#### No need to import this class to your files.

		This class sets up a group of functions that help use shapely objects easier.

		`Point / num`
	"""
	return Point(self.x / num, self.y / num)

def __pointMulScalar(self: Point, num: float) -> Point:
	"""
		#### No need to import this class to your files.

		This class sets up a group of functions that help use shapely objects easier.

		`Point * num`
	"""
	return Point(self.x * num, self.y * num)

def __connectedGeometryRepr(self: Geometry.ConnectedGeometry) -> str:
	coordsList = Geometry.getGeometryCoords(self)
	if len(coordsList) > 0 and len(coordsList) < 4:
		return "%s" % repr(coordsList)
	return "[#%d, v:%s, e:%s]" % (len(coordsList), "Y" if self.is_valid else "N", "Y" if self.is_empty else "N")

def __multiGeometryRepr(self: Geometry.MultiGeometry) -> str:
	return "{%s}" % ", ".join([repr(o) for o in self.geoms])

Point.__neg__ = __pointNeg
Point.__add__ = __pointAdd
Point.__sub__ = __pointSub
Point.__mul__ = __pointMulScalar
Point.__truediv__ = __pointDivScalar

Point.__repr__ = lambda self: "%s" % repr((self.x, self.y))
LinearRing.__repr__ = lambda self: "R%s" % __connectedGeometryRepr(self)
LineString.__repr__ = lambda self: "L%s" % __connectedGeometryRepr(self)
Polygon.__repr__ = lambda self: "P%s" % __connectedGeometryRepr(self)

MultiPoint.__repr__ = __multiGeometryRepr
MultiLineString.__repr__ = __multiGeometryRepr
MultiPolygon.__repr__ = __multiGeometryRepr

warnings.filterwarnings("error") # Turn shapely C++ errors into exceptions for better debugging.
