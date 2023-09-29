import operator
import warnings
from functools import reduce
from math import atan2, cos, degrees, inf, sin, sqrt
from typing import Dict, List, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point
from shapely.geometry.multipoint import MultiPoint
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid
from skimage import transform
from skimage.transform import AffineTransform as AffineTransform

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
	GeometricObject = Union[Polygon, MultiPolygon, LineString, MultiLineString, Point, MultiPoint]

	@staticmethod
	def addCoords(c1: Coords, c2: Coords) -> Coords:
		return(c1[0] + c2[0], c1[1] + c2[1])

	@staticmethod
	def addScalar(c1: Coords, s: float) -> Coords:
		return(c1[0] + s, c1[1] + s)

	@staticmethod
	def getGeometryCoords(geom: ConnectedGeometry) -> CoordsList:
		if not hasattr(geom, "exterior"):
			if not hasattr(geom, "coords"):
				Logger().error("Unknown geometry %s." % repr(geom))
				return []
			return list(geom.coords)
		return list(geom.exterior.coords)

	@staticmethod
	def pointStringId(x: float, y: float) -> str:
		return "%.2f,%.2f" % (x, y)

	@staticmethod
	def lineStringId(line: LineString) -> Tuple[str, ...]:
		coords = Geometry.getGeometryCoords(line)
		return tuple(Geometry.pointStringId(coord[0], coord[1]) for coord in coords)

	@staticmethod
	def vectorsAreEqual(vec1: Vector, vec2: Vector, withinEpsilon = True) -> bool:
		d1 = Geometry.distanceFromVects(vec1, vec2)
		margin = Geometry.EPSILON if withinEpsilon else 0
		return d1 <= margin

	@staticmethod
	def coordsAreAlmostEqual(coords1: Coords, coords2: Coords) -> bool:
		d1 = Geometry.distance(coords1[0], coords1[1], coords2[0], coords2[1])
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
	def sortCoordinatesClockwise(coords: CoordsList) -> CoordsList:
		"""
		(x,y)

		coords = [(0, 1), (1, 0), (1, 1), (0, 0)]
		"""
		center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), coords), [len(coords)] * 2))
		sortedCoords = sorted(coords, key=lambda coord: (-135 - degrees(atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
		return sortedCoords

	@staticmethod
	def isLineSegment(l: LineString) -> bool:
		return isinstance(l, LineString) and len(l.coords) == 2

	@staticmethod
	def orthogonal(p1: Coords, p2: Coords) -> float:
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
		vect = Geometry.distanceVect(x1, y1, x2, y2)
		length = Geometry.vectLength(vect[0], vect[1])
		return length

	@staticmethod
	def distanceFromVects(vect1: Vector, vect2: Vector) -> float:
		return Geometry.distance(vect1[0], vect1[1], vect2[0], vect2[1])

	@staticmethod
	def distanceVect(x1: float, y1: float, x2: float, y2: float) -> Vector:
		dx = x2 - x1
		dy = y2 - y1
		return (dx, dy)

	@staticmethod
	def distanceVectFromPts(pt1: Point, pt2: Point) -> Vector:
		return Geometry.distanceVect(pt1.x, pt1.y, pt2.x, pt2.y)

	@staticmethod
	def getUnitVector(x: float, y: float) -> Vector:
		"""
		Returns a tuple (x, y) of a vector.
		"""
		length = Geometry.distance(0, 0, x, y)
		return (x / length, y / length)

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
	def intersects(o1: GeometricObject, o2: GeometricObject) -> bool:
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
			Logger().warn("Shapely exception caught in intersects(): %s, %s" % (e.__class__.__name__, repr(e)))
			return False

	@staticmethod
	def intersection(o1: GeometricObject, o2: GeometricObject) -> GeometricObject:
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
			return o1.intersection(o2)
		except Exception as e:
			Logger().error("1. Shapely exception.. in intersection(): %s" % repr(e))
			Logger().error("2. Shapely exception.. input args: %s, %s" % (repr(o1), repr(o2)))
			return type(o1)()

	@staticmethod
	def union(polyList: List[Polygon]) -> Union[Polygon, MultiPolygon]:
		validPolyList = []
		for p in polyList:
			parts = make_valid(p)
			validPolyList.append(parts)
		return unary_union(validPolyList)

	@staticmethod
	def subtract(poly1: Polygon, poly2: Polygon) -> Polygon:
		return poly1.difference(poly2)

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
		subtraction = p1.difference(p2) # FIXME: difference() is wrong because it excludes the boundaries of fov from the shadows
		# subtraction = mapRegionPoly.difference(mapRegionPoly.intersection(fovPoly))
		# subtraction = mapRegionPoly.symmetric_difference(fovPoly).difference(fovPoly)
		return Geometry.toGeometryList(subtraction)

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
	def getAffineTransformation(start: CoordsList, end: CoordsList, centerOfRotation: Pose) -> AffineTransform:
		"""
		Estimate the affine transformation matrix that would transform the given polygon from the start state to end state.

		Parameters
		----------
		start : Polygon
			The starting configuration of the polygon.
		end : Polygon
			The starting configuration of the polygon.
		centerOfRotation : Coords
			The center of rotation of the rotation motion.

		Returns
		-------
		AffineTransform
			An affine transformation matrix object.
		"""
		startCoords = np.array(start)
		endCoords = np.array(end)
		startCoords = [(coords[0] - centerOfRotation.x, coords[1] - centerOfRotation.y) for coords in startCoords]
		endCoords = [(coords[0] - centerOfRotation.x, coords[1] - centerOfRotation.y) for coords in endCoords]
		startCoords = np.array(startCoords)
		endCoords = np.array(endCoords)
		matrix = transform.estimate_transform("affine", startCoords, endCoords)
		# matrix = transform.estimate_transform("similarity", startCoords, endCoords)
		return matrix

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
		if param > 1 or param < 0:
			raise ValueError("Parameter should be in range [0, 1]. Given param = %f" % param)
		# Easy cases that do not need calculation
		if param == 0: return AffineTransform(np.identity(3))
		if param == 1: return AffineTransform(transformation.params)
		# Other params
		# scale = [((transformation.scale[0] - 1) * param) + 1, ((transformation.scale[1] - 1) * param) + 1]
		scale = 1
		rotations = Rotation.from_matrix([
			[
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1]
			],
			[
				[transformation.params[0][0], transformation.params[0][1], 0],
				[transformation.params[1][0], transformation.params[1][1], 0],
				[0, 0, 1]
			]
		])
		slerp = Slerp([0, 1], rotations)
		rotation = slerp([0, param, 1])[1].as_euler('xyz')[2]
		# shear = transformation.shear * param
		shear = 0
		translation = [transformation.translation[0] * param, transformation.translation[1] * param]
		# translation = [0, 0]
		parameterizedMatrix = AffineTransform(matrix=None, scale=scale, rotation=rotation, shear=shear, translation=translation)
		return parameterizedMatrix

	@staticmethod
	def applyMatrixTransformToCoordsList(transformation: AffineTransform, coordsList: CoordsList, centerOfRotation: Pose) -> CoordsList:
		coordsList = [(coords[0] - centerOfRotation.x, coords[1] - centerOfRotation.y) for coords in coordsList]
		transformedCoords = transform.matrix_transform(coordsList, transformation.params)
		transformedCoords = [(coords[0] + centerOfRotation.x, coords[1] + centerOfRotation.y) for coords in transformedCoords]
		return transformedCoords

	@staticmethod
	def applyMatrixTransformToPolygon(transformation: AffineTransform, polygon: Polygon, centerOfRotation: Pose) -> Polygon:
		pCoords = Geometry.getGeometryCoords(polygon)
		transformedCoords = Geometry.applyMatrixTransformToCoordsList(transformation, pCoords, centerOfRotation)
		transformedPolygon = Polygon(transformedCoords)
		return transformedPolygon

	@staticmethod
	def applyMatrixTransformToLineString(transformation: AffineTransform, line: LineString, centerOfRotation: Pose) -> LineString:
		pCoords = Geometry.getGeometryCoords(line)
		transformedCoords = Geometry.applyMatrixTransformToCoordsList(transformation, pCoords, centerOfRotation)
		transformedLineString = LineString(transformedCoords)
		return transformedLineString

	@staticmethod
	def findTheLastTimeTheyAreColliding(movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform) -> float:
		"""
			### Remarks
			This assumes the edges are in contact already.
		"""
		NUM_SAMPLES = 100
		latestTime = inf
		# If the edge is not intersecting currently, we don't need to check for the latest time
		if Geometry.intersects(movingEdge, staticEdge):
			return latestTime
		for x in range(1, NUM_SAMPLES, 1):
			fraction = x / NUM_SAMPLES
			newTransform = Geometry.getParameterizedAffineTransformation(transformation, fraction)
			intermediateLine = Geometry.applyMatrixTransformToLineString(newTransform, movingEdge)
			if Geometry.intersects(intermediateLine, staticEdge):
				latestTime = fraction
			else:
				break
		return latestTime

	@staticmethod
	def findTheEarliestTimeTheyAreColliding(movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform) -> float:
		NUM_SAMPLES = 100
		latestTime = inf
		for x in range(0, NUM_SAMPLES, 1):
			fraction = x / NUM_SAMPLES
			newTransform = Geometry.getParameterizedAffineTransformation(transformation, fraction)
			intermediateLine = Geometry.applyMatrixTransformToLineString(newTransform, movingEdge)
			if Geometry.intersects(intermediateLine, staticEdge):
				latestTime = fraction
				break
		return latestTime

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
	return "[#%d, v:%s, e:%s]" % (
		len(self.coords) if isinstance(self, LineString) else len(self.exterior.coords),
		"Y" if self.is_valid else "N",
		"Y" if self.is_empty else "N",
	)

def __multiGeometryRepr(self: Geometry.MultiGeometry) -> str:
	return "{%s}" % ", ".join([repr(o) for o in self])

Point.__neg__ = __pointNeg
Point.__add__ = __pointAdd
Point.__sub__ = __pointSub
Point.__mul__ = __pointMulScalar
Point.__truediv__ = __pointDivScalar

Point.__repr__ = lambda p: "%s" % repr((p.x, p.y))
LinearRing.__repr__ = lambda o: "R%s" % __connectedGeometryRepr(o)
LineString.__repr__ = lambda o: "L%s" % __connectedGeometryRepr(o)
Polygon.__repr__ = lambda o: "P%s" % __connectedGeometryRepr(o)

MultiPoint.__repr__ = __multiGeometryRepr
MultiLineString.__repr__ = __multiGeometryRepr
MultiPolygon.__repr__ = __multiGeometryRepr

# warnings.filterwarnings("error") # Turn shapely C++ errors into exceptions for better debugging.
