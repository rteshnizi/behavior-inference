from typing import Dict, Set, Union
from skimage import transform
from sa_bil.core.model.featureMap import FeatureMap
from sa_bil.core.model.map import Map
from sa_bil.core.model.sensingRegion import SensingRegion
from sa_bil.core.observation.pose import Pose
from sa_bil.core.utils.geometry import Geometry, LineString, Polygon, MultiPolygon

class Sensor:
	def __init__(self, idNum: int, time: float, x: float, y: float, psi: float, coords: Geometry.CoordsList, envMap: Map, featureMap: FeatureMap):
		self.id = idNum
		self.pose = Pose(time, x, y, psi)
		self._originalCoords = [tuple(coord) for coord in coords]
		poly = self._buildVisibilityPolygon(envMap, featureMap)
		self.region = SensingRegion("SR%d" % idNum, [], time, idNum, polygon=poly)

	def __repr__(self):
		return "Sensor%d" % self.id

	def _buildVisibilityPolygon(self, envMap: Map, featureMap: FeatureMap) -> Geometry.CoordsList:
		# sortedCoords = Geometry.sortCoordinatesClockwise(self._originalCoords)
		sortedCoords = self._originalCoords
		polygon = Polygon(sortedCoords)
		for rKey in envMap.regions:
			region = envMap.regions[rKey]
			# FIXME: Currently, the type of sensor is missing.
			# Once its available you should check the type and see which type of seeThrough I should look for
			if featureMap.features[region.type].seeThrough.fromAbove: continue
			nextPolygon = Geometry.subtract(polygon, region.polygon)
			polygon = nextPolygon
		return polygon

	def __getLineSegmentExpandedBb(self, transformation: transform.AffineTransform, lineSeg: LineString, angle: float, centerOfRotation: Geometry.Coords) -> Polygon:
		"""
			Gets a tight bounding box for a line segment that is moving with a constant angular velocity.
		"""
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg, centerOfRotation)
		polygons = []
		for j in range(16):
			v1 = self._expandVertObbWithAngularVelocity(lineSeg.coords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = self._expandVertObbWithAngularVelocity(finalConfig.coords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = self._expandVertObbWithAngularVelocity(finalConfig.coords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = self._expandVertObbWithAngularVelocity(lineSeg.coords[1], angle, centerOfRotation, j & 8 != 0)
			p = Polygon([v1, v2, v3, v4])
			polygons.append(p)
		expandedObb = Geometry.union(polygons)
		expandedObb = expandedObb.convex_hull
		return expandedObb

	def findCollisionsWithExtendedBb(self, past: "Sensor", targetPolygon: Union[Polygon, MultiPolygon]) -> Dict[str, Set[LineString]]:
		"""
			#### Returns
				For each edge of the sensor,
				it returns all the edges of the map that intersect the expanded bounding box of that edge.
		"""
		centerOfRotation = (self.pose.x, self.pose.y)
		transformation = Geometry.getAffineTransformation(self.region.polygon, past.region.polygon, centerOfRotation)
		collisionData = {}
		angle = abs(transformation.rotation)
		for sensorEdgeId in self.region.edges:
			collisionData[sensorEdgeId] = []
			edge = self.region.edges[sensorEdgeId]
			boundingBox = self.__getLineSegmentExpandedBb(transformation, edge, angle, centerOfRotation)
			mapBoundaryVerts = targetPolygon.exterior.coords
			for v1, v2 in zip(mapBoundaryVerts, mapBoundaryVerts[1:]):
				mapEdge = LineString([v1, v2])
				if boundingBox.intersects(mapEdge):
					collisionData[sensorEdgeId].append(mapEdge)
		return collisionData
