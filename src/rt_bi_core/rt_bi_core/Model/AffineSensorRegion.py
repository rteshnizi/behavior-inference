from typing import Dict, Set, Union

from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.Track import Tracks
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose


class AffineSensorRegion(SensorRegion):
	def __init__(
			self,
			centerOfRotation: Pose,
			idNum: int,
			envelop: Geometry.CoordsList,
			fov: Union[Polygon, MultiPolygon, None]=None,
			tracks: Tracks={},
			**kwArgs
		) -> None:
		"""
		Initialize an affine sensor region. An affine sensor region need's a center of rotation.

		Parameters
		----------
		centerOfRotation : Pose
			This will be used to interpolate the rotation motion of the region.
		idNum : int
			Id of the sensor region.
		envelop : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelop of the polygonal region.
		fov: Union[Polygon, MultiPolygon, None], default `None`
			The field-of-view, default forces construction using knowledge-base.
		tracks : Tracks, default `{}`
			The tracklets, as defined in the dissertation, observable in the fov,
			default forces construction using knowledge-base.
		"""
		super().__init__(
			idNum=idNum,
			envelop=envelop,
			tracks=tracks,
			fov=fov,
			**kwArgs
		)
		self.centerOfRotation = centerOfRotation

	@property
	def name(self) -> str:
		"""Attaches: `AFF-` to super `super().name`."""
		return "AFF-%s" % super().name

	def __getLineSegmentExpandedBb(self, transformation: AffineTransform, lineSeg: LineString, angle: float, centerOfRotation: Geometry.Coords) -> Polygon:
		""" Gets a tight bounding box for a line segment that is moving with a constant angular velocity. """
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

	def findCollisionsWithExtendedBb(self, past: "AffineSensorRegion", targetPolygon: Union[Polygon, MultiPolygon]) -> Dict[str, Set[LineString]]:
		"""
			#### Returns
				For each edge of the sensor,
				it returns all the edges of the map that intersect the expanded bounding box of that edge.
		"""
		centerOfRotation = (self.centerOfRotation.x, self.centerOfRotation.y)
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
