from typing import Dict, List, Set, Union, Tuple
from sa_bil.core.observation.sensor import Sensor
from sa_bil.core.utils.geometry import Geometry, Polygon, MultiPolygon, LineString

Events = Tuple[Polygon, float, str, LineString]
"""
#### Type
```
(FOV polygon, timeOfEvent, "ingoing" | "outgoing", LineString)
# map edge relevant to the event
```
"""

CollisionData = Dict[tuple, Dict[str, Set[LineString]]]
"""
#### Type
```
{"sensorId": {"edgeId": {L1, L2, ...}}}
Dict[tuple, Dict[str, Set[LineString]]]
```
"""

CollisionInterval = Tuple[LineString, LineString, float, float]
"""## Collision Interval

A collision interval is a represents an interval of time in which two edges.
The two floats are the two closed bounds of the interval of time when this event happens.
That is, Given a tuple `(L1, L2, T1, T2)`, we get the interval of event: `[ T1 * fov1.t, T2 * fov2.t ]`

"""

class Fov:
	"""A Class to model field of view.
	"""
	def __init__(self, sensors):
		self.__polygon: Union[Polygon, MultiPolygon] = None
		self.__edges: Dict[str, LineString] = None
		self.sensors: Dict[tuple, Sensor] = sensors

	def __repr__(self):
		return "FOV-%s" % repr(self.sensors)

	@property
	def polygon(self) -> Union[Polygon, MultiPolygon]:
		"""The polygonal geometrical representation of the FOV.
		"""
		if self.__polygon is None:
			polygons = [self.sensors[i].region.polygon for i in self.sensors]
			self.__polygon = Geometry.union(polygons)
		return self.__polygon

	@property
	def time(self):
		if len(self.sensors) == 0: return None
		for sensorId in self.sensors:
			return self.sensors[sensorId].pose.time

	@property
	def edges(self) -> Dict[str, LineString]:
		if self.__edges is None:
			for s in self.sensors:
				self.__edges = self.__edges | self.sensors[s].region.edges
		return self.__edges

	def __getEquivalentSensorById(self, sensorId: tuple) -> Sensor:
		return self.sensors[(self.time, sensorId[1])]

	def __getCollidingEdgesBySensor(self, polygon: Union[Polygon, MultiPolygon]) -> CollisionData:
		collisionData: CollisionData = {}
		for sensorId in self.sensors:
			collisionData[sensorId] = self.sensors[sensorId].findCollisionsWithExtendedBb(polygon)

	def __initCollisionIntervals(self, intermediateCollisions: CollisionData) -> CollisionInterval:
		intervals = []
		for sensorEdgeId in intermediateCollisions:
			sensorEdge = intermediateCollisions.edges[sensorEdgeId]
			mapEdges = intermediateCollisions[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			for mapEdge in mapEdges:
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def __estimateLocationWithSlirp(self, past: "Fov", ratio: float) -> Union[Polygon, MultiPolygon]:
		"""
			#### Input

			`past` a configuration of the fov (one-to-one mapping of the sensors) in the past,
			`ratio` a number between `0` (`self`) and `1` (`past`)

			#### Returns
			a `Polygon` or a `MultiPolygon` representing the state of the fov at time ratio.
		"""
		polygons = []
		for sensorId in self.sensors:
			sensor = self.sensors[sensorId]
			pastSensor = past.__getEquivalentSensorById(sensorId)
			centerOfRotation = (self.pose.x, self.pose.y)
			transformation = Geometry.getAffineTransformation(sensor.region.polygon, pastSensor.region.polygon, centerOfRotation)
			intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, ratio)
			polygons.append(Geometry.applyMatrixTransformToPolygon(intermediateTransform, self.polygon, centerOfRotation))
		polygons = Geometry.union(polygons)
		return polygons

	def estimateIntermediateCollisionsWithPolygon(self, past: "Fov", polygon: Union[Polygon, List[Polygon], MultiPolygon]) -> Events:
		"""## Estimate Intermediate Collisions With Polygon

			Given  and the current configuration of it (`self`),
			find all the times when there is a shadow component event.
			That is, as the `past` FOV moves towards its current configuration (`self`),
			save the intermediate configurations for which there is a topological change.
			This function assumes each FOV has gone through an affine transformation between two consecutive time frames.

			Parameters
			----------
			past : Fov
				The `past` configuration of the FOV
			polygon : Polygon | List[Polygon] | MultiPolygon
				The second polygon with whom we are interested in checking intersection of edges.

			Returns
			-------
			Events
				A list of intervals in each of which there is at most one component event.
			"""
		# Begin by collecting the edges are that are in contact at the beginning and at the end of the motion.
		currentCollidingEdges = self.__getCollidingEdgesBySensor(polygon)
		pastCollidingEdges = past.__getCollidingEdgesBySensor(polygon)
		possibleCollisionData: Events = {}
		for sensorId in past.sensors:
			possibleCollisionData[sensorId] = self.sensors[sensorId].findCollisionsWithExtendedBb(past.__getEquivalentSensorById(sensorId), polygon)
		# Remove the edges that we are sure are intersecting.
		for sensorId in currentCollidingEdges:
			for l in currentCollidingEdges[sensorId]:
				if sensorId in possibleCollisionData: possibleCollisionData[sensorId].remove(l)
		for sensorId in pastCollidingEdges:
			for l in pastCollidingEdges[sensorId]:
				if sensorId in possibleCollisionData: possibleCollisionData[sensorId].remove(l)
		collisionIntervals = self.__initCollisionIntervals(intermediateCollisions)
