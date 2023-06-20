from typing import Dict, List, Set, Union

from skimage import transform
from visualization_msgs.msg import Marker

from rt_bi_utils.Geometry import (Geometry, LineString, MultiPolygon, Point, Polygon)
from rt_bi_utils.RViz import Color, KnownColors, RViz
from rt_bi_utils.Ros import RosUtils


class PolygonalRegion:
	"""
		coords will be used to create the polygon.
		If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, boundaryColor: Color, backgroundColor: Color = KnownColors.WHITE, polygon: Polygon = None):
		self.name = name
		self._renderLineWidth = 1
		try:
			self._coordsList = coords if polygon is None else list(polygon.exterior.coords)
		except:
			print("woopsie")
		self._coordsDict = self._buildCoords(self._coordsList)
		self.polygon = Polygon(self._coordsList) if polygon is None else polygon
		self.BOUNDARY_COLOR = boundaryColor
		self.BACKGROUND_COLOR = backgroundColor
		self.TEXT_COLOR = KnownColors.BLACK if RViz.isLightColor(backgroundColor) else KnownColors.WHITE
		self.edges = self._buildEdges()
		self.canvasId = None
		self.textId = None

	def __repr__(self):
		return self.name

	def _buildCoords(self, coords: Geometry.CoordsList) -> Dict[str, Point]:
		d = {}
		for c in coords:
			d[Geometry.pointStringId(c[0], c[1])] = Point(c[0], c[1])
		return d

	def _buildEdges(self) -> Dict[str, LineString]:
		d = {}
		verts = list(self.polygon.exterior.coords)
		for v1, v2 in zip(verts, verts[1:]):
			edgeCoords = [v1, v2]
			edge = LineString(edgeCoords)
			d[Geometry.coordListStringId(edgeCoords)] = edge
		return d

	def _hasEdgeByXy(self, x1: float, y1: float, x2: float, y2: float) -> bool:
		return Geometry.coordListStringId(x1, y1, x2, y2) in self.edges

	def _hasEdgeByName(self, name: str) -> bool:
		return name in self.edges

	def isInsideRegion(self, x: float, y: float) -> bool:
		return Geometry.isXyInsidePolygon(x, y, self.polygon)

	def getEquivalentEdge(self, finalConfig: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords) -> LineString:
		"""
			Given an affine transformation, and the final configuration of an edge after the transformation,
			find the edge that will be in that final configuration after the transformation, and `None` otherwise. Boy didn't I repeat myself?!
		"""
		for edge in self.edges.values():
			afterTransformation = Geometry.applyMatrixTransformToLineString(transformation, edge, centerOfRotation)
			if Geometry.lineSegmentsAreAlmostEqual(finalConfig, afterTransformation): return edge
		return None

	def getCollidingEdgesByEdge(self, targetPolygon: Union[Polygon, MultiPolygon]) -> Dict[str, Set[LineString]]:
		"""## Get Colliding Edges By Edge
		Just like the name says.

		Parameters
		----------
		targetPolygon : Union[Polygon, MultiPolygon]
			The against whom we are checking collisions.

		Returns
		-------
		Dict[str, Set[LineString]]
			Map from the string of edge of the FOV to edge of the target polygon.
		"""
		collisionData = {}
		for edgeId in self.edges:
			collisionData[edgeId] = Geometry.getAllIntersectingEdgesWithLine(self.edges[edgeId], targetPolygon)
		return collisionData

	def intersectsRegion(self, other: "PolygonalRegion") -> bool:
		return Geometry.polygonAndPolygonIntersect(self.polygon, other.polygon)

	def union(self, others: List["PolygonalRegion"]) -> Polygon:
		return Geometry.union([r.polygon for r in others].append(self.polygon))

	def getCommonEdge(self, other) -> LineString:
		for e in self.edges:
			if other._hasEdgeByName(e): return self.edges[e]
		return None

	def render(self, renderText = False, fill = False) -> List[Marker]:
		msg = []
		if fill:
			RosUtils.Logger.warn("Cannot fill polygons yet...")
		msg.append(RViz.CreatePolygon(self._coordsList, self.BOUNDARY_COLOR, 1, self.name))
		if renderText:
			msg.append(RViz.CreateText(self.polygon.centroid.xy, self.name, self.TEXT_COLOR))

	def clearRender(self) -> None:
		if self.canvasId is not None:
			pass
			self.canvasId = None
		if self.textId is not None:
			pass
			self.textId = None
