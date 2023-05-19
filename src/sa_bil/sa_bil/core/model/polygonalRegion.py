from shapely.geometry import LineString, Point, Polygon
from typing import Dict, List, Set, Union
from skimage import transform
from sa_bil.renderer import Renderer
from sa_bil.core.utils.geometry import Geometry, MultiPolygon

class PolygonalRegion:
	"""
		coords will be used to create the polygon.
		If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, boundaryColor="RED", backgroundColor="", polygon: Polygon = None):
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

	def _hasEdgeByXy(self, x1, y1, x2, y2):
		return Geometry.coordListStringId(x1, y1, x2, y2) in self.edges

	def _hasEdgeByName(self, name):
		return name in self.edges

	def isInsideRegion(self, x, y):
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

	def intersectsRegion(self, other: "PolygonalRegion"):
		return Geometry.polygonAndPolygonIntersect(self.polygon, other.polygon)

	def union(self, others: List["PolygonalRegion"]) -> Polygon:
		return Geometry.union([r.polygon for r in others].append(self.polygon))

	def getCommonEdge(self, other) -> LineString:
		for e in self.edges:
			if other._hasEdgeByName(e): return self.edges[e]
		return None

	def render(self, canvas, renderText=False, hashFill=False, hashDensity=25):
		if self.canvasId is not None: self.clearRender(canvas)
		if hashDensity not in [75, 50, 25, 12]: raise AssertionError("Density should be one of 75, 50, 25, or 12.")
		self.canvasId = Renderer.CreatePolygon(canvas, self._coordsList, outline=self.BOUNDARY_COLOR, fill=self.BACKGROUND_COLOR, width=self._renderLineWidth, tag=self.name, hashFill=hashFill, hashDensity=hashDensity)
		if renderText: self.textId = Renderer.CreateText(canvas, [self.polygon.centroid.x, self.polygon.centroid.y], self.name, tag=self.name, color="White" if self.BACKGROUND_COLOR.upper() == "BLACK" else "Black")

	def clearRender(self, canvas):
		if self.canvasId is not None:
			Renderer.RemoveShape(canvas, self.canvasId)
			self.canvasId = None
		if self.textId is not None:
			Renderer.RemoveShape(canvas, self.textId)
			self.textId = None
