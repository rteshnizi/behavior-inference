from math import nan
from typing import Dict, List, Set, Union

from skimage import transform
from visualization_msgs.msg import Marker

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry, LineString, MultiPolygon, Point, Polygon
from rt_bi_utils.RViz import Color, KnownColors, RViz


class PolygonalRegion:
	"""
		The base class for all polygonal regions.
		Provides much of the structure and functionality.
	"""

	class RegionType:
		"""All the types of `PolygonalRegion`."""
		BASE = "B"
		MAP = "M"
		SENSING = "Z"
		SHADOW = "S"
		SYMBOL = "A"

	def __init__(
		self,
		idNum: int,
		envelop: Geometry.CoordsList,
		envelopColor: Color,
		interiorColor: Color = KnownColors.WHITE,
		interior: Union[Polygon, MultiPolygon, None] = None,
		regionType: RegionType = RegionType.BASE,
		renderLineWidth = 1,
		timeNanoSecs = 0,
		**kwArgs
	):
		"""
		Initialize an polygonal region.

		Parameters
		----------
		idNum : int
			Id of the sensor region.
		envelop : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelop of the polygonal region.
		envelopColor: Color
			The color of the envelop when/if rendered.
		interiorColor: Color, default `White`
			The color of the interior of the region when/if rendered.
		interior: Union[Polygon, MultiPolygon, None], default `None`
			The interior of the region, if it is separate from its envelop, default forces construction.
		regionType: RegionType, default `RegionType.BASE`
			The type of this region.
		renderLineWidth: int, default `1`
			The width of the rendered lines.
		timeNanoSecs: int, default `0`
			For regions that require a timestamp, this variable holds a float NanoSecond time.
		"""
		self.__name = "%s-%d" % (regionType, idNum)
		self.__RENDER_LINE_WIDTH = renderLineWidth
		self.__envelop = envelop
		self.__coordsDict = self.__buildCoords(self.__envelop)
		self.__interiorPolygon = Polygon(self.__envelop) if interior is None else interior
		self.__regionType = regionType
		self.__ENVELOP_COLOR = envelopColor
		self.__INTERIOR_COLOR = interiorColor
		self.__TEXT_COLOR = KnownColors.BLACK if RViz.isLightColor(interiorColor) else KnownColors.WHITE
		self.__timeNanoSecs = timeNanoSecs
		if len(kwArgs) > 0 : RosUtils.Logger().warn("Unassigned keyword args ignored: %s" % repr(kwArgs))
		self.edges: Dict[str, LineString] = self.__buildEdges()
		"""
		A Dictionary of edge identifier to ObjectModel.
		The edge identifier is the
		"""

	def __repr__(self) -> str:
		return self.name

	def __buildCoords(self, coords: Geometry.CoordsList) -> Dict[str, Point]:
		d = {}
		for c in coords:
			d[Geometry.pointStringId(c[0], c[1])] = Point(c[0], c[1])
		return d

	def __buildEdges(self) -> Dict[str, LineString]:
		d = {}
		verts = list(self.interior.exterior.coords)
		for v1, v2 in zip(verts, verts[1:]):
			edgeCoords = [v1, v2]
			edge = LineString(edgeCoords)
			d[Geometry.coordListStringId(edgeCoords)] = edge
		return d

	@property
	def envelop(self) -> Geometry.CoordsList:
		"""The list of the coordinates of the vertices ."""
		return self.__envelop

	@envelop.setter
	def envelop(self, _: Geometry.CoordsList) -> None:
		raise NotImplementedError("No setter.")

	@property
	def name(self) -> str:
		"""A pre-designed string identifier, in the following format: `"%s-%d" % (regionType, idNum)`"""
		return self.__name

	@property
	def timeNanoSecs(self) -> float:
		"""If there is a timestamp associated, this returns the time, and `nan` otherwise."""
		return self.__timeNanoSecs

	@timeNanoSecs.setter
	def timeNanoSecs(self, _: float) -> None:
		raise NotImplementedError("No setter.")

	@property
	def envelopColor(self) -> Color:
		"""The color used to render the boundary of this polygon."""
		return self.__ENVELOP_COLOR

	@envelopColor.setter
	def envelopColor(self, _: Color) -> None:
		raise NotImplementedError("No setter.")

	@property
	def interiorColor(self) -> Color:
		"""The color used for rendering the background of this polygon (to fill)."""
		return self.__INTERIOR_COLOR

	@interiorColor.setter
	def interiorColor(self, _: Color) -> None:
		raise NotImplementedError("No setter.")

	@property
	def regionType(self) -> RegionType:
		"""A short string identifier for the type of the polygon."""
		return self.__regionType

	@regionType.setter
	def regionType(self, _: RegionType) -> None:
		raise NotImplementedError("No setter.")

	@property
	def interior(self) -> Polygon:
		"""The Geometric description of the region."""
		return self.__interiorPolygon

	@interior.setter
	def interior(self, _: Polygon) -> None:
		raise NotImplementedError("Changing the polygon after __init__ has not been thought out yet.")

	def hasEdge(self, e: LineString) -> bool:
		idCandidate1 = Geometry.lineStringId(e)
		idCandidate2 = Geometry.lineStringId(e.reverse())
		return idCandidate1 in self.edges or idCandidate2 in self.edges

	def isInsideRegion(self, x: float, y: float) -> bool:
		return Geometry.isXyInsidePolygon(x, y, self.interior)

	def getEquivalentEdge(self, finalConfig: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords) -> Union[LineString, None]:
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
		return Geometry.polygonAndPolygonIntersect(self.interior, other.interior)

	def union(self, others: List["PolygonalRegion"]) -> Polygon:
		allPolygons = [r.interior for r in others]
		allPolygons.append(self.interior)
		return Geometry.union(allPolygons)

	def getCommonEdge(self, other: "PolygonalRegion") -> Union[LineString, None]:
		for e in self.edges:
			if other.hasEdge(e): return self.edges[e]
		return None

	def render(self, renderText = False, fill = False) -> List[Marker]:
		msgs = []
		if fill:
			RosUtils.Logger().info("Cannot fill polygons yet.")
		msgs.append(RViz.CreatePolygon(self.name, self.__envelop, self.__ENVELOP_COLOR, self.__RENDER_LINE_WIDTH))
		if renderText:
			msgs.append(RViz.CreateText("%s_txt" % self.name, self.interior.centroid.xy, self.name, self.__TEXT_COLOR))
		return msgs

	def clearRender(self) -> None:
		RosUtils.Logger().info("No implementation for %s" % PolygonalRegion.clearRender.__name__)
		return
