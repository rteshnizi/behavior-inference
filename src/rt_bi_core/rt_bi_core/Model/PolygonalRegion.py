from typing import Dict, List, Union

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
	Edges = Dict[str, LineString]
	"""
	A dictionary of edge identifier to `LineString`.
	The edge identifier is a string.
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
		envelope: Geometry.CoordsList,
		envelopeColor: Color,
		interiorColor: Color = KnownColors.WHITE,
		interior: Union[Polygon, MultiPolygon, None] = None,
		regionType: RegionType = RegionType.BASE,
		renderLineWidth = 1,
		**kwArgs
	):
		"""
		Initialize an polygonal region.

		Parameters
		----------
		idNum : int
			Id of the sensor region.
		envelope : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelope of the polygonal region.
		envelopeColor: Color
			The color of the envelope when/if rendered.
		interiorColor: Color, default `White`
			The color of the interior of the region when/if rendered.
		interior: Union[Polygon, MultiPolygon, None], default `None`
			The interior of the region, if it is separate from its envelope, default forces construction.
		regionType: RegionType, default `RegionType.BASE`
			The type of this region.
		renderLineWidth: int, default `1`
			The width of the rendered lines.
		"""
		self.__name = "%s-%d" % (regionType, idNum)
		self.__RENDER_LINE_WIDTH = renderLineWidth
		self.__envelope = envelope
		self.__coordsDict = self.__buildCoords(self.__envelope)
		self.__interiorPolygon = Polygon(self.__envelope) if interior is None else interior
		self.__regionType = regionType
		self.__ENVELOPE_COLOR = envelopeColor
		self.__INTERIOR_COLOR = interiorColor
		self.__TEXT_COLOR = KnownColors.BLACK if RViz.isLightColor(interiorColor) else KnownColors.WHITE
		if len(kwArgs) > 0 : RosUtils.Logger().warn("Unassigned keyword args ignored: %s" % repr(kwArgs))
		self.__edges: PolygonalRegion.Edges = self.__buildEdges()

	def __repr__(self) -> str:
		return self.name

	def __buildCoords(self, coords: Geometry.CoordsList) -> Dict[str, Point]:
		d = {}
		for c in coords:
			d[Geometry.pointStringId(c[0], c[1])] = Point(c[0], c[1])
		return d

	def __buildEdges(self) -> Edges:
		d = {}
		verts = list(self.interior.exterior.coords)
		for v1, v2 in zip(verts, verts[1:]):
			edgeCoords = [v1, v2]
			edge = LineString(edgeCoords)
			d[Geometry.coordListStringId(edgeCoords)] = edge
		return d

	@property
	def envelope(self) -> Geometry.CoordsList:
		"""The list of the coordinates of the vertices ."""
		return self.__envelope

	@property
	def name(self) -> str:
		"""A pre-designed string identifier, in the following format: `"%s-%d" % (regionType, idNum)`"""
		return self.__name

	@property
	def envelopeColor(self) -> Color:
		"""The color used to render the boundary of this polygon."""
		return self.__ENVELOPE_COLOR

	@property
	def interiorColor(self) -> Color:
		"""The color used for rendering the background of this polygon (to fill)."""
		return self.__INTERIOR_COLOR

	@property
	def regionType(self) -> RegionType:
		"""A short string identifier for the type of the polygon."""
		return self.__regionType

	@property
	def interior(self) -> Polygon:
		"""The Geometric description of the region."""
		return self.__interiorPolygon

	@property
	def edges(self) -> Edges:
		"""
		A dictionary of edge identifier to `LineString`.
		The edge identifier is a string.
		"""
		return self.__edges

	def forceUpdateInteriorPolygon(self, polygon: Polygon) -> None:
		"""Deliberately did not use a setter to not casually just update the interior."""
		self.__interiorPolygon = polygon
		return

	def hasEdge(self, e: LineString) -> bool:
		idCandidate1 = Geometry.lineStringId(e)
		idCandidate2 = Geometry.lineStringId(e.reverse())
		return idCandidate1 in self.__edges or idCandidate2 in self.__edges

	def isInsideRegion(self, x: float, y: float) -> bool:
		return Geometry.isXyInsidePolygon(x, y, self.interior)

	def getEquivalentEdge(self, finalConfig: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords) -> Union[LineString, None]:
		"""
			Given an affine transformation, and the final configuration of an edge after the transformation,
			find the edge that will be in that final configuration after the transformation, and `None` otherwise. Boy didn't I repeat myself?!
		"""
		for edge in self.__edges.values():
			afterTransformation = Geometry.applyMatrixTransformToLineString(transformation, edge, centerOfRotation)
			if Geometry.lineSegmentsAreAlmostEqual(finalConfig, afterTransformation): return edge
		return None

	def intersectsRegion(self, other: "PolygonalRegion") -> bool:
		return Geometry.polygonAndPolygonIntersect(self.interior, other.interior)

	def union(self, others: List["PolygonalRegion"]) -> Polygon:
		allPolygons = [r.interior for r in others]
		allPolygons.append(self.interior)
		return Geometry.union(allPolygons)

	def getCommonEdge(self, other: "PolygonalRegion") -> Union[LineString, None]:
		for e in self.__edges:
			if other.hasEdge(e): return self.__edges[e]
		return None

	def render(self, renderText = False, fill = False) -> List[Marker]:
		msgs = []
		if fill:
			RosUtils.Logger().info("Cannot fill polygons yet.")
		msgs.append(RViz.CreatePolygon(self.name, self.__envelope, self.__ENVELOPE_COLOR, self.__RENDER_LINE_WIDTH))
		if renderText:
			msgs.append(RViz.CreateText("%s_txt" % self.name, self.interior.centroid.xy, self.name, self.__TEXT_COLOR))
		return msgs

	def clearRender(self) -> None:
		RosUtils.Logger().info("No implementation for %s" % PolygonalRegion.clearRender.__name__)
		return
