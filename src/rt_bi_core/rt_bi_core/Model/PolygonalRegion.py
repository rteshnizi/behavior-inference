from enum import Enum
from typing import Dict, Sequence, Union

from skimage import transform
from visualization_msgs.msg import Marker

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry, LineString, MultiPolygon, Polygon
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
	class RegionType(Enum):
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
		self.__idNum = idNum
		self.__name = "%s-%d" % (regionType.value, idNum)
		self.__RENDER_LINE_WIDTH = renderLineWidth
		self.__interiorPolygon = Polygon(envelope) if interior is None else interior
		self.__envelope = envelope if interior is None else Geometry.getGeometryCoords(self.__interiorPolygon)
		self.__regionType = regionType
		self.__DEFAULT_ENVELOPE_COLOR = envelopeColor
		self.__INTERIOR_COLOR = interiorColor
		self.__TEXT_COLOR = KnownColors.BLACK if RViz.isLightColor(interiorColor) else KnownColors.WHITE
		if len(kwArgs) > 0 : RosUtils.Logger().warn("Unassigned keyword args ignored: %s" % repr(kwArgs))
		self.__edges: PolygonalRegion.Edges = self.__buildEdges()

	def __repr__(self) -> str:
		return self.name

	def __buildEdges(self) -> Edges:
		d = {}
		verts = list(self.interior.exterior.coords)
		for v1, v2 in zip(verts, verts[1:]):
			edgeCoords = [v1, v2]
			edge = LineString(edgeCoords)
			d[Geometry.lineStringId(edge)] = edge
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
		return self.__DEFAULT_ENVELOPE_COLOR

	@property
	def idNum(self) -> int:
		"""Id number of this polygonal region of this certain type."""
		return self.__idNum

	@property
	def interior(self) -> Polygon:
		"""The Geometric description of the region."""
		return self.__interiorPolygon

	@property
	def interiorColor(self) -> Color:
		"""The color used for rendering the background of this polygon (to fill)."""
		return self.__INTERIOR_COLOR

	@property
	def regionType(self) -> RegionType:
		"""A short string identifier for the type of the polygon."""
		return self.__regionType

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

	def edgeId(self, e: LineString) -> Union[str, None]:
		"""### Edge Id
		Returns the Id of a given edge if it belongs to the region, or `None` otherwise.

		Parameters
		----------
		e : LineString
			The edge to receive its Id.

		Returns
		-------
		Union[str, None]
			The string Id or `None`.
		"""
		idCandidate1 = Geometry.lineStringId(e)
		idCandidate2 = Geometry.lineStringId(e.reverse())
		if idCandidate1 in self.__edges: return idCandidate1
		if idCandidate2 in self.__edges: return idCandidate2
		return None

	def getEquivalentEdge(self, finalConfig: LineString, transformation: transform.AffineTransform) -> Union[LineString, None]:
		"""
			Given an affine transformation, and the final configuration of an edge after the transformation,
			find the edge that will be in that final configuration after the transformation, and `None` otherwise. Boy didn't I repeat myself?!
		"""
		for edge in self.__edges.values():
			afterTransformation = Geometry.applyMatrixTransformToLineString(transformation, edge)
			if Geometry.lineSegmentsAreAlmostEqual(finalConfig, afterTransformation): return edge
		return None

	def render(self, renderText: bool = False, fill: bool = False, envelopeColor: Union[Color, None] = None) -> Sequence[Marker]:
		msgs = []
		if fill:
			RosUtils.Logger().debug("Cannot fill polygons yet.")
		envelopColor = envelopeColor if envelopeColor is not None else self.__DEFAULT_ENVELOPE_COLOR
		msgs.append(RViz.createPolygon(self.name, Geometry.getGeometryCoords(self.interior), envelopColor, self.__RENDER_LINE_WIDTH))
		if renderText:
			msgs.append(RViz.createText("%s_txt" % self.name, self.interior.centroid.xy, self.name, self.__TEXT_COLOR))
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = []
		msgs.append(RViz.removeMarker(self.name))
		msgs.append(RViz.removeMarker("%s_txt" % self.name))
		return msgs
