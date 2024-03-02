from abc import ABC, abstractproperty
from enum import Enum
from typing import Sequence, TypeAlias, TypeVar

from rt_bi_commons.Shared.Color import RGBA, ColorNames, ColorUtils
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RViz import RViz

_T = TypeVar("_T", bound=type["Polygon"])

class Polygon(ABC):
	"""
		The base class for all polygonal regions.
		Provides much of the structure and functionality.
	"""

	from rt_bi_commons.Utils.Msgs import NANO_CONVERSION_CONSTANT
	from rt_bi_commons.Utils.RViz import DEFAULT_RENDER_DURATION_NS

	Id = NxUtils.Id

	Edges: TypeAlias = dict[str, Shapely.LineString]
	"""
	A dictionary of edge identifier to `Shapely.LineString`.
	The edge identifier is a string.
	"""
	class Types(Enum):
		BASE = "B"
		"""Indicates unset value."""
		DYNAMIC = "D"
		MOVING = "M"
		SENSING = "Z"
		SHADOW = "X"
		STATIC = "S"
		TARGET = "T"

	type = Types.BASE

	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: GeometryLib.CoordsList,
			envelopeColor: RGBA,
			interiorColor: RGBA = ColorNames.GREY_DARK,
			interior: Shapely.Polygon | None = None,
			renderLineWidth = 1,
			predicates: list[Msgs.RtBi.Predicate] = [],
			timeNanoSecs: int = -1,
			**kwArgs
		) -> None:
		"""
		:param str polygonId: Id of the polygon.
		:param str regionId: Id of the regular region owning this polygon.
		:param CoordsList envelope: The list of the coordinates of the vertices of the envelope of the polygonal region.
		:param RGBA envelopeColor: The color of the envelope when/if rendered.
		:param RGBA interiorColor: The color of the interior of the region when/if rendered, defaults to `ColorNames.GREY_DARK`.
		:param interior: The interior of the region, if it is separate from its envelope, `None` forces construction. defaults to `None`.
		:type interior: `Shapely.Polygon` or `None`
		:param int renderLineWidth: The width of the rendered lines, defaults to ``1``.
		:param list predicates: The predicates associated with this polygon, defaults to ``[]``.
		:type predicates: `list[Msgs.RtBi.Predicate]`
		:param int timeNanoSecs: Time of the predicate evaluations, defaults to ``-1`` which indicates the polygon is static.
		"""
		self.__id = Polygon.Id(regionId=regionId, polygonId=polygonId, overlappingRegionId="", overlappingPolygonId="")
		self.__RENDER_LINE_WIDTH = renderLineWidth
		self.__interiorPolygon = Shapely.Polygon(envelope) if interior is None else interior
		self.__envelope = GeometryLib.getGeometryCoords(self.__interiorPolygon) if len(envelope) == 0 else envelope
		self.__DEFAULT_ENVELOPE_COLOR = envelopeColor
		self.__INTERIOR_COLOR = interiorColor
		self.__TEXT_COLOR = ColorNames.BLACK if ColorUtils.isLightColor(interiorColor) else ColorNames.WHITE
		self.__predicates = predicates
		self.__timeNanoSecs = timeNanoSecs
		if len(kwArgs) > 0 : Ros.Log(f"Unassigned keyword args ignored: {repr(kwArgs)}")
		self.__edges: Polygon.Edges = self.__buildEdges()

	def __repr__(self) -> str:
		return self.shortName

	def __buildEdges(self) -> Edges:
		d = {}
		verts = GeometryLib.getGeometryCoords(self.interior)
		for v1, v2 in zip(verts, verts[1:]):
			edgeCoords = [v1, v2]
			edge = Shapely.LineString(edgeCoords)
			d[GeometryLib.lineStringId(edge)] = edge
		return d

	@property
	def timeNanoSecs(self) -> int:
		"""
		The timestamp associated with the region in NanoSeconds.
		A negative time value indicates the polygon is static.
		"""
		return self.__timeNanoSecs

	@timeNanoSecs.setter
	def timeNanoSecs(self, t: int) -> None:
		if self.__timeNanoSecs >= 0 and t < 0:
			Ros.Logger().error(f"Polygon {self.shortName} ignoring attempt to change the time value from {self.__timeNanoSecs} to {t}. Negative time means static polygon.")
			return
		if self.__timeNanoSecs > t:
			Ros.Logger().warn(f"Changing the time value from {self.__timeNanoSecs} to an older time {t}.")
		self.__timeNanoSecs = t
		return

	@property
	def predicates(self) -> list[Msgs.RtBi.Predicate]:
		return self.__predicates

	@property
	def envelope(self) -> GeometryLib.CoordsList:
		"""The list of the coordinates of the vertices ."""
		return self.__envelope

	@property
	def shortName(self) -> str:
		""" Easy to read name. **Do not use it in any meaningful way.**"""
		return f"{self.type.value}-{self.__id.polygonId.split('#')[-1]}"

	@property
	def envelopeColor(self) -> RGBA:
		"""The color used to render the boundary of this polygon."""
		return self.__DEFAULT_ENVELOPE_COLOR

	@property
	def id(self) -> Id:
		return self.__id

	@id.setter
	def id(self, value: Id) -> None:
		self.__id = value
		return

	@property
	def interior(self) -> Shapely.Polygon:
		"""The Geometric description of the region."""
		return self.__interiorPolygon

	@property
	def centroid(self) -> GeometryLib.Coords:
		"""The centroid of the interior."""
		return self.__interiorPolygon.centroid

	@property
	def bounds(self) -> tuple[GeometryLib.Coords, GeometryLib.Coords]:
		"""``[(minX, minY), (maxX, maxY)]``"""
		(minX, minY, maxX, maxY) = self.__interiorPolygon.bounds
		return ((minX, minY), (maxX, maxY))

	@property
	def interiorColor(self) -> RGBA:
		"""The color used for rendering the background of this polygon (to fill)."""
		return self.__INTERIOR_COLOR

	@property
	def edges(self) -> Edges:
		"""
		A dictionary of edge identifier to `Shapely.LineString`.
		The edge identifier is a string.
		"""
		return self.__edges

	@abstractproperty
	def centerOfRotation(self) -> GeometryLib.Coords:
		"""The center of rotation of a polygonal region."""
		...

	def forceUpdateInteriorPolygon(self, polygon: Shapely.Polygon) -> None:
		"""Deliberately did not use a setter to not casually just update the interior."""
		self.__interiorPolygon = polygon
		return

	def edgeId(self, e: Shapely.LineString) -> str | None:
		"""
		Returns the Id of a given edge if it belongs to the region, or `None` otherwise.

		Parameters
		----------
		:param Shapely.LineString e: The edge to receive its Id.
		"""
		idCandidate1 = GeometryLib.lineStringId(e)
		idCandidate2 = GeometryLib.lineStringId(e.reverse())
		if idCandidate1 in self.__edges: return idCandidate1
		if idCandidate2 in self.__edges: return idCandidate2
		return None

	def getEquivalentEdge(self, finalConfig: Shapely.LineString, transformation: AffineTransform) -> Shapely.LineString | None:
		"""
			Given an affine transformation, and the final configuration of an edge after the transformation,
			find the edge that will be in that final configuration after the transformation, and `None` otherwise. Boy didn't I repeat myself?!
		"""
		for edge in self.__edges.values():
			afterTransformation = GeometryLib.applyMatrixTransformToLineString(transformation, edge)
			if GeometryLib.lineSegmentsAreAlmostEqual(finalConfig, afterTransformation): return edge
		return None

	def render(self, durationNs: int = DEFAULT_RENDER_DURATION_NS, renderText: bool = False, envelopeColor: RGBA | None = None) -> Sequence[RViz.Msgs.Marker]:
		msgs = []
		envelopColor = envelopeColor if envelopeColor is not None else self.__DEFAULT_ENVELOPE_COLOR
		marker = RViz.createPolygon(
			self.id,
			GeometryLib.getGeometryCoords(self.interior),
			envelopColor,
			self.__RENDER_LINE_WIDTH,
			durationNs=durationNs,
		)
		msgs.append(marker)
		if renderText:
			textCoords = GeometryLib.getGeometryCoords(self.interior.centroid)[0]
			marker = RViz.createText(
				self.id,
				textCoords,
				self.shortName,
				self.__TEXT_COLOR,
				durationNs=durationNs,
				idSuffix="txt"
			)
			msgs.append(marker)
		return msgs

	def clearRender(self) -> Sequence[RViz.Msgs.Marker]:
		msgs = []
		msgs.append(RViz.removeMarker(self.id))
		msgs.append(RViz.removeMarker(self.id, "txt"))
		return msgs

	def toRegularSpaceMsg(self) -> Msgs.RtBi.RegularSpace:
		msg = Msgs.RtBi.RegularSpace()
		msg.id = self.id
		msg.polygons = [Msgs.toStdPolygon(self.interior)]
		msg.spec.color = ColorNames.toStr(self.envelopeColor)
		msg.spec.name = []
		return msg
