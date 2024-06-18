from abc import ABC, abstractmethod
from enum import Enum
from typing import Literal

from rt_bi_commons.Shared.Color import RGBA, ColorNames, ColorUtils
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RViz import RViz

PolygonFactoryKeys = Literal[
	"centerOfRotation",
	"envelope",
	"envelopeColor",
	"hIndex",
	"interior",
	"interiorColor",
	"polygonId",
	"predicates",
	"regionId",
	"renderLineWidth",
	"subPartId",
	"timeNanoSecs",
	"tracklets",
]

class Polygon(ABC):
	"""
		The base class for all polygonal regions.
		Provides much of the structure and functionality.
	"""

	from rt_bi_commons.Utils.Msgs import NANO_CONVERSION_CONSTANT
	from rt_bi_commons.Utils.RViz import DEFAULT_RENDER_DURATION_NS

	Id = NodeId
	"""
	A dictionary of edge identifier to `Shapely.LineString`.
	The edge identifier is a string.
	"""
	class Types(Enum):
		BASE = "B"
		"""Indicates unset value."""
		DYNAMIC = Msgs.RtBi.RegularSet.DYNAMIC
		AFFINE = Msgs.RtBi.RegularSet.AFFINE
		SENSING = Msgs.RtBi.RegularSet.SENSING
		SHADOW = "X"
		STATIC = Msgs.RtBi.RegularSet.STATIC
		TARGET = Msgs.RtBi.RegularSet.TARGET

	type = Types.BASE
	Type = Literal[Types.BASE]

	def __init__(
			self,
			polygonId: str,
			regionId: str,
			subPartId: str,
			envelope: GeometryLib.CoordsList,
			envelopeColor: RGBA,
			predicates: Predicates,
			timeNanoSecs: int,
			hIndex: int,
			interiorColor: RGBA = ColorNames.GREY_DARK,
			interior: Shapely.Polygon | None = None,
			renderLineWidth = 2.5,
			**kwArgs
		) -> None:
		"""
		:param str polygonId: Id of the polygon.
		:param str regionId: Id of the regular region owning this polygon.
		:param CoordsList envelope: The list of the coordinates of the vertices of the envelope of the polygonal region.
		:param RGBA envelopeColor: The color of the envelope when/if rendered.
		:param predicates: The predicates associated with this polygon, defaults to ``[]``.
		:type predicates: `list[Msgs.RtBi.Predicate]` or `Predicates`
		:param int timeNanoSecs: Time of the predicate evaluations. A negative value indicates unset.
		:param RGBA interiorColor: The color of the interior of the region when/if rendered, defaults to `ColorNames.GREY_DARK`.
		:param interior: The interior of the region, if it is separate from its envelope, `None` forces construction. defaults to `None`.
		:type interior: `Shapely.Polygon` or `None`
		:param int renderLineWidth: The width of the rendered lines, defaults to ``1``.
		"""
		self.__id = Polygon.Id(
			regionId=regionId,
			polygonId=polygonId,
			subPartId=subPartId,
			timeNanoSecs=timeNanoSecs,
			hIndex=hIndex,
		)
		self.__RENDER_LINE_WIDTH = renderLineWidth
		self.__interiorPolygon = Shapely.Polygon(envelope) if interior is None else interior
		self.__interiorPolygon = Shapely.set_precision(self.__interiorPolygon, GeometryLib.EPSILON)
		self.__envelope = GeometryLib.getGeometryCoords(self.__interiorPolygon) if len(envelope) == 0 else envelope
		self.__DEFAULT_ENVELOPE_COLOR = envelopeColor
		self.__INTERIOR_COLOR = interiorColor
		self.__TEXT_COLOR = ColorNames.BLACK if ColorUtils.isLightColor(interiorColor) else ColorNames.WHITE
		self.__predicates = predicates
		if len(kwArgs) > 0 : Ros.Log(f"Unassigned keyword args ignored: {repr(kwArgs)}")
		self.__edges: list[Shapely.LineString] = []
		self.__buildEdges()

	def __repr__(self) -> str:
		accState = "O" if self.isAccessible else "|"
		return f"{self.shortName}[{accState}]"

	def __buildEdges(self) -> None:
		verts = GeometryLib.getGeometryCoords(self.interior)
		for v1, v2 in zip(verts, verts[1:]):
			edgeCoords = [v1, v2]
			self.__edges.append(Shapely.LineString(edgeCoords))
		return

	@property
	def timeNanoSecs(self) -> int:
		"""
		The timestamp associated with the region in NanoSeconds.
		A negative time value indicates the polygon is static.
		"""
		return self.id.timeNanoSecs

	@timeNanoSecs.setter
	def timeNanoSecs(self, t: int) -> None:
		if self.id.timeNanoSecs >= 0 and t < 0:
			Ros.Logger().error(f"Polygon {self.shortName} ignoring attempt to change the time value from {self.id.timeNanoSecs} to {t}. Negative time means static polygon.")
			return
		if self.id.timeNanoSecs > t:
			Ros.Logger().warn(f"Changing the time value from {self.id.timeNanoSecs} to an older time {t}.")
		self.__id = Polygon.Id(
				regionId=self.__id.regionId,
				polygonId=self.__id.polygonId,
				subPartId=self.__id.subPartId,
				timeNanoSecs=t,
				hIndex=self.id.hIndex,
			)
		return

	@property
	def predicates(self) -> Predicates:
		return self.__predicates

	@predicates.setter
	def predicates(self, val: Predicates) -> None:
		self.__predicates = val

	@property
	def isAccessible(self) -> bool:
		return "accessible" not in self.predicates or self.predicates["accessible"]

	@property
	def envelope(self) -> GeometryLib.CoordsList:
		"""The list of the coordinates of the vertices ."""
		return self.__envelope

	@property
	def shortName(self) -> str:
		""" Easy to read name. **Do not use it in any meaningful way.**"""
		(regionId, polyId) = self.__id.shortNames()
		# hIndex = f"[{self.id.hIndex}]" if self.id.hIndex >= 0 else ""
		hIndex = f"[{self.id.hIndex}]"
		subpart = self.__id.subPartId
		return f"{self.type.value}{hIndex}-{regionId}-{polyId}-{subpart}"

	@property
	def envelopeColor(self) -> RGBA:
		"""The color used to render the boundary of this polygon."""
		return self.__DEFAULT_ENVELOPE_COLOR

	@property
	def id(self) -> Id:
		return self.__id

	@id.setter
	def id(self, value: Id) -> None:
		assert (
			self.id.polygonId == value.polygonId and
			self.id.regionId == value.regionId
		), f"Changing polygon and region Ids is not allowed. Existing = {repr(self.id)} vs New = {repr(value)}"
		self.__id = value
		return

	@property
	def interior(self) -> Shapely.Polygon:
		"""The Geometric description of the region."""
		return self.__interiorPolygon

	@property
	def centroid(self) -> GeometryLib.Coords:
		"""The centroid of the interior."""
		p: Shapely.Point = self.__interiorPolygon.centroid
		if p.is_empty:
			Ros.Logger().error(f"Attempted to get centroid of the empty polygon {repr(self.id)}.")
			return (-25.0, -25.0)
		return GeometryLib.toCoords(p)

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
	def hasTrack(self) -> bool:
		return False

	@property
	def edges(self) -> list[Shapely.LineString]:
		"""
		A dictionary of edge identifier to `Shapely.LineString`.
		The edge identifier is a string.
		"""
		return self.__edges

	def intersects(self, other: "Polygon") -> bool:
		return GeometryLib.intersects(self.interior, other.interior)

	def hasCommonEdge(self, other: "Polygon") -> bool:
		return GeometryLib.haveOverlappingEdge(self.interior, other.interior)

	@property
	@abstractmethod
	def centerOfRotation(self) -> GeometryLib.Coords:
		"""The center of rotation of a polygonal region."""
		...

	def getEquivalentEdge(self, finalConfig: Shapely.LineString, transformation: AffineTransform) -> Shapely.LineString | None:
		"""
			Given an affine transformation, and the final configuration of an edge after the transformation,
			find the edge that will be in that final configuration after the transformation, and `None` otherwise. Boy didn't I repeat myself?!
		"""
		for edge in self.edges:
			afterTransformation = GeometryLib.applyMatrixTransformToLineString(transformation, edge)
			if GeometryLib.lineSegmentsAreAlmostEqual(finalConfig, afterTransformation): return edge
		return None

	def createMarkers(self, durationNs: int = DEFAULT_RENDER_DURATION_NS, renderText: bool = False, envelopeColor: RGBA | None = None, stamped: bool = True) -> list[RViz.Msgs.Marker]:
		"""Creates all relevant markers for a given polygon."""
		msgs = []
		if Ros.IsProfiling(): return msgs
		coords = GeometryLib.getGeometryCoords(self.interior)
		if len(coords) == 0:
			Ros.Logger().error(f"Empty polygon detected in createMarkers(): {self.shortName}")
			return msgs
		envelopColor = envelopeColor if envelopeColor is not None else self.envelopeColor
		unstampedId = self.id if stamped else self.id.copy(timeNanoSecs=-1, hIndex=-1)
		marker = RViz.createPolygon(
			unstampedId,
			coords,
			envelopColor,
			self.__RENDER_LINE_WIDTH,
			durationNs=durationNs,
		)
		msgs.append(marker)
		if renderText:
			textCoords = self.centroid
			marker = RViz.createText(
				unstampedId,
				textCoords,
				self.shortName,
				self.__TEXT_COLOR,
				durationNs=durationNs,
				idSuffix="txt"
			)
			msgs.append(marker)
		return msgs

	def deleteMarkers(self) -> list[RViz.Msgs.Marker]:
		"""Create markers to delete all markers.

		:return: Markers set to delete.
		:rtype: `list[RViz.Msgs.Marker]`
		"""
		msgs = []
		msgs.append(RViz.removeMarker(self.id))
		msgs.append(RViz.removeMarker(self.id, idSuffix="txt"))
		return msgs

	def asRegularSetMsg(self) -> Msgs.RtBi.RegularSet:
		msg = Msgs.RtBi.RegularSet()
		msg.id = self.id.regionId
		msg.stamp = Msgs.toTimeMsg(self.timeNanoSecs)
		msg.predicates = self.predicates.asMsgArr()
		polyMsg = Msgs.RtBi.Polygon()
		polyMsg.id = self.id.polygonId
		polyMsg.region = Msgs.toPolygonMsg(self.interior)
		polyMsg.center_of_rotation = Msgs.toPointMsg(self.centerOfRotation)
		msg.polygons = [polyMsg]
		msg.set_type = self.type.value
		# msg.color = ColorNames.toStr(self.envelopeColor)
		# msg.name = []
		return msg
