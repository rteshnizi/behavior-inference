from abc import ABC

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Shared.Pose import Coords, CoordsList
from rt_bi_core.Spatial.Polygon import Polygon


class AffinePolygon(Polygon, ABC):
	"""The base class for all moving polygons."""
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: CoordsList,
			envelopeColor: RGBA,
			centerOfRotation: Coords,
			overlappingPolygonId: Polygon.Id = Polygon.Id("", "", "", ""),
			overlappingPolygonType: Polygon.Types = Polygon.Types.BASE,
			**kwArgs
		) -> None:
		"""
		:param str polygonId: Id of the polygon.
		:param str regionId: Id of the regular region owning this polygon.
		:param CoordsList envelope: The list of the coordinates of the vertices of the envelope of the polygonal region.
		:param RGBA envelopeColor: The color of the envelope when/if rendered.
		:param Coords centerOfRotation: This will be used to interpolate the rotation motion of the region.
		:param RGBA interiorColor: The color of the interior of the region when/if rendered, defaults to `ColorNames.GREY_DARK`.
		:param interior: The interior of the region, if it is separate from its envelope, `None` forces construction. defaults to `None`.
		:type interior: `Shapely.Polygon` or `None`
		:param int renderLineWidth: The width of the rendered lines, defaults to ``1``.
		:param list predicates: The predicates associated with this polygon, defaults to ``[]``.
		:type predicates: `list[Msgs.RtBi.Predicate]`
		:param int timeNanoSecs: Time of the predicate evaluations, defaults to ``-1`` which indicates the polygon is static.
		:param overlappingRegionId: The id of the static regions which overlaps with this region, defaults to `[]`.
		:type overlappingRegionId: Polygon.Id
		:param Types overlappingRegionType: The id of the region which overlaps with this region, defaults to `Types.BASE`

		.. seealso::
			:class:`rt_bi_core.Polygons.DynamicPolygon.DynamicPolygon`.
			:class:`rt_bi_core.Polygons.Polygon.Polygon`.
		"""
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			envelope=envelope,
			envelopeColor=envelopeColor,
			**kwArgs
		)
		self.id = Polygon.Id(self.id.regionId, self.id.polygonId, overlappingPolygonId.regionId, overlappingPolygonId.polygonId)
		self.overlappingPolygonType = overlappingPolygonType
		self.__centerOfRotation = centerOfRotation

	@property
	def centerOfRotation(self) -> Coords:
		return self.__centerOfRotation
