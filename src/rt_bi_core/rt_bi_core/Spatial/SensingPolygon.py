from typing import Literal, Sequence

from visualization_msgs.msg import Marker

import rt_bi_commons.Utils.Ros as Ros
from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.Tracklet import Tracklet


class SensingPolygon(AffinePolygon):
	type: Literal[AffinePolygon.Types.SENSING] = AffinePolygon.Types.SENSING
	"""
	An Affine Polygon within which we are able to observe moving targets.
	"""
	def __init__(self, tracklets: dict[str, Tracklet] = {}, **kwArgs) -> None:
		"""
		:param dict tracklets: Observed moving objects inside a sensing polygon, defaults to ``{}``.
		:type tracklets: `dict[AffinePolygon.Id, Tracklet]`
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
		:param list overlappingRegionIds: The id of the static regions which overlaps with this region, defaults to `[]`.
		:type overlappingRegionIds: list[str]
		:param Types overlappingRegionTypes: The id of the region which overlaps with this region, defaults to `Types.BASE`

		.. seealso::
			For kwArgs see
			* :class:`rt_bi_core.Polygons.AffinePolygon.AffinePolygon`
			* :class:`rt_bi_core.Polygons.DynamicPolygon.DynamicPolygon`
			* :class:`rt_bi_core.Polygons.Polygon.Polygon`
		"""
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.GREEN_LIGHT),
			**kwArgs
		)
		self.__tracklets: dict[str, Tracklet] = {}
		for i in tracklets:
			if not tracklets[i].vanished:
				self.__tracklets[i] = tracklets[i]

	@property
	def tracklets(self) -> dict[str, Tracklet]:
		"""The information about every tracklet, if any, within this sensing region."""
		return self.__tracklets

	@property
	def hasTrack(self) -> bool:
		"""
		Whether this sensing region contains any track observations information.

		:return: `True` if there is any tracklets information, `False` otherwise.
		:rtype: bool
		"""
		return len(self.__tracklets) > 0

	def render(self, renderTracklet = True, envelopeColor: RGBA | None = None, durationNs: int = AffinePolygon.DEFAULT_RENDER_DURATION_NS) -> Sequence[Marker]:
		msgs = super().render(renderText=False, envelopeColor=envelopeColor, durationNs=durationNs)
		if renderTracklet:
			for i in self.__tracklets: Ros.ConcatMessageArray(msgs, self.__tracklets[i].render(durationNs=durationNs))
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = super().clearRender()
		for tracklet in self.__tracklets: Ros.ConcatMessageArray(msgs, self.__tracklets[tracklet].clearRender())
		return msgs
