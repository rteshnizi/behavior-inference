from typing import Literal, Sequence

from visualization_msgs.msg import Marker

import rt_bi_commons.Utils.Ros as RosUtils
from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.Tracklet import Tracklet
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.Polygons.AffinePolygon import AffinePolygon


class SensingPolygon(AffinePolygon):
	"""
	An Affine Polygon within which we are able to observe moving targets.
	"""
	def __init__(self, tracklets: dict[str, Tracklet] = {}, **kwArgs) -> None:
		"""
		:param dict tracklets: Observed moving objects inside a sensing polygon, defaults to ``{}``.
		:type tracklets: `dict[str, Tracklet]`

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

	@property
	def regionType(self) -> Literal[AffinePolygon.Types.SENSING]:
		return self.Types.SENSING

	def render(self, renderTracklet = True, envelopeColor: RGBA | None = None, durationNs: int = AffinePolygon.DEFAULT_RENDER_DURATION_NS) -> Sequence[Marker]:
		msgs = super().render(renderText=False, envelopeColor=envelopeColor, durationNs=durationNs)
		if renderTracklet:
			for i in self.__tracklets: RosUtils.ConcatMessageArray(msgs, self.__tracklets[i].render(durationNs=durationNs))
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = super().clearRender()
		for tracklet in self.__tracklets: RosUtils.ConcatMessageArray(msgs, self.__tracklets[tracklet].clearRender())
		return msgs
