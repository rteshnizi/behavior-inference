from typing import List, Sequence, Union

from visualization_msgs.msg import Marker

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.Tracklet import Tracklet
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import Color, KnownColors

COLOR_PALETTE = ["Green", "Purple", "Gold"]
NUM_COLORS = len(COLOR_PALETTE)

class SensorRegion(AffineRegion):
	"""
	An affine sensor region.
	"""
	def __init__(
			self,
			centerOfRotation: Pose,
			idNum: int,
			envelope: Geometry.CoordsList,
			timeNanoSecs: int,
			interior: Union[Polygon, MultiPolygon, None] = None,
			tracks: List[Tracklet] = [],
			**kwArgs
		) -> None:
		"""
		Initialize a sensor region.

		Parameters
		----------
		centerOfRotation : Pose
			This will be used to interpolate the rotation motion of the region.
		idNum : int
			Id of the sensor region.
		envelope : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelope of the polygonal region.
		envelopeColor: Color
			The color of the envelope when/if rendered.
		regionType: RegionType, default `RegionType.BASE`
			The type of this region.
		timeNanoSecs: int
			A timestamp representing absolute value of time in nanosecond (ns).
		fov: Union[Polygon, MultiPolygon, None], default `None`
			The field-of-view, default forces construction using knowledge-base.
		tracks : Tracks, default `{}`
			The tracklets, as defined in the dissertation, observable in the fov,
			default forces construction using knowledge-base.
		"""
		super().__init__(
			centerOfRotation=centerOfRotation,
			idNum=idNum,
			envelope=envelope,
			envelopeColor=KnownColors.GREEN,
			regionType=AffineRegion.RegionType.SENSING,
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)
		self.__tracks = tracks

	@property
	def tracks(self) -> List[Tracklet]:
		"""The information about every tracklet, if any, within this sensing region."""
		return self.__tracks

	@property
	def hasTrack(self) -> bool:
		"""
		Whether this sensing region contains any track observations information.

		Returns
		-------
		bool
			`True` if there is any tracklets information, `False` otherwise.
		"""
		return len(self.__tracks) > 0

	def render(self, envelopeColor: Union[Color, None] = None) -> Sequence[Marker]:
		msgs = super().render(renderText=False, envelopeColor=envelopeColor)
		for tracklet in self.__tracks: RosUtils.ConcatMessageArray(msgs, tracklet.render())
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = super().clearRender()
		for tracklet in self.__tracks: RosUtils.ConcatMessageArray(msgs, tracklet.clearRender())
		return msgs
