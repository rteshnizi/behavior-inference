from typing import List, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.FeatureMap import FeatureMap
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.RViz import KnownColors

COLOR_PALETTE = ["Green", "Purple", "Gold"]
NUM_COLORS = len(COLOR_PALETTE)

class SensorRegion(PolygonalRegion):
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(
			self,
			idNum: int,
			envelope: Geometry.CoordsList,
			fov: Union[Polygon, MultiPolygon, None] = None,
			tracks: Tracklets = {},
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
		fov: Union[Polygon, MultiPolygon, None], default `None`
			The field-of-view, default forces construction using knowledge-base.
		tracks : Tracks, default `{}`
			The tracklets, as defined in the dissertation, observable in the fov,
			default forces construction using knowledge-base.
		"""
		super().__init__(
			idNum=idNum,
			envelope=envelope,
			envelopeColor=KnownColors.GREEN,
			interior=fov,
			regionType=PolygonalRegion.RegionType.SENSING,
			**kwArgs
		)
		self.__tracks = tracks

	def __repr__(self):
		return "%s@%.2f" % (super().__repr__(), self.timeNanoSecs)

	@property
	def fov(self) -> Union[Polygon, MultiPolygon]:
		"""The FOV."""
		return self.interior

	@property
	def tracks(self) -> Tracklets:
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

	def updateVisibilityPolygon(self, regions: List[PolygonalRegion], featureMap: FeatureMap) -> Geometry.CoordsList:
		# sortedCoords = Geometry.sortCoordinatesClockwise(self._originalCoords)
		sortedCoords = self.__originalCoords
		polygon = Polygon(sortedCoords)
		for region in regions:
			# FIXME: Currently, the type of sensor is missing.
			# Once its available you should check the type and see which type of seeThrough I should look for
			if featureMap.features[region.type].visibleFromAbove: continue
			nextPolygon = Geometry.subtract(polygon, region.interior)
			polygon = nextPolygon
		return polygon

	def render(self) -> List[Marker]:
		msg = super().render(False)
		for trackId in self.__tracks:
			track = self.__tracks[trackId]
			msg += track.render()
		return msg

	def clearRender(self) -> None:
		return super().clearRender()
