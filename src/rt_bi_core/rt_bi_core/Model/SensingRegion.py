from typing import List

from shapely.geometry import Polygon
from rt_bi_utils.RViz import KnownColors
from visualization_msgs.msg import Marker

from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.Track import Tracks
from rt_bi_utils.Geometry import Geometry

COLOR_PALETTE = ["Green", "Purple", "Gold"]
NUM_COLORS = len(COLOR_PALETTE)

class SensingRegion(PolygonalRegion):
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, timeNanoSecs: float, idNum: int, polygon: Polygon = None, tracks: Tracks = {}) -> None:
		super().__init__(name, coords, KnownColors.GREEN, polygon=polygon)
		self.timeNanoSecs: float = float(timeNanoSecs)
		self.tracks = tracks
		self._renderLineWidth = 4

	def __repr__(self):
		return "%s-%.2f" % (super().__repr__(), self.timeNanoSecs)

	@property
	def containsTracks(self) -> bool:
		return len(self.tracks) > 0

	def render(self) -> List[Marker]:
		msg = super().render(False)
		for trackId in self.tracks:
			track = self.tracks[trackId]
			msg += track.render()
		return msg

	def clearRender(self) -> None:
		return super().clearRender()
