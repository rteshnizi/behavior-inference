from shapely.geometry import Polygon
from sa_bil.core.model.PolygonalRegion import PolygonalRegion
from sa_bil.core.observation.track import Tracks
from sa_bil.core.utils.geometry import Geometry

COLOR_PALETTE = ["Green", "Purple", "Gold"]
NUM_COLORS = len(COLOR_PALETTE)

class SensingRegion(PolygonalRegion):
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, timestamp: float, idNum: int, polygon: Polygon = None, tracks: Tracks = {}):
		super().__init__(name, coords, COLOR_PALETTE[idNum % NUM_COLORS], polygon=polygon)
		self.time = timestamp
		self.tracks = tracks
		self._renderLineWidth = 4

	def __repr__(self):
		return "%s-%.2f" % (super().__repr__(), self.time)

	@property
	def containsTracks(self) -> bool:
		return len(self.tracks) > 0

	def render(self, canvas):
		super().render(canvas, False)
		for trackId in self.tracks:
			track = self.tracks[trackId]
			track.render(canvas)

	def clearRender(self, canvas):
		for trackId in self.tracks:
			track = self.tracks[trackId]
			track.clearRender(canvas)
		return super().clearRender(canvas)
