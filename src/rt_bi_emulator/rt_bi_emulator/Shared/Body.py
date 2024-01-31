from rt_bi_commons.Shared.Pose import Pose
from rt_bi_commons.Utils.Geometry import Geometry


class Body:
	def __init__(self, id: int, location: Pose, centerOfRotation: Geometry.Coords, spatialRegion: Geometry.CoordsList) -> None:
		self.id: int = id
		self.location = location
		self.spatialRegion = spatialRegion
		self.centerOfRotation = centerOfRotation

	def __repr__(self) -> str:
		name = "#%d" % self.id
		return "BD-%s:%s" % (name, repr(self.location))
