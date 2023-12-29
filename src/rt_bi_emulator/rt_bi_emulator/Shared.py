
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Pose import Pose


class Target:
	def __init__(self, id: int, location: Pose, spatialRegion: Geometry.CoordsList) -> None:
		self.id: int = id
		self.location = location
		self.spatialRegion = spatialRegion

	def __repr__(self) -> str:
		name = "#%d" % self.id
		return "TG-%s:%s" % (name, repr(self.location))
