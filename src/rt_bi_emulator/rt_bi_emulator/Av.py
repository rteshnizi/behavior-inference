from rt_bi_utils.Pose import Pose
from rt_bi_utils.Geometry import Geometry


class Av:
	def __init__(self, id: int, p: Pose, fov: Geometry.CoordsList) -> None:
		self.robotId: int = id
		self.pose = p
		self.fov = fov
