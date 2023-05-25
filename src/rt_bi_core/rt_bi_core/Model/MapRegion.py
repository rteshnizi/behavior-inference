from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_utils.Geometry import Geometry
from sa_msgs.msg import PoseArray

class MapRegion(PolygonalRegion):
	def __init__(self, name: str, coords: PoseArray, typeStr: str, featureDefinition: Feature):
		self.isObstacle: bool = featureDefinition.traversability.car < 100
		coordList: Geometry.CoordsList = []
		for i in range(len(coords.traj)):
			coordList.append((coords.traj[i].x, coords.traj[i].y))
		super().__init__(name, coordList, "Grey", "Black" if self.isObstacle else "")
		self.type = typeStr

	def render(self, renderText=False):
		super().render(renderText)
