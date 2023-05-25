from sa_bil.core.model.PolygonalRegion import PolygonalRegion
from sa_bil.core.model.featureMap import Feature
from sa_bil.core.utils.geometry import Geometry
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
