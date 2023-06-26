from typing import List
from rt_bi_utils.RViz import KnownColors
from visualization_msgs.msg import Marker

from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry
from sa_msgs.msg import PoseArray as SaPoseArray


class MapRegion(PolygonalRegion):
	def __init__(self, name: str, coords: SaPoseArray, typeStr: str, featureDefinition: Feature):
		self.isObstacle: bool = featureDefinition.traversability.car < 100
		coordList: Geometry.CoordsList = []
		for i in range(len(coords.traj)):
			coordList.append((coords.traj[i].x, coords.traj[i].y))
		super().__init__(name, coordList, KnownColors.GREY, KnownColors.BLACK if self.isObstacle else KnownColors.TRANSPARENT)
		self.type = typeStr

	def render(self, renderText = False) -> List[Marker]:
		return super().render(renderText, fill=True)
