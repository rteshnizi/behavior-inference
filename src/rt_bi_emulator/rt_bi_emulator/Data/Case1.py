from typing import List, Tuple
from rt_bi_utils.SaMsgs import SaMsgs

def getPoses() -> List[Tuple[int, int]]:
	pass

class FeatureInfoIndividual:
	def __init__(
		self,
		polygon_shape_list = SaMsgs.createSaPoseArrayMessage([(0, 10), (0, 100), (500, 100), (500, 10)]),
		feature_name = "BallPark",
		traversability_gv_car = 100,
		traversability_gv_tank = 100,
		visibility_av = "yes",
		type = "BallPark",
	) -> None:
		self.polygon_shape_list = polygon_shape_list
		self.feature_name = feature_name
		self.traversability_gv_car = traversability_gv_car
		self.traversability_gv_tank = traversability_gv_tank
		self.visibility_av = visibility_av
		self.type = type

class Case1:
	FeatureIndividuals = [
		FeatureInfoIndividual(),
	]
