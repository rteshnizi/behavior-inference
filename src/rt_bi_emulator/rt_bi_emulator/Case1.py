from typing import List, Tuple
from rt_bi_emulator.Av import Av
from rt_bi_utils.SaMsgs import SaMsgs
from rt_bi_utils.Pose import Pose

def getPoses() -> List[Tuple[int, int]]:
	pass

BOTTOM_LEFT = (0, 10)
BOTTOM_RIGHT = (0, 100)
TOP_RIGHT = (500, 100)
TOP_LEFT = (500, 10)

class FeatureInfoIndividual:
	def __init__(
		self,
		polygon_shape_list = SaMsgs.createSaPoseArrayMsg([BOTTOM_LEFT, BOTTOM_RIGHT, TOP_RIGHT, TOP_LEFT]),
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

NANO_CONVERSION_CONSTANT = 10 ** 9
CENTER_OF_ROTATION = (120.148, 133.236)
COORDS_1 = [(200.612, -24.140), (244.985, 008.992), (120.148, 133.236)]
COORDS_2 = [(283.602, 065.354), (296.297, 119.257), (120.804, 134.190)]
class Case1:
	FeatureIndividuals = [
		FeatureInfoIndividual(),
	]
	CENTER_OF_ROTATION = (120.148, 133.236)
	Avs: List[Av] = [
		Av(1, Pose(00 * NANO_CONVERSION_CONSTANT, *COORDS_1[-1], 60), COORDS_1),
		Av(1, Pose(45 * NANO_CONVERSION_CONSTANT, *COORDS_2[-1], 60), COORDS_2),
	]
