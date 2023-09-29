from rt_bi_utils.SaMsgs import SaMsgs

BOTTOM_LEFT = (0, 10)
BOTTOM_RIGHT = (0, 100)
TOP_RIGHT = (500, 100)
TOP_LEFT = (500, 10)

class FeatureInfoIndividualPy:
	"""This class represents the data model used in the TAMU SA Semantic Map."""
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

class Case1:
	FeatureIndividuals = [
		FeatureInfoIndividualPy(),
	]
