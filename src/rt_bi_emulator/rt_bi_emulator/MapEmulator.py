import rclpy
from sa_msgs.msg import FeatureInfoIndividual
from sa_msgs.srv import QueryFeature

import rt_bi_utils.Ros as RosUtils
from rt_bi_emulator.Map.FeatureInfoIndividual import Case1
from rt_bi_utils.RtBiNode import RtBiNode
from rt_bi_utils.SaMsgs import SaMsgs


class MapEmulator(RtBiNode):
	def __init__(self):
		super().__init__(node_name="rt_bi_emulator_sa_map")
		SaMsgs.createSaFeatureQueryService(self, self.__featureInfoQueryCallback)

	def __featureInfoQueryCallback(self, request: QueryFeature.Request, response: QueryFeature.Response) -> QueryFeature.Response:
		self.log("Received QueryFeature \"%s\" request." % request.name)
		if request.name == "map":
			# The response to this query is all the map regions without their feature definitions
			for feature in Case1.FeatureIndividuals:
				responseFeature = FeatureInfoIndividual()
				responseFeature.feature_name = feature.feature_name
				responseFeature.type = feature.type
				responseFeature.polygon_shape_list = feature.polygon_shape_list
				RosUtils.AppendMessage(response.feature_info_individual, responseFeature)
		else:
			# The response to this query is the feature definition of the given map feature
			for individual in Case1.FeatureIndividuals:
				if individual.feature_name == request.name:
					responseFeature = FeatureInfoIndividual()
					responseFeature.feature_name = individual.feature_name
					responseFeature.traversability_gv_car = individual.traversability_gv_car
					responseFeature.traversability_gv_tank = individual.traversability_gv_tank
					responseFeature.visibility_av= individual.visibility_av
					RosUtils.AppendMessage(response.feature_info_individual, responseFeature)
					break
			if len(response.feature_info_individual) == 0:
				self.get_logger().error("Feature name \"%s\" not in Map Dictionary. **Returning empty feature**" % request.name)
				return response
		self.log("Response has %d features." % len(response.feature_info_individual))
		return response

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseConfigFileParameters(self) -> None:
		return super().parseConfigFileParameters()

	def render(self) -> None:
		return super().render()

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = MapEmulator()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
