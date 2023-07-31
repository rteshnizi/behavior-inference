import rclpy
from rclpy.node import Node

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.SaMsgs import SaMsgs
from rt_bi_emulator.Case1 import Case1
from sa_msgs.msg import FeatureInfoIndividual
from sa_msgs.srv import QueryFeature


class MapEmulator(Node):
	""" The Viewer ROS Node """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_map_emulator")
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		SaMsgs.createSaFeatureQueryService(self, self.__featureInfoQueryCallback)

	def __featureInfoQueryCallback(self, request: QueryFeature.Request, response: QueryFeature.Response) -> QueryFeature.Response:
		RosUtils.Logger().info("Received QueryFeature \"%s\" request..." % request.name)
		if request.name == "map":
			# The response to this query is all the map regions without their feature definitions
			for feature in Case1.FeatureIndividuals:
				responseFeature = FeatureInfoIndividual()
				responseFeature.feature_name = feature.feature_name
				responseFeature.type = feature.type
				responseFeature.polygon_shape_list = feature.polygon_shape_list
				response.feature_info_individual.append(responseFeature)
		else:
			# The response to this query is the feature definition of the given map feature
			for individual in Case1.FeatureIndividuals:
				if individual.feature_name == request.name:
					responseFeature = FeatureInfoIndividual()
					responseFeature.feature_name = individual.feature_name
					responseFeature.traversability_gv_car = individual.traversability_gv_car
					responseFeature.traversability_gv_tank = individual.traversability_gv_tank
					responseFeature.visibility_av= individual.visibility_av
					response.feature_info_individual.append(responseFeature)
					break
			if len(response.feature_info_individual) == 0:
				self.get_logger().error("Feature name \"%s\" not in Map Dictionary. **Returning empty feature**" % request.name)
				return FeatureInfoIndividual()
		self.get_logger().info("Response has %d features." % len(response.feature_info_individual))
		return response


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
