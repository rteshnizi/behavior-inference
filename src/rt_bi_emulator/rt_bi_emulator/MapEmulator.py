import rclpy
from rclpy.node import Node, Publisher, Service
from rclpy.duration import Duration

import rt_bi_utils.Ros as RosUtils
from rt_bi_emulator.Data.Case1 import Case1
from sa_msgs.msg import FeatureInfoIndividual
from sa_msgs.srv import QueryFeature


class MapEmulator(Node):
	""" The Viewer ROS Node """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_emulator")
		self.get_logger().info("Map Emulator is starting...")
		self.__MAP_SERVICE_NAME = "/sa_map/feature_query"
		self.__mapService: Service
		self.__createServices()

	def __createServices(self) -> None:
		self.__mapService = self.create_service(QueryFeature, self.__MAP_SERVICE_NAME, self.__featureInfoQueryResponse)
		return

	def __featureInfoQueryResponse(self, request: QueryFeature.Request, response: QueryFeature.Response) -> QueryFeature.Response:
		RosUtils.Logger().info("Received %s service request..." % self.__MAP_SERVICE_NAME)
		if request == "map":
			# The response to this query is all the map regions without their feature definitions
			for feature in Case1.FeatureIndividuals:
				responseFeature = FeatureInfoIndividual()
				responseFeature.feature_name = feature.feature_name
				responseFeature.type = feature.type
				responseFeature.polygon_shape_list = feature.polygon_shape_list
				response.feature_info_individual.append(responseFeature)
		else:
			# The response to this query is the feature definition of the given map feature
			feature = next(filter((lambda f: f.feature_name == request), Case1.FeatureIndividuals), None)
			if feature is not None:
				responseFeature = FeatureInfoIndividual()
				responseFeature.traversability_gv_car = feature.traversability_gv_car
				responseFeature.traversability_gv_tank = feature.traversability_gv_tank
				responseFeature.visibility_av= feature.visibility_av
				response.feature_info_individual.append(responseFeature)
		RosUtils.Logger().info("Response has %d features." % len(response.feature_info_individual))
		return response


def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	mapNode = MapEmulator()
	rclpy.spin(mapNode)
	mapNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
