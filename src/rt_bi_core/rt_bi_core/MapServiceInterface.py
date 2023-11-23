from functools import partial
from typing import Dict, List, Sequence

import rclpy
from rclpy.clock import Duration
from rclpy.node import Client, Node
from sa_msgs.msg import FeatureInfoIndividual
from sa_msgs.srv import QueryFeature
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs


class MapServiceInterface(Node):
	""" This Node listens to all the messages published on the topics related to the map and renders them. """
	def __init__(self, **kwArgs):
		""" Create a Viewer ROS node. """
		newKw = { "node_name": "rt_bi_core_map", **kwArgs}
		super().__init__(**newKw)
		self.MAP_QUERY_NAME = "map"
		self.__strNameToIdNum: Dict[str, int] = dict()
		"""This map helps us assume nothing about the meaning of the names assigned to map regions."""
		if self.__class__.__name__ == MapServiceInterface.__name__:
			self.get_logger().debug("%s is initializing." % self.get_fully_qualified_name())
			RosUtils.SetLogger(self.get_logger())
			self.__regions: List[MapRegion] = []
			self.mapClient = SaMsgs.createSaFeatureQueryClient(self)
			(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))
		else:
			self.get_logger().debug("%s finished super class service init." % self.get_fully_qualified_name())

	def __registerRegionId(self, featureName: str) -> int:
		if featureName in self.__strNameToIdNum:
			self.get_logger().error("Duplicate feature name encountered: %s... region will be ignored." % featureName)
			return self.__strNameToIdNum[featureName]
		idNum = len(self.__strNameToIdNum)
		self.__strNameToIdNum[featureName] = idNum
		return idNum

	def __parseFeatureDefinition(self, region: MapRegion, request: QueryFeature.Request, response: QueryFeature.Response) -> List[Feature]:
		parsedFeatures: List[Feature] = []
		if not isinstance(response.feature_info_individual, Sequence): return parsedFeatures
		if len(response.feature_info_individual) > 1:
			raise RuntimeError("Currently we parse a single feature at a time.")
		individualFeature = response.feature_info_individual[0]
		visibilityAv = individualFeature.visibility_av
		traversabilityCar = individualFeature.traversability_gv_car
		traversabilityTank = individualFeature.traversability_gv_tank
		feature = Feature(request.name, {
				"visibility_av": visibilityAv,
				"traversability_gv_car": traversabilityCar,
				"traversability_gv_tank": traversabilityTank,
		})
		region.featureDefinition = feature
		self.get_logger().debug("Assigned feature definition \"%s\" to region %s." % (request.name, repr(region)))
		parsedFeatures.append(feature)
		return parsedFeatures

	def parsePolygonShapeList(self, request: QueryFeature.Request, response: QueryFeature.Response, queryFeatures: bool = True) -> List[MapRegion]:
		self.get_logger().debug("Parsing response to \"%s\" query." % request.name)
		if request.name != self.MAP_QUERY_NAME:
			self.get_logger().error("Ignoring response to query %s. Cannot be parsed via %s" % (request.name, self.parsePolygonShapeList.__name__))
			return []
		regions: List[MapRegion] = []
		individualFeature: FeatureInfoIndividual
		for individualFeature in response.feature_info_individual:
			fName = individualFeature.feature_name
			poseArray = individualFeature.polygon_shape_list
			coords = SaMsgs.convertSaPoseListToCoordsList(poseArray.traj)
			idNum = self.__registerRegionId(fName)
			region = MapRegion(idNum=idNum, envelope=coords, timeNanoSecs=-1)
			regions.append(region)
			if queryFeatures:
				subRequest = QueryFeature.Request()
				subRequest.name = fName
				self.get_logger().debug("Querying feature definition for \"%s\"." % subRequest.name)
				boundParser = partial(self.__parseFeatureDefinition, region)
				RosUtils.SendClientRequest(self, self.mapClient, subRequest, boundParser)

		self.get_logger().debug("Parsed %d items." % len(regions))
		if isinstance(self, MapServiceInterface):
			self.__regions= regions
		return regions

	def render(self) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().info("Skipping map render... RViz is not ready yet to receive messages.")
			return
		if len(self.__regions) == 0:
			self.get_logger().info("Skipping map render... No regions to render.")
			return
		message = MarkerArray()
		for region in self.__regions:
			RosUtils.ConcatMessageArray(message.markers, region.render())
		self.__rvizPublisher.publish(message)
		return

	def requestMap(self, mapClient: Client) -> None:
		self.get_logger().debug("Initializing map polygons for %s." % self.get_fully_qualified_name())
		if not mapClient.service_is_ready():
			RosUtils.WaitForServicesToStart(self, mapClient)
		request = QueryFeature.Request()
		request.name = self.MAP_QUERY_NAME
		RosUtils.SendClientRequest(self, mapClient, request, self.parsePolygonShapeList)
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = MapServiceInterface()
	node.requestMap(node.mapClient)
	while node.context.ok():
		node.get_clock().sleep_for(Duration(seconds=2, nanoseconds=0))
		node.render()
		node.requestMap(node.mapClient)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
