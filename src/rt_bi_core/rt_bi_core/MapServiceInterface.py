from typing import Dict, List, Union

import rclpy
from rclpy.clock import Duration
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry, Polygon
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs
from sa_msgs.msg import FeatureInfoIndividual
from sa_msgs.srv import QueryFeature


class MapServiceInterface(Node):
	""" This Node listens to all the messages published on the topics related to the map and renders them. """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_core_map")
		self.__MAP_QUERY_NAME = "map"
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__regions: Union[Dict[str, PolygonalRegion], None] = None
		self.__regionNames: List[str] = []
		self.__polygon: Union[Polygon, None] = None
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self)
		self.__mapClient = SaMsgs.createSaFeatureQueryClient(self)

	@property
	def regions(self) -> Dict[str, PolygonalRegion]:
		if self.__regions is not None: return self.__regions
		return {}

	@property
	def polygon(self) -> Polygon:
		if self.__polygon is None:
			polygons = [self.regions[r].polygon for r in self.regions]
			self.__polygon = Geometry.union(polygons)
		return self.__polygon

	def __parseFeatureQueryResponse(self, request: QueryFeature.Request, response: QueryFeature.Response) -> None:
		self.get_logger().info("Parsing response to %s query..." % request.name)
		if request.name == self.__MAP_QUERY_NAME:
			self.__parsePolygonShapeList(response)
			self.__render()
		else:
			self.__parseFeatureDefinition(request.name, response)
		return

	def __parsePolygonShapeList(self, response: QueryFeature.Response) -> None:
		regions = {}
		individualFeature: FeatureInfoIndividual
		i = 0
		for individualFeature in response.feature_info_individual:
			fName = individualFeature.feature_name
			coords = individualFeature.polygon_shape_list
			# FIXME: Make sure this is correct with Anant
			regions[fName] = MapRegion(fName, coords, fName)
			self.__regionNames.append(fName)
			i += 1
		self.get_logger().info("Parsed %d items." % i)
		self.__regions = regions
		return

	def __parseFeatureDefinition(self, featureName: str, response: QueryFeature.Response) -> List[PolygonalRegion]:
		individualFeature: FeatureInfoIndividual
		updatedRegions: List[PolygonalRegion] = []
		for individualFeature in response.feature_info_individual:
			visibilityAv = individualFeature.visibility_av
			traversabilityCar = individualFeature.traversability_gv_car
			traversabilityTank = individualFeature.traversability_gv_tank
			feature = Feature(featureName, {
					"visibility_av": visibilityAv,
					"traversability_gv_car": traversabilityCar,
					"traversability_gv_tank": traversabilityTank,
			})
			try:
				self.regions[featureName].featureDefinition = feature
				updatedRegions.append(self.__regions[featureName])
			except KeyError as _:
				self.get_logger().error("Region with name \"%s\" not found in existing regions" % featureName)
		return updatedRegions

	def __render(self, regions: List[PolygonalRegion] = None):
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().warn("Skipping map render... RViz is not ready yet to receive messages.")
			return
		if regions is None:
			self.get_logger().info("Rendering map...")
			regionList = self.regions.values()
		else:
			self.get_logger().info("Rendering regions %s..." % repr([r.name for r in regions]))
			regionList = regions
		if len(regionList) == 0:
			self.get_logger().warn("Skipping render... no regions to update.")
			return
		message = MarkerArray()
		for region in regionList:
			message.markers += region.render()
		self.get_logger().info("MarkerArray about to be sent with %d markers." % len(message.markers))
		self.__rvizPublisher.publish(message)
		return

	def __clearRender(self):
		self.get_logger().warn("__clearRender is not implemented.")
		# for region in self.regions.values():
		# 	# region.clearRender()
		return

	def initializeMap(self) -> None:
		self.get_logger().info("Initializing map polygons...")
		if not self.__mapClient.service_is_ready():
			RosUtils.WaitForServicesToStart(self, self.__mapClient)
		request = QueryFeature.Request()
		request.name = self.__MAP_QUERY_NAME
		RosUtils.SendClientRequest(self, self.__mapClient, request, self.__parseFeatureQueryResponse)
		return

	def queryDefinitions(self) -> None:
		for name in self.__regionNames:
			if name == "":
				self.get_logger().warn("Region with empty name noticed in region names!")
				continue
			request = QueryFeature.Request()
			request.name = name
			self.get_logger().info("Sending QueryFeature for \"%s\"..." % name)
			RosUtils.SendClientRequest(self, self.__mapClient, request, self.__parseFeatureQueryResponse)
		self.__render()
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = MapServiceInterface()
	node.initializeMap()
	while node.context.ok():
		node.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))
		node.queryDefinitions()
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
