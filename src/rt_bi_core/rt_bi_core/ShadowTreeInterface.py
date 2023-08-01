from typing import List

import rclpy
from rclpy.node import Node

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensingRegion import SensingRegion
from rt_bi_core.ShadowTree.ShadowTree import ShadowTree
from rt_bi_utils.SaMsgs import SaMsgs
from sa_msgs.msg import FeatureInfoIndividual, RobotState
from sa_msgs.srv import QueryFeature


class ShadowTreeInterface(Node):
	"""
	This Node listens to all the messages published on the topics related to the shadow tree.
	"""
	def __init__(self) -> None:
		""" Create a Shadow Tree Interface node. """
		super().__init__("rt_bi_core_shadow_tree")
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__shadowTree: ShadowTree = ShadowTree()
		self.__mapClient = SaMsgs.createSaFeatureQueryClient(self)
		self.__initializeMap()
		SaMsgs.subscribeToSaRobotStateTopic(self, self.__parseRobotStateMessage)
		# start a timer to query the feature definitions every 5 secs
		self.create_timer(5, self.__queryDefinitions)

	def __parseRobotStateMessage(self, update: RobotState) -> None:
		if update is None:
			self.get_logger().warn("Received empty RobotState!")
			return False
		if (
			self.__sensors is not None and
			update.robot_id in self.__sensors and
			hash(repr(update)) == hash(repr(self.__sensors[update.robot_id]))
		):
			return False

		self.get_logger().info("Updating sensor %d definition..." % update.robot_id)
		if self.__sensors is None:
			self.__sensors = {}
		coords = SaMsgs.convertSaPoseListToCoordsList(update.fov.corners)
		sensor = SensingRegion("S-%d" % update.robot_id, coords, self.get_clock().now().nanoseconds, update.robot_id)
		self.__sensors[update.robot_id] = sensor
		self.__updateSensingRegions([sensor])
		return

	def __initializeMap(self) -> None:
		self.get_logger().info("Initializing map polygons...")
		if not self.__mapClient.service_is_ready():
			RosUtils.WaitForServicesToStart(self, self.__mapClient)
		request = QueryFeature.Request()
		request.name = self.__MAP_QUERY_NAME
		RosUtils.SendClientRequest(self, self.__mapClient, request, self.__parseFeatureQueryResponse)
		return

	def __queryDefinitions(self) -> None:
		for name in self.__regionNames:
			if name == "":
				self.get_logger().warn("Region with empty name noticed in region names!")
				continue
			request = QueryFeature.Request()
			request.name = name
			self.get_logger().info("Sending QueryFeature for \"%s\"..." % name)
			RosUtils.SendClientRequest(self, self.__mapClient, request, self.__parseFeatureQueryResponse)
		self.__updateMap()
		return

	def __parseFeatureQueryResponse(self, request: QueryFeature.Request, response: QueryFeature.Response) -> None:
		self.get_logger().info("Parsing response to %s query..." % request.name)
		if request.name == self.__MAP_QUERY_NAME:
			self.__parsePolygonShapeList(response)
			self.__updateMap()
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

	def __updateMap(self, regions: List[MapRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Map update for shadow tree %s" % repr(regions))
		self.__shadowTree.updateMap(timeNanoSecs=updateTime, regions=regions)
		return

	def __updateNamedRegions(self, regions: List[PolygonalRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Named update for shadow tree %s" % repr(regions))
		self.__shadowTree.updateNamedRegions(timeNanoSecs=updateTime, regions=regions)
		return

	def __updateSensingRegions(self, regions: List[SensingRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("FOV update for shadow tree %s" % repr(regions))
		self.__shadowTree.updateSensingRegions(timeNanoSecs=updateTime, regions=regions)
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = ShadowTreeInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
