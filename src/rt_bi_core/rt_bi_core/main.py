import rclpy
from rt_bi_core.Interface.MapInterface import MapInterface
from rt_bi_utils.Renderer import RViz

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	mapNode = MapInterface()
	rViz = RViz("rt_bi_utils_rviz")
	rclpy.spin(mapNode)
	rclpy.spin(rViz)
	rViz.destroy_node()
	mapNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
