import rclpy
from rt_bi_core.Interface.MapInterface import MapInterface

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	mapNode = MapInterface()
	rclpy.spin(mapNode)
	mapNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
