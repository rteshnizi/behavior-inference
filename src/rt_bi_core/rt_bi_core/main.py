import rclpy
from rt_bi_core.Interface.MapInterface import MapInterface

def main(args=None):
	"""
	Start the viewer.
	"""
	rclpy.init(args=args)
	mapNode = MapInterface()
	rclpy.spin(mapNode)
	return

if __name__ == "__main__":
	main()
