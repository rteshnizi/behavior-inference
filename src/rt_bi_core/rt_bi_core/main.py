import rclpy
from sa_bil.viewer import Viewer

def main(args=None):
	"""
	Start the viewer.
	"""
	rclpy.init(args=args)
	viewer = Viewer()
	rclpy.spin(viewer)
	return

if __name__ == "__main__":
	main()
