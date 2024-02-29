import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.RegionsSubscriber import MapSubscriber


class MapRenderer(MapSubscriber):
	""" This Node listens to all static and dynamic map region updates and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_map", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)

	def onMapUpdated(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} map updated.")
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	node = MapRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
