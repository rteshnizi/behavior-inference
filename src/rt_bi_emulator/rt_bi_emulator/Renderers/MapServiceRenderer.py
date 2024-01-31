import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.MapRegionsSubscriber import MapRegionsSubscriber


class MapServiceRenderer(MapRegionsSubscriber):
	""" This Node listens to all the messages published on the topics related to the map and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_map", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)

	def onMapUpdated(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} received map update.")
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	node = MapServiceRenderer()
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()