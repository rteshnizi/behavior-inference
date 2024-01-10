import rclpy

from rt_bi_utils.RtBiNode import RtBiNode


class SpGraphInterface(RtBiNode):
	def __init__(self) -> None:
		super().__init__(node_name="rt_bi_core_sp_graph")
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseConfigFileParameters(self) -> None:
		return super().parseConfigFileParameters()

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	ba = SpGraphInterface()
	rclpy.spin(ba)
	ba.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
