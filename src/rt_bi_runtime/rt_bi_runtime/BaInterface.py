import json
from typing import Dict, List, Literal, Set, Union

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import rt_bi_utils.Ros as RosUtils
from rt_bi_runtime.Model.BehaviorAutomaton import BehaviorAutomaton
from rt_bi_utils.Color import RGBA, toHexStr


class BaRosNode(Node):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self) -> None:
		""" Create a Behavior Automaton node. """
		super().__init__(node_name="rt_bi_core_bi") # type: ignore - parameter_overrides: List[Parameter] = None
		self.get_logger().info("%s is initializing." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__declareParameters()
		self.__name: str = self.get_fully_qualified_name()
		self.__states: List[str] = []
		self.__transitions: List[List[List[str]]] = []
		self.__start: str = ""
		self.__accepting: List[str] = []
		self.__parseConfigFileParameters()
		self.__ba = BehaviorAutomaton(self.__name, self.__states, self.__transitions, self.__start, self.__accepting)
		return

	def __repr__(self) -> str:
		return self.__name

	def __declareParameters(self) -> None:
		self.get_logger().debug("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("name", Parameter.Type.STRING)
		self.declare_parameter("states", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("start", Parameter.Type.STRING)
		self.declare_parameter("accepting", Parameter.Type.STRING_ARRAY)
		return

	def __parseConfigFileParameters(self) -> None:
		self.get_logger().debug("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__name = self.get_parameter("name").get_parameter_value().string_value
		self.__states = list(self.get_parameter("states").get_parameter_value().string_array_value)
		for transition in list(self.get_parameter("transitions").get_parameter_value().string_array_value):
			self.__transitions.append(json.loads(transition))
		self.__start = self.get_parameter("start").get_parameter_value().string_value
		self.__accepting = list(self.get_parameter("accepting").get_parameter_value().string_array_value)
		return

	def render(self) -> None:
		MatplotlibHelper.createBehaviorAutomatonFigure(self, self.__ba)
		return

	def destroy_node(self) -> None:
		MatplotlibHelper.closeFig(self.__ba)
		return super().destroy_node()


class MatplotlibHelper:
	import matplotlib.pyplot as plt
	import networkx as nx
	from matplotlib import use
	from matplotlib.animation import FuncAnimation
	from matplotlib.axes import Axes
	from matplotlib.figure import Figure
	from matplotlib.markers import MarkerStyle

	from rt_bi_runtime.Model.BehaviorAutomaton import BehaviorAutomaton
	from rt_bi_utils.Color import RgbaNames
	from rt_bi_utils.NetworkX import NxUtils

	use("QtAgg")
	# CSpell: ignore: mpl_toolkits, set_facecolor, facecolor, edgecolor, LINETO, dtype, edgecolors
	__graphToFigs: Dict[str, Figure] = {}
	__graphToAxes: Dict[str, Axes] = {}
	__graphToAnimation: Dict[str, FuncAnimation] = {}
	__rosNode: Node
	KnownMarkers = Union[Literal["o"], Literal["v"], Literal["^"], Literal["<"], Literal[">"], Literal["8"], Literal["s"], Literal["p"], Literal["*"], Literal["h"], Literal["H"], Literal["D"], Literal["d"], Literal["P"], Literal["X"]]

	class PredefinedMarkerStyles:
		import numpy as np
		from matplotlib.markers import MarkerStyle
		from matplotlib.patches import PathPatch
		from matplotlib.path import Path as MarkerPath

		__innerCirclePath = MarkerPath.circle(center=(0.0, 0.0), radius=0.80)
		__outerCirclePath = MarkerPath.circle(center=(0.0, 0.0), radius=1.00)
		__vertices = np.concatenate((__innerCirclePath.vertices, __outerCirclePath.vertices))
		__codes = np.concatenate((__innerCirclePath.codes, __outerCirclePath.codes))
		__markerPath = MarkerPath(__vertices, __codes)
		doubleCircle = MarkerStyle(__markerPath)

	class NodeStyle:
		def __init__(self, marker: Union["MatplotlibHelper.KnownMarkers", "MatplotlibHelper.MarkerStyle"], nodeColor: RGBA, boundaryColor: RGBA, boundaryWidth: float = 1.0, size: int = 600) -> None:
			self.marker: Union["MatplotlibHelper.KnownMarkers", "MatplotlibHelper.MarkerStyle"] = marker
			self.size: int = size
			self.nodeColor: str = toHexStr(nodeColor)
			self.boundaryColor: str = toHexStr(boundaryColor)
			self.boundaryWidth: float = boundaryWidth

	__acceptingStateStyle = NodeStyle(PredefinedMarkerStyles.doubleCircle, RgbaNames.TRANSPARENT, RgbaNames.BLACK)
	__nonAcceptingStateStyle = NodeStyle("o", RgbaNames.TRANSPARENT, RgbaNames.BLACK)
	__activeAcceptingStateStyle = NodeStyle(__acceptingStateStyle.marker, RgbaNames.TRANSPARENT, RgbaNames.RED, 2)
	__activeNonAcceptingStateStyle = NodeStyle("o", RgbaNames.TRANSPARENT, RgbaNames.RED, 2)

	@classmethod
	def __drawActiveNonAcceptingStates(cls, ba: BehaviorAutomaton) -> None:
		cls.__drawNodes(ba, (ba.activeStates() - ba.acceptingStates), cls.__activeNonAcceptingStateStyle)
		return

	@classmethod
	def __drawActiveAcceptingStates(cls, ba: BehaviorAutomaton) -> None:
		cls.__drawNodes(ba, (ba.activeStates() & ba.acceptingStates), cls.__activeAcceptingStateStyle)
		return

	@classmethod
	def __drawInactiveNonAcceptingStates(cls, ba: BehaviorAutomaton) -> None:
		cls.__drawNodes(ba, (ba.nonActiveStates() - ba.acceptingStates), cls.__nonAcceptingStateStyle)
		return

	@classmethod
	def __drawInactiveAcceptingStates(cls, ba: BehaviorAutomaton) -> None:
		cls.__drawNodes(ba, (ba.nonActiveStates() & ba.acceptingStates), cls.__acceptingStateStyle)
		return

	@classmethod
	def __drawNodes(cls, ba: BehaviorAutomaton, nodes: Set[str], style: NodeStyle) -> None:
		ax = cls.__graphToAxes[ba.name]
		cls.nx.draw_networkx_nodes(
			ba,
			pos=ba.renderLayout,
			ax=ax,
			nodelist=nodes,
			node_size=style.size,
			node_shape=style.marker, # type: ignore - according to the docs, specification is as matplotlib.scatter marker
			node_color=style.nodeColor,
			edgecolors=style.boundaryColor
		)
		cls.nx.draw_networkx_labels(ba, ba.renderLayout, ax=ax)
		return

	@classmethod
	def __drawEdges(cls, ba: BehaviorAutomaton) -> None:
		ax = cls.__graphToAxes[ba.name]
		cls.nx.draw_networkx_edges(ba, ba.renderLayout, ax=ax)
		labels = cls.nx.get_edge_attributes(ba, "descriptor")
		cls.nx.draw_networkx_edge_labels(ba, ba.renderLayout, labels, ax=ax)
		return

	@classmethod
	def __updatePlot(cls, ba: BehaviorAutomaton) -> None:
		if ba.name not in cls.__graphToFigs: return
		ax = cls.__graphToAxes[ba.name]
		ax.clear()
		cls.__drawInactiveAcceptingStates(ba)
		cls.__drawInactiveNonAcceptingStates(ba)
		cls.__drawActiveAcceptingStates(ba)
		cls.__drawActiveNonAcceptingStates(ba)
		cls.__drawEdges(ba)
		return

	@classmethod
	def activateFigure(cls, ba: BehaviorAutomaton) -> Figure:
		return cls.plt.figure(ba.name)

	@classmethod
	def createBehaviorAutomatonFigure(cls, rosNode: Node, ba: BehaviorAutomaton) -> None:
		if ba.name in cls.__graphToFigs: cls.closeFig(ba)
		cls.__rosNode = rosNode
		fig = cls.activateFigure(ba)
		ax = fig.add_subplot()
		cls.__graphToFigs[ba.name] = fig
		cls.__graphToAxes[ba.name] = ax
		cls.plt.axis("off")
		fig.set_facecolor("grey")
		animation = cls.FuncAnimation(fig, cls.__updatePlot, frames=ba.baGenerator, interval=250)
		cls.__graphToAnimation[ba.name] = animation
		cls.plt.show()
		return

	@classmethod
	def closeFig(cls, ba: BehaviorAutomaton) -> None:
		if ba.name not in cls.__graphToFigs: return
		cls.plt.close(cls.__graphToFigs[ba.name])
		cls.__graphToAnimation.pop(ba.name)
		cls.__graphToFigs.pop(ba.name)
		cls.__graphToAxes.pop(ba.name)
		return


def main(args=None):
	rclpy.init(args=args)
	ba = BaRosNode()
	ba.render()
	rclpy.spin(ba)
	ba.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
