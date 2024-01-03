from typing import Dict, Literal, Set, Union

from rclpy.node import Node

from rt_bi_utils.Color import RGBA, toHexStr


class BaMatplotlibRenderer:
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
		def __init__(self, marker: Union["BaMatplotlibRenderer.KnownMarkers", "BaMatplotlibRenderer.MarkerStyle"], nodeColor: RGBA, boundaryColor: RGBA, boundaryWidth: float = 1.0, size: int = 600) -> None:
			self.marker: Union["BaMatplotlibRenderer.KnownMarkers", "BaMatplotlibRenderer.MarkerStyle"] = marker
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
