import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D # leave this here, otherwise fig.gca(projection="3d") won't work
from math import nan, isnan
from queue import Queue
from typing import Dict, List, Tuple, Union

from sa_bil.core.model.shadowRegion import ShadowRegion
from sa_bil.core.spec.lambdas import NfaLambda
from sa_bil.core.spec.spaceTime import ProjectiveSpaceTimeSet
from sa_bil.core.spec.timeRegion import TimeInterval

Queue.__repr__ = lambda q: repr(q.queue)

class GraphAlgorithms:
	_allFigs = set()

	@staticmethod
	def bfs(shadowTree: nx.DiGraph, start: str, goalFunc: NfaLambda.NfaLambdaFunc) -> Union[List[str], None]:
		q = Queue()
		visited = set()
		q.put([start])
		while not q.empty():
			path = q.get()
			n = path[-1]
			visited.add(n)
			nodeData = shadowTree.nodes[n]
			spaceTimeSetOfNode = ProjectiveSpaceTimeSet(nodeData["region"].polygon, TimeInterval(nodeData["fromTime"], nodeData["toTime"], True, True))
			# if nodeData["type"] == "sensor": continue # FIXME: check if there is a track head here
			if goalFunc(spaceTimeSetOfNode):
				return path
			for child in shadowTree.adj[n]:
				if child in visited: continue
				newPath = list(path)
				newPath.append(child)
				q.put(newPath)
		return None

	@staticmethod
	def isConnectedBfs(graph: nx.DiGraph, start, end, premittedBeams = []):
		q = Queue()
		visited = set()
		q.put([start])
		while not q.empty():
			path = q.get()
			n = path[-1]
			visited.add(n)
			if n == end: return path
			if (n != start) and (GraphAlgorithms.isBeamNode(n)) and (n not in premittedBeams): continue
			for child in graph.adj[n]:
				if child in visited: continue
				newPath = list(path)
				newPath.append(child)
				q.put(newPath)
		return None

	@staticmethod
	def chainGraphs(graphs: List[nx.DiGraph], startInd = 0, endInd = nan) -> nx.DiGraph:
		if isnan(endInd): endInd = len(graphs) - 1
		if startInd == endInd: return nx.DiGraph(graphs[startInd])
		if not startInd < endInd: raise IndexError("startIndex must be less than endIndex")
		chained = graphs[startInd]
		for i in range(startInd + 1, endInd + 1):
			chained = nx.compose(chained, graphs[i])
		return chained

	@staticmethod
	def getBeamLeaves(graph: nx.DiGraph):
		leaves = []
		for n in graph.nodes():
			if GraphAlgorithms.isBeamNode(n) and graph.out_degree(n) == 0:
				leaves.append(n)
		return leaves

	@staticmethod
	def getBeamName(passingLane, notPassingLane):
		return "%s¦|%s" % (passingLane, notPassingLane)

	@staticmethod
	def isBeamNode(n: str) -> bool:
		return "¦|" in n

	@staticmethod
	def isShadowRegion(graph: nx.DiGraph, n: str) -> bool:
		_isShadow = True if "type" in graph.nodes[n] and graph.nodes[n]["type"] == "shadow" else False
		if _isShadow: return True
		_isShadow = True if "region" in graph.nodes[n] and isinstance(graph.nodes[n]["region"], ShadowRegion) else False
		return _isShadow

	@staticmethod
	def getBeamEnds(n: str) -> Tuple[str, str]:
		parts = n.split("¦|")
		return (parts[0], parts[1])

	@staticmethod
	def cloneNodeProps(frm: dict, to: dict) -> bool:
		for key in frm: to[key] = frm[key]

	@staticmethod
	def multiPartiteLayout(g: nx.DiGraph, pos: Dict[str, Tuple[float, float]]=None):
		y = 0
		pos = {}
		for subG in g.graphs:
			keys = sorted(subG.nodes, key=lambda n: subG.nodes[n]["region"].polygon.centroid.x)
			i = 0
			for n in keys:
				nn = g._generateTemporalName(n , subG.time)
				pos[nn] = [0, 0]
				pos[nn][0] = i / len(subG.nodes)
				pos[nn][1] = y
				i += 1
			y += 2
		return pos

	@staticmethod
	def displayGraphAuto(g: nx.DiGraph, displayGeomGraph, displaySpringGraph):
		"""
		Detects beam and non-beam nodes for the correct coloring
		"""
		grnNodes = []
		symNodes = []
		redNodes = []
		for n in g.nodes:
			if n.startswith("FOV"):
				grnNodes.append(n)
			elif n.startswith("SYM"):
				symNodes.append(n)
			else:
				redNodes.append(n)
		if displayGeomGraph:
			GraphAlgorithms.displayGeometricGraph(g)
		if displaySpringGraph:
			GraphAlgorithms.displaySpringGraph(g, grnNodes, redNodes, symNodes)

	@staticmethod
	def displaySpringGraph(g: nx.DiGraph, greenNodes: list, redNodes: list, symbolNodes: list):
		greenNodesWithSymbols = greenNodes.copy()
		redNodesWithSymbols = redNodes.copy()
		for symNode in symbolNodes:
			if g.nodes[symNode]["region"].inFov:
				greenNodesWithSymbols.append(symNode)
			else:
				redNodesWithSymbols.append(symNode)
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		# pos = nx.spring_layout(g)
		# pos = nx.kamada_kawai_layout(g, scale=-1)
		# pos = nx.kamada_kawai_layout(g, weight="fromTime", scale=-1)
		# pos = nx.multipartite_layout(g, subset_key="fromTime", align="horizontal")
		pos = GraphAlgorithms.multiPartiteLayout(g)
		nx.draw_networkx_nodes(g, pos, nodelist=greenNodesWithSymbols, node_color="palegreen")
		nx.draw_networkx_nodes(g, pos, nodelist=redNodesWithSymbols, node_color="tomato")
		normalEdges = []
		temporalEdges = []
		for e in g.edges:
			if "isTemporal" in g.edges[e] and g.edges[e]["isTemporal"]:
				temporalEdges.append(e)
			else:
				normalEdges.append(e)
		nx.draw_networkx_edges(g, pos, edgelist=normalEdges, edge_color="blue")
		nx.draw_networkx_edges(g, pos, edgelist=temporalEdges, edge_color="red", width=2)
		nx.draw_networkx_labels(g, pos, font_family="DejaVu Sans", font_size=10)
		plt.axis("off")
		fig.set_facecolor("grey")
		fig.show()
		return fig

	@staticmethod
	def getNodeCoordinates(g, nodeName) -> tuple:
		return (g.nodes[nodeName]["centroid"].x, g.nodes[nodeName]["centroid"].y, g.nodes[nodeName]["fromTime"])

	@staticmethod
	def displayGeometricGraph(g: nx.DiGraph):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		ax = fig.add_subplot(projection="3d")
		xFov = []
		xpFov = None
		yFov = []
		ypFov = None
		zFov = []
		zpFov = None
		xSymbol = []
		ySymbol = []
		zSymbol = []
		xShadow = []
		yShadow = []
		zShadow = []
		for node in g.nodes:
			if "type" not in g.nodes[node]:
				raise "Graph node needs a type"
			elif g.nodes[node]["type"] == "sensor":
				(xFov, yFov) = g.nodes[node]["region"].polygon.exterior.coords.xy
				zFov = [g.nodes[node]["fromTime"]] * len(xFov)
				ax.plot(xFov, yFov, zFov, "g-")
				# ax.text(g.nodes[node]["centroid"].x, g.nodes[node]["centroid"].y, g.nodes[node]["fromTime"], node, color="darkgreen")
				if xpFov is not None:
					for i in range(len(xFov) - 1):
						ax.plot([xpFov[i], xFov[i]], [ypFov[i], yFov[i]], [zpFov[i], zFov[i]], "g-")
				(xpFov, ypFov, zpFov) = (xFov, yFov, zFov)
			elif g.nodes[node]["type"] == "symbol":
				(xSymbol, ySymbol) = g.nodes[node]["region"].polygon.exterior.coords.xy
				zSymbol = [g.nodes[node]["fromTime"]] * len(xSymbol)
				ax.plot(xSymbol, ySymbol, zSymbol, "b-")
				# ax.text(g.nodes[node]["centroid"].x, g.nodes[node]["centroid"].y, g.nodes[node]["fromTime"], node, color="darkblue")
			elif g.nodes[node]["type"] == "shadow":
				(xShadow, yShadow) = g.nodes[node]["region"].polygon.exterior.coords.xy
				zShadow = [g.nodes[node]["fromTime"]] * len(xShadow)
				ax.plot(xShadow, yShadow, zShadow, "k-")
				# ax.text(g.nodes[node]["centroid"].x, g.nodes[node]["centroid"].y, g.nodes[node]["fromTime"], node, color="maroon")
			else:
				raise "Unknown node type: %s" % g.nodes[node]["type"]
		ax.autoscale()
		fig.set_facecolor("grey")
		fig.show()
		return fig

	@staticmethod
	def _getTimedNodeName(nodeName, timestamp):
		return "%s-%.1f" % (nodeName, timestamp)

	@staticmethod
	def killDisplayedGraph(fig):
		plt.close(fig)
		GraphAlgorithms._allFigs.remove(fig)

	@staticmethod
	def killAllDisplayedGraph():
		for fig in GraphAlgorithms._allFigs:
			plt.close(fig)
		GraphAlgorithms._allFigs.clear()
