from queue import Queue
from typing import Dict, List, Tuple, Union

import networkx as nx
from matplotlib.pyplot import Figure, axis, figure, pause
from mpl_toolkits.mplot3d import Axes3D  # cspell: disable-line, leave this here, otherwise fig.gca(projection="3d") won't work

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Lambda import NfaLambda
from rt_bi_core.BehaviorAutomaton.SpaceTime import ProjectiveSpaceTimeSet
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.BehaviorAutomaton.TimeRegion import TimeInterval
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_core.Model.Tracklet import Tracklet, Tracklets
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, Polygon

Queue.__repr__ = lambda q: repr(q.queue)
___a = Axes3D # This is here to avoid a warning for unused import

class ShadowTree(nx.DiGraph):
	"""The implementation of a Shadow Tree in python as described in the dissertation of Reza Teshnizi."""

	def __init__(self):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(name="ShadowTree V3")
		self.__fig: Union[Figure, None] = None
		self.__history: List[ConnectivityGraph] = []
		self.componentEvents: List[List[Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """

		self.redPolys: List[Polygon] = [] # Used for debugging
		self.bluePolys: List[Polygon] = [] # Used for debugging
		self.lines: List[LineString] = [] # Used for debugging

	@property
	def history(self) -> Tuple[ConnectivityGraph, ...]:
		"""The list of the graphs."""
		return tuple(self.__history)

	def __repr__(self) -> str:
		return self.name

	def __getLowerAndUpperNode(self, n1: str, n2: str) -> Tuple[str, str]:
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def __addNode(self, n: str, timeNanoSecs: int) -> None:
		self.add_node(n)
		self.nodes[n]["fromTime"] = timeNanoSecs
		self.nodes[n]["toTime"] = timeNanoSecs # This will be updated accordingly when adding temporal edges
		return

	def __addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime = None, toTime = None) -> None:
		(lower, upper) = self.__getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)
		return

	def __shadowsAreConnectedTemporally(self, previousGraph: ConnectivityGraph, currentGraph: ConnectivityGraph, previousShadow: dict, currentShadow: dict, centerOfRotation: Geometry.Coords) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the translation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		previousP: Polygon = previousShadow["region"].polygon
		currentP: Polygon = currentShadow["region"].polygon
		intersectionOfShadows: Polygon = previousP.intersection(currentP)
		for fov in previousGraph.__sensorNodes:
			if fov not in previousGraph.nodes: continue
			previousFovRegion: SensorRegion = previousGraph.nodes[fov]["region"]
			if fov not in currentGraph.nodes: continue
			currentFovRegion: SensorRegion = currentGraph.nodes[fov]["region"]
			transformation: AffineTransform = Geometry.getAffineTransformation(previousFovRegion.interior, currentFovRegion.interior, centerOfRotation)
			ps = []
			for e in currentFovRegion.__edges:
				edgeC = currentFovRegion.__edges[e]
				edgeP = previousFovRegion.getEquivalentEdge(edgeC, transformation, centerOfRotation)
				if edgeP is None:
					raise AssertionError("No equivalent edge found based on transformation.")
				polygon = Polygon([edgeP.coords[0], edgeP.coords[1], edgeC.coords[1], edgeC.coords[0]])
				ps.append(polygon)
			u = Geometry.union(ps)
			p = intersectionOfShadows.difference(u)
			if p.area > 0.01:
				return True
		return False

	def __interpolateTrack(self, previousTracks: Tracklets, currentTracks: Tracklets, eventTime: float, eventFraction: float) -> Tracklets:
		"""
			This function assumes members in previousTracks and currentTracks all have the same timestamp.
		"""
		if len(previousTracks) == 0 or len(currentTracks) == 0: return currentTracks
		if eventFraction == 0: return previousTracks
		if eventFraction == 1: return currentTracks
		interpolatedTracks = {}
		(currentTime, _) = next(iter(currentTracks))
		for (prevTime, trackId) in previousTracks:
			previousPose = previousTracks[(prevTime, trackId)].pose
			currentPose = currentTracks[(currentTime, trackId)].pose
			x = ((eventFraction * (currentPose.x - previousPose.x)) + previousPose.x)
			y = ((eventFraction * (currentPose.y - previousPose.y)) + previousPose.y)
			psi = ((eventFraction * (currentPose.psi - previousPose.psi)) + previousPose.psi)
			interpolatedTracks[(eventTime, trackId)] = Tracklet(trackId, eventTime, x, y, psi, isInterpolated=True)
		return interpolatedTracks

	def __appendConnectivityGraphPerEvent(self, regions: List[AffineRegion], events: CtCd.CollisionEvent, pastCGraph: ConnectivityGraph, nowSensors: Tracklets, symbols: Dict[str, Symbol], startTime: float, endTime: float) -> List[ConnectivityGraph]:
		graphs = []
		for event in events:
			eventTime = ((event[1] * (endTime - startTime)) + startTime)
			interpolatedTracks = self.__interpolateTrack(pastCGraph, nowSensors, eventTime, event[1])
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTime}
			graph = ConnectivityGraph(eventTime, regions, event[0], filteredTracks, symbols)
			graphs.append(graph)
		return graphs

	def __addTemporalEdges(self, eventGraphs: List[ConnectivityGraph], centerOfRotation: Geometry.Coords) -> None:
		for nowGraph in eventGraphs:
			isInitialGraph = (len(self.__history) == 0)
			self.__appendToHistory(nowGraph)
			if isInitialGraph: return
			pastGraph = self.__history[-2]
			# Add temporal edges between FOVs
			for sensor in pastGraph.interior:
				fovNodeInShadowTree = self.generateTemporalName(sensor, pastGraph.timeNanoSecs)
				if fovNodeInShadowTree not in self.nodes: continue
				fovNodeInCurrentGraph = self.generateTemporalName(sensor, nowGraph.timeNanoSecs)
				if fovNodeInCurrentGraph not in self.nodes: continue
				self.__addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between symbols
			for symNode in pastGraph.symbolNodes:
				symNodeInShadowTree = self.generateTemporalName(symNode, pastGraph.time)
				symNodeInCurrentGraph = self.generateTemporalName(symNode, nowGraph.timeNanoSecs)
				self.__addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between shadows
			for shadowNodeInPastGraph in pastGraph.shadowNodes:
				for shadowNodeInNowGraph in nowGraph.__shadowNodes:
					previousShadowNodeRegion = pastGraph.nodes[shadowNodeInPastGraph]["region"]
					currentShadowNodeRegion = nowGraph.nodes[shadowNodeInNowGraph]["region"]
					if previousShadowNodeRegion.polygon.intersects(currentShadowNodeRegion.polygon):
						if self.__shadowsAreConnectedTemporally(pastGraph, nowGraph, pastGraph.nodes[shadowNodeInPastGraph], nowGraph.nodes[shadowNodeInNowGraph], centerOfRotation):
							shadowNodeInShadowTree = self.generateTemporalName(shadowNodeInPastGraph, pastGraph.timeNanoSecs)
							shadowNodeInNowGraph = self.generateTemporalName(shadowNodeInNowGraph, nowGraph.timeNanoSecs)
							self.nodes[shadowNodeInShadowTree]["toTime"] = nowGraph.timeNanoSecs
							self.__addEdge(shadowNodeInShadowTree, shadowNodeInNowGraph, isTemporal=True)
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		for node in graph.nodes:
			temporalName = self.generateTemporalName(node, graph.timeNanoSecs)
			self.__addNode(temporalName, graph.timeNanoSecs)
			self.shallowCopyNode(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self.generateTemporalName(edge[0], graph.timeNanoSecs)
			to = self.generateTemporalName(edge[1], graph.timeNanoSecs)
			self.__addEdge(frm, to, False)
		self.__history.append(graph)
		RosUtils.Logger().info("ShadowTree history length %d." % len(self.__history))
		return

	def multiPartiteLayout(self) -> Dict[str, Tuple[float, float]]:
		y = 0
		pos: Dict[str, Tuple[float, float]] = {}
		for subG in self.history:
			keys = sorted(subG.nodes, key=lambda n: subG.nodes[n]["region"].interior.centroid.x)
			i = 0
			for n in keys:
				nn = self.generateTemporalName(n , subG.timeNanoSecs)
				pos[nn] = [0, 0]
				pos[nn][0] = i / len(subG.nodes)
				pos[nn][1] = y
				i += 1
			y += 2
		return pos

	def bfs(self, start: str, goalFunc: NfaLambda.NfaLambdaFunc) -> Union[List[str], None]:
		q = Queue()
		visited = set()
		q.put([start])
		while not q.empty():
			path = q.get()
			n = path[-1]
			visited.add(n)
			nodeData = self.nodes[n]
			spaceTimeSetOfNode = ProjectiveSpaceTimeSet(nodeData["region"].interior, TimeInterval(nodeData["fromTime"], nodeData["toTime"], True, True))
			# if nodeData["type"] == "sensor": continue # FIXME: check if there is a track head here
			if goalFunc(spaceTimeSetOfNode):
				return path
			for child in self.adj[n]:
				if child in visited: continue
				newPath = list(path)
				newPath.append(child)
				q.put(newPath)
		return None

	def updateNamedRegions(self, timeNanoSecs: int, regions: List[SymbolRegion]) -> None:
		return

	def updateSensors(self, timeNanoSecs: int, regions: List[SensorRegion], symbols: Dict[str, Symbol] = {}) -> None:

		RosUtils.Logger().info("<====================================CtCd: %d====================================>" % timeNanoSecs)
		if len(self.history) == 0:
			RosUtils.Logger().error("Initial CGraph cannot be created from from sensors.")
		else:
			pastCGraph = self.history[-1]

			# There are there possibilities for sensor names. We have decided to require specific messages for S1.
			# 1. S1 is in pastSensors but not in nowSensors ----> S1 has turned off -> the shadows around S1 have merged.
			# 2. S2 is not in pastSensors but is in nowSensors -> S2 has turned on --> the shadows around S2 have splitted.
			# 3. S3 is in pastSensors but not in nowSensors ----> S3 has moved ------> the shadows around S3 have evolved.

			if len(pastCGraph.fieldOfView) > 0:
				fovRegions = FieldOfView(regions)
				for rName in pastCGraph.fieldOfView:
					fovRegions.addConnectedComponent(pastCGraph.fieldOfView[rName])
			else:
				fovRegions = regions

			nowCGraph = ConnectivityGraph(timeNanoSecs=timeNanoSecs, mapRegions=pastCGraph.mapPerimeter, fovRegions=fovRegions)
			# 1. Sensors that have turned off -> the shadows around S1 have merged.
			turnedOffSensors = pastCGraph.fieldOfView - nowCGraph.fieldOfView
			if len(turnedOffSensors) > 0:
				RosUtils.Logger().info("S1.1.1 -> Sensors turned off: %s" % repr(turnedOffSensors))
			else:
				RosUtils.Logger().info("S1.1.2 -> No recently turned off sensor.")

			# 2. Sensors that have turned on --> the shadows around S2 have splitted. No real algorithm needed, the code does it, on its own.
			turnedOnSensors = nowCGraph.fieldOfView - pastCGraph.fieldOfView
			if len(turnedOnSensors) > 0:
				RosUtils.Logger().info("S2.1.1 -> Sensors turned on: %s" % repr(turnedOnSensors))
				self.__appendToHistory(nowCGraph)
			else:
				RosUtils.Logger().info("S2.1.2 -> No newly turned on sensor.")

			# 3. Sensors that have moved ------> the shadows around S3 have evolved.
			evolvedSensors = pastCGraph.fieldOfView & nowCGraph.fieldOfView
			if len(evolvedSensors) > 0:
				RosUtils.Logger().info("S3.1.1 -> Sensors evolved: %s" % repr(evolvedSensors))

				events = CtCd.estimateIntermediateCollisionsWithPolygon(pastCGraph.fieldOfView, nowCGraph.fieldOfView, pastCGraph.mapPerimeter.interior)
				RosUtils.Logger().info("Appending CGraphs to Shadow Tree for %d events." % len(events))
				eventGraphs = self.__appendConnectivityGraphPerEvent(
					regions=regions,
					events=events,
					pastCGraph=pastCGraph,
					nowSensors=nowCGraph.fieldOfView,
					symbols=symbols,
					startTime=pastCGraph.timeNanoSecs,
					endTime=timeNanoSecs
				)
				for g in eventGraphs: self.__appendToHistory(g)
			# graphPacks.sort(key=lambda x:x[0].time)
			# for (graph, centerOfRotation) in graphPacks:
			# 	self._addTemporalEdges(graph, centerOfRotation)
		RosUtils.Logger().info("</===============================End CtCd: %d===================================>" % timeNanoSecs)
		return

	def updateMap(self, timeNanoSecs: int, regions: List[MapRegion]) -> None:
		if len(self.__history) == 0:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, mapRegions=regions, fovRegions=[])
			self.__appendToHistory(currentConnectivityG)
		return

	def renderSpringGraph(self, greenNodes: List[str], redNodes: List[str], symbolNodes: List[str]) -> None:
		"""
		# Deprecated

		Parameters
		----------
		greenNodes : List[str]
			Rendered as green nodes.
		redNodes : List[str]
			Rendered as red nodes.
		symbolNodes : List[str]
			Rendered as a symbol.
		"""
		RosUtils.Logger().error("Using RViz rendering...")
		raise DeprecationWarning("Using RViz rendering...")
		RosUtils.Logger().info("RENDER -> fov nodes: %s" % repr(greenNodes))
		RosUtils.Logger().info("RENDER -> shd nodes: %s" % repr(redNodes))
		greenNodesWithSymbols = greenNodes.copy()
		redNodesWithSymbols = redNodes.copy()
		for symNode in symbolNodes:
			if self.nodes[symNode]["region"].inFov:
				greenNodesWithSymbols.append(symNode)
			else:
				redNodesWithSymbols.append(symNode)
		if self.__fig is None:
			fig = figure(self.name)
			fig.set_facecolor("grey") # cspell: disable-line
			axis("off")
		else:
			fig = self.__fig
		# pos = nx.spring_layout(g)
		# pos = nx.kamada_kawai_layout(g, scale=-1) # cspell: disable-line
		# pos = nx.kamada_kawai_layout(g, weight="fromTime", scale=-1) # cspell: disable-line
		# pos = nx.multipartite_layout(g, subset_key="fromTime", align="horizontal")

		pos = self.multiPartiteLayout()
		nx.draw_networkx_nodes(self, pos, nodelist=greenNodesWithSymbols, node_color="PaleGreen")
		nx.draw_networkx_nodes(self, pos, nodelist=redNodesWithSymbols, node_color="Tomato")
		normalEdges = []
		temporalEdges = []
		for e in self.edges:
			if "isTemporal" in self.edges[e] and self.edges[e]["isTemporal"]:
				temporalEdges.append(e)
			else:
				normalEdges.append(e)
		nx.draw_networkx_edges(self, pos, edgelist=normalEdges, edge_color="blue") # cspell: disable-line
		nx.draw_networkx_edges(self, pos, edgelist=temporalEdges, edge_color="red", width=2) # cspell: disable-line
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		pause(0.05)
		return

	@staticmethod
	def shallowCopyNode(frm: dict, to: dict) -> None:
		for key in frm: to[key] = frm[key]
		return

	@staticmethod
	def generateTemporalName(name: str, time: int) -> str:
		""" Use this function to safely generate (typo-free and in uniform format) the names for the temporal edges. """
		return "%s-%d" % (name, time)
