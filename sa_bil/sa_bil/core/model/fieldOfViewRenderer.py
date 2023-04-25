from sa_bil.core.gui.drawing import Drawing
from sa_bil.core.model.connectivityGraph import ConnectivityGraph
from sa_bil.core.model.sensingRegion import SensingRegion
from sa_bil.core.observation.track import Tracks

class FieldOfViewRenderer:
	"""
	Used for debug rendering ONLY
	"""
	def __init__(self):
		self._previousCGraph: ConnectivityGraph = None
		self._timeLabelCanvasId = None

	def render(self, cGraph: ConnectivityGraph, canvas):
		if self._previousCGraph is not None and self._previousCGraph.time != cGraph.time: self.clearRender(canvas)
		for fovName in cGraph.fovNodes:
			sensingRegion: SensingRegion = cGraph.nodes[fovName]["region"]
			sensingRegion.render(canvas)
			for trackId in sensingRegion.tracks:
				sensingRegion.tracks[trackId].render(canvas)
		for shadowName in cGraph.shadowNodes:
			shadowRegion = cGraph.nodes[shadowName]["region"]
			shadowRegion.render(canvas)
		timeStr = "N/A" if cGraph is None else "%.2f" % cGraph.time
		labelStr = "t = %s" % timeStr
		self._timeLabelCanvasId = Drawing.CreateText(canvas, [-2, -5], labelStr, "TIME-LABEL")
		self._previousCGraph = cGraph

	def clearRender(self, canvas):
		if self._previousCGraph is None: return
		for fov in self._previousCGraph.fovNodes:
			fovNodeData = self._previousCGraph.nodes[fov]
			fovNodeData["region"].clearRender(canvas)
			if "tracks" in fovNodeData:
				for trackId in fovNodeData["tracks"]:
					fovNodeData["tracks"][trackId].clearRender(canvas)
		for shadowName in self._previousCGraph.shadowNodes:
			shadowRegion = self._previousCGraph.nodes[shadowName]["region"]
			shadowRegion.clearRender(canvas)
		Drawing.RemoveShape(canvas, self._timeLabelCanvasId)
		self._timeLabelCanvasId = None
		self._previousCGraph = None
