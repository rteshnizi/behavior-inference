import tkinter as tk
from typing import List
from sa_bil.core.utils.geometry import Geometry # I put this on top here to assign shapely repr functions

from sa_bil.core.gui.canvas import Canvas
from sa_bil.core.gui.drawing import Drawing
from sa_bil.core.observation.observations import Observation, Observations
from sa_bil.core.model.map import Map
from sa_bil.core.model.shadowTreeV2 import ShadowTreeV2
from sa_bil.core.model.fieldOfViewRenderer import FieldOfViewRenderer
from sa_bil.core.model.connectivityGraph import ConnectivityGraph
from sa_bil.core.spec.specification import Specification
from sa_bil.core.utils.graph import GraphAlgorithms

class App(tk.Frame):
	def __init__(self, bil):
		self.bil = bil
		self.map: Map = self.bil.map
		self.observations: Observations = self.bil.observations
		self.specs: List[Specification] = self.bil.specs
		self.shadowTree: ShadowTreeV2 = None
		self._maxObservationIndex = len(self.observations) - 1
		self.master = tk.Tk()
		self.fovRenderer = FieldOfViewRenderer()
		self.master.title("Canvas")
		self.master.geometry("1000x900")
		self.master.protocol("WM_DELETE_WINDOW", self._onClose)

		# Create & Configure frame
		tk.Grid.rowconfigure(self.master, 0, weight=1)
		tk.Grid.columnconfigure(self.master, 0, weight=1)
		self.frame = tk.Frame(self.master)
		self.frame.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
		tk.Grid.rowconfigure(self.frame, 0, minsize=25)
		tk.Grid.rowconfigure(self.frame, 1, minsize=25)
		tk.Grid.rowconfigure(self.frame, 2, minsize=850)
		super().__init__(self.master)
		self.fovLabel = tk.StringVar(master=self.master)
		self.fovIndex = 0
		self.fovLabel.set(self._observationLabelText)
		self.eventLabel = tk.StringVar(master=self.master)
		self.eventIndex = 0
		self.eventDrawingId = []
		self.dbgActionValue = tk.StringVar(self.master)
		self.dbgActionValue.set("Debug Actions") # set the default option
		self.dbgActions = {
			"< Observation": self._prevFov,
			"Observation >": self._nextFov,
			"Create Conn. Graph": self._createConnectivityGraph,
			"Single Layer Shadow Tree": self._createSingleLayerShadowTree,
			"Create Shadow Tree": self._createShadowTree,
			"Validate Spec": self._validateSpec
		}
		self.specDropdownText = tk.StringVar(self.master)
		self.selectedSpec = "simple-2" # set the default option
		self.specDropdownText.set("Selected Spec: %s" % self.selectedSpec)
		self.dbgCheckboxes = {
			"Render Trajectory": tk.IntVar(master=self.master, value=1),
			"Space-Time Diagram": tk.IntVar(master=self.master, value=1),
			"Shadow Tree Graph": tk.IntVar(master=self.master, value=1),
			"Show FOV": tk.IntVar(master=self.master, value=0),
			"Show Event": tk.IntVar(master=self.master, value=0),
		}
		self._createTkGridRow0()
		self._createTkGridRow1()
		self.canvas = Canvas(master=self.frame, app=self, row=2, col=0)
		self._renderMap()
		self._renderTrajectories()
		self._renderSpec(self.selectedSpec)

	def _onClose(self):
		GraphAlgorithms.killAllDisplayedGraph()
		self.master.destroy()

	def _renderMap(self):
		self.map.render(self.canvas.tkCanvas)
		# Draw Axis
		# Drawing.CreateLine(self.canvas.tkCanvas, [[0, 0], [5, 0]], "PURPLE", "X-AXIS", width=2, arrow=True)
		# Drawing.CreateText(self.canvas.tkCanvas, [5, -5], "X", "X-AXIS-L")
		# Drawing.CreateLine(self.canvas.tkCanvas, [[0, 0], [0, 5]], "PURPLE", "Y-AXIS", width=2, arrow=True)
		# Drawing.CreateText(self.canvas.tkCanvas, [-5, 5], "Y", "Y-AXIS-L")

	def _renderTrajectories(self, force=False):
		if not force and not self.shouldRenderTrajectory: return
		trajectories = self.observations.trajectories
		t = self.observations.timeArray[self.fovIndex] if self.shouldShowFOV else None
		for trajectory in trajectories:
			trajectory.render(self.canvas.tkCanvas, time=t)

	def _clearTrajectories(self, force=False):
		if not force and self.shouldRenderTrajectory: return
		trajectories = self.observations.trajectories
		for trajectory in trajectories:
			trajectory.clear(self.canvas.tkCanvas)

	def _onSpecChange(self, specName: str):
		self._clearSpec()
		self._renderSpec(specName)
		self.specDropdownText.set("Selected Spec: %s" % self.selectedSpec)

	def _clearSpec(self):
		self.spec.nfa.killDisplayedGraph()
		self.spec.clearRender(self.canvas.tkCanvas)

	def _renderSpec(self, specName: str):
		self.selectedSpec = specName
		self.spec.render(self.canvas.tkCanvas)

	@property
	def _maxEventIndex(self):
		if self.shadowTree is None: return 0
		return len(self.shadowTree.graphs) - 1

	@property
	def _eventLabelText(self):
		if self.shadowTree is None: return "N/A"
		return "0 ≤ Ev %d ≤ %d" % (self.eventIndex, self._maxEventIndex)

	@property
	def _observationLabelText(self):
		# return "0 ≤ FOV %d ≤ %d" % (self._fovIndex, self._maxFovIndex)
		return "[%d] %d FOVs --> %s" % (self.fovIndex, self._maxObservationIndex + 1, self._eventLabelText)

	def _toggleTrajectory(self, force=False):
		self._clearTrajectories(force)
		self._renderTrajectories(force)

	def _toggleEvents(self):
		self._clearEvents()
		self._renderEvents()

	def _clearEvents(self, force=False):
		self.fovRenderer.clearRender(self.canvas.tkCanvas)
		self._renderTrajectories(force=True)

	def _renderEvents(self):
		if self.shadowTree is None: return
		if not self.shouldRenderEvent: return
		self._clearFov()
		self._clearTrajectories(force=True)
		self.fovLabel.set(self._observationLabelText)
		cGraph = self.shadowTree.graphs[self.eventIndex]
		self.fovRenderer.render(cGraph, self.canvas.tkCanvas)

	def _changeEvent(self, showNext: bool):
		self._clearEvents(force=True)
		self.eventIndex += 1 if showNext else -1
		self.eventIndex = min(self._maxEventIndex, self.eventIndex)
		self.eventIndex = max(0, self.eventIndex)
		self._renderEvents()

	def _nextEvent(self):
		self._changeEvent(True)

	def _prevEvent(self):
		self._changeEvent(False)

	def _onDbgAction(self, actionName: str):
		callback = self.dbgActions[actionName]
		callback()
		self.dbgActionValue.set("Debug Actions")

	def _toggleFov(self):
		self._clearFov()
		self._toggleTrajectory(force=True)
		self._renderFov()

	def _clearFov(self):
		self.fovRenderer.clearRender(self.canvas.tkCanvas)
		self._renderTrajectories(force=True)

	def _renderFov(self):
		if not self.shouldShowFOV: return
		self._clearEvents(force=True)
		self._clearTrajectories(force=True)
		cGraph = ConnectivityGraph(self.map, self.observationToRender.fov.polygon , self.observationToRender.tracks, self.observationToRender.time, self.spec.validators)
		self.fovRenderer.render(cGraph, self.canvas.tkCanvas)

	def _changeFov(self, showNext: bool):
		self._clearFov()
		self.fovIndex += 1 if showNext else -1
		self.fovIndex = min(self._maxObservationIndex, self.fovIndex)
		self.fovIndex = max(0, self.fovIndex)
		self.fovLabel.set(self._observationLabelText)
		self._toggleTrajectory(force=True)
		self._renderFov()

	def _nextFov(self):
		self._changeFov(True)

	def _prevFov(self):
		self._changeFov(False)

	@property
	def shouldRenderEvent(self) -> bool:
		return self.dbgCheckboxes["Show Event"].get() == 1

	@property
	def shouldRenderTrajectory(self) -> bool:
		return self.dbgCheckboxes["Render Trajectory"].get() == 1

	@property
	def shouldShowFOV(self) -> bool:
		return self.dbgCheckboxes["Show FOV"].get() == 1

	@property
	def displaySpaceTime(self) -> bool:
		return self.dbgCheckboxes["Space-Time Diagram"].get() == 1

	@property
	def displayShadowTree(self) -> bool:
		return self.dbgCheckboxes["Shadow Tree Graph"].get() == 1

	@property
	def spec(self) -> Specification:
		return next(spec for spec in self.specs if spec.name == self.selectedSpec)

	@property
	def observationToValidate(self) -> Observation:
		return self.observations.getObservationByIndex(self.validationIndex)

	@property
	def observationToRender(self) -> Observation:
		return self.observations.getObservationByIndex(self.fovIndex)

	def _showSpecGraph(self):
		self.spec.nfa.killDisplayedGraph()
		self.spec.nfa.displayGraph()

	def _createConnectivityGraph(self):
		cGraph = ConnectivityGraph(self.map, self.observationToRender.fov.polygon, self.observationToRender.tracks, self.observationToRender.time, self.spec.validators)
		cGraph.displayGraph(self.displaySpaceTime, self.displayShadowTree)

	def _createSingleLayerShadowTree(self):
		if self.fovIndex == self._maxObservationIndex:
			previousIndex = self.fovIndex - 1
			nextIndex = self.fovIndex
		else:
			previousIndex = self.fovIndex
			nextIndex = self.fovIndex + 1
		fovs = [self.observations.getObservationByIndex(previousIndex).fov, self.observations.getObservationByIndex(nextIndex).fov]
		self.shadowTree = ShadowTreeV2(self.map, fovs, self.spec.validators)
		self.shadowTree.displayGraph(self.displaySpaceTime, self.displayShadowTree)
		[Drawing.CreatePolygon(self.canvas.tkCanvas, list(p.exterior.coords), "RED", "", 2, "RED-P") for p in self.shadowTree.redPolys]
		[Drawing.CreatePolygon(self.canvas.tkCanvas, list(p.exterior.coords), "BLUE", "", 2, "BLUE-P") for p in self.shadowTree.bluePolys]
		# Drawing.CreateCircle(self.canvas.tkCanvas, self.shadowTree.blueVert[0], self.shadowTree.blueVert[1], 3, "BLUE", "VV")
		[Drawing.CreateLine(self.canvas.tkCanvas, list(l.coords), "MAROON", "LL", 2) for l in self.shadowTree.lines]
		self.fovLabel.set(self._observationLabelText)

	def _createShadowTree(self):
		fovs = [self.observations[o].fov for o in self.observations]
		# self.shadowTree = ShadowTree(False, self.map, fovs, self.spec.validators, self.observations.tracks)
		self.shadowTree = ShadowTreeV2()
		self.shadowTree._update(self.map, None,    fovs[0], self.spec.validators, self.observations.tracks)
		self.shadowTree._update(self.map, fovs[0], fovs[1], self.spec.validators, self.observations.tracks)
		self.shadowTree.displayGraph(self.displaySpaceTime, self.displayShadowTree)
		self.fovLabel.set(self._observationLabelText)

	def _createButton(self, row, col, text, callback, textVariable=None):
		tk.Grid.columnconfigure(self.frame, col, weight=1)
		btn = None
		if textVariable is not None:
			btn = tk.Button(self.frame, textvariable=textVariable)
		else:
			btn = tk.Button(self.frame)
			btn["text"] = text
		btn["command"] = callback
		btn.grid(row=row, column=col, sticky=tk.N + tk.S + tk.E + tk.W)

	def _createTkGridRow0(self):
		row = 0
		column = 0
		actionNames = list(self.dbgActions.keys())
		dbgActionsMenu = tk.OptionMenu(self.frame, self.dbgActionValue, *actionNames, command=self._onDbgAction)
		dbgActionsMenu.grid(row=row, column=column, sticky="ew")
		column += 1

		specList = [spec.name for spec in self.specs]
		specMenu = tk.OptionMenu(self.frame, self.specDropdownText, *specList, command=self._onSpecChange)
		specMenu.grid(row=row, column=column, sticky="ew")
		column += 1

		self._createButton(row, column, "Spec NFA", self._showSpecGraph)
		column += 1

		self._createButton(row, column, "< Event", self._prevEvent)
		column += 1
		self._createButton(row, column, "Event >", self._nextEvent)
		column += 1
		return

	def _createTkGridRow1(self):
		column = 0
		tk.Grid.columnconfigure(self.frame, 0, weight=1)
		label = tk.Label(master=self.frame, textvariable=self.fovLabel)
		label.grid(row=1, column=column, columnspan=1)

		column = 1
		for (text, variable) in self.dbgCheckboxes.items():
			tk.Grid.columnconfigure(self.frame, column, weight=1)
			checkbox = tk.Checkbutton(master=self.frame, text=text, variable=variable)
			if text == "Show FOV":
				# Skip adding this checkbox, but kept the code for debug
				# checkbox["command"] = self._toggleFov
				# checkbox.grid(row=1, column=column)
				pass
			elif text == "Render Trajectory":
				# Skip adding this checkbox, but kept the code for debug
				# checkbox["command"] = self._toggleTrajectory
				# checkbox.grid(row=1, column=column)
				pass
			elif text == "Show Event":
				checkbox["command"] = self._toggleEvents
				checkbox.grid(row=1, column=column)
				column += 1
			else:
				checkbox.grid(row=1, column=column)
				column += 1
		return

	def _validateSpec(self):
		print("Validating specification %s" % repr(self.spec))
		certificate = self.spec.nfa.readAll(self.map, self.observations)
		print("Following Space time set satisfied the spec:\n\t%s" % repr(certificate))
		# self.previousObservation = observation
		# spec.nfa.displayGraph()
		return
