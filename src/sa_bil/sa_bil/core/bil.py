import os
from sa_bil.renderer import Renderer
# from sa_bil.core.gui.app import App
from sa_bil.core.observation.observations import Observations
from sa_bil.core.parser import EnvironmentParser, ObservationParser, SpecParser
from sa_bil.core.spec.lambdas import Prototypes

class Bil(object):
	def __init__(self, loadFromFile=False):
		Prototypes.initialize()
		self.loadFromFile = loadFromFile
		# self.dataPrototype = "TemporalEdgeTest-1"
		# self.dataPrototype = "TemporalEdgeTest-2"
		# self.dataPrototype = "TwistTest-NoTraj"
		# self.dataPrototype = "TwistTest-Traj1"
		self.dataPrototype = "Two-Rotating-Boxes"
		# self.dataPrototype = "Prototype-1"
		# self.dataPrototype = "Prototype-2"
		# self.dataPrototype = "MovingSensor-0"
		Renderer.init(self.dataPrototype)
		self.mockDataDir = os.path.abspath(os.path.join("data", "Mock", self.dataPrototype))
		self.envParser = EnvironmentParser(self.mockDataDir)
		self.featureMap = None
		self.map = None
		self.observations: Observations = None
		(self.featureMap, self.map) = self.envParser.parse()
		if loadFromFile:
			self.obsParser = ObservationParser(self.mockDataDir)
			self.observations = self.obsParser.parseNew(self.map, self.featureMap)
			# (self.scenario.observationOlds, self.scenario.agents) = self.obsParser.parse(self.map, self.scenario.fov)
		self.specParser = SpecParser(self.mockDataDir)
		self.specs = self.specParser.parse()
		# self.app = App(self)
		self.previousObservation = None

	def run(self):
		self.app.mainloop()
