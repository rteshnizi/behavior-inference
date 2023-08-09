from rt_bi_core.BehaviorAutomaton.Validator import Validator
from rt_bi_core.BehaviorAutomaton.Nfa import NFA

class Specification:
	def __init__(self, name: str, specificationDict: str):
		self.name: str = name
		self._specificationDict: str = specificationDict
		self._shapeIds: str = None
		self._regionSymbols = []
		self.validators = self._buildSymbolMap()
		self.nfa = NFA(self.name, self.states, self.transitions, self.validators)

	@property
	def states(self):
		return self._specificationDict["states"]

	@property
	def transitions(self):
		return self._specificationDict["transitions"]

	@property
	def _validatorsDefs(self):
		return self._specificationDict["validators"]

	def __repr__(self):
		return "SPEC-%s" % self.name

	def _buildSymbolMap(self):
		symbols = {}
		for symName in self._validatorsDefs:
			symbols[symName] = Validator(symName, self._validatorsDefs[symName])
			if symbols[symName].isRegion: self._regionSymbols.append(symName)
		return symbols

	def render(self):
		for symName in self._regionSymbols:
			region = self.validators[symName].value
			region.render(renderText=True)

	def clearRender(self):
		for symName in self._regionSymbols:
			region = self.validators[symName].value
			region.clearRender()
