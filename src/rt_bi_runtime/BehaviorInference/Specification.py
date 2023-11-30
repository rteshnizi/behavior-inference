from typing import List, Literal, Union

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorInference.Nfa import Nfa
from rt_bi_core.BehaviorInference.Symbol import Symbol

SpecName = Literal["TET1", "TWIST"]

class Specification:
	def __init__(self, name: SpecName, specificationDict: str):
		self.name: SpecName = name
		self.__specificationDict: str = specificationDict
		self.__shapeIds: Union[str, None] = None
		self.__regionSymbols: List[str] = []
		self.validators = self.__buildSymbolMap()
		self.nfa = Nfa(self.name, self.states, self.transitions, self.validators)

	@property
	def states(self):
		return self.__specificationDict["states"]

	@property
	def transitions(self):
		return self.__specificationDict["transitions"]

	@property
	def __validatorsDefs(self):
		return self.__specificationDict["validators"]

	def __repr__(self):
		return "SPEC-%s" % self.name

	def __buildSymbolMap(self):
		symbols = {}
		for symName in self.__validatorsDefs:
			idNum = RosUtils.RegisterRegionId(symName)
			symbols[symName] = Symbol(symName, self.__validatorsDefs[symName], idNum)
			if symbols[symName].isRegion: self.__regionSymbols.append(symName)
		return symbols

	def render(self):
		for symName in self.__regionSymbols:
			region = self.validators[symName].value
			region.render(renderText=True)

	def clearRender(self):
		for symName in self.__regionSymbols:
			region = self.validators[symName].value
			region.clearRender()
