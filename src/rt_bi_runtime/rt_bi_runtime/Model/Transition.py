import json
import re as RegEx

from rt_bi_runtime.Model.Symbol import Symbol


class Transition:
	def __init__(self, symbol: str, consuming: bool, nextState: str):
		self.symbol = symbol
		self.consuming = consuming
		self.nextState = nextState

	def __repr__(self):
		return "Δ(%s, %s)" % (self.symbol, "T" if self.consuming else "F")
