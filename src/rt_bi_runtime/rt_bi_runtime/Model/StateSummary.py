from typing import List


class StateSummary:
	"""
		This is the weirdest name I could use.
		This represents a penny in what Dr. Shell referred to as a stack of pennies that will track a possible pose and its state.
		A penny represents an active hypothesis about the placement of a hidden target.
	"""
	def __init__(self, state: str, path: List[str]):
		"""
		Create a penny.

		Parameters
		----------
		state : str
			The name of the state in the NFA graph.
		path : List[str]
			The path in the shadow graph that has so far carried this penny over the transitions.
		"""
		self.state = state
		self.path = path

	def __repr__(self):
		return "(%s, %s)" % (self.state, self.path)

	def __hash__(self):
		return hash(repr(self))
