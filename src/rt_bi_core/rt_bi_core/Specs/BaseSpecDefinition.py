from typing import Dict, List


class BaseSpec:
	__slots__ = ("states", "transitions", "validators")
	def __init__(self, states: List[str], transitions: Dict[str, List[str]], validators: Dict[str, str]) -> None:
		self.states: List[str] = states
		"""The name of all the states in the behavior automaton. The two states `"START"` and `"END"` are always expected."""
		self.transitions: Dict[str, List[str]] = transitions
		"""
		A dictionary to define the transitions in the behavior automaton.

		* The key is the name of the state and the value is a list of all the outgoing transitions as described next.

		* The value is a list of outgoing transitions, each of which written as a pair: `((Validator, Bool), State)`.
		in which, (Validator, Bool) is a tuple wherein Validator is the string name of a validator from `self.validators`
		and Bool is a boolean indicating whether this transition would consume the input symbol and block the next transition until another inference is triggered.
		State is the name of the corresponding outgoing state.
		"""
		self.validators: Dict[str, str] = validators
		"""
		Validators is a dictionary from a short name to a long python function name.
		The python function name must be a string which is a python relative path in this current package.
		"""
