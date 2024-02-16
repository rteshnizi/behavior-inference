from pathlib import Path
from typing import cast

from rt_bi_commons.Utils import Ros
from rt_bi_runtime.Model.TransitionToSparql import TransitionParser


class Transition:
	def __init__(self, transitionStr: str, grammarFilePath: Path) -> None:
		self.transitionStr: str = transitionStr
		self.__parser = TransitionParser(grammarFilePath)

	def __repr__(self) -> str:
		return f"{self.transitionStr}"

	def __hash__(self) -> int:
		return hash(self.transitionStr)

	def __eq__(self, other: "Transition") -> bool:
		return self.transitionStr == other.transitionStr

	@property
	def toSparqlFilter(self) -> str:
		try:
			sparqlFilterStr = cast(str, self.__parser.parse(self.transitionStr))
			return sparqlFilterStr
		except Exception as e:
			Ros.Logger().error(f"Failed to evaluate transition function: {str(e)}")
			raise e
