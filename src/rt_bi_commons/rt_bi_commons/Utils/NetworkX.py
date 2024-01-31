
from typing import Any, Dict, Mapping, Tuple


class NxUtils:
	NxDefaultLayout = Mapping[Any, Any]
	GraphLayout = Dict[str, Tuple[float, float]]
	"""A dictionary from node to an (X, Y) coordinate."""
