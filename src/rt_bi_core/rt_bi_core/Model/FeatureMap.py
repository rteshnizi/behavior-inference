from typing import Dict, Union


class FeatureMap:
	"""
	Data structure to represent features as described by Semantic Map.
	"""
	def __init__(self, json: dict):
		self.features: Dict[str, Feature] = {}

class Feature:
	def __init__(self, name, raw: dict = { "visibility_av": "yes", "traversability_gv_car": 1.0, "traversability_gv_tank": 1.0 }):
		self._raw = raw
		self.name = name
		self.visibleFromAbove = raw["visibility_av"] == "yes"
		self.traversability = Traversability(raw["traversability_gv_car"], raw["traversability_gv_tank"])

	def __repr__(self) -> str:
		return "{ %s - visibleAbv: %s, Trv: { %s } }" % repr(self.name, repr(self.visibleFromAbove), repr(self.traversability))

class Traversability:
	def __init__(self, car: float, tank: float) -> None:
		self.car = car
		self.tank = tank

	def __repr__(self) -> str:
		return "{c: %d, t: %d}" % (self.car, self.tank)
