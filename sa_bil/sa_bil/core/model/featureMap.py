from typing import Dict

class FeatureMap:
	"""
	Data structure to represent features as described by Semantic Map.
	"""
	def __init__(self, json: dict):
		self.features: Dict[str, Feature] = {}

class Feature:
	def __init__(self, name, raw: dict):
		self._raw = raw
		self.name = name
		self.visibleFromAbove = raw["visibility_AV"] == "yes"
		self.traversability = Traversability(raw["traversability_car"], raw["traversability_tank"])

	def __repr__(self) -> str:
		return "{ %s - visibleAbv: %s, Trv: { %s } }" % repr(self.name, repr(self.visibleFromAbove), repr(self.traversability))

class Traversability:
	def __init__(self, car: int, tank: int) -> None:
		self.car = car
		self.tank = tank

	def __repr__(self) -> str:
		return "{c: %d, t: %d}" % (self.car, self.tank)
