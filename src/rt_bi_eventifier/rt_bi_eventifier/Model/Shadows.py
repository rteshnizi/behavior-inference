from typing import List, Sequence, Set

from visualization_msgs.msg import Marker

from rt_bi_core.Model.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion
from rt_bi_utils.RViz import KnownColors


class Shadows(RegularAffineRegion[ShadowRegion]):
	"""A Class to model Shadows."""
	def __init__(self, regions: List[ShadowRegion] = []):
		"""
		A dictionary from `region.name` to the `ShadowRegion` object.
		"""
		super().__init__(regions=regions)

	def __and__(self, others: "Shadows") -> Set[str]:
		return super().__and__(others)

	def __add__(self, others: "Shadows") -> Set[str]:
		return super().__add__(others)

	def __sub__(self, others: "Shadows") -> Set[str]:
		return super().__sub__(others)

	def __getitem__(self, regionName: str) -> ShadowRegion:
		return super().__getitem__(regionName)

	def addConnectedComponent(self, region: ShadowRegion) -> None:
		if region.regionType != ShadowRegion.RegionType.SHADOW: return
		return super().addConnectedComponent(region)

	def intersection(self, others: "Shadows") -> Set[str]:
		return super().intersection(others)

	def union(self, others: "Shadows") -> Set[str]:
		return super().union(others)

	def difference(self, others: "Shadows") -> Set[str]:
		return super().difference(others)

	def render(self) -> Sequence[Marker]:
		return super().render(envelopeColor=KnownColors.DARK_RED)
