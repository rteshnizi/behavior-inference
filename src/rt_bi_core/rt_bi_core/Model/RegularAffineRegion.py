from typing import List, Set

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion


class RegularAffineRegion(RegularSpatialRegion):
	"""
	A Class to model a set of Regular Affine Region.
	"""

	NANO_CONSTANT = 10 ** 9
	MAX_UPDATE_DELAY_NS = 5 * NANO_CONSTANT
	"""
	### Core assumption:
	We allow at most a delay of 5s between updates from a certain sensor before we declare it off.
	"""

	def __init__(self, regions: List[AffineRegion] = []):
		super().__init__(regions=regions)

	def __and__(self, other: "RegularAffineRegion") -> Set[str]:
		return super().__and__(other)

	def __add__(self, other: "RegularAffineRegion") -> Set[str]:
		return super().__add__(other)

	def __sub__(self, other: "RegularAffineRegion") -> Set[str]:
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> AffineRegion:
		return super().__getitem__(regionName)

	@property
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		else: return self[next(iter(self))].timeNanoSecs

	def addConnectedComponent(self, region: AffineRegion) -> None:
		return super().addConnectedComponent(region)

	def intersection(self, other: "RegularAffineRegion") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "RegularAffineRegion") -> Set[str]:
		return super().union(other)

	def difference(self, other: "RegularAffineRegion") -> Set[str]:
		return super().difference(other)
