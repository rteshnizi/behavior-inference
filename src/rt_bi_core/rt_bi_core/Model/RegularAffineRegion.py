from typing import Generic, List, Set, TypeVar

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.RegularDynamicRegion import RegularDynamicRegion

RegionType = TypeVar("RegionType", bound=AffineRegion)

class RegularAffineRegion(Generic[RegionType], RegularDynamicRegion[RegionType]):
	"""
	A Class to model a set of Regular Affine Region.
	"""

	def __init__(self, regions: List[RegionType] = []):
		super().__init__(regions=regions)

	def __and__(self, other: "RegularAffineRegion") -> Set[str]:
		return super().__and__(other)

	def __add__(self, other: "RegularAffineRegion") -> Set[str]:
		return super().__add__(other)

	def __sub__(self, other: "RegularAffineRegion") -> Set[str]:
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> RegionType:
		return super().__getitem__(regionName)

	@property
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		else: return self[next(iter(self))].timeNanoSecs

	def addConnectedComponent(self, region: RegionType) -> None:
		return super().addConnectedComponent(region)

	def intersection(self, other: "RegularAffineRegion") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "RegularAffineRegion") -> Set[str]:
		return super().union(other)

	def difference(self, other: "RegularAffineRegion") -> Set[str]:
		return super().difference(other)
