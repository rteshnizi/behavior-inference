from typing import Union

from rt_bi_core.Model.TimeRegion import TimeRegion
from rt_bi_utils.Geometry import Geometry, Polygon


class ProjectiveSpaceTimeSet:
	"""
		A projective space time set. That is, the set is space x time.
		If `space` is `None`, there is no space requirement.
	"""
	def __init__(self, space: Union[Polygon, None], time: TimeRegion):
		self.spaceRegion = space
		self.timeRegion = time

	def __repr__(self) -> str:
		return "ST{%s x %s}" % (repr(self.spaceRegion), repr(self.timeRegion))

	def intersects(self, other: "ProjectiveSpaceTimeSet") -> bool:
		if not self.spaceIntersectionCheck(other.spaceRegion): return False
		if not self.timeRegion.intersects(other.timeRegion): return False
		return True

	def spaceIntersectionCheck(self, poly2) -> bool:
		if self.spaceRegion == None: return True
		intersection = Geometry.intersection(self.spaceRegion, poly2)
		try:
			return intersection.area > 0
		except:
			return intersection[0].area > 0
		return False