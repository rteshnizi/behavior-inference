from abc import ABC

from rt_bi_commons.Shared.Pose import Coords
from rt_bi_core.Polygons.DynamicPolygon import DynamicPolygon


class AffinePolygon(DynamicPolygon, ABC):
	"""The base class for all moving polygons."""

	type = DynamicPolygon.Types.BASE
	def __init__(
			self,
			centerOfRotation: Coords,
			overlappingRegionIds: list[str] = [],
			overlappingRegionTypes: DynamicPolygon.Types = DynamicPolygon.Types.BASE,
			**kwArgs
		) -> None:
		"""
		:param Coords centerOfRotation: This will be used to interpolate the rotation motion of the region.
		:param list overlappingRegionIds: The id of the static regions which overlaps with this region, defaults to `[]`.
		:type overlappingRegionIds: list[str]
		:param Types overlappingRegionTypes: The id of the region which overlaps with this region, defaults to `Types.BASE`
		.. seealso::
			:class:`rt_bi_core.Polygons.DynamicPolygon.DynamicPolygon`.
			:class:`rt_bi_core.Polygons.Polygon.Polygon`.
		"""
		super().__init__(**kwArgs)
		self.centerOfRotation: Coords = centerOfRotation
		self.overlappingRegionIds = overlappingRegionIds
		self.overlappingRegionTypes = overlappingRegionTypes
