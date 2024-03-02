from dataclasses import dataclass
from typing import Any, Mapping, TypeVar


class NxUtils:
	@dataclass(frozen=True)
	class Id:
		"""
		An identifier `dataclass` for every polygons.
		:param str regionId: id of the region owning this polygon.
		:param str polygonId: id of the polygon.
		:param str overlappingRegionId: id of the region owning the overlapping polygon, defaults to ``""``.
		:param str overlappingPolygonId: id of the polygon this polygon lies upon, defaults to ``""``.
		"""
		regionId: str
		polygonId: str
		overlappingRegionId: str
		overlappingPolygonId: str

		def __repr__(self) -> str:
			return f"/{self.overlappingRegionId}/{self.overlappingPolygonId}/{self.regionId}/{self.polygonId}/"


	NxDefaultLayout = Mapping[Any, Any]
	__T = TypeVar("__T")
	GraphLayout = dict[__T, tuple[float, float]]
	"""A dictionary from node id to an (X, Y) coordinate."""
