from dataclasses import asdict, dataclass, replace
from json import dumps, loads
from typing import Any


@dataclass(frozen=True, order=True)
class NodeId:
	"""
	An identifier `dataclass` for every graph node.
	:param int timeNanoSecs: The timestamp of the polygon state.
	:param str regionId: id of the region owning this polygon.
	:param str polygonId: id of the polygon.
	"""
	hIndex: int
	timeNanoSecs: int
	regionId: str
	polygonId: str
	subPartId: str

	def __repr__(self) -> str:
		hIndex = f"[{self.hIndex}]" if self.hIndex >= 0 else ""
		return f"{hIndex}@{self.timeNanoSecs}/{self.regionId}/{self.polygonId}/{self.subPartId}"

	def copy(self, timeNanoSecs: int | None = None, hIndex: int | None = None, subPartId: str | None = None) -> "NodeId":
		timeNanoSecs = timeNanoSecs if timeNanoSecs is not None else self.timeNanoSecs
		hIndex = hIndex if hIndex is not None else self.hIndex
		subPartId = subPartId if subPartId is not None else self.subPartId
		return replace(self, timeNanoSecs=timeNanoSecs, hIndex=hIndex, subPartId=subPartId)

	def shortNames(self) -> tuple[str, str]:
		polyId = self.polygonId.split('#')[-1].strip("_")
		regionId = self.regionId.split('/')[-1].strip("_")
		return (regionId, polyId)

	@staticmethod
	def fromJson(jsonStr: str) -> "NodeId":
		try:
			d: dict[str, Any] = loads(jsonStr)
			return NodeId.fromDict(d)
		except Exception as _:
			raise RuntimeError(f"Unable to parse string to NodeId: {jsonStr}")

	@staticmethod
	def fromDict(d: dict[str, Any]) -> "NodeId":
		try:
			return NodeId(
				hIndex=d["hIndex"],
				timeNanoSecs=d["timeNanoSecs"],
				regionId=d["regionId"],
				polygonId=d["polygonId"],
				subPartId=d["subPartId"],
			)
		except Exception as _:
			raise RuntimeError(f"Unable to parse dictionary to NodeId: {d}")

	def stringify(self) -> str:
		d = asdict(self)
		return dumps(d)
