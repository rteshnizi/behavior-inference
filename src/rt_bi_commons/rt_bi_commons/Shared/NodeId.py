from dataclasses import dataclass, replace


@dataclass(frozen=True, order=True)
class NodeId:
	"""
	An identifier `dataclass` for every graph node.
	:param int timeNanoSecs: The timestamp of the polygon state.
	:param str regionId: id of the region owning this polygon.
	:param str polygonId: id of the polygon.
	"""
	timeNanoSecs: int
	regionId: str
	polygonId: str

	def __repr__(self) -> str:
		onStr = ""
		# onStr = f"ON//{self.overlappingRegionId}/{self.overlappingPolygonId}"
		# onStr = "" if onStr == "ON///" else onStr
		return f"@{self.timeNanoSecs}//{self.regionId}/{self.polygonId}/{onStr}"

	def updateTime(self, timeNanoSecs: int) -> "NodeId":
		return replace(self, timeNanoSecs=timeNanoSecs)
