import json
from math import nan
from typing import TypedDict, cast, overload

import requests
from geometry_msgs.msg import Point as PointMsg, Point32 as Point32Msg, Polygon as PolygonMsg, Pose as PoseMsg, Quaternion as QuaternionMsg

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Ros import Logger
from rt_bi_interfaces.msg import Polygon as RtBiPolygonMsg, Predicate, RegularSpace as RegularSpaceMsg, TimeInterval, Traversability
from rt_bi_interfaces.srv import SpaceTime


class SparqlData(TypedDict):
	type: str
	value: str

class SparqlResultHelper:
	def __init__(self, response: requests.Response) -> None:
		self.variables: list[str] = []
		self.results: list[dict[str, SparqlData]] = []
		response.raise_for_status()
		# From this point on we can assume the request succeeded.
		j = response.json()
		self.variables = j["head"]["vars"]
		self.results = j["results"]["bindings"]

	def __getitem__(self, index: int) -> dict[str, SparqlData]:
		return self.results[index]

	def __len__(self) -> int:
		return len(self.results)

	def __repr__(self) -> str:
		return repr(self.results)

	def __isVariable(self, varName: str) -> bool:
		return varName in self.variables

	@overload
	def contains(self, i: int, var: list[str]) -> bool: ...

	@overload
	def contains(self, i: int, var: str) -> bool: ...

	def contains(self, i: int, var: str | list[str]) -> bool:
		if isinstance(var, list):
			return all(n in self[i] for n in var)
		return var in self[i]

	def boolVarValue(self, i: int, varName: str) -> str:
		strVal = self.strVarValue(i, varName)
		if strVal == Traversability.TRUE or strVal == Traversability.FALSE: return strVal
		return ""

	def floatVarValue(self, i: int, varName: str) -> float:
		strVal = self.strVarValue(i, varName)
		if strVal is not None: return float(strVal)
		return nan

	def strVarValue(self, i: int, varName: str) -> str:
		if not self.__isVariable(varName):
			raise KeyError(f"No variable with name\"{varName}\" defined.")
		if varName in self[i]:
			return self[i][varName]["value"]
		return ""

class HttpInterface:
	def __init__(self, fusekiServerAdr: str, rdfStoreName: str) -> None:
		self.__fusekiServerAdr = fusekiServerAdr
		self.__rdfStoreName = rdfStoreName
		return

	@property
	def __SPARQL_URL(self) -> str:
		"""http://192.168.1.164:8090/rt-bi/"""
		return f"{self.__fusekiServerAdr}/{self.__rdfStoreName}/"

	def __parseInterval(self, helper: SparqlResultHelper, i: int) -> tuple[TimeInterval, int]:
		msg = TimeInterval(
			start=Ros.SecToTimeMsg(helper.floatVarValue(i, "minT")),
			end=Ros.SecToTimeMsg(helper.floatVarValue(i, "maxT")),
			include_left=json.loads(helper.boolVarValue(i, "minInclusive")),
			include_right=json.loads(helper.boolVarValue(i, "maxInclusive")),
		)
		i += 1
		return (msg, i)

	def __parsePolygons(self, helper: SparqlResultHelper, i: int, regularRegionId: str) -> tuple[list[RtBiPolygonMsg], int]:
		polys = []
		polyMsg = RtBiPolygonMsg()
		polyMsg.id = helper.strVarValue(i, "polygonId")
		while i < len(helper):
			if helper.strVarValue(i, "polygonId") != polyMsg.id:
				polys.append(polyMsg)
				if helper.strVarValue(i, "regularRegionId") != regularRegionId:
					return (polys, i)
				polyMsg = RtBiPolygonMsg()
				polyMsg.id = helper.strVarValue(i, "polygonId")
			x = helper.floatVarValue(i, "x")
			y = helper.floatVarValue(i, "y")
			Ros.AppendMessage(polyMsg.region.points, Point32Msg(x=x, y=y, z=0.0))
			i += 1
		polys.append(polyMsg)
		return (polys, i)

	def __parseTraversability(self, helper: SparqlResultHelper, i: int) -> tuple[Traversability, int]:
		msg = Traversability(
			legs=helper.boolVarValue(i, "legs"),
			wheels=helper.boolVarValue(i, "wheels"),
			swim=helper.boolVarValue(i, "swims"),
		)
		i += 1
		return (msg, i)

	def __parsePredicates(self, helper: SparqlResultHelper, i: int) -> list[Predicate]:
		predicates = []
		for var in helper.variables:
			if not var.startswith("p_"): continue
			predicate = Predicate()
			predicate.name = var
			if helper.contains(i, var):
				val = helper.strVarValue(i, var)
				if val != Predicate.TRUE and val != Predicate.FALSE:
					raise ValueError(f"Unexpected predicate value: {val}")
				predicate.value = val
			else:
				predicate.value = Predicate.INHERIT
			predicates.append(predicate)
		return predicates

	def staticReachability(self, queryStr: str, res: SpaceTime.Response) -> SpaceTime.Response:
		r = requests.post(self.__SPARQL_URL, data={ "query": queryStr })
		try:
			helper = SparqlResultHelper(r)
			i = 0
			while i < len(helper):
				msg = RegularSpaceMsg()
				msg.id = helper.strVarValue(i, "regularRegionId")
				msg.predicates = self.__parsePredicates(helper, i)
				(msg.polygons, i) = self.__parsePolygons(helper, i, msg.id)
				Ros.AppendMessage(res.spatial_matches, msg)
		except Exception as e:
			Logger().error(f"SPARQL request failed with the following message: {repr(e)}")
			raise e
		return res
