from math import nan
from typing import TypedDict, cast

import requests
from geometry_msgs.msg import Point as PointMsg, Point32 as Point32Msg, Polygon as PolygonMsg, Pose as PoseMsg, Quaternion as QuaternionMsg

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Ros import Logger
from rt_bi_interfaces.msg import Polygon as RtBiPolygonMsg, RegularSpace as RegularSpaceMsg, TimeInterval, Traversability
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

	def __len__(self) -> int:
		return len(self.results)

	def __repr__(self) -> str:
		return repr(self.results)

	def __isVariable(self, varName: str) -> bool:
		return varName in self.variables

	def variablesAreDefined(self, i: int, varNames: list[str]) -> bool:
		return all(n in self.results[i] for n in varNames)

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
		if varName in self.results[i]:
			return self.results[i][varName]["value"]
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

	def parseInterval(self, helper: SparqlResultHelper, i: int) -> tuple[TimeInterval, int]:
		msg = TimeInterval(
			start=helper.floatVarValue(i, "t1"),
			end=helper.floatVarValue(i, "t2"),
			include_left=helper.floatVarValue(i, "minInclusive"),
			include_right=helper.floatVarValue(i, "maxInclusive"),
		)
		i =+ 1
		return (msg, i)

	def parsePolygon(self, helper: SparqlResultHelper, i: int) -> tuple[RtBiPolygonMsg, int]:
		msg = RtBiPolygonMsg(id=helper.strVarValue(i, "polygonId"))
		while i < len(helper):
			if not helper.variablesAreDefined(i, ["polygonId"]): return (msg, i)
			if helper.strVarValue(i, "polygonId") != msg.id: return (msg, i)
			x = helper.floatVarValue(i, "x")
			y = helper.floatVarValue(i, "y")
			Ros.AppendMessage(msg.region.points, Point32Msg(x=x, y=y, z=0.0))
			i += 1
		return (msg, i)

	def parseTraversability(self, helper: SparqlResultHelper, i: int) -> tuple[Traversability, int]:
		msg = Traversability(
			legs=helper.boolVarValue(i, "needsLegs"),
			wheels=helper.boolVarValue(i, "needsWheels"),
			swim=helper.boolVarValue(i, "needsSwim"),
		)
		i =+ 1
		return (msg, i)

	def filterSpaceTime(self, queryStr: str, res: SpaceTime.Response) -> SpaceTime.Response:
		r = requests.post(self.__SPARQL_URL, data={ "query": queryStr })
		try:
			helper = SparqlResultHelper(r)
			regSpaceMsgs: dict[str, RegularSpaceMsg] = {}
			# Assumes results are sorted by regular region's id
			i = 0
			while i < len(helper):
				idStr = cast(str, helper.strVarValue(i, "regularSpaceId"))
				if idStr not in regSpaceMsgs:
					regularSpaceMsg = RegularSpaceMsg(id=idStr)
					regSpaceMsgs[idStr] = regularSpaceMsg
				else:
					regularSpaceMsg = regSpaceMsgs[idStr]
				if helper.variablesAreDefined(i, ["name"]):
					regularSpaceMsg.spec.name = helper.strVarValue(i, "name")
					i += 1
					continue
				if helper.variablesAreDefined(i, ["color"]):
					regularSpaceMsg.spec.color = helper.strVarValue(i, "color")
					i += 1
					continue
				if helper.variablesAreDefined(i, ["minT", "maxT", "minInclusive", "maxInclusive"]):
					(interval, i) = self.parseInterval(helper, i)
					Ros.AppendMessage(regularSpaceMsg.spec.off_intervals, interval)
					continue
				if helper.variablesAreDefined(i, ["needsLegs", "needsWheels", "needsSwim"]):
					(regularSpaceMsg.spec.traversability, i) = self.parseTraversability(helper, i)
					continue
				if helper.variablesAreDefined(i, ["polygonId"]):
					(polygonMsg, i) = self.parsePolygon(helper, i)
					Ros.AppendMessage(regularSpaceMsg.polygons, polygonMsg)
					continue
				else:
					Logger().warn(f"Irregular sparql response structure. Possible reason could be that rq file is updated but this parser is not.")
			res.spatial_matches = list(regSpaceMsgs.values())
		except Exception as e:
			Logger().error(f"SPARQL request failed with the following message: {repr(e)}")
		return res
