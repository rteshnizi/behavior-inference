from typing import TypedDict

import requests
from geometry_msgs.msg import Point as PointMsg, Point32 as Point32Msg, Polygon as PolygonMsg, Pose as PoseMsg, Quaternion as QuaternionMsg

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Ros import Logger
from rt_bi_interfaces.msg import MapRegion as MapRegionMsg
from rt_bi_interfaces.srv import StaticReachability


class SparqlData(TypedDict):
	type: str
	value: str

class SparqlResponseParser:
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

class HttpInterface:
	def __init__(self, fusekiServerAdr: str, rdfStoreName: str) -> None:
		self.__fusekiServerAdr = fusekiServerAdr
		self.__rdfStoreName = rdfStoreName
		return

	@property
	def sparqlUrl(self) -> str:
		"""http://192.168.1.164:8090/rt-bi-ontology/sparql"""
		return f"{self.__fusekiServerAdr}/{self.__rdfStoreName}/sparql"

	def staticReachability(self, queryStr: str, res: StaticReachability.Response) -> StaticReachability.Response:
		r = requests.post(self.sparqlUrl, data={ "query": queryStr })
		try:
			parsedResponse = SparqlResponseParser(r)
			mapRegionMsgs: dict[str, MapRegionMsg] = {}
			for i in range(len(parsedResponse)):
				idData = parsedResponse.results[i]["regionId"]
				xData = parsedResponse.results[i]["xVal"]
				yData = parsedResponse.results[i]["yVal"]
				idStr = f"{idData['value']}"
				if idStr not in mapRegionMsgs:
					mapRegionMsg = MapRegionMsg(id=idStr)
					mapRegionMsg.spec.color = parsedResponse.results[i]["regionColor"]["value"]
					mapRegionMsg.spec.traversability.legs = parsedResponse.results[i]["needsLegs"]["value"] == "true"
					mapRegionMsg.spec.traversability.wheels = parsedResponse.results[i]["needsWheels"]["value"] == "true"
					mapRegionMsg.spec.traversability.swim = parsedResponse.results[i]["needsSwim"]["value"] == "true"
					mapRegionMsgs[idStr] = mapRegionMsg
				else:
					mapRegionMsg = mapRegionMsgs[idStr]
				Ros.AppendMessage(mapRegionMsg.region.points, Point32Msg(x=float(xData["value"]), y=float(yData["value"]), z=0.0))
			res.regions = list(mapRegionMsgs.values())
		except Exception as e:
			Logger().error(f"SPARQL request failed with the following message: {repr(e)}")
		return res
