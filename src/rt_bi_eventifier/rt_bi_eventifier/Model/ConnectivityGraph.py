from dataclasses import dataclass
from typing import Sequence

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import EdgeData, NxUtils
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import GraphPolygon, MapPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon


class ConnectivityGraph(NxUtils.Graph[GraphPolygon]):
	""" The implementation of a Connectivity Graph in python as described in the dissertation. """
	@dataclass(frozen=True)
	class NodeData(NxUtils.NodeData[GraphPolygon]): ...

	def __init__(
			self,
			timeNanoSecs: int,
			mapPolys: list[MapPolygon],
			sensorPolys: list[SensingPolygon],
			rvizPublisher: Ros.Publisher | None = None,
	) -> None:
		super().__init__(rvizPublisher)
		self.timeNanoSecs = timeNanoSecs
		self.__hIndex: int | None = None
		self.__map: list[MapPolygon] = []
		self.__mapIdToIndex: dict[NxUtils.Id, int] = {}
		self.__sensors: list[SensingPolygon] = []
		self.__sensorIdToIndex: dict[NxUtils.Id, int] = {}
		self.__shadows: list[MapPolygon] = []
		self.__antiShadows: list[SensingPolygon] = []
		Ros.Log(f"Constructing Connectivity Graph @ {self.timeNanoSecs}")
		for poly in mapPolys + sensorPolys: poly.id.copy(hIndex=self.__hIndex)
		self.__constructMap(polys=mapPolys)
		self.__constructSensors(polys=sensorPolys)
		self.__constructNodes()
		self.__constructEdges()

	def __repr__(self):
		return f"CGr-{self.timeNanoSecs}(N={len(self.nodes)}, E={len(self.edges)})"

	def __contains__(self, id_: NxUtils.Id | Sequence[NxUtils.Id]) -> bool:
		if isinstance(id_, NxUtils.Id):
			id_ = id_.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
			return super().__contains__(id_)
		sansHIndex = []
		for id_i in id_: sansHIndex.append(id_i.copy(hIndex=-1))
		return super().__contains__(sansHIndex)

	def __constructMap(self, polys: Sequence[MapPolygon]) -> None:
		if len(polys) == 0: return
		assert (
			polys[0].type == MovingPolygon.type or
			polys[0].type == StaticPolygon.type
		), f"Unexpected input polygon type: {polys[0].type}"
		for poly in polys:
			self.__mapIdToIndex[poly.id.copy(timeNanoSecs=-1, hIndex=-1, subPartId="")] = len(self.map)
			self.map.append(poly)
		Ros.Log(f"Stored {len(self.map)} map polygons.")
		return

	def __constructSensors(self, polys: Sequence[SensingPolygon]) -> None:
		if len(polys) == 0: return
		assert polys[0].type == SensingPolygon.type, f"Unexpected input polygon type: {polys[0].type}"
		for poly in polys:
			self.__sensorIdToIndex[poly.id.copy(timeNanoSecs=-1, hIndex=-1, subPartId="")] = len(self.sensors)
			self.sensors.append(poly)
		Ros.Log(f"Stored {len(self.sensors)} sensors.")
		return

	def __constructEdges(self) -> None:
		# Add edges to neighboring nodes
		# FIXME: What to do with tracklets?
		for nodeId1 in self.nodes:
			for nodeId2 in self.nodes:
				if nodeId1 == nodeId2: continue
				poly1 = self.getContent(nodeId1, "polygon")
				poly2 = self.getContent(nodeId2, "polygon")
				if poly1.intersects(poly2) or poly1.hasCommonEdge(poly2):
					self.addEdge(nodeId1, nodeId2, addReverseEdge=True)
		return

	def __constructNodes(self) -> None:
		from uuid import uuid4
		shadowPolys: list[MapPolygon] = []
		antiShadowPolys: list[SensingPolygon] = []
		if len(self.sensors) == 0:
			for mapPoly in self.map:
				shadowPolys.append(type(mapPoly)(
					polygonId=mapPoly.id.polygonId,
					regionId=mapPoly.id.regionId,
					subPartId="",
					envelope=[],
					interior=mapPoly.interior,
					timeNanoSecs= self.timeNanoSecs,
					hIndex=self.hIndex if self.hIndex is not None else -1,
					predicates=mapPoly.predicates,
					centerOfRotation=mapPoly.centerOfRotation,
					envelopeColor=ColorNames.ORANGE,
				))
		else:
			for sensor in self.sensors:
				for mapPoly in self.map:
					sensedPolys = GeometryLib.intersection(mapPoly.interior, sensor.interior)
					sensedPolys = GeometryLib.filterPolygons(sensedPolys)
					for i in range(len(sensedPolys)):
						antiShadowPolys.append(SensingPolygon(
							polygonId=sensor.id.polygonId,
							regionId=sensor.id.regionId,
							subPartId=uuid4().hex,
							envelope=[],
							interior=sensedPolys[i],
							timeNanoSecs= self.timeNanoSecs,
							hIndex=self.hIndex if self.hIndex is not None else -1,
							predicates=mapPoly.predicates,
							centerOfRotation=sensor.centerOfRotation,
						))
			for mapPoly in self.map:
				diff = mapPoly.interior
				for sensor in self.sensors:
					diffPolys = GeometryLib.difference(diff, sensor.interior)
					diff = Shapely.MultiPolygon(diffPolys)
				diff = GeometryLib.filterPolygons(diff)
				for i in range(len(diff)):
					shadowPolys.append(type(mapPoly)(
						polygonId=mapPoly.id.polygonId,
						regionId=mapPoly.id.regionId,
						subPartId=uuid4().hex,
						envelope=[],
						interior=diff[i],
						timeNanoSecs= self.timeNanoSecs,
						hIndex=self.hIndex if self.hIndex is not None else -1,
						predicates=mapPoly.predicates,
						centerOfRotation=mapPoly.centerOfRotation,
						envelopeColor=ColorNames.ORANGE,
					))

		if len(shadowPolys) == 0: Ros.Logger().warn("No shadows produced.")
		for shadow in shadowPolys:
			self.shadows.append(shadow)
			self.addNode(shadow.id, self.NodeData(polygon=shadow))
		for antiShadow in antiShadowPolys:
			self.antiShadows.append(antiShadow)
			self.addNode(antiShadow.id, self.NodeData(polygon=antiShadow))
		Ros.Log(f"Constructed {len(self.shadows)} shadows and {len(self.antiShadows)} anti-shadows.")
		return

	def addNode(self, id: NodeId, content: NodeData) -> NodeId:
		id = id.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		return super().addNode(id, content)

	def addEdge(self, frmId: NodeId, toId: NodeId, addReverseEdge=False, content: EdgeData | None = None) -> None:
		frmId = frmId.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		toId = toId.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		return super().addEdge(frmId, toId, addReverseEdge, content)

	@property
	def hIndex(self) -> int | None:
		return self.__hIndex

	@hIndex.setter
	def hIndex(self, value: int) -> None:
		assert value >= 0, f"hIndex must be non-negative. given value = {value}"
		Ros.Log(f"Setting hIndex of {repr(self)} to {value}")
		self.__hIndex = value
		for poly in self.map + self.sensors + self.shadows + self.antiShadows:
			poly.id = poly.id.copy(hIndex=value)

	@property
	def map(self) -> list[MapPolygon]:
		return self.__map

	@property
	def sensors(self) -> list[SensingPolygon]:
		return self.__sensors

	@property
	def shadows(self) -> list[MapPolygon]:
		return self.__shadows

	@property
	def antiShadows(self) -> list[SensingPolygon]:
		return self.__antiShadows

	def getMapPoly(self, id_: NxUtils.Id) -> MapPolygon:
		id__ = id_.copy(timeNanoSecs=-1, hIndex=-1, subPartId="")
		assert id__ in self.__mapIdToIndex, f"Sensor has gone missing {id_}!"
		return self.map[self.__mapIdToIndex[id__]]

	def getSensor(self, id_: NxUtils.Id) -> SensingPolygon:
		id__ = id_.copy(timeNanoSecs=-1, hIndex=-1, subPartId="")
		assert id__ in self.__sensorIdToIndex, f"Sensor has gone missing {id_}!"
		return self.sensors[self.__sensorIdToIndex[id__]]

	def hasSensor(self, id_: NxUtils.Id) -> bool:
		id__ = id_.copy(timeNanoSecs=-1, hIndex=-1, subPartId="")
		return id__ in self.__sensorIdToIndex

	def createNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for antiShadow in self.antiShadows:
			Ros.ConcatMessageArray(markers, antiShadow.createMarkers(renderText=True, stamped=False))
		for shadow in self.shadows:
			Ros.ConcatMessageArray(markers, shadow.createMarkers(renderText=True, stamped=False))
		id = RViz.Id(hIndex=-1, timeNanoSecs=-1, regionId="CGraph", polygonId="Name", subPartId="")
		marker = RViz.createText(id, coords=(100, -10), text=f"{repr(self)}", outline=ColorNames.WHITE, idSuffix="txt")
		Ros.AppendMessage(markers, marker)
		return markers

	def createEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
		return []

	def logGraphNodes(self) -> None:
		Ros.Log("\tAntiShadows", [(s, s.timeNanoSecs) for s in self.antiShadows])
		Ros.Log("\tShadows", [(s, s.timeNanoSecs) for s in self.shadows])
		return
