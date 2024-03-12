from dataclasses import dataclass
from typing import Sequence, TypeVar

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import EdgeData, NodeData, NxUtils
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
		self.__sensors: list[SensingPolygon] = []
		self.__shadows: list[MapPolygon] = []
		Ros.Log(f"Constructing Connectivity Graph @ {self.timeNanoSecs}")
		for poly in mapPolys + sensorPolys: poly.id.copy(hIndex=self.__hIndex)
		self.__constructMap(polys=mapPolys)
		self.__constructSensors(polys=sensorPolys)
		self.__constructShadows()

	def __repr__(self):
		return f"CGr-{self.timeNanoSecs}"

	def addNode(self, id: NodeId, content: NodeData) -> NodeId:
		id = id.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		return super().addNode(id, content)

	def addEdge(self, frmId: NodeId, toId: NodeId, addReverseEdge=False, content: EdgeData | None = None) -> None:
		frmId = frmId.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		toId = toId.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		return super().addEdge(frmId, toId, addReverseEdge, content)

	def __constructMap(self, polys: Sequence[MapPolygon]) -> None:
		if len(polys) == 0: return
		assert (
			polys[0].type == MovingPolygon.type or
			polys[0].type == StaticPolygon.type
		), f"Unexpected input polygon type: {polys[0].type}"
		for poly in polys:
			self.map.append(poly)
		Ros.Log(f"Constructed {len(self.map)} map polygons.")
		return

	def __addSensorNode(self, poly: SensingPolygon) -> None:
		self.sensors.append(poly)
		self.addNode(poly.id, self.NodeData(polygon=poly))
		for sensor in self.sensors:
			if poly.id == sensor.id: continue
			if GeometryLib.intersects(poly.interior, sensor.interior):
				self.addEdge(poly.id, sensor.id, addReverseEdge=True)
		# for mapPoly in self.map:
		# 	intersectionPolys = poly & mapPoly
		# 	if intersectionPolys.is_empty:
		# 		self.__addSensorNodes(poly)
		# 	for i in range(len(intersectionPolys)):
		# 		poly = SensingPolygon(
		# 			polygonId=f"{poly.id.polygonId}-{i}",
		# 			regionId=poly.id.regionId,
		# 			envelope=[],
		# 			interior=intersectionPolys[i],
		# 			centerOfRotation=poly.centerOfRotation,
		# 			timeNanoSecs=self.timeNanoSecs,
		# 			predicates=mapPoly.predicates,
		# 			tracklets=poly.tracklets,
		# 		)
		return

	def __constructSensors(self, polys: Sequence[SensingPolygon]) -> None:
		if len(polys) == 0: return
		assert polys[0].type == SensingPolygon.type, f"Unexpected input polygon type: {polys[0].type}"
		for poly in polys:
			self.__addSensorNode(poly)
		Ros.Log(f"Constructed {len(self.sensors)} sensors.")
		return

	def __addShadowNode(self, shadow: MapPolygon) -> None:
		# Assuming each shadow polygon belongs to a connected component.
		self.shadows.append(shadow)
		self.addNode(shadow.id, self.NodeData(polygon=shadow))
		# Add edges to neighboring shadows
		for otherShadow in self.shadows:
			if shadow.id == otherShadow.id: continue
			if shadow.hasCommonEdge(otherShadow):
				self.addEdge(shadow.id, otherShadow.id, addReverseEdge=True)
		# Edge between shadows and sensors
		# FIXME: What to do with tracklets?
		for sensor in self.sensors:
			if sensor.hasCommonEdge(shadow):
				self.addEdge(shadow.id, sensor.id, addReverseEdge=True)
		return

	def __constructShadows(self) -> None:
		shadowPolys: list[MapPolygon] = []
		if len(self.sensors) == 0:
			for mapPoly in self.map:
				shadowPolys.append(type(mapPoly)(
					polygonId=mapPoly.id.polygonId,
					regionId=mapPoly.id.regionId,
					envelope=[],
					interior=mapPoly.interior,
					timeNanoSecs= self.timeNanoSecs,
					hIndex=self.hIndex if self.hIndex is not None else -1,
					predicates=mapPoly.predicates,
					centerOfRotation=mapPoly.centerOfRotation,
					envelopeColor=ColorNames.ORANGE,
				))
		else:
			for mapPoly in self.map:
				diff = mapPoly.interior
				for sensor in self.sensors:
					diffPolys = GeometryLib.difference(diff, sensor.interior)
					diff = Shapely.MultiPolygon(diffPolys)
				diff = GeometryLib.toGeometryList(diff)
				for i in range(len(diff)):
					shadowPolys.append(type(mapPoly)(
						polygonId=f"{mapPoly.id.polygonId}-{i}",
						regionId=mapPoly.id.regionId,
						envelope=[],
						interior=diff[i],
						timeNanoSecs= self.timeNanoSecs,
						hIndex=self.hIndex if self.hIndex is not None else -1,
						predicates=mapPoly.predicates,
						centerOfRotation=mapPoly.centerOfRotation,
						envelopeColor=ColorNames.ORANGE,
					))
		if len(shadowPolys) == 0: Ros.Logger().warn("No shadows produced.")
		for poly in shadowPolys:
			self.__addShadowNode(poly)
		Ros.Log(f"Constructed {len(self.shadows)} shadows.")
		return

	@property
	def hIndex(self) -> int | None:
		return self.__hIndex

	@hIndex.setter
	def hIndex(self, value: int) -> None:
		assert value >= 0, f"hIndex must be non-negative. given value = {value}"
		Ros.Log(f"Setting hIndex of {repr(self)} to {value}")
		self.__hIndex = value
		for poly in self.map + self.sensors + self.shadows:
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

	def getNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for sensor in self.sensors:
			Ros.ConcatMessageArray(markers, sensor.createMarkers(stamped=False))
		for shadow in self.shadows:
			Ros.ConcatMessageArray(markers, shadow.createMarkers(stamped=False))
		id = RViz.Id(hIndex=-1, timeNanoSecs=-1, regionId="CGraph", polygonId="Sensors")
		marker = RViz.createText(id, coords=(100, -10), text=f"Z = {len(self.sensors)}", outline=ColorNames.WHITE, idSuffix="txt")
		Ros.AppendMessage(markers, marker)
		id = RViz.Id(hIndex=-1, timeNanoSecs=-1, regionId="CGraph", polygonId="Shadows")
		marker = RViz.createText(id, coords=(100, -25), text=f"X = {len(self.shadows)}", outline=ColorNames.WHITE, idSuffix="txt")
		Ros.AppendMessage(markers, marker)
		id = RViz.Id(hIndex=-1, timeNanoSecs=-1, regionId="CGraph", polygonId="Time")
		marker = RViz.createText(id, coords=(100, -50), text=f"T = {self.timeNanoSecs}", outline=ColorNames.WHITE, idSuffix="txt")
		Ros.AppendMessage(markers, marker)
		return markers

	def getEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
		return []

	def logGraphNodes(self) -> None:
		Ros.Log("\tSensors", [(s, s.timeNanoSecs) for s in self.sensors])
		Ros.Log("\tShadows", [(s, s.timeNanoSecs) for s in self.shadows])
		return
