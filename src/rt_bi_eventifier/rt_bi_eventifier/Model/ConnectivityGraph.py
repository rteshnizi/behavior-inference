from dataclasses import dataclass
from typing import Sequence

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import GraphPolygon, MapPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.ShadowPolygon import ShadowPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon


class ConnectivityGraph(NxUtils.Graph[GraphPolygon]):
	""" The implementation of a Connectivity Graph in python as described in the dissertation. """
	@dataclass(frozen=True)
	class NodeData(NxUtils.NodeData[GraphPolygon]): ...

	def __init__(
			self,
			timeNanoSecs: int,
			mapPolys: Sequence[MapPolygon],
			sensorPolys: Sequence[SensingPolygon],
			rvizPublisher: Ros.Publisher | None = None,
	) -> None:
		super().__init__(rvizPublisher)
		self.timeNanoSecs = timeNanoSecs
		Ros.Log(f"Constructing Connectivity Graph @ {self.timeNanoSecs}")
		self.__map: list[MapPolygon] = []
		self.__sensors: list[SensingPolygon] = []
		self.__shadows: list[ShadowPolygon] = []
		Ros.Log("Constructing map.")
		self.__constructMap(polys=mapPolys)
		# Ros.Log("Constructing sensors.")
		# self.__constructSensors(polys=sensorPolys)
		Ros.Log("Constructing shadows.")
		self.__constructShadows()
		Ros.Log(f"Connectivity Graph containing {len(self.nodes)} nodes made.")

	def __repr__(self):
		return f"CGr-{self.timeNanoSecs}"

	def addNode(self, polygon: GraphPolygon) -> None:
		assert (polygon.type == SensingPolygon.type or polygon.type == ShadowPolygon.type), f"Unexpected node type {polygon.type}"

		if polygon.type == SensingPolygon.type: self.sensors.append(polygon)
		if polygon.type == ShadowPolygon.type: self.shadows.append(polygon)

		super().addNode(polygon.id, self.NodeData(polygon=polygon))
		return

	def __constructMap(self, polys: Sequence[MapPolygon]) -> None:
		if len(polys) == 0: return
		assert (
			polys[0].type == MovingPolygon.type or
			polys[0].type == StaticPolygon.type
		), f"Unexpected input polygon type: {polys[0].type}"
		for poly in polys:
			self.map.append(poly)
		return

	def __addSensorNodes(self, poly: SensingPolygon) -> None:
		self.addNode(poly)
		for sensor in self.sensors:
			if poly.id == sensor.id: continue
			if GeometryLib.intersects(poly.interior, sensor.interior):
				self.addEdge(poly.id, sensor.id, addReverseEdge=False)
		return

	def __constructSensors(self, polys: Sequence[SensingPolygon]) -> None:
		if len(polys) == 0: return
		assert polys[0].type == SensingPolygon.type, f"Unexpected input polygon type: {polys[0].type}"

		for poly in polys:
			for mapPoly in self.map:
				intersectionPolys = poly & mapPoly
				for i in range(len(intersectionPolys)):
					poly = SensingPolygon(
						polygonId=f"{poly.id.polygonId}-{i}",
						regionId=poly.id.regionId,
						envelope=[],
						interior=intersectionPolys[i],
						centerOfRotation=poly.centerOfRotation,
						timeNanoSecs=self.timeNanoSecs,
						predicates=mapPoly.predicates,
						tracklets=poly.tracklets,
					)
					self.__addSensorNodes(poly)
		Ros.Log(f"Constructed Sensors from {len(polys)} polys into {len(self.sensors)} partitions.")
		return

	def __addShadowNode(self, shadow: ShadowPolygon) -> None:
		# Assuming each shadow polygon belongs to a connected component.
		self.addNode(shadow)
		# Add edges to neighboring shadows
		for otherShadow in self.shadows:
			if shadow.id == otherShadow.id: continue
			if shadow.hasCommonEdge(otherShadow):
				self.addEdge(shadow.id, otherShadow.id, addReverseEdge=True)
		# FIXME: Add tracklet edges
		# for sensorId in self.sensors:
		# 	sensorPoly = self.sensors[sensorId]
		# 	if not sensorPoly.hasTrack: continue
		# 	if GeometryLib.haveOverlappingEdge(sensorPoly.interior, shadowPoly.interior):
		# 		for i in sensorPoly.tracklets:
		# 			tracklet = sensorPoly.tracklets[i]
		# 			if tracklet.spawned:
		# 				self.addEdge(shadowPoly.id, sensorPoly.id, addReverseEdge=False)
		# 			if tracklet.vanished:
		# 				self.addEdge(sensorPoly.id, shadowPoly.id, addReverseEdge=False)
		return

	def __constructShadows(self) -> None:
		shadowPolys: list[ShadowPolygon] = []
		if len(self.sensors) == 0:
			for mapPoly in self.map:
				shadowPolys.append(ShadowPolygon(
					polygonId=mapPoly.id.polygonId,
					regionId=mapPoly.id.regionId,
					envelope=[],
					interior=mapPoly.interior,
					timeNanoSecs= self.timeNanoSecs,
					predicates=mapPoly.predicates,
				))
		else:
			for mapPoly in self.map:
				for sensor in self.sensors:
					diff = mapPoly - sensor
					for i in range(len(diff)):
						shadowPolys.append(ShadowPolygon(
							polygonId=f"{mapPoly.id.polygonId}-{i}",
							regionId=mapPoly.id.regionId,
							envelope=[],
							interior=diff[i],
							timeNanoSecs= self.timeNanoSecs,
							predicates=mapPoly.predicates,
						))
						i += 1
		if len(shadowPolys) == 0: Ros.Logger().warn("No shadows produced.")
		for poly in shadowPolys:
			self.__addShadowNode(poly)
		return

	@property
	def map(self) -> list[MapPolygon]:
		return self.__map

	@property
	def sensors(self) -> list[SensingPolygon]:
		return self.__sensors

	@property
	def shadows(self) -> list[ShadowPolygon]:
		return self.__shadows

	def getNodeMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for shadowSet in self.sensors:
			Ros.ConcatMessageArray(markers, shadowSet.createMarkers())
		for shadowSet in self.shadows:
			Ros.ConcatMessageArray(markers, shadowSet.createMarkers())
		# Ros.ConcatMessageArray(markers, self.map.createMarkers())
		return markers

	def getEdgeMarkers(self) -> list[RViz.Msgs.Marker]:
		return []
