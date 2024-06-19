from dataclasses import dataclass
from typing import Final, Sequence

from networkx import relabel_nodes

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.NetworkX import EdgeData, NxUtils
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import GraphPolygon, MapPolygon
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet


class ConnectivityGraph(NxUtils.Graph[GraphPolygon]):
	""" The implementation of a Connectivity Graph in python as described in the dissertation. """
	@dataclass(frozen=True)
	class NodeData(NxUtils.NodeData[GraphPolygon]): ...

	TRACKLET_EXIT_MAX_DISTANCE: Final[int] = 10
	def __init__(
			self,
			timeNanoSecs: int,
			mapPolys: list[MapPolygon],
			sensorPolys: list[SensingPolygon],
			rvizPublisher: Ros.Publisher | None = None,
	) -> None:
		super().__init__(rvizPublisher)
		self.timeNanoSecs = timeNanoSecs
		self.__hIndex: int = -1
		self.__map: list[MapPolygon] = []
		self.__mapIdToIndex: dict[NxUtils.Id, int] = {}
		self.__sensors: list[SensingPolygon] = []
		self.__sensorIdToIndex: dict[NxUtils.Id, int] = {}
		self.__shadows: list[MapPolygon] = []
		self.__antiShadows: list[SensingPolygon] = []
		self.__tracklet: Tracklet | None = None
		self.__numberOfPolygonVerts = -1
		Ros.Log(f"Constructing Connectivity Graph @ {self.timeNanoSecs}")
		for poly in mapPolys + sensorPolys: poly.id.copy(hIndex=self.__hIndex)
		self.__constructMap(polys=mapPolys)
		self.__constructSensors(polys=sensorPolys)
		self.__constructNodes()
		self.__constructEdges()

	@property
	def hasTrack(self) -> bool:
		return self.__tracklet is not None

	@property
	def fovEvent(self) -> bool:
		return self.hasTrack and self.track.entered or self.track.exited

	@property
	def track(self) -> Tracklet:
		assert self.__tracklet is not None, "First test with hasTrack"
		return self.__tracklet

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
			polys[0].type == AffinePolygon.type or
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
		if self.hasTrack and not self.fovEvent: return
		# Add edges to neighboring nodes
		for nodeId1 in self.nodes:
			poly1 = self.getContent(nodeId1, "polygon")
			if not poly1.isAccessible: continue
			for nodeId2 in self.nodes:
				if nodeId1 == nodeId2: continue
				poly2 = self.getContent(nodeId2, "polygon")
				if self.hasTrack and (poly1.type != SensingPolygon.type and poly2.type != SensingPolygon.type): continue
				if not poly2.isAccessible: continue
				# Sensors only have outgoing edges to shadows when track has exited
				if poly1.type == SensingPolygon.type:
					if poly2.type != SensingPolygon.type:
						if not poly1.trackExited: continue
				# Sensors only have ingoing edges from shadows when track has entered
				if poly2.type == SensingPolygon.type:
					if poly1.type != SensingPolygon.type:
						if not poly2.trackEntered: continue
				# if poly1.type == SensingPolygon.type and poly2.type == SensingPolygon.type: continue
				if poly1.intersects(poly2) or poly1.hasCommonEdge(poly2):
					other = None
					if poly1.type == SensingPolygon.type and poly1.hasTrack:
						other = poly2
					elif poly2.type == SensingPolygon.type and poly2.hasTrack:
						other = poly1
					# The line below would work if there was no delay.
					# if other is not None and self.__tracklet is not None and GeometryLib.distance(GeometryLib.toPoint(self.__tracklet), other.interior) > self.TRACKLET_EXIT_MAX_DISTANCE:
					if other is not None and self.hasTrack:
						p = GeometryLib.toPoint(self.track)
						if self.track.exited and not GeometryLib.intersects(p, other.interior): continue
						if self.track.entered and GeometryLib.distance(p, other.interior) > self.TRACKLET_EXIT_MAX_DISTANCE: continue
					self.addEdge(nodeId1, nodeId2)
		return

	def __extractTracklets(self, sensor: SensingPolygon, subPoly: Shapely.Polygon) -> dict:
		""" Only the subpart of the sensor that contains the tracklets should get it.
		* In cases of tracklet exiting, the closest subpart, takes it
		"""
		tracklets = {}
		for tId in sensor.tracklets:
			tracklet = sensor.tracklets[tId]
			p = GeometryLib.toPoint(tracklet)
			if not tracklet.exited:
				if GeometryLib.intersects(subPoly, p):
					tracklets[tId] = tracklet
					self.__tracklet = tracklet
			else:
				if GeometryLib.distance(subPoly, p) <= self.TRACKLET_EXIT_MAX_DISTANCE:
					tracklets[tId] = tracklet
					self.__tracklet = tracklet
		return tracklets

	def __constructNodes(self) -> None:
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
					envelopeColor=mapPoly.envelopeColor,
				))
		else:
			for sensor in self.sensors:
				for mapPoly in self.map:
					sensedPolys = GeometryLib.intersection(mapPoly.interior, sensor.interior)
					sensedPolys = GeometryLib.filterPolygons(sensedPolys)
					(rName, polyName) = mapPoly.id.shortNames()
					for i in range(len(sensedPolys)):
						tracklets = self.__extractTracklets(sensor, sensedPolys[i])
						antiShadowPolys.append(SensingPolygon(
							polygonId=sensor.id.polygonId,
							regionId=sensor.id.regionId,
							subPartId=f"{rName}-{polyName}-{i}",
							envelope=[],
							interior=sensedPolys[i],
							timeNanoSecs= self.timeNanoSecs,
							hIndex=self.hIndex if self.hIndex is not None else -1,
							predicates=mapPoly.predicates,
							centerOfRotation=sensor.centerOfRotation,
							tracklets=tracklets,
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
						subPartId=f"{i}",
						envelope=[],
						interior=diff[i],
						timeNanoSecs= self.timeNanoSecs,
						hIndex=self.hIndex if self.hIndex is not None else -1,
						predicates=mapPoly.predicates,
						centerOfRotation=mapPoly.centerOfRotation,
						envelopeColor=mapPoly.envelopeColor,
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
		self.__numberOfPolygonVerts += 0 if content.polygon is None else len(content.polygon.edges)
		return super().addNode(id, content)

	def addEdge(self, frmId: NodeId, toId: NodeId, addReverseEdge=False, content: EdgeData | None = None) -> None:
		frmId = frmId.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		toId = toId.copy(hIndex=-1) # CGraph nodes are do not have an hIndex
		return super().addEdge(frmId, toId, addReverseEdge, content)

	@property
	def hIndex(self) -> int:
		return self.__hIndex

	@hIndex.setter
	def hIndex(self, value: int) -> None:
		assert value >= 0, f"hIndex must be non-negative. given value = {value}"
		Ros.Log(f"Setting hIndex of {repr(self)} to {value}")
		self.__hIndex = value
		relabeledNodes = {}
		for id_ in self.nodes:
			poly = self.getContent(id_, "polygon")
			poly.id = poly.id.copy(hIndex=value)
			relabeledNodes[id_] = poly.id
		# Ros.Log(f"Labels", [(k, relabeledNodes[k]) for k in relabeledNodes])
		relabel_nodes(self, relabeledNodes, copy=False)
		for poly in self.map + self.sensors + self.shadows + self.antiShadows:
			poly.id = poly.id.copy(hIndex=value)
		return

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

	@property
	def numberOfPolygonVertices(self) -> int:
		return self.__numberOfPolygonVerts

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
