from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Shared.Pose import Pose
from rt_bi_commons.Utils.RViz import DEFAULT_RENDER_DURATION_NS, RViz


class Tracklet(Pose):
	__ID_PREFIX = "trk"

	def __init__(self, idStr: str, timeNanoSecs: int, x: float, y: float, angleFromX: float, spawned = False, vanished = False) -> None:
		super().__init__(timeNanoSecs, x, y, angleFromX)
		self.id = idStr
		self.__rVizId = RViz.Id(timeNanoSecs=timeNanoSecs, regionId=self.id, polygonId=Tracklet.__ID_PREFIX)
		self.spawned = spawned
		self.vanished = vanished

	def __repr__(self) -> str:
		s = f"{self.id}/{Tracklet.__ID_PREFIX}"
		if self.spawned and self.vanished: return "%s%s" % ("[+/-]", s)
		if self.spawned: return "%s%s" % ("[+]", s)
		if self.vanished: return "%s%s" % ("[-]", s)
		return s

	def createMarkers(self, durationNs: int = DEFAULT_RENDER_DURATION_NS) -> list[RViz.Msgs.Marker]:
		msgs = []
		if self.spawned: color: RGBA = ColorNames.GREEN
		elif self.vanished: color: RGBA = ColorNames.RED
		else: color: RGBA = ColorNames.CYAN
		msgs.append(RViz.createCircle(self.__rVizId, self.x, self.y, radius=13, outline=color, width=3.0, durationNs=durationNs))
		msgs.append(RViz.createText(self.__rVizId, (self.x, self.y), repr(self), ColorNames.WHITE, durationNs=durationNs, idSuffix="txt"))
		return msgs

	def deleteMarkers(self) -> list[RViz.Msgs.Marker]:
		msgs = []
		msgs.append(RViz.removeMarker(self.__rVizId))
		msgs.append(RViz.removeMarker(self.__rVizId, idSuffix="txt"))
		return msgs
