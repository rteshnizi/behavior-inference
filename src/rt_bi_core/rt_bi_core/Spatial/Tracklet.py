from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Shared.Pose import Pose
from rt_bi_commons.Utils.Msgs import NANO_CONVERSION_CONSTANT, Msgs
from rt_bi_commons.Utils.RViz import DEFAULT_RENDER_DURATION_NS, RViz


class Tracklet(Pose):
	__ID_PREFIX = "trk"

	def __init__(self, idStr: str, timeNanoSecs: int, hIndex: int, x: float, y: float, angleFromX: float, entered = False, exited = False) -> None:
		super().__init__(timeNanoSecs, x, y, angleFromX)
		self.id = idStr
		self.__rVizId = RViz.Id(hIndex=hIndex, timeNanoSecs=timeNanoSecs, regionId=self.id, polygonId=Tracklet.__ID_PREFIX, subPartId="")
		self.entered = entered
		self.exited = exited

	@property
	def hIndex(self) -> int:
		return self.__rVizId.hIndex

	def __repr__(self) -> str:
		(rId, polyId) = self.__rVizId.shortNames()
		s = f"{rId}/{polyId}"
		if self.entered and self.exited: return "%s%s" % ("[+/-]", s)
		if self.entered: return "%s%s" % ("[+]", s)
		if self.exited: return "%s%s" % ("[-]", s)
		return s

	def asMsg(self) -> Msgs.RtBi.Tracklet:
		msg = Msgs.RtBi.Tracklet()
		msg.id = self.id
		msg.pose = Msgs.toPoseMsg(self)
		msg.entered = self.entered
		msg.exited = self.exited
		return msg

	def createMarkers(self, durationNs: int = DEFAULT_RENDER_DURATION_NS) -> list[RViz.Msgs.Marker]:
		msgs = []
		if self.entered: color: RGBA = ColorNames.GREEN
		elif self.exited: color: RGBA = ColorNames.RED
		else: color: RGBA = ColorNames.CYAN
		coords = (self.x, self.y)
		durationNs = 3 * NANO_CONVERSION_CONSTANT if self.exited else durationNs
		msgs.append(RViz.createCircle(self.__rVizId.copy(timeNanoSecs=-1, hIndex=-1), coords, radius=5, outline=color, width=2.0, durationNs=durationNs))
		msgs.append(RViz.createText(self.__rVizId.copy(timeNanoSecs=-1, hIndex=-1), coords, repr(self), ColorNames.WHITE, durationNs=durationNs, idSuffix="txt"))
		return msgs

	def deleteMarkers(self) -> list[RViz.Msgs.Marker]:
		msgs = []
		msgs.append(RViz.removeMarker(self.__rVizId))
		msgs.append(RViz.removeMarker(self.__rVizId, idSuffix="txt"))
		return msgs
