from typing import Literal, Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_commons.Shared.TimeInterval import TimeInterval
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_commons.Utils.RViz import RGBA, ColorNames
from rt_bi_core.AffineRegion import AffineRegion
from rt_bi_interfaces.msg import MapRegion as MapRegionMsg, SpaceSpec, TimeInterval as TimeIntervalMsg


class MapRegion(AffineRegion):
	def __init__(self,
		id: str,
		envelope: Geometry.CoordsList,
		timeNanoSecs: int = 0,
		interior: Union[Polygon, MultiPolygon, None] = None,
		offIntervals: list[TimeInterval] = [],
		spec: SpaceSpec = SpaceSpec(),
		**kwArgs,
	) -> None:
		super().__init__(
			centerOfRotation=(0.0, 0.0),
			id=id,
			envelope=envelope,
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.WHITE),
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)
		self.centerOfRotation = Geometry.toCoords(self.interior.centroid)
		self.offIntervals: list[TimeInterval] = offIntervals
		self.spec: SpaceSpec = spec
		return

	@property
	def regionType(self) -> Literal[AffineRegion.RegionType.MAP]:
		return self.RegionType.MAP

	@property
	def resolvedBgColor(self) -> RGBA:
		return ColorNames.fromString(self.spec.color)

	def toMsg(self) -> MapRegionMsg:
		msg = MapRegionMsg()
		msg.id = self.id
		msg.region = self.toPolygonMsg()
		msg.spec.color = ColorNames.toStr(self.envelopeColor)
		msg.spec.off_intervals = []
		for interval in self.offIntervals:
			intervalMsg = TimeIntervalMsg()
			intervalMsg.start = Ros.NanoSecToTimeMsg(interval.startNanoSecs)
		return msg

	def render(self, renderText = False) -> Sequence[Marker]:
		self.BACKGROUND_COLOR = self.resolvedBgColor
		return super().render(renderText)
