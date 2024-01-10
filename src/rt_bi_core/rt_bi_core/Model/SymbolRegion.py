from typing import Dict, Literal, Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_utils.Color import RGBA
from rt_bi_utils.Geometry import Geometry, Polygon
from rt_bi_utils.RViz import ColorNames


class SymbolRegion(AffineRegion):
	__colors: Dict[AffineRegion.RegionType, RGBA] = {
		AffineRegion.RegionType.BASE: ColorNames.DARK_RED,
		AffineRegion.RegionType.MAP: ColorNames.YELLOW,
		AffineRegion.RegionType.SENSING: ColorNames.CYAN,
		AffineRegion.RegionType.SHADOW: ColorNames.BLUE,
		AffineRegion.RegionType.SYMBOL: ColorNames.DARK_MAGENTA,
		AffineRegion.RegionType.TARGET: ColorNames.RED,
	}
	def __init__(
			self,
			centerOfRotation: Geometry.Coords,
			idNum: int,
			envelope: Geometry.CoordsList,
			timeNanoSecs: int,
			overlappingRegionType: AffineRegion.RegionType,
			overlappingRegionId: int,
			interior: Union[Polygon, None] = None,
			**kwArgs
		):
		super().__init__(
			centerOfRotation=centerOfRotation,
			idNum=idNum,
			envelope=envelope,
			envelopeColor=self.__colors[overlappingRegionType],
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)
		self.__overlappingRegionId = overlappingRegionId
		self.overlappingRegionType = overlappingRegionType

	@property
	def regionType(self) -> Literal[AffineRegion.RegionType.SYMBOL]:
		return self.RegionType.SYMBOL

	@property
	def name(self) -> str:
		return f"{super().shortName}-{self.overlappingRegionType.value}-{self.__overlappingRegionId}"

	def render(self, envelopeColor: Union[RGBA, None] = None, durationNs: int = -1) -> Sequence[Marker]:
		return super().render(renderText=True, envelopeColor=envelopeColor, durationNs=durationNs)
