from typing import Dict, Literal, Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Utils.Geometry import Geometry, Polygon
from rt_bi_commons.Utils.RViz import ColorNames
from rt_bi_core.AffineRegion import AffineRegion


class SymbolRegion(AffineRegion):
	__colors: Dict[AffineRegion.RegionType, RGBA] = {
		AffineRegion.RegionType.BASE: ColorNames.RED_DARK,
		AffineRegion.RegionType.MAP: ColorNames.YELLOW,
		AffineRegion.RegionType.SENSING: ColorNames.CYAN,
		AffineRegion.RegionType.SHADOW: ColorNames.BLUE,
		AffineRegion.RegionType.SYMBOL: ColorNames.MAGENTA_DARK,
		AffineRegion.RegionType.TARGET: ColorNames.RED,
	}
	def __init__(
			self,
			centerOfRotation: Geometry.Coords,
			id: str,
			envelope: Geometry.CoordsList,
			timeNanoSecs: int,
			overlappingRegionType: AffineRegion.RegionType,
			overlappingRegionId: int,
			interior: Union[Polygon, None] = None,
			**kwArgs
		):
		super().__init__(
			centerOfRotation=centerOfRotation,
			id=id,
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
