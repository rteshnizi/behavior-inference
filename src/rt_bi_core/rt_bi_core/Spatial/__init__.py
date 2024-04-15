from typing import Any, TypeAlias

from typing_extensions import TypeVar

from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.DynamicPolygon import DynamicPolygon
from rt_bi_core.Spatial.Polygon import Polygon, PolygonFactoryKeys
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_core.Spatial.TargetPolygon import TargetPolygon

# Polygon Aliases
MapPolygon: TypeAlias = AffinePolygon | DynamicPolygon | StaticPolygon
GraphPolygon: TypeAlias = MapPolygon | SensingPolygon

_T_Polygon = TypeVar("_T_Polygon", bound=Polygon)


def PolygonFactory(PolyCls: type[_T_Polygon], kwArgs: dict[PolygonFactoryKeys, Any]) -> _T_Polygon:
	assert(
		PolyCls.type == AffinePolygon.type or
		PolyCls.type == DynamicPolygon.type or
		PolyCls.type == SensingPolygon.type or
		PolyCls.type == StaticPolygon.type or
		PolyCls.type == TargetPolygon.type
	), f"Unexpected Polygon type in factory: {PolyCls.type}"

	if PolyCls.type == StaticPolygon.type:
		kwArgs.pop("centerOfRotation")

	return PolyCls(**kwArgs)
