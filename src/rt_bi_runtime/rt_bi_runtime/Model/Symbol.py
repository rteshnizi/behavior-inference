from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_runtime.Model.Lambda import LambdaType, NfaLambda
from rt_bi_utils.RViz import KnownColors


class Symbol:
	def __init__(self, name: str, lambdaStr: str, id: int) -> None:
		self.name = name
		# for Time, value is a float
		# myStr = "from sa_bil.core.utils.geometry import Geometry;" + lambdaStr + ";"
		self.lambdaObj: NfaLambda = eval(lambdaStr)
		self._lambdaString = lambdaStr
		# for Region, value is a polygonalRegion
		if self.isRegion:
			self.value = PolygonalRegion(id, envelope=[], envelopeColor=KnownColors.PURPLE, interior=self.lambdaObj.spaceTimeSet.spaceRegion)
		else:
			self.value = self.lambdaObj.spaceTimeSet.timeRegion

	@property
	def isRegion(self):
		"""
		Just for the sake of debugging
		"""
		return self.lambdaObj.type == LambdaType.Region

	def __repr__(self):
		return self._lambdaString

	def execute(self, p):
		return self.lambdaObj.func(p)
