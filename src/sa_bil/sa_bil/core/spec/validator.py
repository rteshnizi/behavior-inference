import sa_bil.core # If you remove this, this file will throw an error
from sa_bil.core.model.polygonalRegion import PolygonalRegion
from sa_bil.core.observation.pose import Pose
from sa_bil.core.spec.lambdas import NfaLambda, Prototypes, LambdaType
class Validator(object):
	def __init__(self, name, lambdaStr):
		self.name = name
		# for Time, value is a float
		# myStr = "from sa_bil.core.utils.geometry import Geometry;" + lambdaStr + ";"
		self.lambdaObj: NfaLambda = eval(lambdaStr)
		self._lambdaString = lambdaStr
		# for Region, value is a polygonalRegion
		if self.isRegion:
			self.value = PolygonalRegion("sym-%s" % self.name, [], "BLUE", polygon=self.lambdaObj.spaceTimeSet.spaceRegion)
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
