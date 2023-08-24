from enum import Enum
from functools import partial
from math import inf
from typing import Callable, Union

from rt_bi_core.BehaviorAutomaton.SpaceTime import ProjectiveSpaceTimeSet
from rt_bi_core.BehaviorAutomaton.TimeRegion import TimeInterval
from rt_bi_utils.Geometry import Polygon


class LambdaType(Enum):
	Region = 1
	Time = 2

class NfaLambda:
	NfaLambdaFunc = Callable[[ProjectiveSpaceTimeSet], bool]
	def __init__(self, func, spaceTimeDescriptor: Union[list, str], lambdaType: LambdaType):
		self._originalFuncName = func.__name__
		if lambdaType == LambdaType.Region:
			# For region the description is a list of polygon vertices
			space = Polygon(spaceTimeDescriptor)
			time = TimeInterval((-1 * inf), inf, False, False)
		else:
			# For time, it's a string representation of the interval
			space = None
			intervalNums = spaceTimeDescriptor[1:-1].split(",")
			time = TimeInterval(float(intervalNums[0]), float(intervalNums[1]), spaceTimeDescriptor[0] == "[", spaceTimeDescriptor[-1] == "]")
		self.spaceTimeSet = ProjectiveSpaceTimeSet(space, time)
		self.func: NfaLambda.NfaLambdaFunc = partial(func, self.spaceTimeSet)
		self.type: LambdaType = lambdaType

	def __repr__(self):
		return "λ(%s, %s)" % (self._originalFuncName, self.type.name)
