from rt_bi_core.BehaviorAutomaton import LambdaType, NfaLambda, ProjectiveSpaceTimeSet
from rt_bi_utils import ObjectLiteral

class Prototypes:
	"""
	Each object literal represents one spec's validators

	Each validator receives two arguments:
	* first argument is the space-time of interest for this validator, and
	* the second one is the space-time of the NFA
	"""
	P2 = None
	TET1 = None
	TWIST = None
	@staticmethod
	def initialize():
		Prototypes.P2 = ObjectLiteral(
			# A = NfaLambda(Prototypes.funcP2A, [[29, 46], [56, 46], [56, -1], [29, -1]], LambdaType.Region),
			# B = NfaLambda(Prototypes.funcP2B, [[19, 201], [41, 201], [41, 154], [19, 154]], LambdaType.Region),
			# C = NfaLambda(Prototypes.funcP2C, [[180, 140], [195, 140], [195, 160], [180, 160]], LambdaType.Region),
			A = NfaLambda(Prototypes.funcP2A, [[40, 30], [60, 30], [60, 20], [40, 20]], LambdaType.Region),
			B = NfaLambda(Prototypes.funcP2B, [[30, 180], [50, 180], [50, 170], [30, 170]], LambdaType.Region),
			C = NfaLambda(Prototypes.funcP2C, [[180, 140], [195, 140], [195, 160], [180, 160]], LambdaType.Region),
			T0 = NfaLambda(Prototypes.funcP2T0, "[10,10]", LambdaType.Time)
		)
		Prototypes.TET1 = ObjectLiteral(
			A = NfaLambda(Prototypes.funcP2A, [[10, 40], [20, 40], [20, 30], [10, 30]], LambdaType.Region),
			B = NfaLambda(Prototypes.funcP2B, [[90, 40], [80, 40], [80, 30], [90, 30]], LambdaType.Region),
			T0 = NfaLambda(Prototypes.funcP2T0, "[10,10]", LambdaType.Time)
		)
		Prototypes.TWIST = ObjectLiteral(
			A = NfaLambda(Prototypes.funcP2A, [[10, 75], [20, 75], [20, 65], [10, 65]], LambdaType.Region),
			B = NfaLambda(Prototypes.funcP2B, [[145, 15], [140, 15], [140, 10], [145, 10]], LambdaType.Region),
			T0 = NfaLambda(Prototypes.funcP2T0, "[11,11]", LambdaType.Time)
		)

	@staticmethod
	def funcP2A(mySpaceTime: ProjectiveSpaceTimeSet, spaceTimeOfQuery: ProjectiveSpaceTimeSet) -> bool:
		return mySpaceTime.intersect(spaceTimeOfQuery)

	@staticmethod
	def funcP2B(mySpaceTime: ProjectiveSpaceTimeSet, spaceTimeOfQuery: ProjectiveSpaceTimeSet) -> bool:
		return mySpaceTime.intersect(spaceTimeOfQuery)

	@staticmethod
	def funcP2C(mySpaceTime: ProjectiveSpaceTimeSet, spaceTimeOfQuery: ProjectiveSpaceTimeSet) -> bool:
		return mySpaceTime.intersect(spaceTimeOfQuery)

	@staticmethod
	def funcP2T0(mySpaceTime: ProjectiveSpaceTimeSet, spaceTimeOfQuery: ProjectiveSpaceTimeSet) -> bool:
		return mySpaceTime.intersect(spaceTimeOfQuery)
