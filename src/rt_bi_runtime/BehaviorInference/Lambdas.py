from rt_bi_runtime.BehaviorInference.Lambda import LambdaType, NfaLambda
from rt_bi_runtime.BehaviorInference.SpaceTime import ProjectiveSpaceTimeSet
from rt_bi_utils.Python import ObjectLiteral


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
			A = NfaLambda(Prototypes.delta, [[40, 30], [60, 30], [60, 20], [40, 20]], LambdaType.Region),
			B = NfaLambda(Prototypes.delta, [[30, 180], [50, 180], [50, 170], [30, 170]], LambdaType.Region),
			C = NfaLambda(Prototypes.delta, [[180, 140], [195, 140], [195, 160], [180, 160]], LambdaType.Region),
			T0 = NfaLambda(Prototypes.delta, "[10,10]", LambdaType.Time)
		)
		Prototypes.TET1 = ObjectLiteral(
			A = NfaLambda(Prototypes.delta, [[10, 40], [20, 40], [20, 30], [10, 30]], LambdaType.Region),
			B = NfaLambda(Prototypes.delta, [[90, 40], [80, 40], [80, 30], [90, 30]], LambdaType.Region),
			T0 = NfaLambda(Prototypes.delta, "[10,10]", LambdaType.Time)
		)
		Prototypes.TWIST = ObjectLiteral(
			A = NfaLambda(Prototypes.delta, [[10, 75], [20, 75], [20, 65], [10, 65]], LambdaType.Region),
			B = NfaLambda(Prototypes.delta, [[145, 15], [140, 15], [140, 10], [145, 10]], LambdaType.Region),
			T0 = NfaLambda(Prototypes.delta, "[11,11]", LambdaType.Time)
		)

	@staticmethod
	def delta(mySpaceTime: ProjectiveSpaceTimeSet, spaceTimeOfQuery: ProjectiveSpaceTimeSet) -> bool:
		"""### Delta

		Parameters
		----------
		mySpaceTime : ProjectiveSpaceTimeSet
			Mine
		spaceTimeOfQuery : ProjectiveSpaceTimeSet
			The other

		Returns
		-------
		bool
			Whether the transition should happen.
		"""
		return mySpaceTime.intersects(spaceTimeOfQuery)
