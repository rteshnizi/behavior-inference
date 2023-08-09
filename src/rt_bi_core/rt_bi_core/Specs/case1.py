from rt_bi_utils import ObjectLiteral


specification1 = ObjectLiteral(
	states=["START", "S-0", "S-1", "END"],
	transitions={
		"START": ["((T0, False), S-0)"],
		"S-0": ["((A, True), S-1)"],
		"S-1": ["((B, True), END)"]
	},
	validators={
		"A": "bil.spec.lambdas.Prototypes.P2.A",
		"B": "bil.spec.lambdas.Prototypes.P2.B",
		"T0": "bil.spec.lambdas.Prototypes.P2.T0",
		"C": "bil.spec.lambdas.Prototypes.P2.C"
	},
)
