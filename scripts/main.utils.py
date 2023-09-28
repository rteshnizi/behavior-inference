import logging
from argparse import ArgumentParser, Namespace
from json import loads
from math import degrees, radians
from typing import List

from rt_bi_utils.Geometry import AffineTransform, Geometry
from rt_bi_utils.Pose import Pose

logging.basicConfig(filemode=None, format="[%(levelname)s]: %(message)s", force=True, level=logging.INFO)

#region Configuration of CLI Arguments
argParser = ArgumentParser(
	description="Provides CLI access to some of Reza Teshnizi's python utility functionality.",
)

subParsers = argParser.add_subparsers(
	help="The utility command to execute.",
	dest="command",
)

#region Rotation Parser
rotationParser = subParsers.add_parser(
	"rotation",
	usage="%(prog)s \"[[283.602, 65.354], [296.297, 119.257], [120.804, 134.190]]\" -d 30 -c \"[120.804, 134.190]\"",
)
rotationParser.add_argument(
	"coordinates",
	type=str,
	help="The list of coordinates to be rotated."
)
rotationParser.add_argument(
	"--center", "-c",
	required=True,
	type=str,
	help="A coordinate to be used as the center of rotation.",
)
mxg = rotationParser.add_mutually_exclusive_group(required=True)
mxg.add_argument(
	"--degrees", "-d",
	type=float,
	help="The degrees to be used for rotation.",
)
mxg.add_argument(
	"--radians", "-r",
	type=float,
	help="The radians to be used for rotation.",
)
#endregion # cSpell: disable-line

#region Get Angle Parser
rotationParser = subParsers.add_parser(
	"angle",
	usage="%(prog)s \"[[283.602, 65.354], [296.297, 119.257], [120.804, 134.190]]\" \"[[200.612, -24.140], [244.985, 8.992], [120.148, 133.236]]\" -c \"[120.804, 134.190]\"",
)
rotationParser.add_argument(
	"coords1",
	type=str,
	help="First list of coordinates."
)
rotationParser.add_argument(
	"coords2",
	type=str,
	help="Second list of coordinates."
)
rotationParser.add_argument(
	"--center", "-c",
	required=True,
	type=str,
	help="A coordinate to be used as the center of rotation.",
)
#endregion # cSpell: disable-line

#endregion # cSpell: disable-line

def getAngle(coords1: str, coords2: str, center: str) -> str:
	coords1: Geometry.CoordsList = loads(coords1)
	coords2: Geometry.CoordsList = loads(coords2)
	center: Geometry.Coords = loads(center)
	center: Pose = Pose(0, center[0], center[1], 0)
	transformation = Geometry.getAffineTransformation(coords1, coords2, center)
	return (transformation.rotation, degrees(transformation.rotation))

def rotateCoords(coords: str, angleD: float, angleR: float, center: str) -> List[List[float]]:
	coords: Geometry.CoordsList = loads(coords)
	center: Geometry.Coords = loads(center)
	center: Pose = Pose(0, center[0], center[1], 0)
	angleR = radians(angleD) if angleR is None else angleR
	transformation = AffineTransform(rotation=angleR)
	transformed = Geometry.applyMatrixTransformToCoordsList(transformation, coords, center)
	transformed = [list(c) for c in transformed]
	return transformed

def main(args: Namespace) -> None:
	if args.command == "rotation":
		logging.info(rotateCoords(args.coordinates, args.degrees, args.radians, args.center))
	elif args.command == "angle":
		angles = getAngle(args.coords1, args.coords2, args.center)
		logging.info("%f rad = %f deg" % angles)
	return

if __name__ == "__main__":
	names = argParser.parse_args()
	main(names)