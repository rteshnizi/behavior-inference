import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rt_bi_emulator"

setup(
	name=package_name,
	version="0.0.4",
	packages = find_packages(exclude=["test"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
		(os.path.join("share", package_name, "launch"), glob("launch/*")),
		(os.path.join("share", package_name, "config"), glob("config/*")),
	],
	install_requires= [
		"setuptools",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="A package to emulate a live scenario for debugging of the BI project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"MP = rt_bi_emulator.MapEmulator:main",
			"AV = rt_bi_emulator.AvEmulator:main",
			# "GvEmulator = rt_bi_emulator.GvEmulator:main",
		],
	},
)
