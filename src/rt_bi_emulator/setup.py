import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rt_bi_emulator"

setup(
	name=package_name,
	version="0.0.10",
	packages = find_packages(exclude=["test"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
		(os.path.join("share", package_name, "launch"), glob("launch/*")),
		(os.path.join("share", package_name, "config"), glob("config/*")),
	],
	install_requires= [
		"setuptools==58.2.0",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="A package to emulate a live scenario for debugging of the BI project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"EMP = rt_bi_emulator.Emulators.DynamicMapEmulator:main",
			"ESE = rt_bi_emulator.Emulators.SensorEmulator:main",
			"ESY = rt_bi_emulator.Emulators.SymbolRegionEmulator:main",
			"ETG = rt_bi_emulator.Emulators.TargetEmulator:main",
			"IMP = rt_bi_emulator.Renderers.MapServiceRenderer:main",
			"ISE = rt_bi_emulator.Renderers.SensorTopicRenderer:main",
			"ISY = rt_bi_emulator.Renderers.SymbolTopicRenderer:main",
			"ITG = rt_bi_emulator.Renderers.TargetTopicRenderer:main",
			"SMP = rt_bi_emulator.Emulators.SaMapEmulator:main",
		],
	},
)
