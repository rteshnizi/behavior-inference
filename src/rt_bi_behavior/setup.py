import os
from glob import glob

from setuptools import find_packages, setup

packageName = "rt_bi_behavior"

setup(
	name=packageName,
	version="0.9.0",
	packages = find_packages(exclude=["test", "launch"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + packageName]),
		("share/" + packageName, ["package.xml"]),
		(os.path.join("share", packageName, "launch"), glob("launch/*")),
		(os.path.join("share", packageName, "config"), glob("config/*")),
	],
	install_requires= [
		"networkx~=3.2",
		"setuptools==58.2.0",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="The behavior automaton implementation for the Behavior Inference project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"BA = rt_bi_behavior.BaNode:main",
			"BA_RENDERER = rt_bi_behavior.DotRenderer:main",
		],
	},
)
