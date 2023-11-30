import os
import pathlib
from glob import glob

from setuptools import find_packages, setup

packageName = pathlib.Path(__file__).parent.parent.name

setup(
	name=packageName,
	version="0.0.6",
	packages = find_packages(exclude=["test", "launch"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + packageName]),
		("share/" + packageName, ["package.xml"]),
		(os.path.join("share", packageName, "launch"), glob("launch/*")),
		(os.path.join("share", packageName, "config"), glob("config/*")),
	],
	install_requires= [
		"setuptools==58.2.0",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="The runtime environment of the Behavior Inference project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"BI = rt_bi_core.BehaviorInferenceRuntime:main",
		],
	},
)
