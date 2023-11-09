import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rt_bi_core"

setup(
	name=package_name,
	version="0.0.5",
	packages = find_packages(exclude=["test", "launch"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
		(os.path.join("share", package_name, "launch"), glob("launch/*")),
		(os.path.join("share", package_name, "config"), glob("config/*")),
	],
	install_requires= [
		"isort",
		"networkx~=3.0",
		"setuptools==58.2.0",
		"numpy~=1.24"
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="The core package of the Behavior Inference project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"BA = rt_bi_core.BehaviorAutomatonInterface:main",
			"MI = rt_bi_core.MapServiceInterface:main",
			"SI = rt_bi_core.SensorTopicInterface:main",
			"ST = rt_bi_core.ShadowTreeInterface:main",
		],
	},
)
