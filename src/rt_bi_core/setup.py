import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rt_bi_core"

setup(
	name=package_name,
	version="0.0.1",
	packages = find_packages(exclude=["test", "launch"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
		(os.path.join("share", package_name, "launch"), glob("launch/*")),
		(os.path.join("share", package_name, "config"), glob("config/*")),
	],
	install_requires= [
		"networkx~=3.0",
		"scikit-image~=0.19.3",
		"setuptools",
		"shapely~=2.0.1",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="The core package of the Behavior Inference project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"MapServiceInterface = rt_bi_core.MapServiceInterface:main",
			"ShadowTreeInterface = rt_bi_core.ShadowTreeInterface:main",
		],
	},
)
