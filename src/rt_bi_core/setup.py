import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rt_bi_core"

setup(
	name=package_name,
	version="0.0.0",
	packages = find_packages(exclude=["test"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
		(os.path.join("share", package_name, "launch"), glob("launch/*")),
		(os.path.join("share", package_name, "config"), glob("config/*")),
	],
	install_requires= [
		"numpy~=1.24.2",
		"pillow>=8.3.2",
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
			"MapInterface = rt_bi_core.MapInterface:main",
			"ShadowTreeInterface = rt_bi_core.ShadowTreeInterface:main",
		],
	},
)
