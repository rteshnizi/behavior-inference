from setuptools import setup

package_name = "rt_bi_utils"

setup(
	name=package_name,
	version="0.0.1",
	packages=[package_name],
	data_files=[
		("share/ament_index/resource_index/packages",
			["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
	],
	install_requires=[
		"scikit-image~=0.19.3",
		"setuptools",
		"shapely~=2.0.1",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="Utility classes, objects, and functions not specific to the BI project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
		],
	},
)
