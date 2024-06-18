
from setuptools import find_packages, setup

package_name = "rt_bi_commons"

setup(
	name=package_name,
	version="0.9.0",
	packages = find_packages(exclude=["test"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
	],
	install_requires=[
		"lark~=1.1",
		"networkx~=3.2.0",
		"scikit-image~=0.19.3",
		"scipy~=1.8.0",
		"setuptools==58.2.0",
		"shapely~=2.0.1",
		"typing_extensions~=4.9.0",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="Common classes, objects, and functions shared among components in rt_bi_* packages.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
		],
	},
)
