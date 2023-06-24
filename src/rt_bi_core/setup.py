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
		"jsoncomment~=0.4.2",
		"numpy~=1.24.2",
		"pillow>=8.3.2",
		"scikit-image~=0.19.3",
		"setuptools",
		"shapely~=2.0.1",
	],
	zip_safe=True,
	maintainer="reza",
	maintainer_email="rht1369@gmail.com",
	description="TODO: Package description",
	license="TODO: License declaration",
	tests_require=["pytest"],
	entry_points={
		"console_scripts": [
			"main = rt_bi_core.main:main"
		],
	},
)
