import os
from glob import glob
from setuptools import find_packages, setup

sa_bil = "sa_bil"

setup(
	name=sa_bil,
	version="0.0.0",
	packages = find_packages(exclude=['test']),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + sa_bil]),
		("share/" + sa_bil, ["package.xml"]),
		(os.path.join("share", sa_bil, "launch"), glob("launch/*")),
		# (os.path.join("share", sa_bil, "config"), glob("config/*")),
		# (os.path.join("share", sa_bil, "scripts"), glob("scripts/*")),
	],
	install_requires= [
		"jsoncomment~=0.4.2",
		"numpy~=1.24.2",
		"pillow>=8.3.2",
		"sa_msgs",
		"visualization_msgs",
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
			"viewer = sa_bil.viewer:main"
		],
	},
)
