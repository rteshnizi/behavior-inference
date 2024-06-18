import os
from glob import glob

from setuptools import find_packages, setup

packageName = "rt_bi_runtime"

setup(
	name=packageName,
	version="0.9.0",
	packages = find_packages(exclude=["test", "launch"]),
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + packageName]),
		("share/" + packageName, ["package.xml"]),
		(os.path.join("share", packageName, "launch"), glob("launch/*")),
		(os.path.join("share", packageName, "config"), glob("config/*")),
		(os.path.join("share", packageName, "rdf"), glob("rdf/*")),
		(os.path.join("share", packageName, "sparql"), glob("sparql/*")),
	],
	install_requires= [
		"requests~=2.25",
		"setuptools==58.2.0",
	],
	zip_safe=True,
	maintainer="Reza Teshnizi",
	maintainer_email="reza.teshnizi@gmail.com",
	description="The runtime environment of the Behavior Inference project.",
	license="UNLICENSED",
	entry_points={
		"console_scripts": [
			"DD_RDF = rt_bi_runtime.RdfStoreNode:main",
			"CS_MGR = rt_bi_runtime.ColdStartManager:main"
		],
	},
)
