from setuptools import setup

sa_bil = 'sa_bil'
core = 'sa_bil/core'
gui = 'sa_bil/core/gui'
model = 'sa_bil/core/model'
observation = 'sa_bil/core/observation'
spec = 'sa_bil/core/spec'
utils = 'sa_bil/core/utils'

setup(
	name=sa_bil,
	version='0.0.0',
	packages=[sa_bil, core, gui, model, observation, spec, utils],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + sa_bil]),
		('share/' + sa_bil, ['package.xml']),
	],
	install_requires= [
		"setuptools",
		"numpy==1.24.2",
		"shapely==2.0.1",
		"jsoncomment==0.4.2",
		"scikit-image==0.19.3",
	],
	zip_safe=True,
	maintainer='reza',
	maintainer_email='rht1369@gmail.com',
	description='TODO: Package description',
	license='TODO: License declaration',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'viewer = sa_bil.viewer:main'
		],
	},
)
