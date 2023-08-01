"""
Reusable utility classes and functions for Python
"""

class ObjectLiteral:
	"""
	This class works like a JS class.
	The parameters passed as key value will be accessible as key value.
	"""
	def __init__(self, **kwds):
		self.__dict__.update(kwds)

	def __repr__(self):
		return 'Literal(%s)' % repr(self.__dict__)

class Singleton(type):
	"""
	https://stackoverflow.com/a/6798042/750567

	Parameters
	----------
	type : _type_
		_description_

	Returns
	-------
	_type_
		_description_
	"""
	_instances = {}
	def __call__(cls, *args, **kwargs):
		if cls not in cls._instances:
			cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
		return cls._instances[cls]
