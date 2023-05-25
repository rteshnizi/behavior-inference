"""
Reusable utility classes and functions for Python
"""

class ObjectLiteral:
	def __init__(self, **kwds):
		self.__dict__.update(kwds)

	def __repr__(self):
		return 'Literal(%s)' % repr(self.__dict__)
