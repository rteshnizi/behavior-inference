import random
from math import sqrt
from typing import TypeAlias

numeric: TypeAlias = int | float

RGBA: TypeAlias = tuple[numeric, numeric, numeric, numeric]
""" A tuple that represents an RGBA value. Values between [0-1]. """
RGB: TypeAlias = tuple[numeric, numeric, numeric]
""" A tuple that represents an RGB value. Values between [0-1]. """

class ColorNames:
	BLACK: RGBA = 			(0, 0, 0, 1)
	BLUE: RGBA = 			(0, 0, 1, 1)
	BLUE_DARK: RGBA = 		(0, 0, 0.35, 1)
	BLUE_LIGHT: RGBA = 		(0, 0.8, 1, 1)
	CYAN: RGBA =	 		(0, 1, 1, 1)
	CYAN_DARK: RGBA = 		(0, 0.5, 0.5, 1)
	GREEN: RGBA = 			(0, 1, 0, 1)
	GREEN_DARK: RGBA = 		(0, 0.25, 0, 1)
	GREEN_LIGHT: RGBA = 	(0.25, 0.95, 0.25, 1)
	GREY: RGBA = 			(0.5, 0.5, 0.5, 1)
	GREY_DARK: RGBA = 		(0.35, 0.35, 0.35, 1)
	GREY_LIGHT: RGBA = 		(0.85, 0.85, 0.85, 1)
	MAGENTA: RGBA = 		(1, 0, 1, 1)
	MAGENTA_DARK: RGBA =	(0.85, 0, 0.85, 1)
	MAROON: RGBA = 			(0.5, 0, 0, 1)
	ORANGE: RGBA = 			(1, 0.647, 0, 1)
	PURPLE: RGBA = 			(0.36, 0.35, 0.83, 1)
	RED: RGBA = 			(1, 0, 0, 1)
	RED_DARK: RGBA =		(0.95, 0.25, 0.25, 1)
	TRANSPARENT: RGBA = 	(0, 0, 0, 0)
	WHITE: RGBA = 			(1, 1, 1, 1)
	YELLOW: RGBA =			(1, 1, 0, 1)
	YELLOW_DARK: RGBA =		(0.8, 0.8, 0, 1)

	@classmethod
	def fromString(cls, name: str) -> RGBA:
		try:
			return getattr(cls, name)
		except:
			return cls.BLACK

	@classmethod
	def toStr(cls, c: RGBA) -> str:
		for att in dir(cls):
			if getattr(cls, att) == c: return att
		return ColorUtils.toHexStr(c)

class ColorUtils:
	@staticmethod
	def avgColor(c1: RGBA, c2: RGBA) -> RGBA:
		return ((c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2, (c1[2] + c2[2]) / 2, (c1[3] + c2[3]) / 2)

	@staticmethod
	def toHexStr(color: RGBA) -> str:
		return "#" + "".join(format(int(round(val * 255)), "02x") for val in color)

	@staticmethod
	def randomColor(alpha = 1.0) -> RGBA:
		return (random.uniform(0.1, 0.8), random.uniform(0.1, 0.8), random.uniform(0.1, 0.8), alpha)

	@staticmethod
	def randomLightColor(alpha = 1.0) -> RGBA:
		color = ColorUtils.randomColor(alpha)
		while not ColorUtils.isLightColor(color): color = ColorUtils.randomColor(alpha)
		return color

	@staticmethod
	def inverseColor(color: RGBA, inverseAlpha = False) -> RGBA:
		return (1 - color[0], 1 - color[1], 1 - color[2], 1 - color[3] if inverseAlpha else color[3])

	@staticmethod
	def isLightColor(rgbColor: RGBA | RGB) -> bool:
		"""
		https://stackoverflow.com/a/58270890/750567

		Parameters
		----------
		rgbColor : Color
			The input color which we would like to determine whether it is light or dark.

		Returns
		-------
		bool
			The truth of "it is light" statement about the color.
		"""
		r = rgbColor[0]
		g = rgbColor[1]
		b = rgbColor[2]
		hsp = sqrt(0.299 * (r * r) + 0.587 * (g * g) + 0.114 * (b * b))
		if (hsp > 0.5): return True
		else: return False
