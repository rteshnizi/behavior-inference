import random
from tkinter import LAST
from shapely.geometry import Point

class Drawing:
	TRANSLATION_X = None
	TRANSLATION_Y = None
	SCALE = None

	@staticmethod
	def init(dataPrototype: str):
		if dataPrototype == "Prototype-1":
			Drawing.TRANSLATION_X = 60
			Drawing.TRANSLATION_Y = 60
			Drawing.SCALE = 7.2
		elif dataPrototype == "Prototype-2":
			Drawing.TRANSLATION_X = 10
			Drawing.TRANSLATION_Y = 205
			Drawing.SCALE = 3.95
		elif dataPrototype == "MovingSensor-0":
			Drawing.TRANSLATION_X = 105
			Drawing.TRANSLATION_Y = 105
			Drawing.SCALE = 3.75
		elif dataPrototype.startswith("TemporalEdgeTest"):
			Drawing.TRANSLATION_X = 10
			Drawing.TRANSLATION_Y = 125
			Drawing.SCALE = 5
		elif dataPrototype.startswith("TwistTest-NoTraj"):
			Drawing.TRANSLATION_X = 10
			Drawing.TRANSLATION_Y = 125
			Drawing.SCALE = 5.5
			# Drawing.TRANSLATION_X = 70
			# Drawing.TRANSLATION_Y = 175
			# Drawing.SCALE = 3.5
		elif dataPrototype.startswith("TwistTest-Traj1"):
			Drawing.TRANSLATION_X = 10
			Drawing.TRANSLATION_Y = 125
			Drawing.SCALE = 5.5
		elif dataPrototype.startswith("Two-Rotating-Boxes"):
			Drawing.TRANSLATION_X = 10
			Drawing.TRANSLATION_Y = 275
			Drawing.SCALE = 1.95
		else:
			raise RuntimeError("Please set drawing constants for data folder \"%s\"" % dataPrototype)

	@staticmethod
	def _translateCoords(coord):
		c = [Drawing.SCALE * (coord[0] + Drawing.TRANSLATION_X), Drawing.SCALE * ((-1 * coord[1]) + Drawing.TRANSLATION_Y)]
		return c

	@staticmethod
	def RandomColorString():
		return "#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])

	@staticmethod
	def CreateCircle(canvas, centerX, centerY, radius, outline, tag, fill="", width=1):
		"""
		Returns shape id

		center: Point

		radius: number

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		c = Drawing._translateCoords([centerX, centerY])
		centerX = c[0]
		centerY = c[1]
		topLeft = Point((centerX - radius, centerY - radius))
		bottomRight = Point((centerX + radius, centerY + radius))
		shape = canvas.create_oval((topLeft.x, topLeft.y, bottomRight.x, bottomRight.y), outline=outline, fill=fill, width=width, tag=tag)
		# bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def CreatePolygon(canvas, coords, outline, fill, width, tag, hashFill=False, hashDensity=25):
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		if hashDensity not in [75, 50, 25, 12]: raise AssertionError("Density should be one of 75, 50, 25, or 12.")
		coords = [Drawing._translateCoords(c) for c in coords]
		hashStr = "gray%d" % hashDensity if hashFill else ""
		shape = canvas.create_polygon(coords, outline=outline, fill=fill, width=width, tag=tag, stipple=hashStr)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def CreateLine(canvas, coords, color, tag, width=1, dash=(), arrow=False):
		"""
		Returns shape id, or None if there are no points.

		coords: A list of coordinate pairs [x, y]

		color: color string (empty string for transparent)

		width: number; default is 1

		dash: Dash pattern, given as a list of segment lengths. Only the odd segments are drawn.

		tag: a unique identifier (use entity name)
		"""
		if len(coords) == 0: return None
		coords = [Drawing._translateCoords(c) for c in coords]
		shape = canvas.create_line(coords, fill=color, width=width, dash=dash, tag=tag, arrow=LAST if arrow else None)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def CreateText(canvas, coords, text, tag, color="Black", fontSize=10):
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		text: to be rendered

		color: color string (default black)

		fontSize: number; default is 10
		"""
		coords = Drawing._translateCoords(coords)
		shape = canvas.create_text(coords[0], coords[1], text=text, fill=color, font="Consolas %d" % fontSize, tag=tag)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def RemoveShape(canvas, shapeId):
		"""
		Remove a shape from canvas
		"""
		canvas.delete(shapeId)
