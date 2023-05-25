import random
import time
from typing import Union
from rclpy.node import Node
from visualization_msgs.msg import Marker
from shapely.geometry import Point

class RViz(Node):
	"""
		This node prepares the visualization messages for R-Viz.
		It does not send those messages.
	"""
	def init(self):
		"""Create a RViz utility node."""
		super().__init__("rt_bi_utils_rviz")
		self.get_logger().info("RViz...")
		self.TRANSLATION_X = 0
		self.TRANSLATION_Y = 0
		self.SCALE = 1

	def _translateCoords(self, coord):
		self.get_logger().info("Translate Coord %s" % repr(coord))
		return
		c = [self.SCALE * (coord[0] + self.TRANSLATION_X), self.SCALE * ((coord[1]) + self.TRANSLATION_Y)]
		return c

	def RandomColorString(self):
		return "#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])

	def CreateCircle(self, canvas, centerX, centerY, radius, outline, tag, fill="", width=1):
		"""
		Returns shape id

		center: Point

		radius: number

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		self.get_logger().info("Render Circle @ %f, %f" % (centerX, centerY))
		return
		marker = Marker()
		marker.header.frame_id = "/bil_frame"
		marker.header.stamp = time.time()
		marker.ns = "bil"
		marker.id
		c = self._translateCoords([centerX, centerY])
		centerX = c[0]
		centerY = c[1]
		topLeft = Point((centerX - radius, centerY - radius))
		bottomRight = Point((centerX + radius, centerY + radius))
		shape = canvas.create_oval((topLeft.x, topLeft.y, bottomRight.x, bottomRight.y), outline=outline, fill=fill, width=width, tag=tag)
		# bindMouseEvent(canvas, shape)
		return shape

	def CreatePolygon(self, canvas, coords, outline, fill, width, tag, hashFill=False, hashDensity=25):
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		self.get_logger().info("Render Poly %s" % repr(coords))
		return
		if hashDensity not in [75, 50, 25, 12]: raise AssertionError("Density should be one of 75, 50, 25, or 12.")
		coords = [self._translateCoords(c) for c in coords]
		hashStr = "gray%d" % hashDensity if hashFill else ""
		shape = canvas.create_polygon(coords, outline=outline, fill=fill, width=width, tag=tag, stipple=hashStr)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	def CreateLine(self, canvas, coords, color, tag, width=1, dash=(), arrow=False):
		"""
		Returns shape id, or None if there are no points.

		coords: A list of coordinate pairs [x, y]

		color: color string (empty string for transparent)

		width: number; default is 1

		dash: Dash pattern, given as a list of segment lengths. Only the odd segments are drawn.

		tag: a unique identifier (use entity name)
		"""
		self.get_logger().info("Render Line %s" % repr(coords))
		return
		if len(coords) == 0: return None
		coords = [self._translateCoords(c) for c in coords]
		shape = canvas.create_line(coords, fill=color, width=width, dash=dash, tag=tag, arrow=LAST if arrow else None)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	def CreateText(self, canvas, coords, text, tag, color="Black", fontSize=10):
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		text: to be rendered

		color: color string (default black)

		fontSize: number; default is 10
		"""
		self.get_logger().info("Render %s" % text)
		return
		coords = self._translateCoords(coords)
		shape = canvas.create_text(coords[0], coords[1], text=text, fill=color, font="Consolas %d" % fontSize, tag=tag)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	def RemoveShape(self, canvas, shapeId):
		"""
		Remove a shape from canvas
		"""
		self.get_logger().info("Clear shape...")
		return
		canvas.delete(shapeId)
