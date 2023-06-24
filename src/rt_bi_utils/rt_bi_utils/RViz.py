import random
from ctypes import c_int32
from math import cos, sin, sqrt
from typing import Tuple
from zlib import adler32

from geometry_msgs.msg import Point as PointMsg
from visualization_msgs.msg import Marker

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry

Color = Tuple[float, float, float, float]
""" A tuple that represents an RGBA value. Values between [0-1]. """

class KnownColors:
	TRANSPARENT: Color = 	[0, 0, 0, 0]
	WHITE: Color = 			[1, 1, 1, 1]
	LIGHT_GREY: Color = 	[0.75, 0.75, 0.75, 1]
	GREY: Color = 			[0.5, 0.5, 0.5, 1]
	DARK_GREY: Color = 		[0.25, 0.25, 0.25, 1]
	BLACK: Color = 			[0, 0, 0, 1]
	RED: Color = 			[1, 0, 0, 1]
	GREEN: Color = 			[0, 1, 0 , 1]
	BLUE: Color = 			[0, 0, 1, 1]

class RViz:
	"""
		This class only prepares the visualization messages for R-Viz.
		All the shapes are rendered as Markers.
		Any closed shape is rendered as a LINE_STRIP in which
		the points list contains the same point as its first and last elements.
		http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines#The_Code_Explained
	"""

	TRANSLATION_X = 0
	TRANSLATION_Y = 0
	SCALE = 1
	NAMESPACE = "rt_bi_core"
	FRAME_ID = "map"

	@staticmethod
	def __translateCoords(coord: Geometry.Coords) -> Geometry.Coords:
		RosUtils.Logger().info("Translate Coord %s" % repr(coord))
		RosUtils.Logger().warn("No translation done.")
		return coord
		c = [RViz.SCALE * (coord[0] + RViz.TRANSLATION_X), RViz.SCALE * ((coord[1]) + RViz.TRANSLATION_Y)]
		return c

	@staticmethod
	def __createPointMessage(x: float, y: float, z = 0.0) -> PointMsg:
		"""
		Create a Point Msg.

		Parameters
		----------
		x : float
		y : float
		z : float, optional
			by default 0.0
		Returns
		-------
		PointMsg
		"""
		p = PointMsg()
		p.x = x
		p.y = y
		p.z = z
		return p

	@staticmethod
	def __setMarkerColor(marker: Marker, color: Color) -> Marker:
		marker.color.r = float(color[0])
		marker.color.g = float(color[1])
		marker.color.b = float(color[2])
		marker.color.a = float(color[3])
		return marker

	@staticmethod
	def __setMarkerId(marker: Marker, strId: str) -> Marker:
		uInt = adler32(strId.encode("utf-8"))
		marker.id = c_int32(uInt).value
		return marker

	@staticmethod
	def __setMarkerHeader(marker: Marker) -> Marker:
		marker.ns = RViz.NAMESPACE
		marker.action = Marker.ADD
		marker.pose.orientation.w = 1.0
		marker.header.frame_id = RViz.FRAME_ID
		marker.header.stamp = RosUtils.RosTimeStamp()
		return marker

	@staticmethod
	def randomColor(alpha = 1.0) -> Color:
		return (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), alpha)

	@staticmethod
	def inverseColor(color: Color, inverseAlpha = False) -> Color:
		return (1 - color[0], 1 - color[1], 1 - color[2], 1 - color[3] if inverseAlpha else color[3])

	@staticmethod
	def isLightColor(rgbColor: Color) -> bool:
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
		[r, g, b, a] = rgbColor
		hsp = sqrt(0.299 * (r * r) + 0.587 * (g * g) + 0.114 * (b * b))
		if (hsp > 0.5): return True
		else: return False

	@staticmethod
	def CreateCircle(strId: str, centerX: float, centerY: float, radius: float, outline: Color, width = 1.0) -> Marker:
		"""
		Returns shape id

		center: Point

		radius: number

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		RosUtils.Logger().info("Render circle ID %s" % strId)
		circle = Marker()
		circle = RViz.__setMarkerHeader(circle)
		circle = RViz.__setMarkerId(circle, strId)
		circle.type = Marker.LINE_STRIP
		circle = RViz.__setMarkerColor(circle, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		circle.scale.x = width
		for i in range(32):
			p = RViz.__createPointMessage(centerX + (radius * cos(i)), centerY + (radius * sin(i)))
			circle.points.append(p)
		p = RViz.__createPointMessage(centerX + (radius * cos(0)), centerY + (radius * sin(0)))
		circle.points.append(p)
		return circle

	@staticmethod
	def CreatePolygon(strId: str, coords: Geometry.CoordsList, outline: Color, width: float) -> Marker:
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		RosUtils.Logger().info("Render polygon ID %s with coords %s" % (strId, repr(coords)))
		polygon = Marker()
		polygon = RViz.__setMarkerHeader(polygon)
		polygon = RViz.__setMarkerId(polygon, strId)
		polygon.type = Marker.LINE_STRIP
		polygon = RViz.__setMarkerColor(polygon, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		polygon.scale.x = float(width)
		for (x, y) in coords:
			polygon.points.append(RViz.__createPointMessage(x, y))

		polygon.points.append(RViz.__createPointMessage(*coords[-1]))
		return polygon

	@staticmethod
	def CreateLine(strId: str, coords: Geometry.CoordsList, outline: Color, tag: str, width = 1.0, arrow = False) -> Marker:
		"""
		Returns shape id, or None if there are no points.

		coords: A list of coordinate pairs [x, y]

		color: color string (empty string for transparent)

		width: number; default is 1

		dash: Dash pattern, given as a list of segment lengths. Only the odd segments are drawn.

		tag: a unique identifier (use entity name)
		"""
		RosUtils.Logger().info("Render line strip ID %s" % strId)
		lineSeg = Marker()
		lineSeg = RViz.__setMarkerHeader(lineSeg)
		lineSeg = RViz.__setMarkerId(lineSeg, strId)
		lineSeg.type = Marker.LINE_STRIP
		lineSeg = RViz.__setMarkerColor(lineSeg, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		lineSeg.scale.x = width
		for (x, y) in coords:
			lineSeg.points.append(RViz.__createPointMessage(x, y))
		return lineSeg

	@staticmethod
	def CreateText(strId: str, coords: Geometry.Coords, text: str, color: Color, fontSize = 10) -> Marker:
		"""
		Returns shape id

		coords: [x, y]

		text: to be rendered

		color: color string (default black)

		fontSize: number; default is 10
		"""
		RosUtils.Logger().info("Render text ID %s with content \"%s\"" % (strId, text))
		RosUtils.Logger().error("Render of text is not implemented.")
		text = Marker()
		text = RViz.__setMarkerHeader(text)
		text = RViz.__setMarkerId(text, strId)
		return text

	@staticmethod
	def RemoveShape() -> None:
		"""
		Remove a shape from canvas
		"""
		RosUtils.Logger().info("Clear shape...")
		RosUtils.Logger().warn("Maybe deprecated function?")
		return
