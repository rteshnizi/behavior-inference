import random
from ctypes import c_int32
from math import cos, sin, sqrt
from typing import Tuple, Union
from zlib import adler32

from geometry_msgs.msg import Point as PointMsg
from rclpy.node import Node, Publisher, Timer
from visualization_msgs.msg import Marker, MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry

numeric = Union[int, float]
Color = Tuple[numeric, numeric, numeric, numeric]
""" A tuple that represents an RGBA value. Values between [0-1]. """

class KnownColors:
	TRANSPARENT: Color = 	(0, 0, 0, 0)
	WHITE: Color = 			(1, 1, 1, 1)
	LIGHT_GREY: Color = 	(0.75, 0.75, 0.75, 1)
	GREY: Color = 			(0.5, 0.5, 0.5, 1)
	DARK_GREY: Color = 		(0.25, 0.25, 0.25, 1)
	BLACK: Color = 			(0, 0, 0, 1)
	RED: Color = 			(1, 0, 0, 1)
	MAROON: Color = 		(0.502, 0, 0, 1)
	GREEN: Color = 			(0, 1, 0, 1)
	BLUE: Color = 			(0, 0, 1, 1)
	PURPLE: Color = 		(0.36, 0.25, 0.83, 1)

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
	FRAME_ID = "map"

	@staticmethod
	def isRVizReady(node: Node, publisher: Publisher) -> bool:
		if node.executor is None:
			node.get_logger().error("No Executor.")
			return False
		if any(n for n in node.executor.get_nodes() if n.get_name().lower().find("rviz") > -1):
			node.get_logger().warn("No node containing the name RViz was found.")
			return False
		if publisher.get_subscription_count() == 0:
			node.get_logger().warn("No subscribers to visualization messages.")
			return False
		return True

	@staticmethod
	def createRVizPublisher(node: Node, topic: str) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, MarkerArray, topic)

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
	def __setMarkerPose(marker: Marker, coords: Geometry.Coords) -> Marker:
		marker.pose.position.x = float(coords[0])
		marker.pose.position.y = float(coords[1])
		return marker

	@staticmethod
	def __setMarkerId(marker: Marker, strId: str) -> Marker:
		uInt = adler32(strId.encode("utf-8"))
		marker.id = c_int32(uInt).value
		return marker

	@staticmethod
	def __setMarkerHeader(marker: Marker) -> Marker:
		marker.ns = RosUtils.NAMESPACE
		marker.action = Marker.ADD
		marker.pose.orientation.w = 1.0
		marker.header.frame_id = RViz.FRAME_ID
		marker.header.stamp = RosUtils.RosTimeStamp()
		return marker

	@staticmethod
	def randomColor(alpha = 1.0) -> Color:
		return (random.uniform(0.1, 0.8), random.uniform(0.1, 0.8), random.uniform(0.1, 0.8), alpha)

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
	def createCircle(strId: str, centerX: float, centerY: float, radius: float, outline: Color, width = 1.0) -> Marker:
		"""
		Returns shape id

		center: Point

		radius: number

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		RosUtils.Logger().debug("Render circle id %s." % strId)
		circle = Marker()
		circle = RViz.__setMarkerHeader(circle)
		circle = RViz.__setMarkerId(circle, strId)
		circle.type = Marker.LINE_STRIP
		circle = RViz.__setMarkerColor(circle, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		circle.scale.x = width
		for i in range(32):
			p = RViz.__createPointMessage(centerX + (radius * cos(i)), centerY + (radius * sin(i)))
			RosUtils.AppendMessage(circle.points, p)
		p = RViz.__createPointMessage(centerX + (radius * cos(0)), centerY + (radius * sin(0)))
		RosUtils.AppendMessage(circle.points, p)
		return circle

	@staticmethod
	def createPolygon(strId: str, coords: Geometry.CoordsList, outline: Color, width: float) -> Marker:
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		RosUtils.Logger().debug("Render polygon id %s." % strId)
		polygon = Marker()
		polygon = RViz.__setMarkerHeader(polygon)
		polygon = RViz.__setMarkerId(polygon, strId)
		polygon.type = Marker.LINE_STRIP
		polygon = RViz.__setMarkerColor(polygon, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		polygon.scale.x = float(width)
		for (x, y) in coords:
			RosUtils.AppendMessage(polygon.points, RViz.__createPointMessage(x, y))

		RosUtils.AppendMessage(polygon.points, RViz.__createPointMessage(*coords[0]))
		return polygon

	@staticmethod
	def createLine(strId: str, coords: Geometry.CoordsList, outline: Color, width = 1.0, arrow = False) -> Marker:
		"""
		Returns shape id, or None if there are no points.

		coords: A list of coordinate pairs [x, y]

		color: color string (empty string for transparent)

		width: number; default is 1

		dash: Dash pattern, given as a list of segment lengths. Only the odd segments are drawn.

		tag: a unique identifier (use entity name)
		"""
		RosUtils.Logger().debug("Render line strip id %s." % strId)
		lineSeg = Marker()
		lineSeg = RViz.__setMarkerHeader(lineSeg)
		lineSeg = RViz.__setMarkerId(lineSeg, strId)
		lineSeg.type = Marker.LINE_STRIP
		lineSeg = RViz.__setMarkerColor(lineSeg, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		lineSeg.scale.x = float(width)
		for (x, y) in coords:
			RosUtils.AppendMessage(lineSeg.points, RViz.__createPointMessage(x, y))
		return lineSeg

	@staticmethod
	def createText(strId: str, coords: Geometry.Coords, text: str, outline: Color = KnownColors.BLACK, fontSize = 10.0) -> Marker:
		"""Create a text Marker message.

		Parameters
		----------
		strId : str
			Used for refreshing the same element over successive frames.
		coords : Geometry.Coords
			The placement.
		text : str
			The text.
		outline : Color, optional
			The color, by default KnownColors.BLACK
		fontSize : float, optional
			The font size, by default 10.0

		Returns
		-------
		Marker
			The marker message.
		"""
		# RosUtils.Logger().info("Render text ID %s with content \"%s\"" % (strId, text))
		RosUtils.Logger().debug("Render text id %s." % strId)
		textMarker = Marker()
		textMarker = RViz.__setMarkerHeader(textMarker)
		textMarker = RViz.__setMarkerId(textMarker, strId)
		textMarker.type = Marker.TEXT_VIEW_FACING
		textMarker.text = text
		textMarker = RViz.__setMarkerPose(textMarker, coords)
		textMarker = RViz.__setMarkerColor(textMarker, outline)
		textMarker.scale.z = float(fontSize)
		return textMarker

	@staticmethod
	def removeShape(strId: str) -> None:
		"""
		Remove a shape from RViz.
		"""
		RosUtils.Logger().debug("Clear rendered RViz shape id %s." % strId)
		marker = Marker()
		marker = RViz.__setMarkerHeader(marker)
		marker.action = Marker.DELETE # To remove shape
		marker = RViz.__setMarkerId(marker, strId)
		return

	@staticmethod
	def removeAllShapes() -> None:
		"""
		Remove all shapes from RViz
		"""
		RosUtils.Logger().debug("Clear all rendered shapes from RViz.")
		marker = Marker()
		marker = RViz.__setMarkerHeader(marker)
		marker.action = Marker.DELETEALL # To remove shape # CSpell: disable-line
		return
