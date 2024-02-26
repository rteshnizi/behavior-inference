from ctypes import c_int32
from math import cos, sin
from typing import Tuple, Union
from zlib import adler32

from geometry_msgs.msg import Point as PointMsg
from rclpy.node import Publisher, Timer
from visualization_msgs.msg import Marker, MarkerArray

import rt_bi_commons.Utils.Ros as RosUtils
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Utils.Geometry import Geometry

NANO_CONVERSION_CONSTANT = 10 ** 9

class RViz:
	"""
		This class only prepares the visualization messages for R-Viz.
		All the shapes are rendered as Markers.
		Any closed shape is rendered as a LINE_STRIP in which
		the points list contains the same point as its first and last elements.
		http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines#The_Code_Explained
	"""

	FRAME_ID = "map"

	@staticmethod
	def isRVizReady(node: RtBiNode, publisher: Publisher) -> bool:
		if node.executor is None:
			node.get_logger().error("No Executor.")
			return False
		if any(n for n in node.executor.get_nodes() if n.get_name().lower().find("rviz") > -1):
			node.log("No node containing the name RViz was found.")
			return False
		if publisher.get_subscription_count() == 0:
			node.log("No subscribers to visualization messages.")
			return False
		return True

	@staticmethod
	def createRVizPublisher(node: RtBiNode, topic: str) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, MarkerArray, topic)

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
		p.x = float(x)
		p.y = float(y)
		p.z = float(z)
		return p

	@staticmethod
	def __setMarkerColor(marker: Marker, color: RGBA) -> Marker:
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
	def __setMarkerHeader(marker: Marker, durationNs: int) -> Marker:
		marker.ns = RosUtils.NAMESPACE
		marker.action = Marker.ADD
		if durationNs > 0:
			marker.lifetime = RosUtils.DurationMsg(int(durationNs / NANO_CONVERSION_CONSTANT), durationNs % NANO_CONVERSION_CONSTANT)
		marker.pose.orientation.w = 1.0
		marker.header.frame_id = RViz.FRAME_ID
		marker.header.stamp = RosUtils.Now(None).to_msg()
		return marker

	@staticmethod
	def createCircle(strId: str, centerX: float, centerY: float, radius: float, outline: RGBA, width = 1.0, durationNs: int = -1) -> Marker:
		RosUtils.Logger().debug("Render circle id %s." % strId)
		circle = Marker()
		circle = RViz.__setMarkerHeader(circle, durationNs)
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
	def createPolygon(strId: str, coords: Geometry.CoordsList, outline: RGBA, width: float, durationNs: int = -1) -> Marker:
		"""Create a polygon Marker message.

		Parameters
		----------
		strId : str
			Used for refreshing the same element over successive frames.
		coords : Geometry.Coords
			The placement.
		outline : Color
			The color of the outline.
		width : float
			The width of the rendered outline.

		Returns
		-------
		Marker
			The marker message.
		"""
		RosUtils.Logger().debug("Render polygon id %s." % strId)
		polygon = Marker()
		polygon = RViz.__setMarkerHeader(polygon, durationNs)
		polygon = RViz.__setMarkerId(polygon, strId)
		polygon.type = Marker.LINE_STRIP
		polygon = RViz.__setMarkerColor(polygon, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		polygon.scale.x = float(width)
		for (x, y) in coords:
			RosUtils.AppendMessage(polygon.points, RViz.__createPointMessage(x, y))

		if Geometry.coordsAreEqual(coords[0], coords[-1]): return polygon
		# If the last vertex is not the same as the first one, we need to close the loop-back here.
		RosUtils.AppendMessage(polygon.points, RViz.__createPointMessage(*coords[0]))
		return polygon

	@staticmethod
	def createLine(strId: str, coords: Geometry.CoordsList, outline: RGBA, width: float, arrow = False, durationNs: int = -1) -> Marker:
		"""Create a line Marker message.

		Parameters
		----------
		strId : str
			Used for refreshing the same element over successive frames.
		coords : Geometry.Coords
			The placement.
		outline : Color
			The color.
		width : float
			The width of the rendered line.
		arrow : bool, optional
			Whether to render an arrow-head, by default False.

		Returns
		-------
		Marker
			The marker message.
		"""
		RosUtils.Logger().debug("Render line strip id %s." % strId)
		lineSeg = Marker()
		lineSeg = RViz.__setMarkerHeader(lineSeg, durationNs)
		lineSeg = RViz.__setMarkerId(lineSeg, strId)
		lineSeg.type = Marker.LINE_STRIP
		lineSeg = RViz.__setMarkerColor(lineSeg, outline)
		# LINE_STRIP markers use only the x component of scale, for the line width
		lineSeg.scale.x = float(width)
		for (x, y) in coords:
			RosUtils.AppendMessage(lineSeg.points, RViz.__createPointMessage(x, y))
		if arrow: RosUtils.Logger().info("Rendering arrow-head is not implemented.")
		# if not self.renderArrows: continue
		# (dx, dy) = Geometry.getUnitVectorFromAngle(end.angleFromX)
		# dx = dx * self.HEADING_ARROW_LENGTH
		# dy = dy * self.HEADING_ARROW_LENGTH
		# msgs.append(RViz.createLine(canvas, [[p.pt.x, p.pt.y], [p.pt.x + dx, p.pt.y + dy]], color=self.MARKING_COLOR, tag=self.name, arrow=True))
		return lineSeg

	@staticmethod
	def createText(strId: str, coords: Geometry.Coords, text: str, outline: RGBA = ColorNames.BLACK, fontSize = 10.0, durationNs: int = -1) -> Marker:
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
		RosUtils.Logger().debug("Render text id %s." % strId)
		textMarker = Marker()
		textMarker = RViz.__setMarkerHeader(textMarker, durationNs)
		textMarker = RViz.__setMarkerId(textMarker, strId)
		textMarker.type = Marker.TEXT_VIEW_FACING
		textMarker.text = text
		textMarker = RViz.__setMarkerPose(textMarker, coords)
		textMarker = RViz.__setMarkerColor(textMarker, outline)
		textMarker.scale.z = float(fontSize)
		return textMarker

	@staticmethod
	def removeMarker(strId: str) -> Marker:
		"""
		Remove a shape from RViz.
		"""
		RosUtils.Logger().debug("Clear rendered RViz shape id %s." % strId)
		marker = Marker()
		marker = RViz.__setMarkerHeader(marker, -1)
		marker.action = Marker.DELETE # To remove shape
		marker = RViz.__setMarkerId(marker, strId)
		return marker

	@staticmethod
	def removeAllMarkers() -> Marker:
		"""
		Remove all shapes from RViz
		"""
		RosUtils.Logger().debug("Clear all rendered shapes from RViz.")
		marker = Marker()
		marker = RViz.__setMarkerHeader(marker, -1)
		marker.action = Marker.DELETEALL # To remove shape # CSpell: disable-line
		return marker
