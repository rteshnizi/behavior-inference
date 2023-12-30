from math import nan
from typing import Callable, Tuple, Union

from rclpy.node import Node, Publisher, Timer
from sa_msgs.msg import RobotState

import rt_bi_utils.Ros as RosUtils
from rt_bi_interfaces.msg import EstimationMsg


class RtBiEmulator:
	"""
		This class only prepares the messages defined in the rt_bi_eMulator.
	"""
	__RT_BI_EMULATOR_TARGET = "/rt_bi_emulator/target"
	__RT_BI_EMULATOR_ESTIMATION = "/rt_bi_emulator/estimation"

	@staticmethod
	def createTargetPublisher(node: Node, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, RobotState, RtBiEmulator.__RT_BI_EMULATOR_TARGET, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToTargetTopic(node: Node, callbackFunc: Callable[[RobotState], None]) -> None:
		RosUtils.CreateSubscriber(node, RobotState, RtBiEmulator.__RT_BI_EMULATOR_TARGET, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def createEstimationPublisher(node: Node, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, EstimationMsg, RtBiEmulator.__RT_BI_EMULATOR_ESTIMATION, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToEstimationTopic(node: Node, callbackFunc: Callable[[EstimationMsg], None]) -> None:
		RosUtils.CreateSubscriber(node, EstimationMsg, RtBiEmulator.__RT_BI_EMULATOR_ESTIMATION, callbackFunc) # type: ignore - "type[Metaclass_EstimationMsg]" is incompatible with "type[EstimationMsg]"
