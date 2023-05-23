from abc import ABC
from jsoncomment import JsonComment
from rclpy.impl.rcutils_logger import RcutilsLogger
import os

from sa_bil.core.observation.observations import Observations, Observation
from sa_bil.core.observation.fov import Fov
from sa_bil.core.observation.sensor import Sensor
from sa_bil.core.observation.track import Track
from rt_bi_core.Model.FeatureMap import FeatureMap
from rt_bi_core.Model.agent import Agent
from rt_bi_core.Model.observationOld import ObservationOld
from rt_bi_core.Model.teamMember import TeamMember

JsonParser = JsonComment()

class Parser(ABC):
	def __init__(self, dirAbsPath: str, logger: RcutilsLogger):
		self._dirAbsPath = dirAbsPath
		self.logger = logger

class ObservationParser(Parser):
	def __init__(self, dirAbsPath):
		super().__init__(dirAbsPath)
		self._observationsJsonPath = os.path.abspath(os.path.join(self._dirAbsPath, "obs.json"))

	def parse(self, envMap, fov):
		"""
		Returns
		===
		A tuple `(observations, sens)`
		Stories are sensor readings, Agents are sensor locations
		"""
		parsedObservation = JsonParser.loadf(self._observationsJsonPath)
		idNum = 0
		observations = {}
		agents = {}
		for entity in parsedObservation:
			idNum += 1
			# Not a team member
			if "valid" in entity:
				agent = Agent(idNum)
				trajectoryData = entity["Datap"]
				valid = entity["valid"]
				s = ObservationOld(agent, trajectoryData, envMap, valid)
				observations[agent.agentId] = s
			# An AGC member
			else:
				# The FOV list contains field of view per timestamp
				# For each timestamp, it contains a list of individual FOVs per sensor on the vehicle
				# Each FOV per sensor has 2 lists: [x1, x2, x3, ...],[y1, y2, y3, ...]
				member = TeamMember(idNum)
				agents[member.agentId] = member
				for i in range(len(entity["FOV"])):
					# FIXME: For now there is only one sensor per vehicle, we might have to union them later
					xs = entity["FOV"][i][0][0]
					ys = entity["FOV"][i][0][1]
					coords = [[xs[j], ys[j]] for j in range(len(xs))]
					fov.append(coords, entity["Datap"][i][0], agentIndex=len(agents) - 1)
		return (observations, agents)

	def parseNew(self, envMap: Map, featureMap: FeatureMap):
		"""
		Returns
		===
		A tuple `observations`
		Stories are sensor readings, Agents are sensor locations
		"""
		rawObservations = JsonParser.loadf(self._observationsJsonPath)
		observations = Observations()
		# FIXME: Once we process multiple agents, this should change
		lastTrack = None
		for rawObservation in rawObservations:
			tracks = {}
			# for deletedTrack in rawObservation["deletedTracks"]:
			# 	tracks[deletedTrack].peek.vanished = True
			for rawTrack in rawObservation["tracks"]:
				idNum = 1 # FIXME: 1 below is the default ID because we don't trust TTL ids
				tracks[(rawObservation["t"], idNum)] = Track(idNum, rawObservation["t"], rawTrack["pose"][0], rawTrack["pose"][1], rawTrack["pose"][2])
				vanishedLastTime = lastTrack is None or lastTrack.pose.vanished
				if vanishedLastTime:
					tracks[(rawObservation["t"], idNum)].pose.spawn = True
				lastTrack = tracks[(rawObservation["t"], idNum)]
			sensors = {}
			for rawSensor in rawObservation["sensors"]:
				idNum = rawSensor["ID"]
				xs = rawSensor["FoV"][0][0]
				ys = rawSensor["FoV"][0][1]
				coords = [[xs[j], ys[j]] for j in range(len(xs))]
				sensor = Sensor(idNum, rawObservation["t"], rawSensor["pose"][0], rawSensor["pose"][1], rawSensor["pose"][2], coords, envMap, featureMap)
				sensors[(rawObservation["t"], idNum)] = sensor
			fov = Fov(sensors)
			if len(rawObservation["deletedTracks"]) > 0:
				if lastTrack is None: print("Ignoring deleting Track %d. We can't delete a track before one exists." % rawObservation["deletedTracks"][0])
				else: lastTrack.pose.vanished = True
			observation = Observation(rawObservation["t"], fov, tracks)
			observations.addObservation(observation)
		return observations

class SpecParser(Parser):
	def __init__(self, dirAbsPath):
		super().__init__(dirAbsPath)
		self._specsPath = os.path.abspath(os.path.join(self._dirAbsPath, "specs.json"))

	def parse(self):
		"""
		Returns
		===
		A dict `(observations, agents)`
		Stories are sensor readings, Agents are sensor locations
		"""
		specs = []
		specsJson = JsonParser.loadf(self._specsPath)
		for specName in specsJson:
			specs.append(Specification(specName, specsJson[specName]))
		return specs
