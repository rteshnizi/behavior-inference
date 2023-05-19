from sa_bil.core.model.agent import Agent

class TeamMember(Agent):
	def __init__(self, agentId):
		super().__init__(agentId)
		self.name = "T-%d" % self.agentId
		self.isTeamMember = True
