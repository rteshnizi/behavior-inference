class Agent:
	def __init__(self, agentId):
		self.agentId = agentId
		self.name = "E-%d" % self.agentId

	def __repr__(self):
		return "%s-%d" % (self.name, self.agentId)
