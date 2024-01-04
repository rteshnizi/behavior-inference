import networkx as nx


class SpaceTimeGraph(nx.DiGraph):
	def __init__(self, name: str):
		self.name = name
