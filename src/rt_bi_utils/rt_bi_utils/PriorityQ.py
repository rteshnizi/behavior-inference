"""
A wrapper for heapq to enable arbitrary objects in the data array
See https://stackoverflow.com/a/8875823/750567
"""
import heapq
from typing import List, TypeVar, Callable

DataType = TypeVar("DataType")

class PriorityQ(object):
	def __init__(self, key: Callable[[DataType], any], initial: List[DataType]=None) -> None:
		"""
		initial: The initial data array

		key: a function to return the primary key (or the cost) associated with the given item
		"""
		self.__counter = 0
		self.__key = key
		if initial:
			self.__data = [self.__createTuple(item) for item in initial]
			heapq.heapify(self.__data)
		else:
			self.__data = []

	def __repr__(self) -> str:
		return 'Q(count = %d)' % len(self)

	def __len__(self):
		return len(self.__data)

	@property
	def isEmpty(self) -> bool:
		return len(self) == 0

	def __createTuple(self, item):
		"""
		### Remarks

		* The first element of the tuple is the cost associated with the item.
		* The second is just a sequence id in order to avoid comparison between items if the keys happen to be equal
		* The third is the item itself
		"""
		self.__counter += 1 # TODO: Use length of __data? I don't know if this was done on purpose. I can look into it later.
		return (self.__key(item), self.__counter, item)

	def enqueue(self, item: DataType):
		item = self.__createTuple(item)
		heapq.heappush(self.__data, item)

	def dequeue(self):
		# Return the last item of the tuple
		return heapq.heappop(self.__data)[-1]
