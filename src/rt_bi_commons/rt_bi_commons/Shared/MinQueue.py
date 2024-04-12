import heapq
from typing import Callable, Generic, TypeAlias, TypeVar

numeric: TypeAlias = int | float

DataType = TypeVar("DataType")
QueueTuple = tuple[numeric, int, DataType]

class MinQueue(Generic[DataType]):
	"""
	A wrapper for heapq to enable arbitrary objects in the data array.
	The smallest element is always at index 0.

	See https://stackoverflow.com/a/8875823/750567
	"""

	def __init__(self, key: Callable[[DataType], numeric], initial: list[DataType]=[]) -> None:
		"""
		Create a priority queue, whose elements are sorted using the given key function.

		Parameters
		----------
		key : Callable[[DataType], int]
			A function which, given any item of `DataType`, returns the primary key (i.e. the sort order) associated with that item.
			The key/cost is expected to be an `int`.
			This is because, [in python Integers have unlimited precision](href=https://docs.python.org/3/library/stdtypes.html#typesnumeric).
			That is, there is no such thing as an integer overflow.
			This allows infinite axes, such a time, as the key.
		initial : list[DataType], optional
			The initial data array, by default None.
		"""
		self.__counter: int = 0
		"""
		This is a strictly increasing number to keep the tuples in their added order if the keys are equal.

		Note that, in python there is no such thing as an integer overflow,
		so you should not worry about this counter ever showing the wrong number.
		"""
		self.__key: Callable[[DataType], numeric] = key
		"""
		A function to return the primary key (i.e. the sort order) associated with any given item.
		"""
		self.__data: list[QueueTuple] = [self.__createTuple(item) for item in initial]
		"""The internal data structure to keep the items and maintain the heap invariant."""
		heapq.heapify(self.__data)

	def __repr__(self) -> str:
		return 'Q(count = %d)' % len(self)

	def __len__(self) -> int:
		return len(self.__data)

	@property
	def isEmpty(self) -> bool:
		"""
		Whether there is any element stored in the PriorityQ.

		Returns
		-------
		bool
			True if the queue is empty, False otherwise.
		"""
		return len(self) == 0

	@property
	def smallest(self) -> DataType:
		"""
		Return but not remove the smallest item. The smallest item in a minPQ is the first item in the queue.

		Returns
		-------
		`DataType | None`
			Returns `None` if the list is empty otherwise the smallest element in the queue w.r.t. their keys.
		"""
		if self.isEmpty: raise IndexError("MinQueue is empty.")
		return self.__data[0][-1]

	@property
	def peek(self) -> DataType:
		return self.smallest

	def __createTuple(self, item: DataType) -> QueueTuple:
		"""
		### Remarks

		* The first element of the tuple is the cost associated with the item. An Integer is expected.
		* The second is just a sequence id in order to avoid comparison between items if the keys happen to be equal
		* The third is the item itself
		"""
		self.__counter += 1
		return (self.__key(item), self.__counter, item)

	def enqueue(self, item: DataType) -> None:
		"""
		Add new item to the queue, maintaining the queue invariant in accordance to the key function.

		Parameters
		----------
		item : DataType
			The item to be added to the queue.
		"""
		itemTuple = self.__createTuple(item)
		heapq.heappush(self.__data, itemTuple)
		return

	def dequeue(self) -> DataType:
		"""
		Pops (removes) the smallest item off the heap, maintaining the queue invariant in accordance to the key function.

		Returns
		-------
		DataType
			The item stored in the queue.

		Raises
		------
		IndexError
			If the queue is empty.
		"""
		if self.isEmpty: raise IndexError("MinQueue is empty")
		# Return the last item of the tuple
		return heapq.heappop(self.__data)[-1]
