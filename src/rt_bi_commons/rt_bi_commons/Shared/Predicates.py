from typing_extensions import Self

from rt_bi_commons.Utils.Msgs import Msgs


class Predicates(dict[str, bool]):
	def __setitem__(self, p: str, val: str | bool) -> None:
		assert (
			isinstance(val, bool) or
			val == Msgs.RtBi.Predicate.FALSE or
			val == Msgs.RtBi.Predicate.TRUE
		), f"Unexpected value for predicate {p}: {val}"
		if isinstance(val, bool):
			super().__setitem__(p, val)
			return
		if val == Msgs.RtBi.Predicate.TRUE:
			super().__setitem__(p, True)
			return
		if val == Msgs.RtBi.Predicate.FALSE:
			super().__setitem__(p, False)
			return
		return

	def update(self, other: "Predicates") -> Self:
		"""It will not remove predicates. Only adds or updates value."""
		for p in other: self[p] = other[p]
		return self

	def __or__(self, other: "Predicates") -> "Predicates":
		updated = Predicates([])
		updated.update(self)
		updated.update(other)
		return updated

	def asMsgArr(self) -> list[Msgs.RtBi.Predicate]:
		pList = []
		for pName in self:
			pred = Msgs.RtBi.Predicate()
			pred.name = pName
			pred.value = Msgs.RtBi.Predicate.TRUE if self[pName] else Msgs.RtBi.Predicate.FALSE
			pList.append(pred)
		return pList

	@classmethod
	def fromMsgArray(cls, msgs: list[Msgs.RtBi.Predicate]) -> "Predicates":
		d = {}
		for msg in msgs: d[msg.name] = msg.value
		return Predicates(d)
