from typing_extensions import Literal, Self, TypeAlias, cast

from rt_bi_commons.Utils.Msgs import Msgs

PredicateValue: TypeAlias = bool | Literal["inherit"]


class Predicates:
	def __init__(self, raw: list[Msgs.RtBi.Predicate]) -> None:
		self.__d: dict[str, PredicateValue] = {}
		self.__meta: dict[str, str] = {}
		for msg in raw:
			self[msg.name] = msg.value
			self.__meta[msg.name] = msg.meta

	def __repr__(self) -> str:
		return repr(self.__d)

	def __iter__(self): return iter(self.__d)

	def __getitem__(self, p: str): return self.__d[p]

	def __setitem__(self, p: str, val: str | bool) -> None:
		assert (
			isinstance(val, bool) or
			val == Msgs.RtBi.Predicate.INHERIT or
			val == Msgs.RtBi.Predicate.FALSE or
			val == Msgs.RtBi.Predicate.TRUE
		), f"Unexpected value for predicate {p}: {val}"
		assert (
			p not in self.__d or
			not isinstance(self.__d[p], bool)
		), "The boolean value of a predicate is fixed."
		if isinstance(val, bool):
			self.__d[p] = val
			return
		if val == Msgs.RtBi.Predicate.INHERIT:
			# FIXME: TURN predicates to boolean with meta. NO inheriting of props
			self.__d[p] = False
			return
		if val == Msgs.RtBi.Predicate.TRUE:
			self.__d[p] = True
			return
		if val == Msgs.RtBi.Predicate.FALSE:
			self.__d[p] = False
			return
		return

	def __eq__(self, other: "Predicates") -> bool:
		for p in self:
			if p not in other: return False
			if self[p] != other[p]: return False
		return True

	def update(self, other: "Predicates") -> Self:
		return self.__ior__(other)

	def __ior__(self, other: "Predicates") -> Self:
		for p in self:
			if self[p] != Msgs.RtBi.Predicate.INHERIT: continue
			if p not in other: continue
			self[p] = other[p]
		return self

	def __or__(self, other: "Predicates") -> "Predicates":
		updated = Predicates([])
		updated.update(other)
		return updated

	def __contains__(self, p: str) -> bool:
		return p in self.__d

	def inheritFalse(self) -> Self:
		for p in self:
			if self[p] == "INHERIT": self[p] = False
		return self
