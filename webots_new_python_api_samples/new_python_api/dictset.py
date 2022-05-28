"""A DictSet is like an ordinary dictionary but with set-arithmetic operations."""

__author__ = "Justin C. Fisher"

#/*
# * Copyright 2022. Justin Fisher.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# */

from typing import Container, Union, Mapping

class DictSet(dict):
    """A DictSet is like an ordinary dictionary but with set-arithmetic operations.
       Suppose D is a DictSet, M a mapping, C a container/mapping, and S a simple non-mapping container.
       D&C returns the subDictSet of D whose keys are also in C.
       D.intersection(C1, C2, ...) returns D & C1 & C2 ... , much like Python's set.intersection()
       D-C returns the subDictSet of D whose keys are not also in C.
       M-D returns DictSet(M)-D, a DictSet much like mapping M, but with D's keys removed
       S-D returns a python *set* containing the members of S that are not also in D.
       D.difference(C1, C2, ...) returns D - C1 - C2 ... , much like Python's set.difference()
       D|M returns a DictSet containing both D's items and M's (preferring the leftmost DictSet).
       D|S returns a python *set* containing D's keys and C's members (preferring the leftmost DictSet)
       D.union(C1, C2, ...) returns D | C1 | C2 ... , much like Python's set.union()
       In general, these prefer to keep the ordering, keys, and values of DictSets over non, and
       of the leftmost DictSet when two are combined.  (Beware: Python 3.9+ defines A|B for dicts slightly
       differently, preferring the ordering and keys of A and the values of B.)
       As with Python sets, & and | are mostly commutative, aside from these preferences.
       This also supports incremental versions of &=, -=, and |=."""

    def __and__(self, other: Container) -> 'DictSet':
        if isinstance(other, Container): return DictSet((k, v) for k, v in self.items() if k in other)
        return NotImplemented

    __rand__ = __and__

    def __iand__(self, other: Container):  # handles `self &= other`
        """D &= C alters D by removing any entry whose key is not in container C."""
        for key in tuple(key for key in self if key not in other): del self[key]
        return self

    def intersection(self, *others: Container) -> 'DictSet':  # `D.intersection(C1, C2, ...)
        """Returns the subDictSet of D whose keys are in all the others, akin to Python set.intersection()."""
        return DictSet((k, v) for k, v in self.items() if not any(1 for other in others if k not in other))

    def __sub__(self, other: Container) -> 'DictSet':  # `self - other`
        """D - C returns a subDictSet of D retaining all entries whose keys are not in container C."""
        if isinstance(other, Container): return DictSet((k, v) for k, v in self.items() if k not in other)
        return NotImplemented

    def __rsub__(self, other: Container) -> Union['DictSet', set]:  # `other - self` for non-DictSet other
        """M - D returns a DictSet containing mapping M's entries whose keys aren't in DictSet D.
           C - D returns the set of container C's members that aren't keys in DictSet D."""
        if isinstance(other, Mapping): return DictSet(other) - self
        if isinstance(other, set): return other - self.keys()
        if isinstance(other, Container): set(other) - self.keys()
        return NotImplemented

    def __isub__(self, other: Container):  # handles `self -= other`
        """D -= C alters DictSet D by removing any entry whose key is not in container C."""
        for key in tuple(key for key in self if key in other): del self[key]
        return self

    def difference(self, *others: Container) -> 'DictSet':  # `D.difference(C1, C2, ...)
        """Returns the subDictSet of self whose keys are not in any of the others,
           akin to Python's set.difference()."""
        return DictSet((k, v) for k, v in self.items() if not any(1 for other in others if k in other))

    def __or__(self, other: Container) -> Union['DictSet', set]:  # handles `self|other`
        """D|M returns a DictSet containing both DictSet D's items and Mapping M's.
           D|S returns a python *set* containing DictSet D's keys and Container C's members.
           These retain the keys, values, and ordering of the leftmost dictset (unlike Python 3.9+
           dict | which retains the keys and ordering of the leftmost item, and values from the rightmost)."""
        # here, we **self first to give self's ordering and keys priority, later to reassert self's values
        if isinstance(other, Mapping): return DictSet({**self, **other, **self})
        if isinstance(other, Container): return {*self, *other}
        return NotImplemented

    __ror__ = __or__

    def __ior__(self, other: Mapping):  # handles `self |= other`
        """Alters self by adding any entries from other whose keys are not present in it.
           Unlike Python 3.9+'s version of this, this keeps self's values where other disagrees."""
        for key in other:
            if key not in self: self[key] = other[key]
        return self

    def union(self, *others: Container) -> Union['DictSet', set]:  # `D.union(C1, C2, ...)
        """D.union(C1, C2, ...) returns D | C1 | C2 ... akin to Python's set.union()
           If any other isn't a mapping, a set will be returned of keys/elements from all the contributors.
           If all others are mappings, this returns a SetDict with an entry for each distinct key from
           any contributor. Both give highest priority to self, then to earlier others."""
        if any(1 for c in others if not isinstance(c, Mapping)): return set(self).union(*others)
        d = self.copy()  # TODO *** check that this returns a DictSet, not just a dict:
        for other in others:  d |= other
        return d

    # TODO could add comparison operations on keys, though conflict with dict.__eq__'s also comparing values

    def copy(self):
        return DictSet(self)
