"""Members of the SurrogateValue class will behave as their .value for most Python purposes, including arithmetic
   operations, comparisons, being called as a function, iteration, and indexing."""

__author__ = "Justin C. Fisher"

#/*
# * Copyright 2021-2022. Justin Fisher.
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


from math import trunc, floor, ceil
from operator import index

def identity_function(arg): return arg # used to get pickle to pickle a surrogate as its .value

class SurrogateValue:
    """Members of this class will behave as their .value for most Python purposes, including arithmetic
       operations, comparisons, being called as a function, iteration, and indexing.
       The main exception is that .attribute access prioritizes the surrogate over the contained .value,
       so the surrogate may have attributes and methods of its own, and indeed a typical use case is to
       subclass SurrogateValue to create objects that can have some special attributes or methods while also
       behaving as their .value, which itself is of some immutable class that would be hard to subclass, like float.
       Getting surrogate.attr will seek attributes the surrogate lacks in the .value, so the .value's
       own .methods and .attributes will be available via the surrogate so long as names don't collide.
       Setting surrogate.attr = newvalue always affects the surrogate; set surrogate.value.attr to affect the .value.
       Another exception is that assigning variable v = surrogate will make v represent the surrogate itself,
       rather than its current .value, so if that .value changes, so too will the behavior of variable v.
       To get v to instead forever represent the current .value, set v = surrogate.value
       Another exception is that Python isinstance and issubclass will reveal the surrogate's own class,
       though some SurrogateValue subclasses may opt to deepen the masquerade by inheriting from an appropriate ABC
       and/or using a metaclass with a custom __instance_check__ and/or __subclass_check__.
       Another exception is that Python `is` will distinguish between a surrogate and its .value (aside
       from the highly dis-recommended case where a surrogate is its own .value.  DO NOT DO THIS!).
       A final exception is that, since SurrogateValues are mutable -- they can change .value and what they
       are `==` to -- they are not suitable for use in Python sets or as dictionary keys.  For most purposes,
       users should explicitly append .value to employ the current .value in set or dictionary lookups.
    """

    value = None

    # def __init__(self, value=None): # made creation easier for testing purposes
    #     if value is not None: self.value = value

    # Leaving it to subclasses to perhaps define __instance_check__ and/or __subclass_check__, if wanted

    # TODO Python 3.10 added __match_args__.  Not sure if I should define it?  It is supposed to be a class attribute
    #  so defining it as a dynamic property would require metaclass trickery

    # This is called only when the surrogate itself lacks the attribute in question, so surrogate.attr will
    # return the .attr of the surrogate if it has it, otherwise the .attr of the .value if it has it, otherwise error
    def __getattr__(self, attr):
        try:
            return getattr(self.value, attr)
        except AttributeError:
            raise AttributeError(f"Neither the SurrogateValue {self} nor its .value has attribute '{attr}'")

    # copying/pickling
    def __reduce__(self):
        return identity_function, (self.value,) # simple trick to get pickle to pickle self as self.value

    # comparisons
    def __eq__(self,other): return self.value == other
    def __ne__(self,other): return self.value != other
    def __lt__(self,other): return self.value < other
    def __le__(self,other): return self.value <= other
    def __gt__(self,other): return self.value > other
    def __ge__(self,other): return self.value >= other

    # callable
    def __call__(self, *args, **kwargs): self.value(*args, **kwargs)

    # indexing
    def __len__(self): return len(self.value)
    # omitted __length_hint__ because I'm not sure it is working or desired
    # def __length_hint__(self): return self.value.__length_hint__()
    def __contains__(self,item): return item in self.value
    def __iter__(self): return iter(self.value)
    def __reversed__(self): return reversed(self.value)
    def __getitem__(self, item): return self.value[item]
    # not defining __missing__ since __getitem__ will have passed responsibility for that on to self.value
    def __setitem__(self, item, newvalue): self.value[item] = newvalue
    def __delitem__(self, item): del self.value[item]

    # binary arithmetic ops
    def __add__(self, other): return self.value + other
    def __radd__(self, other): return other + self.value
    def __iadd__(self, other):
        self.value += other
        return self
    def __sub__(self, other): return self.value - other
    def __rsub__(self, other): return other - self.value
    def __isub__(self, other):
        self.value -= other
        return self
    def __mul__(self, other): return self.value * other
    def __rmul__(self, other): return other * self.value
    def __imul__(self, other):
        self.value *= other
        return self
    def __matmul__(self, other): return self.value @ other
    def __rmatmul__(self, other): return other @ self.value
    def __imatmul__(self, other):
        self.value @= other
        return self
    def __truediv__(self, other): return self.value / other
    def __rtruediv__(self, other): return other / self.value
    def __itruediv__(self, other):
        self.value /= other
        return self
    def __floordiv__(self, other): return self.value // other
    def __rfloordiv__(self, other): return other // self.value
    def __ifloordiv__(self, other):
        self.value //= other
        return self
    def __mod__(self, other): return self.value % other
    def __rmod__(self, other): return other % self.value
    def __imod__(self, other):
        self.value %= other
        return self
    def __divmod__(self, other): return divmod(self.value , other)
    def __rdivmod__(self, other): return divmod(other, self.value)
    def __idivmod__(self, other):
        self.value = self.value.__idivmod__(other)
        return self
    def __pow__(self, power, modulo=None): return pow(self.value, power, modulo)
    def __rpow__(self, other): return other ** self.value
    def __ipow__(self, other):
        self.value **= other
        return self


    # bitwise ops
    def __lshift__(self, other): return self.value << other
    def __rlshift__(self, other): return other << self.value
    def __ilshift__(self, other):
        self.value <<= other
        return self
    def __rshift__(self, other): return self.value >> other
    def __rrshift__(self, other): return other >> self.value
    def __irshift__(self, other):
        self.value >>= other
        return self
    def __and__(self, other): return self.value & other
    def __rand__(self, other): return other & self.value
    def __iand__(self, other):
        self.value &= other
        return self
    def __xor__(self, other): return self.value ^ other
    def __rxor__(self, other): return other ^ self.value
    def __ixor__(self, other):
        self.value ^= other
        return self
    def __or__(self, other): return self.value | other
    def __ror__(self, other): return other | self.value
    def __ior__(self, other):
        self.value |= other
        return self

    # unary ops
    def __neg__(self): return -self.value
    def __pos__(self): return +self.value
    def __abs__(self): return abs(self.value)
    def __invert__(self): return ~self.value

    def __index__(self): return index(self.value)
    def __int__(self): return int(self.value)
    def __float__(self): return float(self.value)
    def __str__(self): return str(self.value)

    def __round__(self, n=None): return round(self.value, n)
    def __trunc__(self): return trunc(self.value)
    def __floor__(self): return floor(self.value)
    def __ceil__(self): return ceil(self.value)

    def __bool__(self): return bool(self.value)

    def __hash__(self):
        raise NotImplementedError("SurrogateValues cannot be hashed for use in sets or dictionaries because "
                                  "they are mutable: they may change their .value and what they are `==` to. "
                                  "Please append .value to employ the current .value in dictionary or set lookups. "
                                  "If you want to store a SurrogateValue itself in a set or dict, you will likely need "
                                  "to use an immutable placeholder and convert back to the surrogate when you want.")


class surrogate_attribute:
    """This decorator turns a simple attribute declaration into a function that will pass an .attribute reference
       down to the owning item's .value.  The decorated function should typically have the following form:
           @surrogate_attribute
           def attname(self) -> typehint: "docstring"
       This can be used to help typehint and document attributes of SurrogateValues, or in cases where you don't
       want/need full SurrogateValue functionality, this can provide surrogate-like behavior for single attributes.
    """
    def __init__(self, fn):
        self.__doc__ = fn.__doc__
        self.__name__ = fn.__name__
    def __get__(self, instance, cls=None):
        return getattr(instance.value, self.__name__)
    def __set__(self, instance, new_value):
        setattr(instance.value, self.__name__, new_value)





# ========== TESTING ==========================


# class TestCache:
#     counter = 0
#
#     @cached_property
#     def number(self):
#         TestCache.counter += 1
#         print(f"incrementing {TestCache.counter=}")
#         return TestCache.counter
#
# i = TestCache()
# print(f"First reading:  {i.number=}")
# print(f"Second reading: {i.number=}")

#
# class one_cache_property:
#     def __init__(self, getter):
#         self.__doc__ = getter.__doc__
#         self._getter = getter
#         self._name = getter.__name__
#     def __get__(self, instance, cls=None):
#         # if instance is None: return self
#         if instance is None: return "I guess I can return anything"
#         value = instance.cache = self._getter(self) # subsequent lookups will now go straight here
#         return value
#
# class TestOneCache:
#     counter = 0
#
#     @one_cache_property
#     def cache(self):
#         TestOneCache.counter += 1
#         print(f"incrementing {TestOneCache.counter=}")
#         return TestOneCache.counter

# i = TestOneCache()
# print(f"First reading:  {i.cache=}")
# print(f"Second reading: {i.cache=}")
#
#
# class MetaC(type):
#     def __getitem__(cls, item): return "MetaC getitem"
#     at = "meta"
#
#
# class C( metaclass = MetaC ):
#     pass
#
# C.time
#
# class SubCSurrogate( C, SurrogateValue ):
#     value = "subclass value"
#
#     @surrogate_attribute
#     def __len__(self) -> int: "The length of this instance's .value"
#
# s = SubCSurrogate()
# len(s)
#
#
# class JustSubC( C ):
#     pass
#
# j = JustSubC()
#
#
#
# class Vector( list ):
#     def __add__(self,other): return Vector( s+o for s,o in zip(self,other) )
#
#
# class VectorValue( Vector, SurrogateValue ):
#
#     value = [0, 0, 0]
#     def __getitem__(self, i): return
#
# v = Vector( 1,1,1 )
# vv = VectorValue()
#
#
# from collections.abc import Iterable
# class Other:
#     def __init__(self):
#         print("Other.__init__ was called!")
#
# class MyList( list, Other ):
#     def __init__(self, *args):
#         if len(args)==1 and isinstance( args[0], Iterable ): # iterable single arg will extend list
#             list.__init__( self, args[0] )
#         else: # otherwise each given arg will become a vector component
#             list.__init__( self, args )
#
#
# list([1,2,3])
# MyList([1,2,3])
#
# from collections.abc import Iterable
# from ctypes import c_double
# Vec3f = c_double*3
# c3 = (c_double*3)(0,1,2)
#
#
#
# # -----
#
#
# class default_static_getter:
#     """The decorated function will be used to produce a default value for instances that lack this attribute themselves
#        and for references from the class itself.  Note: Unlike Python @staticmethods, a self/instance argument will
#        still be passed to the decorated function, but it will be None when the attribute is accessed from the class.
#        When used to decorate function f of class C which has instance i, references to i.f will return instance i's own
#        .f attribute if it has one, or if it doesn't then f(i) will be returned.  Reference to C.f return f(None).
#        (In contrast, using C.f with a default_getter would return the descriptor itself, not f(None).)
#        Useful for implementing dynamic attributes accessible from the class or its instances, like Robot.time."""
#     # Note: Python prioritizes (1) Class descriptors with __set__ methods (including @property descriptors) over
#     #                          (2) instances' own attributes, and both of these over
#     #                          (3) Class descriptors with only __get__ methods, like this default_getter
#     def __init__(self, getter):
#         self.__doc__ = getter.__doc__
#         self.__name__ = getter.__name__
#         self._getter = getter
#     def __get__(self, instance, cls=None):
#         return self._getter(instance) # subsequent lookups will now go straight here
#
# class static_property:
#     """Much like a Python @property, except accessing this from the class does not return the property itself,
#        but instead returns the result of calling the getter or setter with argument self = None.
#        Like a Python @property, if no setter is explicitly defined, a default setter will be created that simply
#        raises an error. The presence of this setter causes this property will always take precedence
#        over class instance's like-named attributes, so instance.foo will call foo's getter, even if instance
#        does have a .foo attribute in its own __dict__."""
#     def __init__(self, getter):
#         self.__doc__ = getter.__doc__
#         self.__name__ = getter.__name__
#         self._getter = getter
#     def __get__(self, instance, cls=None):
#         return self._getter(instance)
#
#
#
# class Robot:
#     @default_static_getter
#     def time(self)->float:
#         """Time is like a river."""
#         return 0
#
#     @property
#     def testprop(self): return 0
#
#
#
# def StopAndGo(phrase = "stop and go"):
#     for letter in phrase:
#         if letter == ' ':
#             raise StopIteration
#         else:
#             yield letter
#
# class StopAndGo:
#     phrase = "stop and go"
#     def __init__(self, phrase = "stop and go"):
#         self.phrase = phrase
#         self.i = -1
#
#     def __iter__(self): return self
#
#     def __next__(self):
#         self.i += 1
#         if self.i >= len(self.phrase) or self.phrase[self.i]==' ': raise StopIteration
#         return self.phrase[self.i]
#
# it = StopAndGo()
# for letter in it: print(letter)