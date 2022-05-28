"""descriptors.py includes various property-like descriptors that allow for automatic caching (with optional automatic
   updating as simulation time passes), and specifiable levels of protection from unwanted changes, and prioritization
   for property-getter vs instance attributes."""

__author__ = 'Justin C. Fisher'

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

import functools
import core_api # to allow timed_cached_property to refer to core_api.time

def descriptor( getter:callable=None, protected=False, prioritized=False, cached=False ) -> callable:
    """This generic factory can yield a variety of different descriptors, ranging from ordinary Python @properties
       to variants that take lower priority than like-named instance.attributes, and/or that automatically
       cache returned values. When cached=False and protected=prioritized=True, this is equivalent to Python @property.
       When cached=True and protected=prioritized=False, this is equivalent to Python 3.8's @functools.cached_property.
       Suppose f is the decorated function defined in class C, which has instance i.
         getter: if non-None a new descriptor object will be returned with this as its getter, allowing @descriptor
                 to work with no arguments.  Otherwise a class will be returned that can generate such descriptors,
                 allowing decorator uses like @descriptor( cached = True )
         protected: if True, a default setter will be created that raises an error for attempts to set i.f = n.
                    If this protective setter is overwritten, e.g. with @f.setter, the new one will take precedence.
         prioritized: if False, i.f will prioritize returning i's own attribute i.f if it has one,
                      and will return f(i) only if it doesn't. If True, i.f will return f(i) regardless.
                      Python efficiently implements low priority descriptors without setters, whereas
                      it tries to give descriptors with setters (including protected ones like @properties)
                      higher priority, so this then takes a less efficient route to lower their priority again.
                      Achieving high priority requires a setter, so a vanilla one will be created if needed.
         cached: if True, when getting i.f calls f() the result will be stored as i's own attribute i.f in i.__dict__
                 If not prioritized, subsequent readings of i.f return the cached value.
                 If prioritized, subsequent readings of i.f will call f() again, so the cache will be harder to access.
       """
    # For most purposes, cases with a "protected" setter are equivalent to cases with a custom setter:
    # these both would get high priority by default, so require extra effort to achieve low priority.
    # There are 2^3 = 8 possible combinations of cached x prioritized x presence/absence of setter.
    # The cached cases are handled with a single generic getter, reducing the remaining cases to 4.
    # These 4 cases involve 2 where Python's default prioritization is already correct (no setter = low priority;
    # setter = high priority) so a simple getter can be used, 1 case where Python's default prioritization would be
    # too low (no setter != high priority) so we can use a simple getter together with a vanilla setter to achieve
    # higher priority,and 1 case where python's default priority is too high (setter != low priority) so we need
    # a deprioritized getter that manually cedes priority back to instance.attributes.
    # Together, that means we choose between 3 getters (a generic cached one, a simple one, and a deprioritized one)
    # and that we sometimes need to manufacture a setter, for protection, for artificial priority, or when a
    # custom one is provided, with the presence of the setter potentially forcing us to a deprioritized getter.

    class CustomDescriptor:
        def __init__(self, getter):
            self.__doc__ = getter.__doc__
            self.__name__ = getter.__name__
            functools.update_wrapper(self, getter)
            self._getter = getter
            self._setter = True if protected else None

        def __repr__(self): return f"descriptor( {self.__name__} )"

        # Start with a simple getter without any caching or manual deprioritization
        def __get__(self, instance, cls=None):
            if instance is None: return self  # C.f returns descriptor itself
            return self._getter( instance )

        # Alternative getters that we may need in some special cases
        def _deprioritized_get(self, instance, cls=None):
            """This getter prioritizes returning instance.attribute, if it is present, and falls back upon the
               decorated getter when it isn't."""
            # Note: explicit deprioritization like this isn't needed when there is no setter
            if instance is None: return self # C.f returns descriptor itself
            return instance.__dict__.get( self.__name__, self._getter(instance) )

        def _cached_get(self, instance, cls=None):
            """All cached get options are handled by this generic cached getter"""
            if instance is None: return self # C.f returns descriptor itself
            #Python tries to prioritize any descriptor with a setter, so we'll need to manually deprioritize
            if self._setter is not None and not prioritized and self.__name__ in instance.__dict__:
                return instance.__dict__[self.__name__]
            value = self._getter(instance)
            instance.__dict__[self.__name__] = value # cache value f(i) as i.f
            return value

        if cached: # replace with a general purpose cached getter
            __get__ = _cached_get
        elif protected and not prioritized: # cancel the priority that Python gives any descriptor protected by a setter
            __get__ = _deprioritized_get

        # default setters for protected and prioritized classes
        def _vanilla_set(self, instance, value):
            """This does what instance.f = value typically does in python, storing this value in f's __dict__."""
            instance.__dict__[self.__name__] = value

        def _protected_set(self, instance, value):
            raise AttributeError(f"Can't set protected attribute {instance}.{self.__name__}")

        if protected: # protected descriptors define a setter that raises an error to prevent setting
            __set__ = _protected_set
        elif prioritized: # unprotected getters need a trivial setter to achieve priority
            __set__ = _vanilla_set

        def setter(self, setter: callable)->'CustomDescriptor':
            """This decorator allows for defining a setter for this descriptor, much like Python's @property .setter"""
            self._setter = setter  # store the decorated setter, mostly to remember that we have a setter

            @functools.wraps(self._getter) # copy name and docstring of our decorated getter
            def _simple_set(self, instance, value):
                setter(instance, value) # wrap the decorated setter in one that accepts the needed self arg too
            self.__set__ = _simple_set

            if not prioritized and not protected: # adding a setter adds priority that we need to counteract
                if not cached: # generic cached getter already handles priority fine
                    self.__get__ = self._deprioritized_get

            return self
        set = setter # an alias for setter to avoid current linting bugs
        # TODO Pycharm seems to care whether you define f's setter with @f.set vs @f.setter
        #  Using @f.setter makes it wrongly infer that f is an @property with no defined getter, so complains ungettable
        #  Using @f.set avoids this, but we'd prefer to call it @f.setter for consistency with @property
        #  As a compromise, I define both .setter and .set so we can choose between using the familiar @f.setter with
        #  linting complaints, versus using @f.set to avoid complaints at cost of inconsistency.
        #  Hopefully someday the linters will get smart enough to trace back .setter like they do .set, so an option
        #  would be to just do it the way that "should" work with linters, and hope it eventually will.  But when
        #  we're just writing code to hide under the hood, inconsistent naming is probably better than complaints
        #  that would affect user experience.  There may also be potential workarounds that make this a property
        #  subclass with __new__ method to return non-property when a property can't work (unprotected low-priority)

        #TODO Python implements properties as immutable, so generates anew when setter/deleter defined.  There are
        #     obvious memory disadvantages, and not obvious advantages (though perhaps is faster somehow to have
        #     immutable objects that will always behave the same, so don't need so much dynamic checking?)

        #TODO If there are speed improvements for using Python properties, it would be quite feasible to implement
        # any "high priority" or "deprioritizing" descriptor as a property.  Unfortunately, Python properties
        # don't allow any way of achieving automatic low priority, which should have signifant speed advantages over
        # "deprioritizing" a property.

        #TODO at some point should probably provide deleter option
    # if given a function to decorate, e.g., via @descriptor with no args, decorate it
    if getter:
        return CustomDescriptor( getter )
    # otherwise, return this CustomDescriptor which can serve as a function decorator, e.g. via @descriptor(...)
    return CustomDescriptor

def cached_property(fn):
    """A low priority descriptor that stores its return value as the corresponding instance-attribute so future
       attempts to retrieve this value will get this cached value directly without retriggering the property getter.
       Equivalent to Python 3.8 functools.cached_property, but also works in earlier versions."""
    return descriptor( fn, protected = False, prioritized=False, cached=True )

#TODO at some point should probably provide deleter options

# TODO not sure this is actually used or needed
def class_property( getter:callable=None, cached=False ) -> callable:
    """Creates descriptors whose getter will be called when the attribute is accessed from the class or its instances.
       The getter will be called like a Python @classmethod, receiving the class rather than instance as its argument.
       Suppose f is the decorated getter function defined in class C, and S is C or a subclass thereof, with instance i.
       Both S.f and i.f will return f(S), whereas C.f would return f(C).
         cached: if True, the return value will be cached as S.f, which would overwrite this descriptor if S is C.
         getter: if non-None a new descriptor object will be returned with this as its getter, allowing @class_property
                 to work with no arguments.  Otherwise a class will be returned that can generate such descriptors,
                 allowing decorator uses like @class_property(cached = True)
       Using the parlance for descriptors, class_properties are never "protected" (to protect S.f from change
       you'd need to use a metaclass of S, and you'd likely run into difficulties if you wanted S.f and i.f to
       both work dynamically as descriptors) and they have no setter (capturing S.f=n again requires a metaclass, and
       using a descriptor like @f.setter to define a setter in class C's definition is generally incompatible
       with having C.f work as a class_property). Since they lack setters, Python gives S.f lower priority than i.f,
       when i has its own .f, but it'd be unusual to make a class_property vary across instances."""

    class custom_class_property:
        #TODO this may not be needed since I expect these to be immutable, so can build it in from beginning
        _cached = cached

        def __init__(self, getter ):
            self.__doc__ = getter.__doc__
            self.__name__ = getter.__name__
            self._getter = getter

        def __repr__(self): return f"class_property( {self.__name__} )"

        # This simple uncached getter will trigger for S.f, or i.f, calling f(S) in both cases
        def __get__(self, instance, cls=None):
            return self._getter( cls )
        if cached: # replace with a cached getter
            def __get__(self, instance, cls=None):
                #Python tries to prioritize any descriptor with a setter, so we'll need to manually deprioritize
                value = self._getter( cls )
                setattr( cls, self.__name__, value) # will overwrite this descriptor itself unless cls is a subclass
                return value

    # if directly given a function to decorate, e.g., via @class_property with no args, decorate it
    if getter:
        return custom_class_property( getter )
    # otherwise, return this custom_class_property, usable as a function decorator via @class_property(...)
    return custom_class_property

