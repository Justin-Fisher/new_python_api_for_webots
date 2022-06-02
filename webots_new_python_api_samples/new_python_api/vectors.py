"""This contains various vector-like datatypes for use with Webots in the "New Python API".
   These generally allow convenient access to vector components, e.g. as .x or .red.
   These also provide a number of convenient methods, like .magnitude, .angle, and .unit_vector."""

__author__ = 'Justin C. Fisher'

#/*
# * Copyright 2020-2022. Justin Fisher.
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

import collections
import itertools
import operator
from math import sin, cos, acos, sqrt, atan2, trunc, floor, ceil
from typing import Sequence, Iterable, TypeVar, Generic, overload, Container
import sys
from ctypes import c_double, c_void_p, POINTER, c_ubyte
from typing import List, Tuple, Set, Union, Optional

if sys.version_info < (3, 6, 0):
    raise RuntimeError("Python 3.6 or later is required.")
elif sys.version_info >= (3, 8, 0):
    from math import hypot  # This is useful for us, but isn't available until Python 3.8

from surrogate_values import SurrogateValue

#---------------------------------------
#  General utility
#------------------------------------------

# Python unfortunately doesn't fully support type-hinting that a function can accept any iterable (like a vector,
# list, tuple, or generator expression) that will yield a specific number of floats when iterated over.
# As a placeholder we define the names, but have no way yet of hinting the number of elements that will be used.
Iterable2f = Iterable[float]  # TODO: if python ever allows, good to indicate that this should have 2 elements
Iterable3f = Iterable[float]  # TODO: if python ever allows, good to indicate that this should have 3 elements
Iterable4f = Iterable[float]  # TODO: if python ever allows, good to indicate that this should have 4 elements

def clamp( x, lo, hi):
    """Returns the result of clamping x between lo and hi.  I.e.,
       returns x if lo <= x <= hi, otherwise returns whichever
       of those bounds x exceeds.  When x is a vector each component
       of it is clamped.  In that case, lo and hi can be vectors or scalars.""" 
    if hasattr(x,'__len__'):  # x is a sequence, so we'll clamp each component.
        if not hasattr( lo, '__len__' ): lo = [lo for _ in x] # make lo a vector
        if not hasattr( hi, '__len__' ): hi = [hi for _ in x] # make hi a vector
        return output_type[type(x)]([max(loval, min(hival, xval)) for xval, loval, hival in zip(x, lo, hi)])
    else:  # treat x as a singleton
        return max( lo, min( hi, x))

# TODO may not want/need to include this
def bind(func, object, name=None):
    """Bind an instance of function func to object, using the given name or func's
    own __name__ if name is None or not given.  After this, object.name(...) will be
    equivalent to func(object, ...).  If name is given as "" no bound function will 
    be stored on the object.  A bound function instance will be returned regardless,
    and calling it(...) will be equivalent to calling func(object, ...)."""
    if name==None: name = func.__name__  # if no name given, use func's own __name__
    bound_func = func.__get__(object)    # bound_func(...) is equiv to func(object,...)
    if name: setattr(object, name, bound_func ) # store this as object.name
    return bound_func

#--------------------------------------------------------------------------------------------------
# GenericVector abstract class
#--------------------------------------------------------------------------------------------------

class OutputTypeDict(dict):
    """A dict that maps Type1 and (Type1, Type2) to the type of output for a vectorized operation with such input(s).
       Missing entries will be auto-computed and cached on demand.
       Each vector type has an output_priority, and the input with higher priority will determine the output_type,
       with ties going to the one Python calls (usually the leftmost unless rightmost is subclass of leftmost).
       If the winner has a .preferred_output_type that will be used; otherwise its own type will be used.
       By default ordinary vectors have priority 0; colors have priority 1; and surrogate vectors have a less
       exotic preferred_output_type, like Vector or Color."""

    def __missing__(self, key:Union[type, Tuple[type]]):
        t1, t2 = key if isinstance(key, tuple) else (key, None) # type: type
        if issubclass(t1, GenericVector):
            if t2 is not None and issubclass(t2, GenericVector) and t2.output_priority > t1.output_priority:
                answer = t2.preferred_output_type if t2.preferred_output_type is not None else t2
            else:
                answer = t1.preferred_output_type if t1.preferred_output_type is not None else t1
        # otherwise the first argument isn't a vector type
        elif t2 is not None and issubclass(t2, GenericVector):
            answer = t2.preferred_output_type if t2.preferred_output_type is not None else t2
        else:
            answer = Vector
        self[key] = answer
        return answer
output_type = OutputTypeDict()

VectorType = TypeVar('VectorType')
ContentType = TypeVar('ContentType')

class GenericVector(Generic[ContentType]):
    """This abstract class includes various generic vector methods that work equally with both
         (a) Vectors, which are lists and store their components directly as list members, and
         (b) VectorValues, which instead are surrogates that store their components in their .value
       Vector methods that are especially likely to be time-sensitive may be overwritten in one or
       both of these subclasses for efficiency, e.g., by bypassing a call to __getitem__."""

    # The following Sequence-interface will typically be defined or multi-inherited in concrete subclasses
    __len__: callable
    __iter__: callable
    __getitem__:callable
    __setitem__:callable  # optional, is required for property setters like .x and .xz to work

    preferred_output_type: type = None  # vectorized ops will produce this type of output or preserve input type
    output_priority = 0  # for binary ops like v1 + v2, whichever has the higher output_priority will determine type

    #---Presenting vector as string ------

    def __format__(self, format_spec):
        return "["+  ", ".join([format(item, format_spec) for item in self])  +"]"
    def __repr__(self):
        return "["+  ", ".join(str(component) for component in self) +"]"

    # TODO is this still used???
    json_class = 'Vector'
    def toJSON(self):
        """This will be called by a custom JSON encoder, e.g. for emitters to send a serialized representation
           of this vector to a receiver.  Returns a json-serializable dict indicating the class to reconstruct
           (for vectors this will generally be Vector) and args to reconstruct it with."""
        return dict( __class__ = self.json_class, args = tuple(self) )

    # def output_type(self, other = None):
    #     """If other is None, returns an appropriate output type for a unary vectorized operation like abs(self).
    #        Otherwise, returns an appropriate output type for a binary op like self+other.
    #        Each vector type has an output_priority, and the input with higher priority will determine the output_type,
    #        with ties going to the one Python calls (usually the leftmost unless rightmost is subclass of leftmost).
    #        If the winner has a .preferred_output_type that will be used; otherwise its own type will be used.
    #        By default ordinary vectors have priority 0; colors have priority 1; and surrogate vectors have a less
    #        exotic preferred_output_type, like Vector or Color.
    #        This method could be overwritten in subclasses to produce other behavior."""
    #     # TODO could generalize to allow calling from class with non-vector self argument
    #     if isinstance(other, GenericVector) and other.output_priority > self.output_priority:
    #         return other.output_type(self)
    #     return self.preferred_output_type if self.preferred_output_type is not None else type(self)

    #---Referring to vector coordinates ------

    @property
    def x(self: 'GenericVector[ContentType]') -> ContentType:
        """The x/r/red component of this vector, stored as vector[0]"""
        return self[0]
    @x.setter
    def x(self:'GenericVector[ContentType]', new_value: ContentType): self[0] = new_value

    @property
    def y(self:'GenericVector[ContentType]') -> ContentType:
        """The y/g/green component of this vector, stored as vector[1]"""
        return self[1]
    @y.setter
    def y(self:'GenericVector[ContentType]', new_value: ContentType): self[1] = new_value

    @property
    def z(self:'GenericVector[ContentType]') -> ContentType:
        """The z/b/blue component of this vector, stored as vector[2]"""
        return self[2]
    @z.setter
    def z(self:'GenericVector[ContentType]', new_value: ContentType): self[2] = new_value

    @property
    def a(self:'GenericVector[ContentType]') -> ContentType:
        """The a/alpha component of this vector, stored as vector[3]"""
        return self[3]
    @a.setter
    def a(self:'GenericVector[ContentType]', new_value: ContentType): self[3] = new_value

    @property
    def xy(self:VectorType) -> VectorType:
        """A vector copying this vector's first 2 (.x and .y) components.
           Setting vector.xy = new_value alters these two components, broadcasting the new_value if necessary."""
        return output_type[type(self)](self[0], self[1])
    @xy.setter
    def xy( self, new_value ):
        if isinstance(new_value, Sequence):
            self[0], self[1] = new_value[0], new_value[1] # unpack sequence, e.g. vec.xy = nx,ny
        else:
            self[0] = self[1] = new_value # broadcast atom to both slots, e.g. vec.xy=0

    @property
    def xz(self:VectorType) -> VectorType:
        """Getting this returns a vector copying this vector's first 3 components, but with its .y component set to 0.
           Setting vector.xz = new_value alters these components, broadcasting the new_value if necessary.
           When new_value is a sequence its first and last components will be used, e.g in v1.xz = v2.xz.
           This can be useful, for example, for computing distances for vectors as projected onto the xz plane,
           often useful in fairly-flat simulations where the y-axis is vertical.  See also vector.angle_xz."""
        return output_type[type(self)]( self[0], 0, self[2] )
    @xz.setter
    def xz(self, new_value):
        if isinstance(new_value, Sequence):
            self[0], self[2] = new_value[0], new_value[-1] # unpack new_value's first and last to designated slots
        else:
            self[0] = self[2] = new_value # broadcast atom to both slots

    @property
    def yz(self:VectorType) -> VectorType:
        """Getting this returns a vector copying this vector's first 3 components, but with its .x component set to 0.
           Setting vector.yz = new_value alters these components, broadcasting the new_value if necessary.
           When new_value is a sequence its last two components will be used, e.g in v1.yz = v2.yz."""
        return output_type[type(self)]( self[1], 0, self[2] )
    @yz.setter
    def yz(self, new_value):
        if isinstance(new_value, Sequence):
            self[1], self[2] = new_value[-2], new_value[-1] # unpack sequence's last two values to designated slots
        else:
            self[1] = self[2] = new_value # broadcast atom to both slots

    @property
    def xyz(self:VectorType) -> VectorType:
        """vector.xyz returns a vector copying this vector's first 3 components.
           Setting vector.xyz = new_value unpacks the first three components of a sequence-like new_value to the
           corresponding positions in vector, or broadcasts a scalar new_value to all three positions."""
        return output_type[type(self)]( self[0], self[1], self[2] )
    @xyz.setter
    def xyz(self, new_value):
        if isinstance(new_value, Sequence):
            self[0], self[1], self[2] = new_value[0],new_value[1],new_value[2] # unpack sequence
        else:
            self[0] = self[1] = self[2] = new_value # broadcast atom to all three slots

    # --- vector arithmetic -------------------

    def __neg__(self) -> 'Vector':
        """-vec negates each component of vec"""
        return output_type[type(self)](-x for x in self)

    def __abs__(self) -> 'Vector':
        return output_type[type(self)](abs(x) for x in self)

    def __round__(self, n=None) -> 'Vector':
        return output_type[type(self)](round(x, n) for x in self)

    def __int__(self) -> 'Vector':  # TODO seems unable to work, as there is type-checking for if this returns int!!!
        return output_type[type(self)](int(x) for x in self)

    def __add__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](s + o for s, o in zip(self, other))
        return output_type[type(self)](s + other for s in self)  # broadcast scalar

    def __radd__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](o + s for o, s in zip(other, self))
        return output_type[type(self)](other + s for s in self)  # broadcast scalar

    def __sub__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](s - o for s, o in zip(self, other))
        return output_type[type(self), type(other)](s - other for s in self)  # broadcast scalar

    def __rsub__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](o - s for o, s in zip(other, self))
        return output_type[type(self)](other - s for s in self)  # broadcast scalar

    def __mul__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](s * o for s, o in zip(self, other))
        return output_type[type(self)](s * other for s in self)  # broadcast scalar

    def __rmul__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](o * s for o, s in zip(other, self))
        return output_type[type(self)](other * s for s in self)  # broadcast scalar

    def __truediv__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](s / o for s, o in zip(self, other))
        return output_type[type(self)](s / other for s in self)  # broadcast scalar

    def __rtruediv__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](o / s for o, s in zip(other, self))
        return output_type[type(self)](other / s for s in self)  # broadcast scalar

    def __floordiv__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](s // o for s, o in zip(self, other))
        return output_type[type(self)](s // other for s in self)  # broadcast scalar

    def __rfloordiv__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](o // s for o, s in zip(other, self))
        return output_type[type(self)](other // s for s in self)  # broadcast scalar

    def __mod__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](s % o for s, o in zip(self, other))
        return output_type[type(self)](s % other for s in self)  # broadcast scalar

    def __rmod__(self:VectorType, other) -> VectorType:
        if isinstance(other, Iterable): return output_type[type(self), type(other)](o % s for o, s in zip(other, self))
        return output_type[type(self)](other % s for s in self)  # broadcast scalar

    def dot(self, other) -> float:  # self.dot(other) is the dot product of self and other
        return sum(s * o for s, o in zip(self, other))

    # TODO decide whether to continue using @ as dotproduct or make it more numpy-like
    def __matmul__(self, other) -> float:  # dot product
        return sum(s * o for s, o in zip(self, other))

    # --- in-place vector arithmetic (works only on mutable vectors) ---

    def __iadd__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self[i] += o
        else:  # broadcast other
            for i in range(len(self)): self[i] += other
        return self
    def __isub__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self[i] -= o
        else:  # broadcast other
            for i in range(len(self)): self[i] -= other
        return self
    def __imul__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self[i] *= o
        else:  # broadcast other
            for i in range(len(self)): self[i] *= other
        return self
    def __idiv__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self[i] /= o
        else:  # broadcast other
            for i in range(len(self)): self[i] /= other
        return self

    # --- helper functions ---

    @property
    def dist(self) -> float:
        """Returns or alters the scalar magnitude of this vector. When altering the magnitude of a zero-vector, it is
           assumed to point along the x-axis.  Assigning a vector a negative magnitude will reverse its direction.
           Note vector.distance, .dist, .magnitude and .mag are all aliases for this."""
        return sqrt(sum(x * x for x in self))
    if sys.version_info >= (3, 8, 0):  # Python 3.8 added a more efficient way of computing this
        @property
        def dist(self): return hypot(*self) # hypot will have been imported at top if version is high enough
    @dist.setter
    def dist(self, d):
        self[:] = self.unit_vector * d
    mag = magnitude = distance = dist # aliases

    @property
    def unit_vector(self) -> 'Vector':
        """Returns a vector of magnitude 1 pointing in the same direction as this vector.
           If this vector has magnitude 0, its unit_vector will point along the x-axis."""
        mag = self.mag
        if mag == 0:
            return output_type[type(self)](float(i == 0) for i in range(len(self)))  # unit vector along x-axis
        return output_type[type(self)](component / mag for component in self)

    def component_along(self, other) -> float:
        """Returns the length of the component of this vector along another vector.
           Equivalent to the dot product of this vector and the unit_vector of other."""
        if not isinstance(other, Vector): other = Vector(other)
        return self.dot(other.unit_vector)  # dot product of self and unit vector of other

    @property
    def angle(self) -> float:
        """v.angle_xy (or v.angle for short) returns the angle of vector v's component in the x,y plane, computed using
           Python's math.atan2, which treats the positive y-axis as angle 0, and the positive x-axis as angle pi/2.
           This also treats 0-vectors as having angle 0, somewhat unlike Vector.distance.setter which instead takes them
           to be aligned with the x-axis (but there was no way to maintain consistency with all 2D angle functions).
           Setting v.angle = a keeps the magnitude of v, but rotates v's component in the x,y plane to angle a.
           See also angle_xz and angle_yz"""
        return atan2(self[0], self[1])  # 2D angle
    @angle.setter
    def angle(self, a):
        mag = sqrt(self[0]**2 + self[1]**2)
        self[0], self[1] = mag * sin(a), mag * cos(a)

    angle_xy = angle # alternative handle, for consistency with the other pairs

    @property
    def angle_xz(self) -> float:
        """v.angle_xz returns the angle of vector v's component in the x,z plane, computed using Python's math.atan2,
           which treats the positive z-axis as angle 0, and the positive x axis as angle pi/2.
           Setting v.angle_xz = a keeps the magnitude of v, but rotates v's component in the x,z plane to angle a.
           See also angle_xy (aka angle) and angle_yz"""
        return atan2(self[0], self[2])  # 2D angle
    @angle_xz.setter
    def angle_xz(self, a):
        mag = sqrt(self[0]**2 + self[2]**2)
        self[0], self[2] = mag * sin(a), mag * cos(a)

    @property
    def angle_yz(self) -> float:
        """v.angle_yz returns the angle of vector v's component in the y,z plane, computed using Python's math.atan2,
           which treats the positive z-axis as angle 0, and the positive y-axis as angle pi/2.
           Setting v.angle_yz = a keeps the magnitude of v, but rotates v's component in the y,z plane to angle a.
           See also angle_xy (aka angle) and angle_xz"""
        return atan2(self[1], self[2])  # 2D angle
    @angle_yz.setter
    def angle_yz(self, a):
        mag = sqrt(self[1]**2 + self[2]**2)
        self[1], self[2] = mag * sin(a), mag * cos(a)

    def cross(A, B) -> 'Vector':
        """cross product of 3D vectors A and B"""
        cls = (B.__class__ if isinstance(B, list) and B.__class__ != list else A.__class__)
        return output_type[type(A), type(B)]( A.y * B.z - A.z * B.y,
                                              A.z * B.x - A.x * B.z,
                                              A.x * B.y - A.y * B.x )

    def clamp(self, lo, hi) -> 'GenericVector':
        """Adjusts self to be between lo and hi, which may be scalar or vector. Returns self, thus adjusted."""
        n = len(self)
        if not isinstance( lo, Sequence ): lo = [lo]*n # make lo a list
        if not isinstance( lo, Sequence ): hi = [hi]*n # make hi a list
        for i in range( n ):
            self[i] = max(lo[i], min(hi[i], self[i]))
        return self

        # TODO change from using @ to using .dot?

    def rotate_by_matrix(self, rows):
        return output_type[type(self)]([row @ self for row in rows])

    @property
    def transpose(self) -> 'Vector':
        return output_type[type(self)]([Vector(new_row) for new_row in zip(*self)])

    def EulerCompose(A, B):
        """Assuming A and B are both 4D Euler rotation vectors each consisting of an x,y,z
        unit vector,  together with a an angle to rotate around that vector, this returns
        another such vector that composes those two rotations."""
        unitA, unitB = Vector(A[0:3]).unit_vector, Vector(B[0:3]).unit_vector
        sinA, sinB = sin(A[3]), sin(B[3])
        cosA, cosB = cos(A[3]), cos(B[3])
        vec = unitB * sinB * cosA + unitA * sinA * cosB + (unitA * sinA).cross(unitB * sinB)
        mag = acos(cosA * cosB - ((unitA * sinA) @ (unitB * sinB)))
        return output_type[type(A), type(B)](*vec.unit_vector, mag)

#--------------------------------------------------------------------------------------------------
# GenericColor abstract class
#--------------------------------------------------------------------------------------------------

class GenericColor(GenericVector):
    """This abstract class includes various generic methods involving colors that would work with a
       a variety of vector-like data formats, including lists, c-types arrays, and VectorValues.
    """
    output_priority = 1  # vector + color will return a color rather than a (priority 0) vector

    r = red = GenericVector.x
    g = green = GenericVector.y
    b = blue = GenericVector.z
    rgb = GenericVector.xyz
    alpha = GenericVector.a

    def __repr__(self):
        if len(self)==4: return f"[red={self.red}, green={self.green}, blue={self.blue}, alpha={self.alpha}]"
        if len(self)==3: return f"[red={self.red}, green={self.green}, blue={self.blue}]"
        return f"Color{list(self)}"

    @property
    def hexcolor(self) -> int:
        """Returns the result of converting self from 3-component RGB representation with each component ranging 0..1
           to a 24-bit color representation."""
        return (65536 * int(255 * self[0] + 0.499) +
                256 * int(255 * self[1] + 0.499) +
                int(255 * self[2] + 0.499))

    @staticmethod
    def parse_color(red_or_combined:Union[float,int,Iterable]=None, green:float=None, blue:float=None, alpha:float=None,
                    *, hex:int=None, red:float = 0, r:float=0, g:float=0, b:float=0, a:float=None)->Iterable:
        """Flexibly parses the given args and/or keyword args as a color representation, returning some
           iterable of the form (r, g, b) or (r, g, b, a) where each should range 0..1
           If hex or a single non-iterable argument is given, it is treated as a 24-bit color, e.g. 0xFFFFFF.
           If a single iterable argument is given, it is returned, so color components may be drawn from it.
           If 3-4 args are given, they are treated as rgb or rgba components.
           Components may also be given by keywords r,g,b,a or red,green,blue,alpha with missing rgb keywords
           defaulting to 0 and alpha channel defaulting to not being included (leaving the length 3, rather than 4).
           """
        if red_or_combined is not None and (green is hex is None):
            if isinstance(red_or_combined, Iterable): return red_or_combined
            hex = int( red_or_combined )
        if hex is not None:
            return (hex >> 16)/255, (hex >> 8 & 255)/255, (hex & 255)/255  # unpack hex to r,g,b
        if (alpha or a) is not None:
            return red_or_combined or red or r, green or g, blue or b, alpha or a or 0
        return red_or_combined or red or r, green or g, blue or b



#--------------------------------------------------------------------------------------------------
# Concrete Vector subclasses
#--------------------------------------------------------------------------------------------------

# Each Vector subclass combines the vector functionality of GenericVector with some sort of container-like interface
# by which vector components can be accessed

class Vector( GenericVector, list ):
    """This is the most common form of vector, a list with properties like x,y,z,a and r,g,b to access early members,
       and overloaded vector-arithmetic operators that work sensibly with both vector and scalar operands.
       Perhaps unobvious is @ which is vector dot-product.  This also provides properties to access common derivative
       properties like magnitude and unit_vector."""

    def __init__(self, *args: Union[float,Iterable[float]] ):
        """A Vector is a python list with properties like x,y,z,a and r,g,b to access early members,
           and overloaded vector-arithmetic operators that work sensibly with both vector and scalar operands.
           If given a single iterable arg, its contents will be used as the vector components. E.g. `Vector([1,2])`.
           Otherwise, the given args (if any) will become vector components. E.g. `Vector(1,2)`"""
        if len(args)==1 and isinstance( args[0], Iterable ): # if single iterable arg, draw components from it
            list.__init__( self, args[0] )
        else: # otherwise each given arg will become a vector component
            list.__init__( self, args )

class Color(Vector, GenericColor):
    output_priority = 1  # vector + color will return a color rather than a (priority 0) vector

    def __init__(self, red_or_combined:Union[float,int,Iterable]=None,
                       green:float=None,
                       blue:float=None,
                       alpha:float = None,
                       *, hex:int=None, red:float=0, r:float=0, g:float=0, b:float=0, a:float=None):
        """A Color is a vector, with standard vector methods and a few color-specific methods as well.
           Components may be accessed as color.r/.red, color.g/.green, color.b/.blue, and color.a/.alpha.
           color.hex returns a 24-bit integer representation of this color.
           The Color constructor flexibly parses the given args and/or keyword args as a color vectorlike
           representation of the form [r, g, b] or [r, g, b, a] where each should range 0..1
           If hex or a single non-iterable argument is given, it is treated as a 24-bit color, e.g. 0xFFFFFF.
           If a single iterable argument is given, it is returned, so color components may be drawn from it.
           If 3-4 args are given, they are treated as rgb or rgba components.
           Components may also be given by keywords r,g,b,a or red,green,blue,alpha with missing rgb keywords
           defaulting to 0 and alpha channel defaulting to not being included (leaving the length 3, rather than 4).
           """
        if red_or_combined is not None and (green is hex is None):
            if isinstance(red_or_combined, Iterable):
                list.__init__(self, (red_or_combined))
                return
            hex = int(red_or_combined)
        if hex is not None:
            list.__init__(self, ((hex >> 16)/255, (hex >> 8 & 255)/255, (hex & 255)/255)) # unpack hex to r,g,b
        elif alpha is not None or a is not None:
            list.__init__(self, (red_or_combined or red or r, green or g, blue or b, alpha or a or 0))
        else:
            list.__init__(self, (red_or_combined or red or r, green or g, blue or b))

class CTypesVector( GenericVector[ContentType] ):
    _length: int # will be set by subclasses, designates expected length of this vector; __init__ will enforce
    def __init__(self, *args:Union[float,Iterable[float]]):
        if len(args)==1: args = args[0]
        super().__init__(*itertools.islice(args,self._length)) # c_types arrays require the args be given separately
    def __reduce__(self):
        """This would be used by copy() or pickle() to determine how a new copy of this vector could be made.
           We avoid using ctypes' own _pickle method to make whitelisting easier."""
        return self.__class__, tuple(self)

# ctypes arrays are unusual in that they can be indexed and iterated, but lack accessible attributes like __iter__
# and aren't automatically detected as such by abc's
# We register these as Sequence so that isinstance(v, Sequence) and isinstance(v, Iterable) will work, used above


@Sequence.register
class Vec2f( CTypesVector, c_double*2 ):
    """This is a 2-component Vector whose content is contained in a ctypes array of 2 ctypes.c_double floats.
       In addition to Vector functionality, this also inherits ctypes array functionality, including
       a .fromaddress class method that can be used to receive one of these through a ctypes .dll."""
    _length = 2
Vec2f_p = POINTER(Vec2f)

@Sequence.register
class Vec3f( CTypesVector, c_double*3 ):
    """This is a 3-component Vector whose content is contained in a ctypes array of 3 ctypes.c_double floats.
       In addition to Vector functionality, this also inherits ctypes array functionality, including
       a .fromaddress class method that can be used to receive one of these through a ctypes .dll."""
    _length = 3
Vec3f_p = POINTER(Vec3f)

@Sequence.register
class Vec4f( CTypesVector, c_double*4 ):
    """This is a 4-component Vector whose content is contained in a ctypes array of 4 ctypes.c_double floats.
       In addition to Vector functionality, this also inherits ctypes array functionality, including
       a .fromaddress class method that can be used to receive one of these through a ctypes .dll."""
    _length = 4
Vec4f_p = POINTER(Vec4f)

@Sequence.register
class ColorBGRA( CTypesVector[int], c_ubyte*4):
    """This is a 4-component color vector whose content is contained in a ctypes array of 4 bytes, ordered BGRA.
       Color components are accessible as .b/.blue, .g/.green, .r/.red and .a/.alpha.
       Note that ordinary Color vectors are ordered RGBA not BGRA.  BGRA colors are used internally by webots for
       Camera images and Display images. For most other purposes ordinary Color vectors are recommended."""
    _length = 4
    preferred_output_type = Color
    # TODO sort out vector arithmetic between ordinary colors and BGRA colors
    r = red = GenericVector.z
    g = green = GenericVector.y
    b = blue = GenericVector.x
    alpha = GenericVector.a

    @property
    def hexcolor(self) -> int: return self[0]<<16 + self[1]<<8 + self[2]

    @property
    def rgb(self) -> Color: return Color(self[0]<<16 + self[1]<<8 + self[2])

    def __repr__(self): return f"[blue={self.blue}, green={self.green}, red={self.red}, alpha={self.alpha}]"

#TODO this is a placeholder until a genuine rotation class is made
Rotation = Vec4f

class VectorValue(GenericVector, SurrogateValue):
    """Each VectorValue provides vectorlike access to the early members of its .value sequence using properties x,y,z,a
       and r,g,b,a; and it provides various useful operators for mathematical operations involving vectors,
       most of which work sensibly with both vector and scalar arguments.  Perhaps unobvious is @ which is
       vector dot-product.  This also provides properties to access common derivative attributes like
       magnitude and unit_vector.  Vectors are SurrogateValues and hence provide access to whatever attributes
       and methods the .value itself has, with name collisions going to the surrogate rather than the .value."""

    # Each VectorValue must somehow have a .value (often created by subclass) with a sequence-like container interface
    value: collections.abc.Sequence # or if setters like vec.x = nx will be used, must be MutableSequence
    preferred_output_type = Vector  # so vectorvalue.unit_vector will return a Vector, not another surrogate VectorValue

    #The following is a decent general-purpose .value constructor, but most subclasses should arrange .value themselves
    def initialize_value(self, *args ):
        """This uses the given arg(s) to set an initial .value for this VectorValue.
           If no args are given, the .value will be set to an empty list [].
           If a single sequence-like arg is given, it will be used as the .value.
           If a non-sequence-like iterable is given (e.g. a generator expression) it will be iterated into a list.
           If multiple args are given, the .value will combine them into a list.
           Subclasses may set the .value in other ways, including with a .value property."""
        if len(args)==1 and isinstance( args[0], Sequence ): # single sequence-like arg will be used as .value
            self.value = args[0]
        elif len(args)==1 and isinstance( args[0], Iterable ): # other iterable single arg will be packed into list
            self.value = list( args[0] )
        elif len(args)==0: # no args
            self.value = []
        else: # multiple args will be packed together in a list
            self.value = list( args )

    def __reduce__(self):
        """Returns information about how to reconstruct this VectorValue for copy() or pickle().
           As a surrogate, a VectorValue pickles as its own .value, though we'll try to ensure it will be a vector.
           If self.value is None or already a vector, we'll let the SurrogateValue superclass pickle it as is.
           Otherwise (if self.value is something else, hopefully a list or tuple) we'll reconstruct it as a Vector."""
        if self.value is None or isinstance(self.value,GenericVector):
            return super().__reduce__()
        return output_type[type(self)], tuple(self.value)

    # This will inherit a surrogate container interface from SurrogateValue, about as efficient as we could hope

    #---Referring to vector coordinates ------
    # Note: the versions inherited from GenericVector would work fine, so we overwrite only the most used for efficiency

    @property
    def x(self:'VectorValue[ContentType]') -> ContentType:
        """The x/r/red component of this vector, stored as vector[0]"""
        return self.value[0]
    @x.setter
    def x(self:'VectorValue[ContentType]', new_value: ContentType): self.value[0] = new_value

    @property
    def y(self:'VectorValue[ContentType]') -> ContentType:
        """The y/g/green component of this vector, stored as vector[1]"""
        return self.value[1]
    @y.setter
    def y(self:'VectorValue[ContentType]', new_value: ContentType): self.value[1] = new_value

    @property
    def z(self:'VectorValue[ContentType]') -> ContentType:
        """The z/b/blue component of this vector, stored as vector[2]"""
        return self.value[2]
    @z.setter
    def z(self:'VectorValue[ContentType]', new_value: ContentType): self.value[2] = new_value

    @property
    def a(self:'VectorValue[ContentType]') -> ContentType:
        """The a/alpha component of this vector, stored as vector[3]"""
        return self.value[3]
    @a.setter
    def a(self:'VectorValue[ContentType]', new_value: ContentType): self.value[3] = new_value

    # --- vector arithmetic -------------------
    # Again, versions inherited from GenericVector will work, so just defining most commonly used for speed

    def __neg__(self) -> 'Vector':
        """-vec negates each component of vec"""
        return output_type[type(self)]( -x for x in self.value )
    def __abs__(self) -> 'Vector':
        return output_type[type(self)]( abs(x) for x in self.value )
    def __round__(self, n=None ) -> 'Vector':
        return output_type[type(self)]( round(x, n) for x in self.value )
    def __add__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( s + o for s,o in zip(self.value,other))
        return output_type[type(self)]( s + other for s in self.value ) # broadcast scalar
    def __radd__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( o + s for o,s in zip(other, self.value))
        return output_type[type(self)]( other + s for s in self.value ) # broadcast scalar
    def __sub__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( s - o for s,o in zip(self.value,other))
        return output_type[type(self)]( s - other for s in self.value ) # broadcast scalar
    def __rsub__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( o - s for o,s in zip(other, self.value))
        return output_type[type(self)]( other - s for s in self.value ) # broadcast scalar
    def __mul__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( s * o for s,o in zip(self.value,other))
        return output_type[type(self)]( s * other for s in self.value ) # broadcast scalar
    def __rmul__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( o * s for o,s in zip(other, self.value))
        return output_type[type(self)]( other * s for s in self.value ) # broadcast scalar
    def __truediv__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( s / o for s,o in zip(self.value,other))
        return output_type[type(self)]( s / other for s in self.value ) # broadcast scalar
    def __rtruediv__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( o / s for o,s in zip(other, self.value))
        return output_type[type(self)]( other / s for s in self.value ) # broadcast scalar
    def __floordiv__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( s // o for s,o in zip(self.value,other))
        return output_type[type(self)]( s // other for s in self.value ) # broadcast scalar
    def __rfloordiv__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( o // s for o,s in zip(other, self.value))
        return output_type[type(self)]( other // s for s in self.value ) # broadcast scalar
    def __mod__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( s % o for s,o in zip(self.value,other))
        return output_type[type(self)]( s % other for s in self.value ) # broadcast scalar
    def __rmod__(self:VectorType, other) -> VectorType:
        if isinstance(other,Iterable): return output_type[type(self), type(other)]( o % s for o,s in zip(other, self.value))
        return output_type[type(self)]( other % s for s in self.value ) # broadcast scalar

    def dot(self, other) -> float: # self.dot(other) is the dot product of self and other
        return sum( s*o for s,o in zip(self.value,other) )
    # TODO decide whether to continue using @ as dotproduct or make it more numpy-like
    def __matmul__(self, other) -> float: # dot product
        return sum( s*o for s,o in zip(self.value,other) )

    # --- in-place vector arithmetic (works only on mutable vectors) ---

    def __iadd__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self.value[i] += o
        else:  # broadcast other
            for i in range(len(self.value)): self.value[i] += other
        return self
    def __isub__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self.value[i] -= o
        else:  # broadcast other
            for i in range(len(self.value)): self.value[i] -= other
        return self
    def __imul__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self.value[i] *= o
        else:  # broadcast other
            for i in range(len(self.value)): self.value[i] *= other
        return self
    def __idiv__(self,other) -> 'GenericVector':
        if isinstance(other, Iterable): # operate upon corresponding elements of matched seqs
            for i, o in enumerate(other): self.value[i] /= o
        else:  # broadcast other
            for i in range(len(self.value)): self.value[i] /= other
        return self

    # --- helper functions ---

    @property
    def dist(self)->float: return sqrt( sum(x*x for x in self.value) )
    if sys.version_info >= (3, 8, 0): # Python 3.8 added a more efficient way of computing this
        @property
        def dist(self): return hypot( *self.value )
    mag = magnitude = distance = dist # aliases

    @property
    def unit_vector(self)->'Vector':
        """Returns a vector of magnitude 1 pointing in the same direction as this vector.
           If this vector has magnitude 0, its unit_vector will point along the x-axis."""
        mag = self.mag
        if mag == 0:
            return output_type[type(self)](float(i == 0) for i in range(len(self.value)))  # unit vector along x-axis
        return output_type[type(self)]( component / mag for component in self.value )

    def component_along(self,other) -> float:
        """Returns the length of the component of this vector along another vector.
           Equivalent to the dot product of this vector and the unit_vector of other."""
        if not isinstance(other, Vector): other = Vector(other)
        return self.dot( other.unit_vector )  # dot product of self and unit vector of other

    @property
    def angle(self) -> float:
        """v.angle returns the angle of vector v's component in the x,y plane, computed using Python's math.atan2,
           which treats the positive y-axis as angle 0, and the positive x-axis as angle pi/2.
           Setting v.angle = a keeps the magnitude of v, but rotates v's component in the x,y plane to angle a.
           See also angle_xz and angle_yz"""
        return atan2(self.value[0], self.value[1])  # 2D angle
    @angle.setter
    def angle(self, a):
        mag = sqrt(self.value[0]**2 + self.value[1]**2)
        self.value[0], self.value[1] = mag * sin(a), mag * cos(a)
    angle_xy = angle # alternative handle, for consistency with the other pairs

    @property
    def angle_xz(self) -> float:
        """v.angle_xz returns the angle of vector v's component in the x,z plane, computed using Python's math.atan2,
           which treats the positive z-axis as angle 0, and the positive x-axis as angle pi/2.
           Setting v.angle_xz = a keeps the magnitude of v, but rotates v's component in the x,z plane to angle a.
           See also angle_xy (aka angle) and angle_yz"""
        return atan2(self.value[0], self.value[2])  # 2D angle
    @angle_xz.setter
    def angle_xz(self, a):
        mag = sqrt(self.value[0]**2 + self.value[2]**2)
        self.value[0], self.value[2] = mag * sin(a), mag * cos(a)

    @property
    def angle_yz(self) -> float:
        """v.angle_yz returns the angle of vector v's component in the y,z plane, computed using Python's math.atan2,
           which treats the positive z-axis as angle 0, and the positive y-axis as angle pi/2.
           Setting v.angle_yz = a keeps the magnitude of v, but rotates v's component in the y,z plane to angle a.
           See also angle_xy (aka angle) and angle_xz"""
        return atan2(self.value[1], self.value[2])  # 2D angle
    @angle_yz.setter
    def angle_yz(self, a):
        mag = sqrt(self.value[1]**2 + self.value[2]**2)
        self.value[1], self.value[2] = mag * sin(a), mag * cos(a)

    def clamp(self, lo, hi) -> 'VectorValue':
        """Adjusts self to be between lo and hi, which may be scalar or vector. Returns self, thus adjusted."""
        n = len(self.value)
        if not isinstance(lo, Sequence): lo = [lo] * n  # make lo a list
        if not isinstance(lo, Sequence): hi = [hi] * n  # make hi a list
        for i in range( n ):
            self.value[i] = max(lo[i], min(hi[i], self.value[i]))
        return self

    # TODO change from using @ to using .dot?
    def rotate_by_matrix(self, rows):
        return output_type[type(self)]( [row@self for row in rows] )

    @property
    def transpose(self) -> 'Vector':
        return output_type[type(self)]( [Vector( new_row ) for new_row in zip( *self.value ) ] )

# === Dist objects (list-like objects that distribute most operations over their members) ---



MemberType = TypeVar('MemberType')        # Used to indicate the type of members in a Dist
SubmemberType = TypeVar('SubmemberType')  # Used to indicate type of submembers contained within members
OtherType = TypeVar('OtherType')          # Used to indicate the type of other when broadcasting other to self.
class Dist(Generic[MemberType]):
    """A Dist is like a Python list, except most operations on a Dist are distributed over the members (which are
       stored as ._members, and often are Python objects like devices), producing another Dist as output.
       Other operands are "broadcast" Numpy-style, meaning that when a non-string operand matches the length of a Dist,
       the operation will be "vectorized" element-wise, so Dist(0,1) + (0,1) == Dist(0,2), and otherwise,
       an operand will be distributed (or "broadcast") across the whole Dist. E.g., Dist(0,1) + 3 == Dist(3,4).
       `D = Dist(member1, member2, member3)` creates a Dist with the specified members.
       `D = Dist(member, n=3)` creates a Dist containing 3 copies of member.
       `D = Dist(iterable)` creates a Dist whose members are drawn from iterable.       `
       `motors = Dist(robot.Motor)` therefore creates a Dist of all of robot's Motors, in order.
       `D[i]` indexes or slices the Dist, as with python lists.
       `D[i1:i2, X]` returns the Dist of m[X] for m in D[i1:i2]; i.e. peel off the first index to slice D itself,
        and use any remaining indices to slice each of the particular members.
       `D.velocity` returns another Dist consisting of each member's .velocity.
       `D.velocity = v` adjusts each member's .velocity to be the corresponding member of v if v has the same
       length as the Dist, or to be v itself otherwise (equivalent to numpy "broadcasting") .
       `D(arg1, arg2, key1=k1,...)` returns the Dist whose members are the outputs that result from
       calling each member function with those arguments and/or keyword arguments, broadcasting each arg as needed.
       `D.method(...)` therefore produces a Dist of m.method(a) for each member m, again broadcasting args.
       All arithmetic operations are vectorized, broadcasting the other operand where needed:
       `D + 1` returns another Dist like D, with each member being 1 larger. (1 was broadcast across D)
       `D += 1` replaces D's own members with the results of adding 1 to each. (1 was again broadcast)
       `D + D` returns a Dist whose members will be twice those of D (vectorized arithmetic when lengths are same).
       `foo(D)` passes the whole Dist D to function foo. Many operations that foo might do on D would automatically
       be distributed, but some functions may not be able to handle a whole Dist at once.
       `D.broadcast(foo)(arg1, arg2, key1 = k1, ...)` would call a distributed version of foo, broadcast to match D.
       E.g., if D=Dist(1,4,9), then math.sqrt(D) raises a TypeError but D.broadcast(math.sqrt)(D) returns Dist(1,2,3)."""

    # TODO could consider re-using the more complex output-type selection from Vectors?
    _dist_output_type: type  # The type of output this sort of dist will produce (will be set once this class is defined)

    #TODO may want to @overload the three calling signatures
    def __init__(self, *args: Union[MemberType, Iterable[MemberType]], n:int = None):
        if n is None:
            if len(args) == 1 and isinstance(args[0], Iterable):  # treat single iterable args as sequence of args
                args = args[0]
            self.__dict__['_members'] = list(args)
        else:
            self.__dict__['_members'] = [args[0]]*n

    def __repr__(self):
        return repr(self._members)

    # --- Dist container interface ---

    def __len__(self) -> int:
        return len(self._members)

    def __iter__(self) -> Iterable[MemberType]:
        return iter(self._members)

    def __reversed__(self) -> Iterable[MemberType]:
        return iter(reversed(self._members))

    def broadcast(self, other:OtherType) -> 'Dist[OtherType]':
        """Returns a Dist containing as many repetitions of other as self has members.
           Useful for pre-distributing a container-like operand to ensure that it won't be vectorized in an op with
           a like-sized Dist. E.g., if D has 2 members, then, `D.pos = (1,2)` would be vectorized, setting D[0].pos=1
           and D[1].pos=2.  In contrast `D.pos = D.broadcast((1,2))` would instead set each members' .pos to (1,2).
           Also useful for broadcasting a function to call it distributedly.  E.g., if D=Dist(1,4,9), math.sqrt(D)
           raises a TypeError, but D.broadcast(math.sqrt)(D) returns Dist(1,2,3)."""
        return self._dist_output_type(other, n=len(self))

    @overload  # matched_version_of(self, scalar) returns an iterable of scalar's type
    def matched_version_of(self, other:OtherType) -> Iterable[OtherType]: pass
    @overload  # matched_version_of(self, matching_sequence) returns members from that sequence
    def matched_version_of(self, other:Sequence[OtherType]) -> Iterable[OtherType]: pass
    def matched_version_of(self, other):
        """Returns an iterator of items based on other, either members of other (if other is a non-string with the same
           length as self) or else just other itself repeatedly, being "broadcast" numpy-style to match self."""
        self_n = len(self._members)
        try: other_n = len(other)  # First we'll try to see if other is the right length to vectorize
        except: other_n = None
        if self_n == other_n and not(isinstance(other, str)):
            return iter(other)   # vectorize
        else:
            return itertools.repeat(other, self_n)  # broadcast other by repeating as many times as self needs

    @overload  # with_matched_version_of(self, scalar) returns an iterable of (member, scalar) tuples
    def with_matched_version_of(self, other:OtherType) -> Iterable[Tuple[MemberType, OtherType]]: pass
    @overload  # with_matched_version_of(self, matching_sequence) returns an iterable of (member, othermember) tuples
    def with_matched_version_of(self, other:Sequence[OtherType]) -> Iterable[Tuple[MemberType,OtherType]]: pass
    def with_matched_version_of(self, other):
        """Returns an iterator of (m, o) pairs, where each m will be a successive member of self, and each o will be
           based on other, and will either be a corresponding part of other (if other is a non-string with the same
           length as self) or else will be other itself repeatedly, "broadcast" to match self.
           E.g. D.velocity = 0 sets each member's .velocity to 0 (distributing), whereas if D has 2 members,
           then D.velocity = (0,1) would set the first member's .velocity to 0 and the second's to 1 (vectorizing).
           Note strings are always distributed intact, even when they happen to have the same length as the Dist,
           so e.g., even if D happens to have 3 members, D.name = 'foo' will set each to 'foo' not one to 'f' and
           the others to 'o'.  If you really want to distribute a string, use D.name=list('foo').
           If you want to force an item to be broadcast intact, even if it is the right length to vectorize, you
           can manually broadcast it to match D with D.broadcast(item)."""
        return zip(self._members, self.matched_version_of(other))

    def with_matched_args(self, args, kwargs) -> Iterable[Tuple[MemberType, Tuple, dict[str, any]]]:
        """Returns an iterator of (m, a, k) triples, where each m will be a successive member of self,
           each a will be a tuple drawn from args, broadcasted where necessary to match self,
           and each k will be a dictionary of based on kwargs, again broadcast, if necessary, to match self.
           This iterator can provide relevant args and kwargs for successive calls of a distributed function."""

        # args_it will yield successive tuples of args, ready to send as *args to successive calls of some function
        if args:
            args_it = zip(*(self.matched_version_of(a) for a in args))
        else:                                                     # or if no args were given, then just
            args_it = itertools.repeat(args, len(self._members))  # repeatedly yield that empty tuple

        if kwargs:
            # kw_values_it will yield similar tuples of values to associate with the **kwargs in that call
            kw_values_it = zip(*(self.matched_version_of(value) for value in kwargs.values()))
            # kw_dicts_it will yield dictionaries by zipping each key together with its value from that tuple
            kw_dicts_it = (dict(zip(kwargs, kw_values)) for kw_values in kw_values_it)
        else:                                                          # or if no kwargs were given, then just
            kw_dicts_it = itertools.repeat(kwargs, len(self._members)) # repeatedly yield that empty dict

        # returned iterator will yeild (m, a, k) tuples: m = next member of self, a = tuple of args, k = dict of kwargs
        return zip(self, args_it, kw_dicts_it)


    # Dist[...] can return various types depending on what indices are given, so type-hinting requires @overload
    @overload  # D[index] returns a single member
    def __getitem__(self, index:int) -> MemberType: pass
    @overload  # D[slice] returns a Dist of members in the slice
    def __getitem__(self, item:slice) -> 'Dist[MemberType]': pass
    @overload  # D[index1, index2] returns whatever m[index2] returns
    def __getitem__(self:'Dist[MemberType[SubmemberType]]', item:Tuple[int, int]) -> SubmemberType: pass
    @overload  # D[index, slice] returns whatever m[slice] returns, presumed to be another container like m
    def __getitem__(self, item:Tuple[int, slice]) -> 'MemberType': pass
    @overload  # D[slice, index] returns a Dist of whatever sort of submember m[index] returns
    def __getitem__(self:'Dist[MemberType[SubmemberType]]', item:Tuple[slice, int]) -> 'Dist[SubmemberType]': pass
    @overload  # D[slice1, slice2] returns a Dist of whatever m[slice2] returns, presumed to be another container like m
    def __getitem__(self:'Dist[MemberType]', item:Tuple[slice, slice]) -> 'Dist[MemberType]': pass
    def __getitem__(self, item):
        if isinstance(item, tuple):  # D[domain, range]
            domain = item[0]                               # peel off the first index to slice the Dist itself
            range = item[1] if len(item)==2 else item[1:]  # remaining index / tuple-of-indices will slice each member
            #TODO consider vectorizing each member of range rather than always broadcasting, not gonna bother for now
            if isinstance(domain, slice):  # D[slice, range]
                return self._dist_output_type(m[range] for m in self._members[domain] )
            else:                          # D[index, range]
                return self[domain][range]
        if isinstance(item, slice):
            return self._dist_output_type(self._members[item])
        return self._members[item]

    # D[...] = new_value should take different types depending on what indices are given, so we need @overload
    @overload  # D[index] = new_member replaces a single member
    def __setitem__(self, index:int, new_member:MemberType): pass
    @overload  # D[slice] = new_members replaces a slice of members with new members
    def __setitem__(self, slic:slice, new_members:Sequence[MemberType]): pass
    @overload  # D[index1, index2] = new_value sets m.[index2] = new_value, where m is D[index1]
    def __setitem__(self:'Dist[MemberType[SubmemberType]]', item:Tuple[int, int], value_to_broadcast:SubmemberType): pass
    @overload  # D[index, slice] = new_value sets m[slice] = new_value, where m is D[index1]
    def __setitem__(self:'Dist[MemberType[SubmemberType]]', item:Tuple[int, slice], value_to_broadcast:Sequence[SubmemberType]): pass
    @overload  # D[slice, index] = new_value sets m.[index] = new_value, for each m in D[slice]
    def __setitem__(self:'Dist[MemberType[SubmemberType]]', item:Tuple[slice, int], value_to_broadcast:SubmemberType): pass
    @overload  # D[slice1, slice2] = new_value sets m[slice2] = new_value, for each m in D[slice1]
    def __setitem__(self:'Dist[MemberType[SubmemberType]]', item:Tuple[slice, slice], value_to_broadcast:Sequence[SubmemberType]): pass
    def __setitem__(self, item, new_value):
        """`D[index] = new_member` replaces a member of Dist D.
           `D[slice] = new_members` replaces a slice with new members, just as doing this with a list would.
           `D[domain, range] = new_value` sets m[range] = new_value, broadcast to all m in D[domain]. So,
           `D[:, range] = new_value` sets m[range] = new_value, broadcast to all members of D.
           Note: only case in which new_value would be distributed is D[:, range] = new_value, where range consists only
           of ints, as slicing may require a plural new_value to fill the slice, and D[index] can't vectorize."""
        if isinstance(item, tuple):  # D[domain, range] = new_value
            domain = item[0]                               # peel off the first index to slice the Dist itself
            range = item[1] if len(item)==2 else item[1:]  # remaining index / tuple-of-indices will slice each member
            if isinstance(domain, slice):  # D[slice, range] = new_value
                # If we're assigning across all of D[:,...], and into just one slot of each member, can vectorize
                if (domain == slice(None,None,None) and
                        (isinstance(range, int) or isinstance(range, tuple) and all(isinstance(i,int) for i in range))):
                    for m, v in self.with_matched_version_of(new_value):
                        m[range] = new_value
                for m in self._members[domain]:
                    m[range] = new_value
            else:  # D[index, range] = new_value
                self._members[domain][range] = new_value
        else:  # handles both D[index] = new_member and D[slice] = new_members
            self._members[item] = new_value

    # --- Dist distributing dunder methods across members ---

    def __getattr__(self, attr):  # motors.velocity returns Dist of m.velocity for all motors
        """D.attr returns a Dist of m.attr for each member."""
        return self._dist_output_type(getattr(m, attr) for m in self._members)

    def __setattr__(self, attr, value):  # D.attr = value sets each m.attr, broadcasting value if needed
        for m,v in self.with_matched_version_of(value):
            setattr(m, attr, v)

    def __call__(self, *args, **kwargs):  # D.method(args) calls m.method(args) for each member, broadcasting args
        # Note that motors.method will create a Dist of bound methods, and then *its* __call__ will call them
        return self._dist_output_type(f(*a, **k) for f, a, k in self.with_matched_args(args, kwargs))

    # TODO generalize return types and type-hints to allow subclasses like Pair to retain their (sub)class

    # --- Dist vectorized arithmetic ---
    # For type-hinting, we assume that each arithmetic op on a MemberType produces another MemberType

    def __add__(self, other)->'Dist[MemberType]':   # self + other
        return self._dist_output_type(s + o for s, o in self.with_matched_version_of(other))
    def __radd__(self, other)->'Dist[MemberType]':  # other + self
        return self._dist_output_type(o + s for s, o in self.with_matched_version_of(other))
    def __iadd__(self,other) -> 'Dist[MemberType]':  # self += other
        self.__dict__['_members'] = [s + o for s, o in self.with_matched_version_of(other)]
        return self

    def __sub__(self, other)->'Dist[MemberType]':    # self - other
        return self._dist_output_type(s - o for s, o in self.with_matched_version_of(other))
    def __rsub__(self, other)->'Dist[MemberType]':   # other - self
        return self._dist_output_type(o - s for s, o in self.with_matched_version_of(other))
    def __isub__(self,other) -> 'Dist[MemberType]':  # self -= other
        self.__dict__['_members'] = [s - o for s, o in self.with_matched_version_of(other)]
        return self

    def __mul__(self, other)->'Dist[MemberType]':    # self * other
        return self._dist_output_type(s * o for s, o in self.with_matched_version_of(other))
    def __rmul__(self, other)->'Dist[MemberType]':   # other * self
        return self._dist_output_type(o * s for s, o in self.with_matched_version_of(other))
    def __imul__(self,other) -> 'Dist[MemberType]':  # self *= other
        self.__dict__['_members'] = [s * o for s, o in self.with_matched_version_of(other)]
        return self

    def __pow__(self, other)->'Dist[MemberType]':    # self ** other
        return self._dist_output_type(s ** o for s, o in self.with_matched_version_of(other))
    def __rpow__(self, other)->'Dist[MemberType]':   # other ** self
        return self._dist_output_type(o ** s for s, o in self.with_matched_version_of(other))
    def __ipow__(self,other) -> 'Dist[MemberType]':  # self **= other
        self.__dict__['_members'] = [s ** o for s, o in self.with_matched_version_of(other)]
        return self

    def __mod__(self, other)->'Dist[MemberType]':    # self % other
        return self._dist_output_type(s % o for s, o in self.with_matched_version_of(other))
    def __rmod__(self, other)->'Dist[MemberType]':   # other % self
        return self._dist_output_type(o % s for s, o in self.with_matched_version_of(other))
    def __imod__(self,other) -> 'Dist[MemberType]':  # self %= other
        self.__dict__['_members'] = [s % o for s, o in self.with_matched_version_of(other)]
        return self

    def __truediv__(self, other)->'Dist[MemberType]':    # self / other
        return self._dist_output_type(s / o for s, o in self.with_matched_version_of(other))
    def __rtruediv__(self, other)->'Dist[MemberType]':   # other / self
        return self._dist_output_type(o / s for s, o in self.with_matched_version_of(other))
    def __itruediv__(self,other) -> 'Dist[MemberType]':  # self /= other
        self.__dict__['_members'] = [s / o for s, o in self.with_matched_version_of(other)]
        return self

    def __floordiv__(self, other)->'Dist[MemberType]':   # self // other
        return self._dist_output_type(s // o for s, o in self.with_matched_version_of(other))
    def __rfloordiv__(self, other)->'Dist[MemberType]':  # other // self
        return self._dist_output_type(o // s for s, o in self.with_matched_version_of(other))
    def __ifloordiv__(self,other) -> 'Dist[MemberType]':  # self /= other
        self.__dict__['_members'] = [s // o for s, o in self.with_matched_version_of(other)]
        return self

    def __matmul__(self, other)->'Dist[MemberType]':    # self @ other
        return self._dist_output_type(s @ o for s, o in self.with_matched_version_of(other))
    def __rmatmul__(self, other)->'Dist[MemberType]':   # other @ self
        return self._dist_output_type(o @ s for s, o in self.with_matched_version_of(other))
    def __imatmul__(self,other) -> 'Dist[MemberType]':  # self @= other
        self.__dict__['_members'] = [s @ o for s, o in self.with_matched_version_of(other)]
        return self

    # --- Dist vectorized bitwise ops ---
    def __and__(self, other)->'Dist[MemberType]':   # self & other
        return self._dist_output_type(s & o for s, o in self.with_matched_version_of(other))
    def __rand__(self, other)->'Dist[MemberType]':  # other & self
        return self._dist_output_type(o & s for s, o in self.with_matched_version_of(other))
    def __iand__(self,other) -> 'Dist[MemberType]':  # self &= other
        self.__dict__['_members'] = [s & o for s, o in self.with_matched_version_of(other)]
        return self

    def __or__(self, other)->'Dist[MemberType]':    # self | other
        return self._dist_output_type(s | o for s, o in self.with_matched_version_of(other))
    def __ror__(self, other)->'Dist[MemberType]':   # other | self
        return self._dist_output_type(o | s for s, o in self.with_matched_version_of(other))
    def __ior__(self,other) -> 'Dist[MemberType]':  # self |= other
        self.__dict__['_members'] = [s | o for s, o in self.with_matched_version_of(other)]
        return self

    def __xor__(self, other)->'Dist[MemberType]':    # self ^ other
        return self._dist_output_type(s ^ o for s, o in self.with_matched_version_of(other))
    def __rxor__(self, other)->'Dist[MemberType]':   # other ^ self
        return self._dist_output_type(o ^ s for s, o in self.with_matched_version_of(other))
    def __ixor__(self,other) -> 'Dist[MemberType]':  # self ^= other
        self.__dict__['_members'] = [s ^ o for s, o in self.with_matched_version_of(other)]
        return self

    def __lshift__(self, other)->'Dist[MemberType]':    # self << other
        return self._dist_output_type(s << o for s, o in self.with_matched_version_of(other))
    def __rlshift__(self, other)->'Dist[MemberType]':   # other << self
        return self._dist_output_type(o << s for s, o in self.with_matched_version_of(other))
    def __ilshift__(self,other) -> 'Dist[MemberType]':  # self <<= other
        self.__dict__['_members'] = [s << o for s, o in self.with_matched_version_of(other)]
        return self

    def __rshift__(self, other)->'Dist[MemberType]':    # self >> other
        return self._dist_output_type(s >> o for s, o in self.with_matched_version_of(other))
    def __rrshift__(self, other)->'Dist[MemberType]':   # other >> self
        return self._dist_output_type(o >> s for s, o in self.with_matched_version_of(other))
    def __irshift__(self,other) -> 'Dist[MemberType]':  # self >>= other
        self.__dict__['_members'] = [s >> o for s, o in self.with_matched_version_of(other)]
        return self

    # --- Dist vectorized unary ops ---

    def __neg__(self)->'Dist[MemberType]':    # -self
        return self._dist_output_type(-s for s in self._members)

    def __pos__(self)->'Dist[MemberType]':    # +self
        return self._dist_output_type(+s for s in self._members)

    def __abs__(self)->'Dist[MemberType]':    # abs(self)
        return self._dist_output_type(abs(s) for s in self._members)

    def __invert__(self)->'Dist[MemberType]':    # ~self
        return self._dist_output_type(~s for s in self._members)

    def __round__(self, n=None) ->'Dist[MemberType]':  # round(self, n)
        return self._dist_output_type(round(s, n) for s in self._members)

    def __trunc__(self) ->'Dist[MemberType]':  # math.trunc(self)
        return self._dist_output_type(trunc(s) for s in self._members)

    def __floor__(self) ->'Dist[MemberType]':  # math.floor(self)
        return self._dist_output_type(floor(s) for s in self._members)

    def __ceil__(self) ->'Dist[MemberType]':  # math.ceil(self)
        return self._dist_output_type(ceil(s) for s in self._members)
Dist._dist_output_type = Dist



class Pair(Dist[MemberType]):
    """A Pair is a Dist with special methods to refer to its first two elements as .l/.left and .r/.right.
       Often useful for bilaterally symmetrical robots whose devices come in left/right pairs."""

    @property
    def left(self) -> MemberType:
        """pair.left and pair.l return or adjust pair[0]"""
        return self._members[0]
    @left.setter
    def left(self, new_member: MemberType):
        self._members[0] = new_member
    l = left

    @property
    def right(self) -> MemberType:
        """pair.right and pair.r return or adjust pair[1]"""
        return self._members[1]
    @right.setter
    def right(self, new_member: MemberType):
        self._members[1] = new_member
    r = right
Pair._dist_output_type = Pair



