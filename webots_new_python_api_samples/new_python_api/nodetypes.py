"""A partial repository of information about Webots Scene Tree Nodetypes and their fields."""

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

from collections import ChainMap
from typing import Dict

from descriptors import cached_property, descriptor

GEOMETRY_TYPES = set("Box, Capsule, Cone, Cylinder, ElevationGrid, IndexedFaceSet, IndexedLineSet, Mesh, Plane, "
                     "PointSet, Sphere, Rectangle, TexturedBox, TexturedParallelepiped, Extrusion".split(', '))

class MetaNodeInfo(type):
    """This metaclass allows cls.all_annotations to work as a property for NodeInfo classes."""
    @property
    def all_annotations(cls) -> Dict[str, str]:
        """Returns all annotations for the given class, with annotations converted to strings."""
        if not '_all_annotations' in cls.__dict__:
            chainmap = ChainMap(*(c.__annotations__ for c in cls.__mro__ if '__annotations__' in c.__dict__))
            cls._all_annotations = {key: (value if isinstance(value, str) else value.__name__)
                                    for key,value in chainmap.items()}
        return cls._all_annotations

class NodeInfo(metaclass=MetaNodeInfo): pass

class SFNode: pass
class SFVec2f: pass
class SFVec3f: pass
class SFColor: pass
class SFRotation: pass
class SFBool: pass
class SFInt: pass
class SFFloat: pass
class SFString: pass

class MFNode: pass
class MFVec2f: pass
class MFVec3f: pass
class MFColor: pass
class MFRotation: pass
class MFBool: pass
class MFInt: pass
class MFFloat: pass
class MFString: pass

class Group(NodeInfo):
    children: MFNode = []  # {node, PROTO}

class Transform(Group):
    translation: SFVec3f =     (0, 0, 0)     # any vector
    rotation: SFRotation =     (0, 0, 1, 0)  # unit axis, (-inf, inf) angle
    scale: SFVec3f =           (1, 1, 1)     # any vector
    translationStep: SFFloat = 0.01
    rotationStep: SFFloat =    0.261799387

class Solid(Transform):
    name:                SFString = "solid"      # any string
    model:               SFString = ""           # any string
    description:         SFString = ""           # any string
    contactMaterial:     SFString = "default"    # any string
    immersionProperties: MFNode =   [ ]          # {ImmersionProperties, PROTO}
    boundingObject:      SFNode =   None         # {node, PROTO}
    physics:             SFNode =   None         # {Physics, PROTO}
    locked:              SFBool =   False        # {TRUE, FALSE}
    radarCrossSection:   SFFloat =  0.0          # [0, 1]
    recognitionColors:   MFColor =  []           # any color
    linearVelocity:      SFVec3f =  (0, 0, 0)    # any vector
    angularVelocity:     SFVec3f =  (0, 0, 0)    # any vector

class Shape(NodeInfo):
    appearance:  SFNode = None  # {Appearance, PROTO}
    geometry:    SFNode = None  # {Geometry Primitive, PROTO}
    castShadows: SFBool = True  # {TRUE, FALSE}
    isPickable:  SFBool = True  # {TRUE, FALSE}

class Appearance(NodeInfo):
    material:         SFNode =   None           # {Material, PROTO}
    texture:          SFNode =   None           # {ImageTexture, PROTO}
    textureTransform: SFNode =   None           # {TextureTransform, PROTO}
    name:             SFString = "appearance"   # any string

class PBRAppearance(NodeInfo):
    baseColor:            SFColor =  (1, 1, 1)        # any color
    baseColorMap:         SFNode =   None             # {ImageTexture, PROTO}
    transparency:         SFFloat =  0                # [0, 1]
    roughness:            SFFloat =  0                # [0, 1]
    roughnessMap:         SFNode =   None             # {ImageTexture, PROTO}
    metalness:            SFFloat =  1                # [0, 1]
    metalnessMap:         SFNode =   None             # {ImageTexture, PROTO}
    IBLStrength:          SFFloat =  1                # [0, inf)
    normalMap:            SFNode =   None             # {ImageTexture, PROTO}
    normalMapFactor:      SFFloat =  1                # [0, inf)
    occlusionMap:         SFNode =   None             # {ImageTexture, PROTO}
    occlusionMapStrength: SFFloat =  1                # [0, inf)
    emissiveColor:        SFColor =  (0, 0, 0)        # any color
    emissiveColorMap:     SFNode =   None             # {ImageTexture, PROTO}
    emissiveIntensity:    SFFloat =  1                # [0, inf)
    textureTransform:     SFNode =   None             # {TextureTransform, PROTO}
    name:                 SFString = "PBRAppearance"  # any string

class Physics(NodeInfo):
    density:       SFFloat = 1000   # {-1, [0, inf)}
    mass:          SFFloat = -1     # {-1, [0, inf)}
    centerOfMass:  MFVec3f = []     # any vector
    inertiaMatrix: MFVec3f = []     # any two vectors
    damping:       SFNode =  None   # {Damping, PROTO}


GEOMETRY_COMPATRIOT_TYPES = [Solid, Transform, Shape, Appearance, PBRAppearance, Physics]
GEOMETRY_COMPATRIOT_FIELDS = {cls.__name__: set(cls.all_annotations) for cls in GEOMETRY_COMPATRIOT_TYPES}
