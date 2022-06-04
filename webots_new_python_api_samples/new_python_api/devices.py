"""This contains classes and methods for handling Webots robot devices in the "New Python API".
   These will all typically be accessed via the `robot` module."""

__author__ = "Justin C. Fisher"

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

import io
import itertools
import json
import math
import pickle
from functools import wraps
from typing import List, Tuple, Set, Union, Optional, Callable, Any, Sequence, Iterable, TYPE_CHECKING, TypeVar, \
    overload, Type, Generic
from ctypes import c_int, c_double, c_float, c_bool, c_char_p, c_void_p, POINTER, c_char, c_ubyte
import re

import core_api
from core_api import *  # imports WB constants and the ctypes library wb, and timed_cached_property
from webots_warnings import Warn, WarnOnce, use_docstring_as_deprecation_warning
from surrogate_values import SurrogateValue, surrogate_attribute
from descriptors import descriptor, cached_property # more options than python's @property
from vectors import Vector, Color, VectorValue, Vec3f, Vec4f, Iterable2f, Iterable3f, Iterable4f, ColorBGRA, \
    GenericVector

# --- Additional types that can't simply be imported ---

c_float_p = POINTER(c_float)
c_ubyte_p = POINTER(c_ubyte)

NUMPY_CONFIRMED = False  # We want to be able to type-hint functions that return numpy arrays without requiring numpy
if TYPE_CHECKING:  # When type-checking we'll import numpy if available, but not at runtime
    try:
        import numpy as np
        NUMPY_CONFIRMED = True
    except ImportError:
        pass

if NUMPY_CONFIRMED:
    NumpyArrayType = np.ndarray
else:
    NumpyArrayType = TypeVar('NumpyArrayType')

# ---- functions easing transition from old CamelCase notation and python snake_case

def snake_case(name):
    """Returns a new copy of name, with the same number of leading underscores, and each capital L that is
       followed by a lowercase letter converted to _l.  E.g. _CamelToSnake -> _camel_to_snake"""
    stripped_name = name.lstrip('_')
    under_prefix = '_' * (len(name) - len(stripped_name))
    return under_prefix + re.sub('([A-Z])[a-z]', lambda cap: '_'+cap[0].lower(), stripped_name).lstrip('_')


# === General Device SuperClasses ================

DeviceType = TypeVar('DeviceType', bound='Device')

class DeviceList:
    """The one expected instance of this class will be robot.devices which will provide access to the
       number of devices as `len(robot.devices)`, allow iteration over devices via `for d in robot.devices`,
       and allow indexed reference to devices, e.g. by `robot.devices[0]` or `robot.devices['left motor']`.
       This indexing differs from `Device(name_or_index)` in that this can employ slicing like `robot.devices[0:3]`
       or negative indexing like `robot.devices[-1]` and in that this will not re-call the __init__ method of a
       previously constructed device, whereas `Device(name_or_index)` always does."""

    by_index:List['Device'] = [None] * wb.wb_robot_get_number_of_devices()

    # Since we expect the only instance of this class to be devices, we use `devices` in place of the standard `self`
    @property
    def extend_if_needed(devices) -> 'List[Device]':
        """Checks if additional devices have been added to the robot, and if so pads the device list accordingly.
           Returns the contained Python list of devices."""
        n = wb.wb_robot_get_number_of_devices()
        contained_list = devices.by_index
        if n > len(contained_list):
            contained_list.extend([None] * (n - len(contained_list)))
        return contained_list

    def __len__(devices) -> int:
        """The total number of devices this robot has."""
        return wb.wb_robot_get_number_of_devices()

    def __getitem__(devices, index_or_name: Union[int, str, slice]) -> 'Device':
        if isinstance(index_or_name, str):
            d = Device.map_name_to_device.get(index_or_name, None)
        else:
            assert index_or_name < wb.wb_robot_get_number_of_devices() # TODO testing
            d = devices.extend_if_needed[index_or_name]
        if d is not None: return d
        return Device(index_or_name)  # if unable to find existing match, try to create new one

    def register(devices, d: 'Device'):
        """Called by Device.__new__ to register newly created device d in the devicelist"""
        devices.extend_if_needed[d.index] = d

    def __iter__(devices) -> Iterable['Device']:
        for i in range(len(devices)): yield devices[i]

    def __reversed__(devices) -> Iterable['Device']:
        for i in reversed(range(len(devices))): yield devices[i]

    # TODO check that default iteration works!

devices = DeviceList()  # create the single DeviceList instance; this will be imported into robot to be robot.devices

# TODO: consider whether this really adds significant value beyond already having Device(i) and robot.devices[i]
class MetaDevice(type, Generic[DeviceType]):
    """This Device metaclass enables robot.Device[name_or_index] to work as an alternative device "constructor" that
       will return an existing or new python Device object of the appropriate type, or None if no match is possible.
       Device subclasses like Motor inherit this metaclass, so Motor[i] will return a Motor if Device[i] is a Motor,
       and None otherwise.  Unlike the __new__ method triggered by Device(i), Device[i] won't re-trigger the __init__
       method of already-existing devices so is generally preferable in cases where that is likely.
       This does not provide the full container-like functionality that the DeviceList robot.devices provides.
       In particular, both Device[-1] and Device[0:2] would yield None, whereas robot.devices[-1] would wrap back to
       the last device, and robot.devices[0:2] would return a list of two devices.
       """

    def __getitem__(cls: Type[DeviceType], index_or_name: Union[int, str]) -> DeviceType:
        if isinstance(index_or_name, str):  # if given a string name, try looking it up in our dict of known names
            d = Device.map_name_to_device.get(index_or_name, None)
        else:  # if given an index
            if not 0 <= index_or_name < wb.wb_robot_get_number_of_devices(): return None
            d = devices.extend_if_needed[index_or_name]
        if d is None:
            d = Device(index_or_name)  # if unable to find existing match, try to create new one
        return d if isinstance(d, cls) else None

    def __iter__(cls: Type[DeviceType]) -> Iterable[DeviceType]:
        return (d for d in devices if isinstance(d, cls))

    def __reversed__(cls: Type[DeviceType]) -> Iterable[DeviceType]:
        return (d for d in reversed(devices) if isinstance(d, cls))

# === umbrella Device class ===

class Device(metaclass=MetaDevice):
    """A Python Device object is used to control a device node (e.g. motor or sensor) in the simulation.
       A Device may be created by calling robot.Device(n) or calling some subclass like robot.Motor(n), which is
       generally preferable as it will provide better type-hinting and documentation for users of Python editors
       that support such niceties, like PyCharm. The given name_or_index should either be a string corresponding
       to the name field of the relevant device node, or an integer index ranging from 0 up to the number of devices
       the robot has (accessible via len(robot.devices)). If name_or_index is omitted, then the first of the robot's
       devices matching the class will be returned. E.g. robot.Camera() returns its first (and often only) camera.
       If a Python Device object for the relevant device already exists, it will be returned to avoid duplicating
       Python objects controlling the same device node.
       Otherwise, a new Python Device object will be created, belonging to the relevant subclass like LightSensor.
       If no device with the given name/index and a compatible class is found, None is returned.
       Whenever any Sensor device (e.g. GPS or Camera) is created, if `sampling` is omitted or True, the
       sensor will automatically be enabled with the simulation's basic timestep as its sampling period, so it will
       produce a new sensor reading each timestep.  If a positive value for `sampling` is given, that number of
       milliseconds will be used. If `sampling` is None or False, the sensor will not be enabled."""

    map_nodetype_to_subclass = {}  # this dictionary will be populated by __init_subclass__ as subclasses are defined
    map_name_to_device = {}        # will be populated by __new__ as devices are created
    _world = None                  # if the world module is imported by a supervisor, this will be updated to it
                                   # and will then be used to help supplement some gaps in the ordinary robot API

    def __new__(cls: type,
                name_or_index: Union[str, int] = None,
                sampling: Union[int, bool, None] = True) -> Optional['Device']:
        if isinstance(name_or_index, str):  # if given a name, like "motor1"
            tag = wb.wb_robot_get_device(name_or_index.encode())
            if not tag: return None
            name = name_or_index
        elif isinstance(name_or_index, int):  # if given a device index, ranging from 0 to number of devices minus 1
            tag = wb.wb_robot_get_device_by_index(name_or_index)  # if in bounds returns 1+index, else warns&returns 0
            if not tag: return None
            name = None
        elif name_or_index is None:
            return next(iter(cls), None)
        else:
            raise TypeError(f"Argument {name_or_index} passed to {cls.__name__} constructor "
                            f"was not a string name nor an integer index")

        self = devices.extend_if_needed[tag - 1]  # Retrieve existing Device with the given index (i.e. tag-1), or None

        # Check that the class of the found device is appropriate
        if self and cls is Device: return self  # no class check needed for generic Device(n) calls yielding known devices
        new_cls: type = Device.map_nodetype_to_subclass.get(wb.wb_device_get_type(tag), None)
        if new_cls is None:
            WarnOnce(f"Device {name} has Webots NodeType identifier #{wb.wb_device_get_type(tag)},"
                     f" which has no known corresponding Python device subclass.")
            return None
        if issubclass(cls, new_cls): new_cls = cls  # if told to create a more specific subclass, will comply
        if not issubclass(new_cls, cls):  # but if told to make an *incompatible* subclass, raise warning
            WarnOnce(f"Device constructor was told to find a(n) {cls.__name__} with name/index {name_or_index}, "
                     f"but found a(n) {new_cls.__name__} with that name/index instead.")
            return None
        if self: return self  # if found a known device, and the classes matched up, return it!

        # Otherwise, we didn't already have a device with the relevant name/index, so we must create a new one
        self: Device = object.__new__(new_cls)  # object.__new__ will create a new object of class new_cls
        self.tag = tag
        if name: self.name = name
        Device.map_name_to_device[self.name] = self  # remember that this name maps to this device
        devices.register(self)  # store this device in our list of devices by index

        # By default, Sensors will be enabled with sampling period equal to the simulation's basic timestep
        if isinstance(self, Sensor) and sampling:
            self.sampling = sampling
        elif sampling is not True:  # if given a sampling period for a non-Sensor
            try:  # in the spirit of python duck-typing, we'll try to _enable this device anyway
                self._enable(self.tag, sampling)
            except AttributeError:
                WarnOnce(f"{self} was given a sampling period, but is not a Sensor!")

        return self

    def __repr__(self):  # E.g., LightSensor("light sensor 3")
        return f'{type(self).__name__}("{self.name}")'

    def __init_subclass__(subclass, api_name: str = None, **kwargs):
        """This will be called whenever a new subclass of Device is created below.  This does standard things
           including setting the api_name for the subclass (defaults to that subclass' own name),
           and remembering that the corresponding WB_NODE_CONSTANT maps to this subclass to speed future lookups."""
        subclass.api_name = api_name if api_name != None else snake_case(subclass.__name__)
        subclass._nodeTypeConstant = globals().get(f"WB_NODE_{subclass.api_name.upper()}", None)
        if subclass._nodeTypeConstant:
            Device.map_nodetype_to_subclass[subclass._nodeTypeConstant] = subclass
        super().__init_subclass__(**kwargs)  # allows potential multi-inheritance from this

    # Todo: should I leave .name as an attribute set on creation, or make it a property that looks itself up again?

    wb.wb_device_get_name.restype = c_char_p
    @cached_property
    def name(self):
        """The value of this device's name field in the Webots simulation at the time of first access.
           This will typically still be the current value of its name field, unless a supervisor renamed it."""
        return wb.wb_device_get_name(self.tag).decode()

    @use_docstring_as_deprecation_warning
    def getName(self):
        """DEPRECATED: Device.getName() is deprecated. Please use device.name.
           Returns the current value of this device's name field in the Webots simulation. This will almost always
           be equivalent to self.name, set at device creation, but could vary if a supervisor renames devices."""
        return wb.wb_device_get_name(self.tag).decode()

    wb.wb_device_get_model.restype = c_char_p
    @property
    def model(self) -> str:
        """Returns the contents of the model field of this device's node in the simulation."""
        return wb.wb_device_get_model(self.tag)

    @use_docstring_as_deprecation_warning
    def getModel(self) -> str:
        """DEPRECATED. device.getModel() is deprecated. Please use device.model instead."""
        return wb.wb_device_get_model(self.tag)

    def getNodeType(self) -> type:
        """DEPRECATED. Unlike earlier versions of Webots, this now returns the Python class that this device belongs to,
           e.g. Accelerometer. The expression `d.getNodeType()` is now equivalent to the simple Python `type(d)`.
           The recommended way to test whether device d is a Motor is with Python's `isinstance(d, robot.Motor)`.
           For backwards compatibility in ordinary *non-supervisor* controllers, the various nodetype constants
           like Node.ACCELEROMETER are still available, and still yield the expected comparison results, but
           these are deprecated and may eventually be removed.  So old code that checked if device d is an
           accelerometer by saying `d.getNodeType() == Node.ACCELEROMETER` will still work for now, but should be
           changed to say `isinstance(d, robot.Accelerometer)`."""
        classname = self.__class__.__name__
        WarnOnce(f"DEPRECATION: {classname}.getNodeType() is deprecated and may eventually be removed.  "
                 f"To test whether device d is a {classname}, please use isinstance(d, robot.{classname})")
        return type(self)
        # return wb.wb_device_get_node_type(self)  # former version, returns NodeType constant integers

    @property
    def index(self) -> int:
        """Returns the index of this device in this robot's list of devices.  Indices range from 0 to one less than the
           number of devices in this robot.  robot.Device(index) or robot.devices[index] can be used to retrieve
           a device by its index, and len(robot.devices) is the total number of devices in this robot."""
        return self.tag - 1  # indices are 0-indexed, whereas tags are 1-indexed

class Sensor():
    """This abstract class includes Devices and other entities that have similar methods to read and alter their
       sampling period.  Sensors typically also have a .value attribute that returns the most recent reading
       (though some complex sensors, like Cameras, have multiple methods to yield different sorts of readings).
       Sensors with a .value attribute will typically also be SurrogateValues that will behave like their .value
       in many contexts, enabling you to say "sensor > 50" as shorthand for "sensor.value > 50".
       Python often embraces "duck-typing", accepting any argument that can "quack like a duck" in any context
       that would accept ducks, so many sensors have been trained to "quack like" their own .value .
       For Sensors that are stand-alone Devices, when the Device is created, if no sampling argument (or
       sampling = True) is given the sensor will be enabled with the simulation's basic timestep as its
       sampling period, so it will produce a new sensor reading each timestep.  If a positive sampling is given,
       that number of milliseconds will be used instead. If the sampling is None, the sensor will not be enabled."""
    # TODO: it could make sense to temporarily cache sensor .values so that repeated lookups during the same
    #       e.g. using timed_cached_properties

    def __init_subclass__(subclass, api_name: str = None, auto_link_wb_methods = True, **kwargs):
        """This will be called whenever a new subclass of Sensor is created below.
           Unless auto_link_wb_methods is False, this will automatically associate that subclass with
           various wb_api functions for enabling/disabling it."""
        # for Device+Sensor hybrids, Device should have set api_name; and we don't wanna override
        if 'api_name' not in subclass.__dict__:
            subclass.api_name = api_name if api_name is not None else snake_case(subclass.__name__)
        super().__init_subclass__(**kwargs) # allows potential further multi-inhericance
        if auto_link_wb_methods:
            subclass._enable  = getattr(wb, f"wb_{subclass.api_name}_enable")
            subclass._disable = getattr(wb, f"wb_{subclass.api_name}_disable")
            subclass._get_sampling_period = getattr(wb, f"wb_{subclass.api_name}_get_sampling_period")

    def __repr__(self): return type(self).__name__

    @cached_property
    def _tag_if_needed(self):
        """Sensors associated with a particular Device must pass Device tag as an argument; other sensors don't.
           We return a tuple, so that unpacking it with * can insert 1 or 0 arguments, as needed."""
        # some devices may be surrogates so we don't want to trigger their .value, so we look only in own __dict__
        tag = self.__dict__.get('tag', None)
        return (tag,) if tag else tuple()

    @property
    def sampling(self) -> int:
        """The current sampling period for this sensor, the number of milliseconds of simulated time it will take to
           generate each new sensor reading.  Can be read via sensor.sampling or adjusted via
           sensor.sampling = new_sampling_period.  Setting sensor.sampling = True enables that sensor using the
           simulation's basic timestep as its sampling period (done by default on Sensor creation).
           Setting sensor.sampling = None disables that sensor from making new readings."""
        return self._get_sampling_period(*self._tag_if_needed)
    @sampling.setter
    def sampling(self, new_sampling_period: Union[int, bool, None]):
        if new_sampling_period is True: new_sampling_period = wb.wb_robot_get_basic_time_step()
        if new_sampling_period > 0:
            self._enable(*self._tag_if_needed, int(new_sampling_period))
        else:
            self._disable(*self._tag_if_needed)

    # TODO decide whether we really want to deprecate these, or just keep them as redundant?
    @use_docstring_as_deprecation_warning
    def enable(self, sampling_period: Union[int, bool, None] = True):
        """DEPRECATED: sensor.enable() is deprecated. By default sensors are enabled with the basic timestep,
           or use `sampling` keyword argument in device creation, or set sensor.sampling = period"""
        self.sampling = sampling_period

    @use_docstring_as_deprecation_warning
    def disable(self):
        """DEPRECATED: sensor.disable() is deprecated. Please use sensor.sampling = None  instead."""
        self._disable(*self._tag_if_needed)

    @use_docstring_as_deprecation_warning
    def getSamplingPeriod(self) -> int:
        """DEPRECATED: sensor.getSamplingPeriod() is deprecated. Please use sensor.sampling instead."""
        return self._get_sampling_period(*self._tag_if_needed)


class LookUpTableSensor(Device, Sensor, auto_link_wb_methods=False):
    """This abstract intermediate-level class includes Sensor Devices that also have lookup table methods."""
    def __init_subclass__(subclass, api_name: str = None, **kwargs):
        """This will be called whenever a new subclass of LookUpTableSensor is created below.
           This automatically associates that class with the wb_api function for its lookup table."""
        super().__init_subclass__(api_name=api_name, **kwargs)  # Sensor will use api_name too
        subclass._get_lookup_table = getattr(wb, f"wb_{subclass.api_name}_get_lookup_table")
        subclass._get_lookup_table.restype = c_void_p
        subclass._get_lookup_table_size = getattr(wb, f"wb_{subclass.api_name}_get_lookup_table_size")

    @property
    def lookup_table(self) -> Sequence[Vector]:
        """Returns the lookup table that defines how raw readings in this sensor are converted to returned values.
           The lookup table is returned as a list-like sequence of Vec3fs."""
        ArrayType = Vec3f * self._get_lookup_table_size(self.tag)
        return ArrayType.from_address(self._get_lookup_table(self.tag))

    @use_docstring_as_deprecation_warning
    def getLookupTable(self) -> Vector:
        """DEPRECATED: Device.getLookupTable() is deprecated. Please use device.lookup_table instead, and if you want
        to use Python list.methods on it, you may need to use list() to convert the table and/or its rows to lists."""
        return self.lookup_table

# === Specific Device Classes =============================

# === Sensor Devices =========

class Accelerometer(LookUpTableSensor, VectorValue):
    """A python Accelerometer object is used to control an accelerometer sensor node in the Webots simulation.
       `accelerometer = robot.Accelerometer("devicename")` creates an Accelerometer to control the node with that name.
       `accelerometer.value` returns the accelerometer's acceleration, in meters per second squared, along its 3 axes,
       as a 3D Vector, though if any axes have been disabled in the simulation, they will read as float('nan'), so you
       will likely then just want to refer to individual axes, e.g. using `accelerometer.y`.
       For most purposes, an Accelerometer serves as a surrogate for its own .value, so e.g., `accelerometer.magnitude`
       will return the vector magnitude of its .value and `velocity + accelerometer * dt` could be an effective way
       of estimating how the vector `velocity` would change over a period of duration dt.
       Like other sensors, upon creation, an Accelerometer is automatically enabled with the basic timestep as its
       sampling period, adjustable via accelerometer.sampling, or this can be set to None to disable its readings.
       Like other lookup table sensors, you can get information about the lookup table via accelerometer.lookup_table."""

    wb.wb_accelerometer_get_values.restype = c_void_p
    @property
    def value(self) -> Vec3f:
        """The current reading of this accelerometer, given as a 3D vector. Note: for most purposes, the
           accelerometer device itself may be used in place of its .value, e.g., in vector arithmetic."""
        return Vec3f.from_address(wb.wb_accelerometer_get_values(self.tag))
    def getValues(self) -> Vec3f:
        """DEPRECATED. Accelerometer.getValues() is deprecated. Please use accelerometer.value instead,
           or for most purposes you can just use accelerometer in place of its own .value"""
        return self.value

class Altimeter(Device, Sensor, SurrogateValue):
    """A python Altimeter object is used to control an altimeter sensor node in the Webots simulation.
       `altimeter = robot.Altimeter("devicename")` creates an Altimeter to control the altimeter with that name.
       `altimeter.value` returns the altimeter's height above the global reference plane.
       For most purposes, an Altimeter serves as a surrogate for its own .value, so e.g., `altimeter / 2` will be half
       of altimeter.value, and `if altimeter > 10` will be treated as True when altimeter.value > 10.
       Like other sensors, upon creation, an Altimeter is automatically enabled with the basic timestep as its sampling
       period, adjustable via altimeter.sampling, or this can be set to None to disable its readings."""
    wb.wb_altimeter_get_value.restype = c_double
    @property
    def value(self) -> float:
        """Returns this Altimeter's height, in meters, oabove the global reference plane."""
        return wb.wb_altimeter_get_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getValue(self) -> float:
        """DEPRECATED: Altimeter.getValue() is deprecated. Please use altimeter.value instead."""
        return wb.wb_altimeter_get_value(self.tag)




class Compass(LookUpTableSensor, VectorValue):
    """A python Compass object is used to control a compass sensor node in the Webots simulation.
       `compass = robot.Compass("devicename")` creates a Compass to control the compass node with the given name.
       `compass.value` returns a vector that points from the compass device to the simulation's north direction,
       though if any axes have been disabled in the simulation, they will read as `float('nan')`, so you will likely
       then just want to refer to individual axes, e.g. using `compass.y`.
       For most purposes, a Compass serves as a surrogate for its own .value, so e.g., `pos + 2 * compass.unit_vector`
       could be an effective way to compute a location 2 meters north of vector pos, in the compass' frame of reference.
       Like other sensors, upon creation, a Compass is automatically enabled with the basic timestep as its
       sampling period, adjustable via compass.sampling, or this can be set to None to disable its readings.
       Like other lookup table sensors, you can get information about the lookup table via compass.lookup_table."""
    wb.wb_compass_get_values.restype = c_void_p
    @property
    def value(self) -> Vec3f:
        return Vec3f.from_address(wb.wb_compass_get_values(self.tag))
    @use_docstring_as_deprecation_warning
    def getValues(self) -> Vec3f:
        """DEPRECATED: Compass.getValues() is deprecated. Please use compass.value instead, or for most purposes,
           a compass will serve as a surrogate for its own .value."""
        return self.value


class DistanceSensor(LookUpTableSensor, SurrogateValue):
    """A python DistanceSensor object is used to control a distance sensor node in the Webots simulation.
       `ds = robot.DistanceSensor("devicename")` creates a DistanceSensor to control the one with that name.
       `ds.value` returns a float that indicates distance to the closest object in the direction of the sensor,
       as determined by the type of sensor and its lookup table.
       For most purposes, a DistanceSensor serves as a surrogate for its own .value, so e.g., `if ds < 50` would
       compare ds.value to 50, and `(DS1 + DS2)/2` could compute the average of two distance sensors' readings.
       Like other sensors, upon creation, a DistanceSensor is automatically enabled with the basic timestep as its
       sampling period, adjustable via ds.sampling, or this can be set to None to disable its readings.
       Like other lookup table sensors, you can get information about the lookup table via ds.lookup_table."""

    # In python, type constants will be presented as informative strings, not as integers
    type_constant_to_string = {WB_DISTANCE_SENSOR_GENERIC: 'GENERIC',
                               WB_DISTANCE_SENSOR_INFRA_RED: 'INFRA_RED',
                               WB_DISTANCE_SENSOR_SONAR: 'SONAR',
                               WB_DISTANCE_SENSOR_LASER: 'LASER'}

    # TODO these are included for backwards compatibility, could deprecate
    GENERIC = 'GENERIC'
    INFRA_RED = 'INFRA_RED'
    SONAR = 'SONAR'
    LASER = 'LASER'

    def type(self) -> 'str':
        """Returns the type of this distance sensor, which will be 'GENERIC', 'INFRA_RED', 'SONAR', or 'LASER'."""
        return self.type_constant_to_string[wb.wb_distance_sensor_get_type(self.tag)]
    @use_docstring_as_deprecation_warning
    def getType(self) -> 'str':
        """DEPRECATION: DistanceSensor.getType() is deprecated. Please use distance_sensor.type,
           and note that it now returns strings like 'INFRA_RED' or 'SONAR'."""
        return self.type_constant_to_string[wb.wb_distance_sensor_get_type(self.tag)]

    wb.wb_distance_sensor_get_value.restype = c_double
    @property
    def value(self) -> float:
        """Returns the current value of this DistanceSensor, as determined by its lookup table."""
        return wb.wb_distance_sensor_get_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getValue(self) -> float:
        """DEPRECATED: DistanceSensor.getValue() is deprecated. Please use distancesensor.value instead."""
        return wb.wb_distance_sensor_get_value(self.tag)

    wb.wb_distance_sensor_get_max_value.restype = c_double
    @property
    def max_value(self) -> float:
        """Returns the current maximum possible value for this DistanceSensor."""
        return wb.wb_distance_sensor_get_max_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxValue(self) -> float:
        """DEPRECATED: DistanceSensor.getMaxValue() is deprecated. Please use distancesensor.max_value instead."""
        return wb.wb_distance_sensor_get_max_value(self.tag)

    wb.wb_distance_sensor_get_min_value.restype = c_double
    @property
    def min_value(self) -> float:
        """Returns the minimum possible value for this DistanceSensor."""
        return wb.wb_distance_sensor_get_min_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinValue(self) -> float:
        """DEPRECATED: DistanceSensor.getMinValue() is deprecated. Please use distancesensor.min_value instead."""
        return wb.wb_distance_sensor_get_min_value(self.tag)

    wb.wb_distance_sensor_get_aperture.restype = c_double
    @property
    def aperture(self) -> float:
        """Returns the current aperture of this DistanceSensor measured in radians."""
        return wb.wb_distance_sensor_get_aperture(self.tag)
    @use_docstring_as_deprecation_warning
    def getAperture(self) -> float:
        """DEPRECATED: DistanceSensor.getAperture() is deprecated. Please use distancesensor.aperture instead."""
        return wb.wb_distance_sensor_get_aperture(self.tag)


class GPS(Device, Sensor, VectorValue, api_name = 'gps'):
    """A Global Positioning Sensor (GPS) provides information about its absolute position in the simulated world.
       `gps = robot.GPS("device_name")` stores this as `gps`, and enables it to generate readings every timestep.
       `gps.coordinate_system` returns the coordinate system for this GPS, either "LOCAL" (Webots x,y,z coordinates),
       or "WGS84" (latitude-longitude-altitude).
       `gps.value` returns a 3D vector giving this device's location in this coordinate_system.
       `gps` itself can be used as a surrogate for its current value for most purposes.  For example,
       `gps.x` and `gps[0]` both return the first component of gps.value.  Similarly for .y or [1] and .z or [2].
       `gps - vector2` uses vector arithmetic to compute the difference between gps.value and vector2.
       `gps.speed` returns the current 3D speed vector for this gps, again using its coordinate_system.
       `gps.speed.magnitude` returns the 1D scalar magnitude of this speed (just as .magnitude does for other vectors).
       `gps.convert_to_degrees_minutes_seconds(decimal_degree)` returns a string depicting the given decimal_degree
       in degrees-minutes-seconds format.
       Like other sensors, Python automatically enables a GPS to generate readings every timestep, alterable
       by passing the `sampling=` keyword in device creation, or setting `gps.sampling = t` later."""

    LOCAL, WGS84 = 'LOCAL', 'WGS84'  # still defined as constants for backwards compatibility
    coordinate_system_as_string = {WB_GPS_LOCAL_COORDINATE: 'LOCAL', WB_GPS_WGS84_COORDINATE: 'WGS84'}

    wb.wb_gps_get_values.restype = c_void_p
    @property
    def value(self) -> Vec3f:
        """Returns a 3D vector giving this device's global location in the relevant coordinate system.
           For most purposes, you can use the GPS device itself as a surrogate for this value.
           For example, gps.x and gps[0] both return the first component of gps.value, and `gps - vector2` uses vector
           arithmetic to compute the difference between gps.value and vector2."""
        return Vec3f.from_address(wb.wb_gps_get_values(self.tag))
    @use_docstring_as_deprecation_warning
    def getValues(self) -> Vec3f:
        """DEPRECATED: GPS.getValues() is deprecated. Use gps.value instead, or for most purposes you can use
           a GPS device itself as a surrogate for its own value."""
        return self.value

    wb.wb_gps_get_speed_vector.restype = c_void_p
    @property
    def speed(self) -> Vec3f:
        """Returns the current 3D speed vector for this gps, in its 3D coordinate system.
           As with other Vector objects, you can retrieve its scalar magnitude with `gps.speed.magnitude`."""
        return Vec3f.from_address(wb.wb_gps_get_speed_vector(self.tag))
    @use_docstring_as_deprecation_warning
    def getSpeed(self) -> Vec3f:
        """DEPRECATED: GPS.getSpeed() is deprecated. Use gps.speed for the 3D speed vector,
           or gps.speed.magnitude for its 1D scalar magnitude."""
        return self.speed

    @property
    def coordinate_system(self) -> str:
        """Returns the coordinate system for this GPS, either "LOCAL", meaning Webots x,y,z coordinates,
           or "WGS84", meaning latitude-longitude-altitude."""
        return GPS.coordinate_system_as_string[wb.wb_gps_get_coordinate_system(self.tag)]
    @use_docstring_as_deprecation_warning
    def getCoordinateSystem(self) -> str:
        """DEPRECATED: GPS.getCoordinateSystem() is deprecated. Use gps.coordinate_system,
           which returns "LOCAL" or "WGS84"."""
        return self.coordinate_system

    wb.wb_gps_convert_to_degrees_minutes_seconds.restype = c_char_p
    @staticmethod
    def convert_to_degrees_minutes_seconds(decimal_degree: float) -> str:
        """Returns a string version depicting the given decimal_degree in degrees-minutes-seconds format."""
        return wb.wb_gps_convert_to_degrees_minutes_seconds(c_double(decimal_degree)).decode()
    @staticmethod
    @use_docstring_as_deprecation_warning
    def convertToDegreesMinutesSeconds(decimal_degree: float) -> str:
        """DEPRECATED: Webots is shifting to conventional python naming, so GPS.convertToDegreesMinutesSeconds() has
           become GPS.convert_to_degrees_minutes_seconds()."""
        return wb.wb_gps_convert_to_degrees_minutes_seconds(c_double(decimal_degree)).decode()


class Gyro(LookUpTableSensor, VectorValue):
    """A python Gyro object is used to control a gyroscopic ("gyro") sensor node in the Webots simulation.
       `gyro = robot.Gyro("devicename")` creates a Gyro to control the node with that name.
       `gyro.value` returns the gyro's angular velocity, in radians per second, around its 3 axes, as a 3D Vector,
       though if any axes have been disabled in the simulation, they will read as float('nan'), so you
       will likely then just want to refer to individual axes, e.g. using `gyro.y`.
       For most purposes, a Gyro serves as a surrogate for its own .value, so e.g., `gyro * dt` could be an
       effective way of estimating how far the gyro would turn around each axis over a period of duration dt.
       Like other sensors, upon creation, a Gyro is automatically enabled with the basic timestep as its
       sampling period, adjustable via gyro.sampling, or this can be set to None to disable its readings.
       Like other lookup table sensors, you can get information about the lookup table via gyro.lookup_table."""

    wb.wb_gyro_get_values.restype = c_void_p
    @property
    def value(self) -> Vec3f:
        return Vec3f.from_address(wb.wb_gyro_get_values(self.tag))
    @use_docstring_as_deprecation_warning
    def getValues(self) -> Vec3f:
        """DEPRECATED. Gyro.getValues() is deprecated. Use gyro.value instead,
           or for most purposes gyro will serve as a surrogate for its own .value."""
        return self.value


class InertialUnit(Device, Sensor, VectorValue):
    """A python InertialUnit object is used to control an inertial unit sensor node in the Webots simulation.
       `iu = robot.InertialUnit("devicename")` creates an InertialUnit to control the node with that name.
       `iu.value` returns the inertial unit's current rotation, in radians, around the three global axes (aka yaw,
       pitch and roll) as a 3D Vector, though if any axes have been disabled in the simulation, they will read as
       float('nan'), so you will likely then just want to refer to individual axes, e.g. using `iu.y`.
       For most purposes, an InertialUnit serves as a surrogate for its own .value, so e.g., `iu.magnitude`
       will return the vector magnitude of its .value and `iu / 2` would compute yaw/pitch/roll half as extreme
       as iu's current values.
       `iu.quaternion` returns a 4D rotation vector representing the inertial unit's rotation relative to global axes.
       `iu.noise` returns the value of the sensor's noise field which measures Gaussian noise added to the readings.
       Like other sensors, upon creation, an InertialUnit is automatically enabled with the basic timestep as its
       sampling period, adjustable via iu.sampling, or this can be set to None to disable its readings."""

    wb.wb_inertial_unit_get_roll_pitch_yaw.restype = c_void_p
    @property
    def value(self) -> Vec3f:
        return Vec3f.from_address(wb.wb_inertial_unit_get_roll_pitch_yaw(self.tag))
    @use_docstring_as_deprecation_warning
    def getRollPitchYaw(self) -> Vec3f:
        """DEPRECATED: InertialUnit.getRollPitchYaw is deprecated. Use inertialunit.value instead,
           or for most purposes an InertialUnit will serve as a surrogate for its own value."""
        return self.value

    wb.wb_inertial_unit_get_quaternion.restype = c_void_p
    @property
    def quaternion(self) -> Vec3f:
        return Vec4f.from_address(wb.wb_inertial_unit_get_quaternion(self.tag))
    @use_docstring_as_deprecation_warning
    def getQuaternion(self) -> Vec4f:
        """DEPRECATED. InertialUnit.getQuaternion() is deprecated.  Use inertialunit.quaternion instead."""
        return self.quaternion

    wb.wb_inertial_unit_get_noise.restype = c_double
    @property
    def noise(self) -> float:
        """Returns the noise of this InertualUnit."""
        return wb.wb_inertial_unit_get_noise(self.tag)
    @use_docstring_as_deprecation_warning
    def getNoise(self) -> float:
        """DEPRECATED. InertialUnit.getNoise() is deprecated.  Use inertialunit.noise instead."""
        return wb.wb_inertial_unit_get_noise(self.tag)



class LightSensor(LookUpTableSensor, SurrogateValue):
    """A python LightSensor object is used to control a light sensor node in the Webots simulation.
       `ls = robot.LightSensor("devicename")` creates a LightSensor to control the one with that name.
       `ls.value` returns a float indicating the amount of brightness received by that sensor.
       though if any axes have been disabled in the simulation, they will read as `float('nan')`, so you will likely
       then just want to refer to individual axes, e.g. using `ds.y`.
       For most purposes, a LightSensor serves as a surrogate for its own .value, so e.g., `if ls < 50` would
       compare ls.value to 50, and `(LS1 + LS2)/2` could compute the average of two light sensors' readings.
       Like other sensors, upon creation, a LightSensor is automatically enabled with the basic timestep as its
       sampling period, adjustable via ds.sampling, or this can be set to None to disable its readings.
       Like other lookup table sensors, you can get information about the lookup table via ls.lookup_table."""

    wb.wb_light_sensor_get_value.restype = c_double
    @property
    def value(self) -> float:
        """Returns the irradiance (intensity of light) on this LightSensor in watts per square meter."""
        return wb.wb_light_sensor_get_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getValue(self) -> float:
        """DEPRECATED: LightSensor.getValue() is deprecated. Please use lightsensor.value instead."""
        return wb.wb_light_sensor_get_value(self.tag)

class TouchSensor(LookUpTableSensor, SurrogateValue):

    # BUMPER = WB_TOUCH_SENSOR_BUMPER
    # FORCE = WB_TOUCH_SENSOR_FORCE
    # FORCE3D = WB_TOUCH_SENSOR_FORCE3D
    BUMPER = bool
    FORCE = float
    FORCE3D = Vec3f
    TYPES_DICT = {WB_TOUCH_SENSOR_BUMPER: BUMPER,
                  WB_TOUCH_SENSOR_FORCE: FORCE,
                  WB_TOUCH_SENSOR_FORCE3D: FORCE3D}

    @property
    def type(self) -> type:
        """Returns an indicator of the type of this sensor, either bool for bumpers, float for 1D force sensors,
        or Vec3f for 3D force sensors.  These types may also be referred to as
        touchsensor.BUMPER, .FORCE, and .FORCE3D, respectively."""
        return self.TYPES_DICT[wb.wb_touch_sensor_get_type(self.tag)]
    @use_docstring_as_deprecation_warning
    def getType(self) -> type:
        """DEPRECATED: TouchSensor.getType() is deprecated. Please use touchsensor.type instead."""
        return self.TYPES_DICT[wb.wb_touch_sensor_get_type(self.tag)]

    wb.wb_touch_sensor_get_value.restype = c_double
    wb.wb_touch_sensor_get_values.restype = c_void_p
    @property
    def value(self) -> Union[float, Vec3f]:
        """Returns the current reading of this TouchSensor.  For 3D force sensors, this value will be a 3D vector.
           For bumpers and 1D force sensors, it will instead be a float.  For bumpers, the float will be 1.0 or 0.0,
           which Python treats as being equivalent to True and False in boolean contexts.  Note: for most purposes,
           the touchsensor itself can be used as a surrogate for its own .value."""
        if wb.wb_touch_sensor_get_type(self.tag) == WB_TOUCH_SENSOR_FORCE3D:
            return Vec3f.from_address(wb.wb_touch_sensor_get_values(self.tag))
        else:
            return wb.wb_touch_sensor_get_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getValue(self) -> float:
        """DEPRECATED: TouchSensor.getValue() is deprecated.  Use touchsensor.value instead, or for most purposes,
           the touchsensor itself can be used as a surrogate for its own .value."""
        return wb.wb_touch_sensor_get_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getValues(self) -> Vec3f:
        """DEPRECATED: TouchSensor.getValues() is deprecated.  Use touchsensor.value instead, or for most purposes,
           the touchsensor itself can be used as a surrogate for its own .value."""
        return Vec3f.from_address(wb.wb_touch_sensor_get_values(self.tag))

# === Radar ===

class Radar(Device, Sensor, SurrogateValue):
    """This represents a Radar device, which detects nearby targets by their radar profiles.
       By default, each new Radar device is automatically enabled with the basic timestep as its sampling period,
       or you can use radar = Radar(name, sampling = p) to set another sampling period in milliseconds,
       or set it to None to disable sampling for now, alterable later by setting radar.sampling = p.
       Once enabled and a sampling period has elapsed, a Radar device can be used much like a Python list or tuple.
       `len(radar)` indicates how many Radar.Target objects are currently visible.
       `radar[0]` returns the 0th Target.
       `for target in radar` iterates over the currently visible Targets.
       `radar.value` returns a stable tuple (read-only list) of currently visible Targets. Unlike the radar device
        itself, this will not change as time passes; useful if you want to compare what's visible at different times."""

    class Target(ctypes.Structure):
        # Declare fields for c_types conversion from C-API
        _fields_ = [('distance', c_double),
                    ('received_power', c_double),
                    ('speed', c_double),
                    ('azimuth', c_double)]

        # Declare fields for Python linters
        distance: float
        recieved_power: float
        speed: float
        azimuth: float

        @property
        def angle(self) -> float:
            """Returns or adjusts the target's perceived angle, in radians, to the left (+y direction) of the radar's
               +x axis.  This is -target.azimuth, since azimuth is  left/right inverted from other Webots angles."""
            return -self.azimuth
        @angle.setter
        def angle(self, new_angle):
            self.azimuth = -new_angle

        def __repr__(self): return f"Radar.Target(distance={self.distance}, azimuth={self.azimuth})"

    Target_p = POINTER(Target)

    wb.wb_radar_get_target.restype = Target_p   # will return pointer to single Radar.Target
    wb.wb_radar_get_targets.restype = Target_p  # will return pointer to array of Radar.Targets
    @timed_cached_property
    def value(self) -> Sequence[Target]:
        """Returns the sequence of Radar.Target objects currently visible to this radar device,
           stored as a ctypes array that shares memory with the Webots simulation. This means that initially
           reading the targets is very fast, but this value is valid only for the current timestep.
           If you'll want to refer to this value later, copy the information, e.g. with radar.copy().
           Note: for most purposes, the radar device itself can be used as a surrogate for its own .value.
           E.g. you can use 'len(radar)`, `radar[0]` and `for target in radar` without needing `.value`."""
        n = wb.wb_radar_get_number_of_targets(self.tag)
        if n == 0: return []
        ptr = wb.wb_radar_get_targets(self.tag)
        if not ptr: return []
        return ctypes.cast(ptr, POINTER(Radar.Target * n))[0]

    def copy(self) -> Sequence[Target]:
        """Returns a copy of this Radar's array of current targets.  Unlike radar.value, this copy will remain
           valid after the current timestep."""
        return type(self.value).from_buffer_copy(self.value)

    @property
    def array(self) -> NumpyArrayType:
        """Returns the current list of targets as a Numpy array. Rows correspond to targets.
           Each row consists of 4 floats: the target's distance, speed, received_power, and azimuth.
           The resulting array will share memory with Webots' internal version of the data, making creating this
           extremely fast, but this array will cease to be valid at the end of this timestep!
           If you will want later access to this information, you should make a copy, e.g. with Numpy's ndarray.copy().
           (Extremely fast and provides convenience of Numpy, but requires that Numpy be available, and is valid only
           for this timestep.)"""
        import numpy as np
        return np.array(self.value, copy=False)

    wb.wb_radar_get_min_range.restype = c_double
    @property
    def min_range(self: 'Radar') -> float:
        """Returns the current min_range of this Radar."""
        return wb.wb_radar_get_min_range(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinRange(self: 'Radar') -> float:
        """DEPRECATED: Radar.getMinRange() is deprecated. Please use radar.min_range instead."""
        return wb.wb_radar_get_min_range(self.tag)

    wb.wb_radar_get_max_range.restype = c_double
    @property
    def max_range(self) -> float:
        """Returns the current max_range of this Radar."""
        return wb.wb_radar_get_max_range(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxRange(self) -> float:
        """DEPRECATED: Radar.getMaxRange() is deprecated. Please use radar.max_range instead."""
        return wb.wb_radar_get_max_range(self.tag)

    wb.wb_radar_get_horizontal_fov.restype = wb.wb_radar_get_vertical_fov.restype = c_double
    @property
    def FOV(self) -> Vector:
        """Returns a 2D vector whose .x indicates the horizontal field of view, and whose .y indicates vertical."""
        return Vector(wb.wb_radar_get_horizontal_fov(self.tag), wb.wb_radar_get_vertical_fov(self.tag))
    @use_docstring_as_deprecation_warning
    def getHorizontalFov(self):
        """DEPRECATED: Radar.getHorizontalFov() is deprecated. Please use radar.FOV.x instead."""
        return wb.wb_radar_get_horizontal_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def getVerticalFov(self):
        """DEPRECATED: Radar.getVerticalFov() is deprecated. Please use radar.FOV.y instead."""
        return wb.wb_radar_get_vertical_fov(self.tag)

    def __len__(self) -> int:
        """Returns the number of visible Radar.Targets without taking the time to retrieve them."""
        # Overwrites SurrogateValue.__len__ to enable peeking at len() without having to load the whole list
        # TODO Could perhaps check whether the cached .value is uptodate, and if so use its length?
        return wb.wb_radar_get_number_of_targets(self.tag)
    @use_docstring_as_deprecation_warning
    def getNumberOfTargets(self):
        """DEPRECATED: radar.getNumberOfTargets() is deprecated.  Please use len(radar) instead."""
        return wb.wb_radar_get_number_of_targets(self.tag)

    wb.wb_radar_get_target.restype = Target
    @use_docstring_as_deprecation_warning
    def getTarget(self, index:int) -> Target:
        """DEPRECATED: Radar.getTarget(index) is deprecated. Use radar[index] instead."""
        return wb.wb_radar_get_target(self.tag, index)
        # TODO double-check

    @use_docstring_as_deprecation_warning
    def getTargets(self) -> List[Target]:
        """DEPRECATED. Radar.getTargets() is deprecated. An equivalen is list(radar), though for most purposes,
           radar will serve as a surrogate of its own value, with no need to create a separate list."""
        return list(self.value)



# === Image-Sensing Devices (Camera, Lidar, RangeFinder) ===

from typing import TypeVar, Generic, Type
ImagePixelType = TypeVar('ImagePixelType', ColorBGRA, float, 'Lidar.Cloud.Point')

class ImageContainer(Generic[ImagePixelType]):
    """A container for a 2D image, stored as its .value, which consists of rows of items of ImagePixelType, which will
       be `ColorBGRA` for Cameras, `float` for RangeFinder/Lidar images, and Lidar.Cloud.Point for Lidar point-clouds.
       These devices themselves are ImageContainers, as are copies of them (and copies of copies), produced by .copy()
       The ImageContainer class provides a standard container interface, and conversion to other datatypes.
       ACCESSING PIXELS.
       `image.value` is a ctypes array with a container interface similar to a Python nested list. BEWARE: the .value
       of a Camera/RangeFinder/Lidar its .value will share memory with the Webots simulation, making this value very
       fast to access, but also making it valid only during this timestep! Use `image.copy()` if you'll want it later.
       `image[y]` returns row y of the image, a 1D ctypes array of instances of its ImagePixelType
       `image[y,x]` and `image[y][x]` return the pixel in row y, column x. Note y is first!
       Image slicing works with the same limitations as Python nested lists.  E.g. `image[0:2, :]` or `image[0, 0:2]`
       are fine, but `image[0:2, 0:2] is not.  If you have Numpy available, `image.array` has more slicing options.
       ITERATION.
       `image.height` and `len(image)` return the number of rows; `image.width` returns the number of columns.
       `for row in image:` and `for row in image.by_row` each yield successive rows.
       `for pixel in image.by_pixel:` yields successive pixel values, scanning across each row top to bottom.
       `for x,y,pixel in image.enumerated:` yields pixel values along with their coordinates in the image.
       CONVERSION TO OTHER DATATYPES.
       `image.copy()` creates an ImageContainer with a copy of image.value that will remain valid after this timestep.
       `image.array` returns a view of the current readings as a Numpy array which shares memory with the simulation so is
         extremely fast to create, but is valid only for the current timestep (extremely fast, but requires numpy)
       `image.buffer` copies the current value to a python bytes object (fairly fast, but very inconvenient to use)
       `image.list` returns the current readings all together in one long 1D python list (slow)
       `image.nested_list` returns a 2D list of row-like lists of Lidar readings (slow)"""

    # These will be dynamically provided by Device properties, or set in copying
    value: Sequence[Sequence[ImagePixelType]]
    width: int
    height: int

    def __len__(self) -> int:
        """The number of rows in this image."""
        return self.height

    @property
    def size(self) -> Vector:
        """Returns a vector indicating this image's [width, height] in pixels."""
        return Vector(self.width, self.height)

    @property
    def shape(self) -> Tuple[int, int]:
        """Returns a tuple indicating this image's (height, width) in pixels, which corresponds to the .shape
           the corresponding numpy array would have. Note that shape orders y before x."""
        return self.height, self.width

    # Two-dimensional __getitem__ can return a variety of value types; we start by type-declaring them
    @overload  # image[slice] returns a sequence of rows
    def __getitem__(self: 'ImageContainer[ImagePixelType]', item: slice) -> Sequence[Sequence[ImagePixelType]]: pass
    @overload  # image[slice, :] also returns a sequence of rows
    def __getitem__(self: 'ImageContainer[ImagePixelType]', item: Tuple[slice,None]) -> Sequence[Sequence[ImagePixelType]]: pass
    @overload  # image[y] returns a single row
    def __getitem__(self: 'ImageContainer[ImagePixelType]', item: int) -> Sequence[ImagePixelType]: pass
    @overload # image[y, slice] returns a portion of a row
    def __getitem__(self: 'ImageContainer[ImagePixelType]', item: Tuple[int,slice]) -> Sequence[ImagePixelType]: pass
    @overload  # image[y, x] returns a single pixel
    def __getitem__(self: 'ImageContainer[ImagePixelType]', item: Tuple[int,int]) -> ImagePixelType: pass
    # And now the actual definition
    def __getitem__(self, item):
        if isinstance(item, tuple):
            y, x = item
            if x is None: return self.value[y]  # container[y,:]
            return self.value[y][x]
        return self.value[item]

    # --- ImageContainer iteration ---

    def __iter__(self:'ImageContainer[ImagePixelType]') -> Iterable[Sequence[ImagePixelType]]:
        """`for row in imagecontainer` yields successive rows from the image.
           Equivalent to `for row in imagecontainer.by_row`."""
        return iter(self.value)

    @property
    def by_row(self:'ImageContainer[ImagePixelType]') -> Iterable[Sequence[ImagePixelType]]:
        """`for row in imagecontainer.by_row:` yields successive rows from the image."""
        return iter(self.value)

    @property
    def by_pixel(self: 'ImageContainer[ImagePixelType]') -> Iterable[ImagePixelType]:
        """`for value in imagecontainer.by_pixel:` yields successive pixel values, scanning each row, from top down."""
        return itertools.chain.from_iterable(self.value)

    @property
    def enumerated(self: 'ImageContainer[ImagePixelType]') -> Iterable[Tuple[int, int, ImagePixelType]]:
        """`for x,y,value in imagecontainer.enumerated:` scans across each row, from the top down."""
        return ((x, y, value) for y, row in enumerate(self.value) for x, value in enumerate(row))

    # --- ImageContainer conversion to other datatypes ---

    @property
    def array(self) -> NumpyArrayType:
        """Returns the current image as a Numpy array. If performed from a Camera/RangeFinder/Lidar device itself,
           then the resulting array will share memory with Webots' internal version of the image, making creating this
           extremely fast regardless of image size, but this array will cease to be valid at the end of this timestep!
           If you will want later access to this information, you should make a copy, e.g. with Numpy's ndarray.copy().
           (Extremely fast and provides convenience of Numpy, but requires that Numpy be available, and is valid only
           for this timestep.)"""
        # TODO Numpy seems to insist on converting RangeFinder to 32-bit floats rather than re-using the array
        #  of 16-bit floats. Strange because the same approach works fine on other datatypes like c_ubyte
        import numpy as np
        return np.array(self.value, copy=False)
        # return np.ctypeslib.as_array(wb.wb_range_finder_get_range_image(self.tag), (self.height, self.width))

    def copy(self:'ImageContainer[ImagePixelType]') -> 'ImageContainer[ImagePixelType]':
        """Returns a copy of this image, with the same ImageContainer interface that the original device provided."""
        clone = ImageContainer()
        clone.width, clone.height = self.width, self.height
        clone.value = (type(self.value).from_buffer_copy(self.value))
        return clone

    # TODO probably better to just have people call bytes(img) instead?
    @property
    def bytes(self) -> bytes:
        """Returns a python bytes object containing all image values.
           (Fairly fast but inconvenient to use. It's faster to create NumPy arrays with .array instead.)"""
        # return ctypes.cast(wb.wb_range_finder_get_range_image(self.tag), c_char_p * len(self))
        return bytes(self.value)

    @property
    def list(self: 'ImageContainer[ImagePixelType]') -> List[ImagePixelType]:
        """Returns a 1D python list of values, scanning across each row from top down.
           (Comparatively slow and typically less useful than the ImageContainer or a .copy() thereof.)"""
        return list(value for row in self.value for value in row)

    @property
    def nested_list(self: 'ImageContainer[ImagePixelType]') -> List[List[ImagePixelType]]:
        """Returns a python list of row-like lists of pixel values, from the top row to the bottom.
           (Comparatively slow, and not much different from the original ImageContainer or a .copy() thereof.)"""
        return [list(row) for row in self.value]


class Camera(ImageContainer[ColorBGRA], Device, Sensor): # TODO should be ImageContainer[ColorBGRA] but metaclass conflict
    """A Python Camera object is used to control a camera device node in the simulation, which generates 2D color images
       as seen from its perspective.  Each pixel is a `ColorBGRA` object which represents each color components as an
       int ranging 0-255, stored in a ctypes array (typically one that shares memory with the underlying simulation
       so is valid only this timestep), but with standard Vector-like helper methods, including vector arithmetic and
       accessing color components as .b/.blue, .g/.green, .r/.red, and .a/.alpha. Note that these are BGRA, not RGB!
       TODO vector arithmetic between ordinary (RGB) Color and ColorRGBA currently will muddle red and blue channels!
       `camera = robot.Camera("devicename")` creates a Camera to control the rangefinder with that name.
       IMAGECONTAINER INTERFACE (inherited from ImageContainer superclass).
       `camera.width` and `camera.height` are the dimensions of the Camera image, in pixels.
       `camera[y,x]` returns the current reading of the Camera at row y, column x (fast). Note y index comes first!
         At most one of these indices can be a slice, e.g., 0:2.
       `camera[y]` returns row y of the the current Camera reading
       `for row in camera` and `for row in camera.by_row` iterate through rows, from top to bottom.
       `for pixel in camera.by_pixel` and `for x, y, pixel in camera.enumerated` go through all pixels, row after row.
       `camera.value` returns a ctypes array containing the Camera's current reading. This array shares memory
         with Webots, so is valid only for the duration of this timestep. If you'll want access to this information
         later, you'll need to copy it. This array is a sequence of rows, where each row is a sequence of floats.
         This allows indexing by rows, then columns [y][x], but does not allow comma multi-indexing like [y,x].
       `camera.array` returns a view of the current readings as a Numpy array which shares memory with the simulation so is
         extremely fast to create, but is valid only for the current timestep (extremely fast, but requires numpy)
       `camera.copy()` returns a copy of the current readings that (unlike .value and .array) does not share memory with
         Webots' internal representation of the image, so is safe to use on future timesteps. This copy retains the
         same ImageContainer-interface as the Camera itself. (fast, recommended way to copy outside numpy)
       `camera.buffer` copies the current value to a python bytes object (fairly fast, but very inconvenient to use)
       `camera.list` returns the current readings all together in one long 1D python list (slow)
       `camera.nested_list` returns a 2D list of row-like lists of Camera readings (slow)
       `camera.save_image(filename, quality)` saves the current image to file; quality specifies jpeg quality (0-100)
       CAMERA ATTRIBUTES.
       `camera.near` is the distance to the camera's near clipping plane (i.e. the closest it can see).
       `camera.fov` reads or adjusts the camera's field of view in radians, between `.min_fov` and `.max_fov`
       `camera.focal_distance` reads or adjusts focal distance between `.min_focal_distance` and `.max_focal_distance`;
       this would be the distance to focused-upon objects, but requires the camera have a focus node in the simulation.
       `camera.focal_length` is the length between the camera's sensor and the optical center of its lens.
       `camera.exposure` and `camera.exposure = x` read and adjust the camera's exposure, in joules per square meter.
       `camera.min_range` and `camera.max_range` are bounds on the range of detectable objects.
       CAMERA RECOGNITION OBJECTS.
       TODO These are not implemented yet!
       """

    #--- Camera image dimensions (accessed by ImageContainer superclass) ---

    @property
    def width(self) -> int:
        """Returns the current width of this Camera's image in pixels."""
        return wb.wb_camera_get_width(self.tag)
    @use_docstring_as_deprecation_warning
    def getWidth(self) -> int:
        """DEPRECATED: Camera.getWidth() is deprecated. Please use camera.width or camera.size.x."""
        return wb.wb_camera_get_width(self.tag)

    @property
    def height(self) -> int:
        """Returns the current height of this Camera's image, in pixels."""
        return wb.wb_camera_get_height(self.tag)
    @use_docstring_as_deprecation_warning
    def getHeight(self) -> int:
        """DEPRECATED: Camera.getHeight() is deprecated. Please use camera.height or camera.size.y."""
        return wb.wb_camera_get_height(self.tag)

    # --- Other Camera attributes ---

    wb.wb_camera_get_fov.restype = c_double
    @property
    def fov(self) -> float:
        """Returns or adjusts the current field-of-view of this Camera, in radians."""
        return wb.wb_camera_get_fov(self.tag)
    @fov.setter
    def fov(self, new_fov: float):
        wb.wb_camera_set_fov(self.tag, c_double(new_fov))
    @use_docstring_as_deprecation_warning
    def getFov(self) -> float:
        """DEPRECATED: Camera.getFov() is deprecated. Please use camera.fov instead."""
        return wb.wb_camera_get_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def setFov(self, fov: float):
        """DEPRECATED: Camera.setFov(f) is deprecated. Please use camera.fov = f instead."""
        wb.wb_camera_set_fov(self.tag, c_double(fov))

    wb.wb_camera_get_max_fov.restype = c_double
    @property
    def max_fov(self) -> float:
        """Returns the maximum field of view of this Camera, in radians."""
        return wb.wb_camera_get_max_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxFov(self) -> float:
        """DEPRECATED: Camera.getMaxFov() is deprecated. Please use camera.max_fov instead."""
        return wb.wb_camera_get_max_fov(self.tag)

    wb.wb_camera_get_min_fov.restype = c_double
    @property
    def min_fov(self) -> float:
        """Returns the minimum field of view of this Camera, in radians."""
        return wb.wb_camera_get_min_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinFov(self) -> float:
        """DEPRECATED: Camera.getMinFov() is deprecated. Please use camera.min_fov instead."""
        return wb.wb_camera_get_min_fov(self.tag)

    wb.wb_camera_get_exposure.restype = c_double
    @property
    def exposure(self) -> float:
        """Returns the current exposure of this Camera."""
        return wb.wb_camera_get_exposure(self.tag)
    @exposure.setter
    def exposure(self, new_exposure: float):
        wb.wb_camera_set_exposure(self.tag, c_double(new_exposure))
    @use_docstring_as_deprecation_warning
    def getExposure(self) -> float:
        """DEPRECATED: Camera.getExposure() is deprecated. Please use camera.exposure instead."""
        return wb.wb_camera_get_exposure(self.tag)
    def setExposure(self, exposure: float):
        """DEPRECATED: Camera.setExposure(x) is deprecated. Please use camera.exposure = x instead."""
        wb.wb_camera_set_exposure(self.tag, c_double(exposure))

    wb.wb_camera_get_focal_length.restype = c_double
    @property
    def focal_length(self) -> float:
        """Returns the current focal_length of this Camera. XXX"""
        return wb.wb_camera_get_focal_length(self.tag)
    @use_docstring_as_deprecation_warning
    def getFocalLength(self) -> float:
        """DEPRECATED: Camera.getFocalLength() is deprecated. Please use camera.focal_length instead."""
        return wb.wb_camera_get_focal_length(self.tag)

    wb.wb_camera_get_max_focal_distance.restype = c_double
    @property
    def max_focal_distance(self) -> float:
        """Returns the current max_focal_distance of this Camera. XXX"""
        return wb.wb_camera_get_max_focal_distance(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxFocalDistance(self) -> float:
        """DEPRECATED: Camera.getMaxFocalDistance() is deprecated. Please use camera.max_focal_distance instead."""
        return wb.wb_camera_get_max_focal_distance(self.tag)

    wb.wb_camera_get_min_focal_distance.restype = c_double
    @property
    def min_focal_distance(self) -> float:
        """Returns the current min_focal_distance of this Camera. XXX"""
        return wb.wb_camera_get_min_focal_distance(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinFocalDistance(self) -> float:
        """DEPRECATED: Camera.getMinFocalDistance() is deprecated. Please use camera.min_focal_distance instead."""
        return wb.wb_camera_get_min_focal_distance(self.tag)

    wb.wb_camera_get_focal_distance.restype = c_double
    @property
    def focal_distance(self) -> float:
        """Returns or adjusts the focal distance of this Camera."""
        return wb.wb_camera_get_focal_distance(self.tag)
    @focal_distance.setter
    def focal_distance(self, focalDistance: float):
        wb.wb_camera_set_focal_distance(self.tag, c_double(focalDistance))
    @use_docstring_as_deprecation_warning
    def getFocalDistance(self) -> float:
        """DEPRECATED: Camera.getFocalDistance() is deprecated. Please use camera.focal_distance instead."""
        return wb.wb_camera_get_focal_distance(self.tag)
    @use_docstring_as_deprecation_warning
    def setFocalDistance(self, focalDistance: float):
        """DEPRECATED: Camera.setFocalDistance(d) is deprecated. Please use camera.focal_distance=d instead."""
        wb.wb_camera_set_focal_distance(self.tag, c_double(focalDistance))

    wb.wb_camera_get_near.restype = c_double
    @property
    def near(self) -> float:
        """Returns the current near of this Camera. XXX"""
        return wb.wb_camera_get_near(self.tag)
    @use_docstring_as_deprecation_warning
    def getNear(self) -> float:
        """DEPRECATED: Camera.getNear() is deprecated. Please use camera.near instead."""
        return wb.wb_camera_get_near(self.tag)

    # --- Camera getting images ---

    wb.wb_camera_get_image.restype = c_ubyte_p
    @timed_cached_property
    def value(self) -> Sequence[Sequence[ColorBGRA]]:
        """Returns the current image of this Camera as a ctypes array of rows of ColorBGRA pixels.
           Each pixel is a ColorBGRA vector that supports vector arithmetic that themselves
           contain 4 components (ordered BGRA)."""
        width, height = wb.wb_camera_get_width(self.tag), wb.wb_camera_get_height(self.tag)
        c_arraytype = (ColorBGRA * width) * height
        ptr = wb.wb_camera_get_image(self.tag)
        return ctypes.cast(ptr, POINTER(c_arraytype))[0]

    @use_docstring_as_deprecation_warning
    def getImage(self) -> bytes:
        """DEPRECATED: Camera.getImage() is deprecated. A pure equivalent is camera.bytes,
           but other options like camera.array, camera.value, or simply camera itself are likely better."""
        return wb.wb_camera_get_image(self.tag)

    def getImageArray(self) -> List[List[ColorBGRA]]:
        """DEPRECATED: Camera.getImageArray() is deprecated; camera.nested_list is a very close equivalent, or
           for most purposes the camera itself will work like this nested list, e.g. in camera[y][x][c]."""
        return self.nested_list

    # TODO these seem to require that the width be explicitly given, probably better to make another interface
    @staticmethod
    def imageGetRed(*args):
        return wb.wb_camera_image_get_redXXX(*args)

    @staticmethod
    def imageGetGreen(*args):
        return wb.wb_camera_image_get_greenXXX(*args)

    @staticmethod
    def imageGetBlue(*args):
        return wb.wb_camera_image_get_blueXXX(*args)

    @staticmethod
    def imageGetGray(*args):
        return wb.wb_camera_image_get_grayXXX(*args)

    @staticmethod
    def imageGetGrey(*args):
        return wb.wb_camera_image_get_greyXXX(*args)


    def saveImage(self, filename:str, quality: int = 90) -> int:
        """Saves the camera's current image to the given filename.  If the filname ends '.jpg' or '.jpeg' then
           `quality` will be used to determine jpeg quality, 0-100 (default 90).
           Returns a truth-like value (-1) if the save fails, and a false-like value (0) if it succeeds."""
        return wb.wb_camera_save_image(self.tag, filename.encode(), quality)

    # --- camera recognition ---

    # TODO  NOT YET IMPLEMENTED

    class RecognitionObject:
        def __init__(self):
            pass

        # id = property(wb.wb_camera_recognition_object_id_getXXX, wb.wb_camera_recognition_object_id_setXXX)
        # position = property(wb.wb_camera_recognition_object_position_getXXX,
        #                     wb.wb_camera_recognition_object_position_setXXX)
        # orientation = property(wb.wb_camera_recognition_object_orientation_getXXX,
        #                        wb.wb_camera_recognition_object_orientation_setXXX)
        # size = property(wb.wb_camera_recognition_object_size_getXXX, wb.wb_camera_recognition_object_size_setXXX)
        # position_on_image = property(wb.wb_camera_recognition_object_position_on_image_getXXX,
        #                              wb.wb_camera_recognition_object_position_on_image_setXXX)
        # size_on_image = property(wb.wb_camera_recognition_object_size_on_image_getXXX,
        #                          wb.wb_camera_recognition_object_size_on_image_setXXX)
        # number_of_colors = property(wb.wb_camera_recognition_object_number_of_colors_getXXX,
        #                             wb.wb_camera_recognition_object_number_of_colors_setXXX)
        # colors = property(wb.wb_camera_recognition_object_colors_getXXX, wb.wb_camera_recognition_object_colors_setXXX)
        # model = property(wb.wb_camera_recognition_object_model_getXXX, wb.wb_camera_recognition_object_model_setXXX)

        def get_position(self):
            return wb.wb_camera_recognition_object_get_positionXXX(self)

        def get_orientation(self):
            return wb.wb_camera_recognition_object_get_orientationXXX(self)

        def get_size(self):
            return wb.wb_camera_recognition_object_get_sizeXXX(self)

        def get_position_on_image(self):
            return wb.wb_camera_recognition_object_get_position_on_imageXXX(self)

        def get_size_on_image(self):
            return wb.wb_camera_recognition_object_get_size_on_imageXXX(self)

        def get_colors(self):
            return wb.wb_camera_recognition_object_get_colorsXXX(self)

        def get_id(self):
            return wb.wb_camera_recognition_object_get_idXXX(self)

        def get_number_of_colors(self):
            return wb.wb_camera_recognition_object_get_number_of_colorsXXX(self)

        def get_model(self):
            return wb.wb_camera_recognition_object_get_modelXXX(self)


    def hasRecognition(self) -> bool:
        return wb.wb_camera_has_recognition(self.tag)

    # TODO change recognition to an enable-able pseudo-device



    def recognitionEnable(self, samplingPeriod):
        return wb.wb_camera_recognition_enable(self.tag, samplingPeriod)

    def recognitionDisable(self):
        return wb.wb_camera_recognition_disable(self.tag)

    def getRecognitionSamplingPeriod(self):
        return wb.wb_camera_get_recognition_sampling_periodXXX(self.tag)

    # wb.wb_camera_get_recognition_sampling_periodXXX.restype = XXX
    # @property
    # def recognition_sampling_period(self):
    #     """Returns the current recognition_sampling_period of this Camera. XXX"""
    #     return wb.wb_camera_get_recognition_sampling_periodXXX(self.tag)
    # @use_docstring_as_deprecation_warning
    # def getRecognitionSamplingPeriod(self):
    #     """DEPRECATED: Camera.getRecognitionSamplingPeriod() is deprecated. Please use camera.recognition_sampling_period instead."""
    #     return wb.wb_camera_get_recognition_sampling_periodXXX(self.tag)
    # # -----------------------------------------------------
    # wb.wb_camera_get_recognition_number_of_objectsXXX.restype = XXX
    # @property
    # def recognition_number_of_objects(self) -> int:
    #     """Returns the current recognition_number_of_objects of this Camera. XXX"""
    #     return wb.wb_camera_get_recognition_number_of_objectsXXX(self.tag)
    # @use_docstring_as_deprecation_warning
    # def getRecognitionNumberOfObjects(self) -> int:
    #     """DEPRECATED: Camera.getRecognitionNumberOfObjects() is deprecated. Please use camera.recognition_number_of_objects instead."""
    #     return wb.wb_camera_get_recognition_number_of_objectsXXX(self.tag)
    # # -----------------------------------------------------
    # wb.wb_camera_get_recognition_objectsXXX.restype = XXX
    # @property
    # def recognition_objects(self):
    #     """Returns the current recognition_objects of this Camera. XXX"""
    #     return wb.wb_camera_get_recognition_objectsXXX(self.tag)
    # @use_docstring_as_deprecation_warning
    # def getRecognitionObjects(self):
    #     """DEPRECATED: Camera.getRecognitionObjects() is deprecated. Please use camera.recognition_objects instead."""
    #     return wb.wb_camera_get_recognition_objectsXXX(self.tag)
    # # -----------------------------------------------------
    # wb.wb_camera_get_recognition_segmentation_imageXXX.restype = XXX
    # @property
    # def recognition_segmentation_image(self):
    #     """Returns the current recognition_segmentation_image of this Camera. XXX"""
    #     return wb.wb_camera_get_recognition_segmentation_imageXXX(self.tag)
    # @use_docstring_as_deprecation_warning
    # def getRecognitionSegmentationImage(self):
    #     """DEPRECATED: Camera.getRecognitionSegmentationImage() is deprecated. Please use camera.recognition_segmentation_image instead."""
    #     return wb.wb_camera_get_recognition_segmentation_imageXXX(self.tag)

    # wb.wb_camera_get_image_arrayXXX.restype = XXX
    # @property
    # def image_array(self):
    #     """Returns the current image_array of this Camera. XXX"""
    #     return wb.wb_camera_get_image_arrayXXX(self.tag)
    # @use_docstring_as_deprecation_warning
    # def getImageArray(self):
    #     """DEPRECATED: Camera.getImageArray() is deprecated. Please use camera.image_array instead."""
    #     return wb.wb_camera_get_image_arrayXXX(self.tag)
    # # -----------------------------------------------------
    # wb.wb_camera_get_recognition_objectXXX.restype = XXX
    # @property
    # def recognition_object(self, index):
    #     """Returns the current recognition_object of this Camera. XXX"""
    #     return wb.wb_camera_get_recognition_objectXXX(self.tag, index)
    # @use_docstring_as_deprecation_warning
    # def getRecognitionObject(self, index):
    #     """DEPRECATED: Camera.getRecognitionObject() is deprecated. Please use camera.recognition_object instead."""
    #     return wb.wb_camera_get_recognition_objectXXX(self.tag, index)
    # # -----------------------------------------------------
    # @property
    # def recognition_objects(self):
    #     """Returns the current recognition_objects of this Camera. XXX"""
    #     ret = []
    #     for i in range(self.getRecognitionNumberOfObjects()):
    #         ret.append(self.getRecognitionObject(i))
    #     return ret
    # @use_docstring_as_deprecation_warning
    # def getRecognitionObjects(self):
    #     """DEPRECATED: Camera.getRecognitionObjects() is deprecated. Please use camera.recognition_objects instead."""
    #     ret = []
    #     for i in range(self.getRecognitionNumberOfObjects()):
    #         ret.append(self.getRecognitionObject(i))
    #     return ret
    # # -----------------------------------------------------
    # wb.wb_camera_get_recognition_segmentation_image_arrayXXX.restype = XXX
    # @property
    # def recognition_segmentation_image_array(self):
    #     """Returns the current recognition_segmentation_image_array of this Camera. XXX"""
    #     return wb.wb_camera_get_recognition_segmentation_image_arrayXXX(self.tag)
    # @use_docstring_as_deprecation_warning
    # def getRecognitionSegmentationImageArray(self):
    #     """DEPRECATED: Camera.getRecognitionSegmentationImageArray() is deprecated. Please use camera.recognition_segmentation_image_array instead."""
    #     return wb.wb_camera_get_recognition_segmentation_image_arrayXXX(self.tag)

    def getRecognitionNumberOfObjects(self) -> int:
        return wb.wb_camera_get_recognition_number_of_objectsXXX(self.tag)

    def getRecognitionObjects(self):
        return wb.wb_camera_get_recognition_objectsXXX(self.tag)

    def hasRecognitionSegmentation(self):
        return wb.wb_camera_has_recognition_segmentationXXX(self.tag)

    def enableRecognitionSegmentation(self):
        return wb.wb_camera_enable_recognition_segmentationXXX(self.tag)

    def disableRecognitionSegmentation(self):
        return wb.wb_camera_disable_recognition_segmentationXXX(self.tag)

    def isRecognitionSegmentationEnabled(self):
        return wb.wb_camera_is_recognition_segmentation_enabledXXX(self.tag)

    def getRecognitionSegmentationImage(self):
        return wb.wb_camera_get_recognition_segmentation_imageXXX(self.tag)

    def saveRecognitionSegmentationImage(self, filename, quality):
        return wb.wb_camera_save_recognition_segmentation_imageXXX(self.tag, filename, quality)

    def getRecognitionObject(self, index):
        return wb.wb_camera_get_recognition_objectXXX(self.tag, index)

    def getRecognitionObjects(self):
        ret = []
        for i in range(self.getRecognitionNumberOfObjects()):
            ret.append(self.getRecognitionObject(i))
        return ret

    def getRecognitionSegmentationImageArray(self):
        return wb.wb_camera_get_recognition_segmentation_image_arrayXXX(self.tag)

CameraRecognitionObject = Camera.RecognitionObject  # for backwards compatibility



class Lidar(ImageContainer[float], Device, Sensor):
    """A Python Lidar object is used to control a lidar device node in the simulation, which itself
       detects distances to nearby objects at an array of similar angles, and may rotate.
       `lidar = robot.Lidar("devicename")` creates a Lidar to control the rangefinder with that name.
       IMAGECONTAINER INTERFACE (mostly inherited from ImageContainer superclass).
       `lidar.width` and `lidar.height` are the dimensions of the Lidar image, in pixels.
       `lidar[y,x]` returns the current reading of the Lidar at row y, column x (fast). Note y index comes first!
         At most one of these indices can be a slice, e.g., 0:2.
       `lidar[y]` returns row y of the the current Lidar reading
       `for row in lidar` and `for row in lidar.by_row` iterate through rows, from top to bottom.
       `for pixel in lidar.by_pixel` and `for x, y, pixel in lidar.enumerated` go through all pixels, row after row.
       `lidar.value` returns a ctypes array containing the Lidar's current reading. This array shares memory
         with Webots, so is valid only for the duration of this timestep. If you'll want access to this information
         later, you'll need to copy it. This array is a sequence of rows, where each row is a sequence of floats.
         This allows indexing by rows, then columns [y][x], but does not allow comma multi-indexing like [y,x].
       `lidar.array` returns a view of the current readings as a Numpy array which shares memory with the simulation so is
         extremely fast to create, but is valid only for the current timestep (extremely fast, but requires numpy)
       `lidar.copy()` returns a copy of the current readings that (unlike .value and .array) does not share memory with
         Webots' internal representation of the image, so is safe to use on future timesteps. This copy retains the
         same ImageContainer-interface as the Lidar itself. (fast, recommended way to copy outside numpy)
       `lidar.buffer` copies the current value to a python bytes object (fairly fast, but very inconvenient to use)
       `lidar.list` returns the current readings all together in one long 1D python list (slow)
       `lidar.nested_list` returns a 2D list of row-like lists of Lidar readings (slow)
       `lidar.save_image(filename, quality)` saves the current image to file, with quality specify jpeg quality (0-100)
       LIDAR ATTRIBUTES.
       `lidar.frequency` reads or adjusts the rotation frequency between `.min_frequency` and `.max_frequency`
       `lidar.min_range` and `lidar.max_range` are bounds on the range of detectable objects.
       `lidar.fov` and `lidar.vertical_fov` are the horizontal and vertical fields of view, in radians.
       LIDAR POINTCLOUDS.
       TODO These are not implemented yet!
       """

    #--- Lidar image dimensions (accessed by ImageContainer superclass) ---

    @property
    def width(self) -> int:
        """Returns the current horizontal_resolution of this Lidar (the width of its image, in pixels)."""
        return wb.wb_lidar_get_horizontal_resolution(self.tag)
    @use_docstring_as_deprecation_warning
    def getHorizontalResolution(self) -> int:
        """DEPRECATED: Lidar.getHorizontalResolution() is deprecated. Please use lidar.width instead."""
        return wb.wb_lidar_get_horizontal_resolution(self.tag)

    @property
    def height(self) -> int:
        """Returns the current number of layers of this Lidar (the height of its image, in pixels)."""
        return wb.wb_lidar_get_number_of_layers(self.tag)
    @use_docstring_as_deprecation_warning
    def getNumberOfLayers(self) -> int:
        """DEPRECATED: Lidar.getNumberOfLayers() is deprecated. Please use lidar.number_of_layers instead."""
        return wb.wb_lidar_get_number_of_layers(self.tag)

    # ---Other Lidar characteristics---

    wb.wb_lidar_get_min_frequency.restype = c_double
    @property
    def min_frequency(self) -> float:
        """Returns the minimum frequency of this Lidar (for rotating Lidar)."""
        return wb.wb_lidar_get_min_frequency(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinFrequency(self) -> float:
        """DEPRECATED: Lidar.getMinFrequency() is deprecated. Please use lidar.min_frequency instead."""
        return wb.wb_lidar_get_min_frequency(self.tag)

    wb.wb_lidar_get_max_frequency.restype = c_double
    @property
    def max_frequency(self) -> float:
        """Returns the maximum frequency of this Lidar (for rotating Lidar)."""
        return wb.wb_lidar_get_max_frequency(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxFrequency(self) -> float:
        """DEPRECATED: Lidar.getMaxFrequency() is deprecated. Please use lidar.max_frequency instead."""
        return wb.wb_lidar_get_max_frequency(self.tag)

    wb.wb_lidar_get_frequency.restype = c_double
    @property
    def frequency(self) -> float:
        """Returns or adjusts the current frequency of this Lidar (for rotating Lidar)."""
        return wb.wb_lidar_get_frequency(self.tag)
    @frequency.setter
    def frequency(self, frequency: float):
        wb.wb_lidar_set_frequency(self.tag, c_double(frequency))
    @use_docstring_as_deprecation_warning
    def getFrequency(self) -> float:
        """DEPRECATED: Lidar.getFrequency() is deprecated. Please use lidar.frequency instead."""
        return wb.wb_lidar_get_frequency(self.tag)
    def setFrequency(self, frequency: float):
        """DEPRECATED: Lidar.setFrequency(f) is deprecated. Please use lidar.frequency = f instead."""
        wb.wb_lidar_set_frequency(self.tag, c_double(frequency))

    wb.wb_lidar_get_fov.restype = c_double
    @property
    def fov(self) -> float:  # this is just horizontal
        """Returns the current horizontal field of view of this Lidar, in radians."""
        return wb.wb_lidar_get_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def getFov(self) -> float:  # this is just horizontal
        """DEPRECATED: Lidar.getFov() is deprecated. Please use lidar.fov instead."""
        return wb.wb_lidar_get_fov(self.tag)

    wb.wb_lidar_get_vertical_fov.restype = c_double
    @property
    def vertical_fov(self) -> float:
        """Returns the current vertical_fov of this Lidar, in radians."""
        return wb.wb_lidar_get_vertical_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def getVerticalFov(self) -> float:
        """DEPRECATED: Lidar.getVerticalFov() is deprecated. Please use lidar.vertical_fov instead."""
        return wb.wb_lidar_get_vertical_fov(self.tag)

    wb.wb_lidar_get_min_range.restype = c_double
    @property
    def min_range(self) -> float:
        """Returns the current minimum range of this Lidar."""
        return wb.wb_lidar_get_min_range(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinRange(self) -> float:
        """DEPRECATED: Lidar.getMinRange() is deprecated. Please use lidar.min_range instead."""
        return wb.wb_lidar_get_min_range(self.tag)

    wb.wb_lidar_get_max_range.restype = c_double
    @property
    def max_range(self) -> float:
        """Returns the current maximum range of this Lidar."""
        return wb.wb_lidar_get_max_range(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxRange(self) -> float:
        """DEPRECATED: Lidar.getMaxRange() is deprecated. Please use lidar.max_range instead."""
        return wb.wb_lidar_get_max_range(self.tag)

    # --- Lidar range images ---

    wb.wb_lidar_get_range_image.restype = c_float_p
    @timed_cached_property
    def value(self) -> Sequence[Sequence[float]]:
        """The current range image of this lidar, as a 2D ctypes array of floats, organized by row, so lidar.value[y]
           returns a row, and lidar.value[y][x] returns a pixel.  NOTE: this array shares memory with the webots
           simulation, making it fast to access, but it becomes invalid at end of timestep! If you want the information
           for longer, you'll need to copy it, e.g. with lidar.copy(). For most purposes, you can use lidar as a
           surrogate for lidar.value, and this offers more flexible indexing, e.g. lidar[y,x]."""
        width, height = wb.wb_lidar_get_horizontal_resolution(self.tag), wb.wb_lidar_get_number_of_layers(self.tag)
        c_arraytype = (c_float*width)*height
        ptr = wb.wb_lidar_get_range_image(self.tag)
        return ctypes.cast(ptr, POINTER(c_arraytype))[0]

    # Many conversions, like to lists, bytes or numpy.arrays are inherited from ImageContainer.
    # Here we mostly just need to link old deprecated versions to those.

    @use_docstring_as_deprecation_warning
    def getRangeImage(self) -> List[float]:
        """DEPRECATED: Lidar.getRangeImage() is deprecated; lidar.list is equivalent but other options may be better."""
        return self.list

    @use_docstring_as_deprecation_warning
    def getRangeImageArray(self):
        """DEPRECATED: Lidar.getRangeImage() is deprecated; lidar.nested_list is equivalent and lidar itself mostly is."""
        return self.nested_list

    @use_docstring_as_deprecation_warning
    def getLayerRangeImage(self, layer:int):
        """DEPRECATED: Lidar.getLayerRangeImage(y) is deprecated; use lidar[y] instead."""
        return self[layer]

    # --- Lidar Point Clouds ---

    class Cloud(ImageContainer['Lidar.Cloud.Point']):
        class Point(ctypes.Structure, VectorValue):
            """A Lidar.Cloud.Point is a member of a Lidar.Cloud, and indicates the coordinates of a surface-point
               detected by Lidar, in the frame of reference of the Lidar's axes.
               Each Point shares memory with the Webots simulation, which makes Clouds relatively quick to
               access, but makes Clouds and their Points be valid only for the current timestep.  If you will
               want to refer back to their values later, you'll need to make a copy, e.g. with Cloud.copy().
               Lidar points are vector-like, so you can do standard vector arithmetic with them,
               or use helper functions like point.angle and point.distance.  These vectorized ops consider only
               the .x, .y and .z of the Point and not its .layer_id and .time."""
            # Declare fields for c_types conversion from C-API
            _fields_ = [('x', c_float),
                        ('y', c_float),
                        ('z', c_float),
                        ('layer_id', c_int),
                        ('time', c_float)]

            # Re-declare attributes for Python linters
            x: float
            y: float
            z: float
            layer_id: int
            time: float

            @property
            def value(self) -> Vector:
                """The xyz vector information about this Point.
                   For most purposes, you can use each Point as a surrogate for this value."""
                return Vector(self.x, self.y, self.z)

            def __repr__(self): return f"Lidar.Cloud.Point({self.x}, {self.y}, {self.z})"


        # --- Lidar.Cloud initialization ---

        def __init__(self, lidar: 'Lidar'):
            self.lidar = lidar
            self.tag = lidar.tag
            wb.wb_lidar_enable_point_cloud(self.tag)  # automatically enable (share's parent lidar's sampling period)

        def __repr__(self):
            return f"{self.lidar}.cloud"

        wb.wb_lidar_is_point_cloud_enabled.restype = c_bool
        @property
        def sampling(self)->bool:
            """Setting lidar.cloud.sampling = True will enable lidar.cloud to return a 2D array of Lidar.Cloud.Points
               each time the lidar produces a reading. This is enabled automatically on first reference to lidar.cloud.
               Setting it to False will disable this, which will save simulation processing time.
               The current setting may be read as lidar.cloud.sampling"""
            return wb.wb_lidar_is_point_cloud_enabled(self.tag)
        @sampling.setter
        def sampling(self=None, new_value:bool = True):
            if new_value:
                wb.wb_lidar_enable_point_cloud(self.tag)
            else:
                wb.wb_lidar_disable_point_cloud(self.tag)

        # --- Lidar.Cloud value and dimensions, needed by ImageContainer superclass ---

        @property
        def width(self) -> int:
            """The width of this Lidar.Cloud, i.e. the width or horizontal resolution of this Lidar."""
            return wb.wb_lidar_get_horizontal_resolution(self.tag)

        @property
        def height(self) -> int:
            """The height of this Lidar.Cloud, i.e. the height or number of layers of this Lidar."""
            return wb.wb_lidar_get_number_of_layers(self.tag)

        wb.wb_lidar_get_point_cloud.restype = POINTER(Point)
        @timed_cached_property
        def value(self) -> Sequence[Sequence[ColorBGRA]]:
            """Returns the current value of this Lidar.Cloud as a ctypes array of rows of Lidar.Cloud.Point pixels.
               Each Point has its own .value that is an xyz vector pointing to a surface detected by lidar, and
               each Point serves as a surrogate for this value, so e.g. Points can be used in vector arithmetic.
               Each Point also has a .layer_id indicating which layer in the Lidar sampled that point
               (i.e. its y/row index in the cloud) and .time indicating when the Lidar last sampled that point.
               the Li vector that supports vector arithmetic that themselves."""
            width, height = wb.wb_lidar_get_horizontal_resolution(self.tag), wb.wb_lidar_get_number_of_layers(self.tag)
            c_arraytype = height * (width * Lidar.Cloud.Point)
            ptr = wb.wb_lidar_get_point_cloud(self.tag)
            return ctypes.cast(ptr, POINTER(c_arraytype))[0]

        @property
        def array(self):  # overriding superclass to return just xyz components
            """If numpy is available, this returns a numpy array with shape (height, width, 3) whose last dimension
               contains x,y,z values of points in this point cloud, as viewed from the lidar's frame of reference.
               This array shares memory with the Webots simulation making it very fast to create, but also making it
               valid only for this timestep.  If you'll want to refer to its values later, make a copy, e.g
               with numpy's ndarray.copy().
               Note: this array does not explicitly include each point's .layer_id (though that is implicit in each
               point's location along the first/height axis of the array) nor does it contain each point's .time (since
               numpy requires that array elements be evenly spaced in memory, and the .time value is not evenly
               spaced with .x, .y, and .z, due to the intervening .layer_id which has an incompatible datatype).
               If you want layer_id and time in numpy, use numpy.array(lidar.cloud.value, copy=False) to create a
               numpy 2D structured array within which you may index ['layer_id'] and ['time'].  However this structured
               array will not allow standard indexing or slicing along a third dimension, and cannot easily be made to
               engage in vectorized operations due to the heterogeneity of its data, which is why lidar.cloud.array
               returns the more homogenous and directly usable xyz-only version.
               Requires numpy.  If numpy is not available this will raise an ImportError."""
            import numpy as np
            width, height = wb.wb_lidar_get_horizontal_resolution(self.tag), wb.wb_lidar_get_number_of_layers(self.tag)
            return np.array(self.value, copy=False).view('<f4').reshape((height, width, 5))[:, :, :3]

    @cached_property
    def cloud(self) -> Cloud:
        """Provides access to this Lidar's Cloud (aka its "point-cloud"), which provides access to detailed information
           about the surface points this Lidar has detected.
           Whenever you first refer to lidar.cloud, or if you set lidar.cloud.sampling = True, the Cloud will
           automatically be enabled to produce new readings whenever the lidar itself does.
           Setting lidar.cloud.sampling = None disables readings, which will speed up the simulation again.
           When enabled, `lidar.cloud` works as another ImageContainer, similar to `lidar` itself, except that where
           the lidar's basic image's pixels were simply floats indicating the distance to the detected surface in
           that direction, the cloud's pixels are instead Lidar.Cloud.Point objects which are vector-like surrogates
           for the x,y,z vector to the detected point from the frame of reference of the Lidar (also available as
           point.value), and also allow point.layer_id and point.time to return the point's layer and detection time.
           You can do the standard ImageContainer things with lidar.cloud, like indexing (cloud[row] or cloud[row, x]),
           iterating (for row in cloud, or for x,y,point in cloud.enumerated) or fast Numpy conversion to cloud.array,
           though note that this returns a (height x width x 3) array of x,y,z coordinates."""
        return Lidar.Cloud(self)  # now future references will retrieve this directly; automatically enables itself

    # --- Deprecated methods for handling Lidar.Clouds ---

    @use_docstring_as_deprecation_warning
    def enablePointCloud(self):
        """DEPRECATED. Lidar.enablePointCloud() is deprecated. Your first reference to lidar.cloud automatically enables
           it, or you can use lidar.cloud.sampling = True."""
        return wb.wb_lidar_enable_point_cloud(self.tag)
    @use_docstring_as_deprecation_warning
    def disablePointCloud(self):
        """DEPRECATED. Lidar.disablePointCloud() is deprecated. Use lidar.cloud.sampling = None."""
        return wb.wb_lidar_disable_point_cloud(self.tag)
    @use_docstring_as_deprecation_warning
    def isPointCloudEnabled(self):
        """DEPRECATED. Lidar.isPointCloudEnabled() is deprecated. Use lidar.cloud.sampling."""
        return wb.wb_lidar_is_point_cloud_enabled(self.tag)

    @use_docstring_as_deprecation_warning
    def getNumberOfPoints(self) -> int:
        """DEPRECATED: Lidar.getNumberOfPoints() is deprecated. This is equivalent to lidar.width * lidar.height."""
        return wb.wb_lidar_get_number_of_points(self.tag)

    @use_docstring_as_deprecation_warning
    def getPointCloud(self, data_type='list'):
        """DEPRECATED. Lidar.getPointCloud(type) is deprecated. Strict equivalents are lidar.cloud.nested_list
           and bytes(lidar.cloud), though for most purposes using lidar.cloud or lidar.cloud.array is better."""
        if data_type == 'buffer':
            return bytes(self.cloud)
        if data_type == 'list':
            return self.cloud.nested_list
        raise TypeError(f"Lidar.getPointCloud data_type cannot be {data_type}.  It must be 'list' or 'buffer'.")

    @use_docstring_as_deprecation_warning
    def getLayerPointCloud(self, layer: int, data_type='list'):
        """DEPRECATED. Lidar.getLayerPointCloud(i, type) is deprecated. Strict equivalents are list(lidar.cloud[i])
           and bytes(lidar.cloud[i]), though simply using lidar.cloud[i] is often better."""
        if data_type == 'buffer':
            return bytes(self.cloud[layer])
        if data_type == 'list':
            return list(self.cloud[layer])
        raise TypeError(f"Lidar.getLayerPointCloud data_type cannot be {data_type}.  It must be 'list' or 'buffer'.")


class RangeFinder(ImageContainer[float], Device, Sensor):
    """A Python RangeFinder object is used to control a rangefinder device node in the simulation, which itself
       detects distances to nearby objects at an array of similar angles.
       `rf = robot.RangeFinder("devicename")` creates a RangeFinder to control the rangefinder with that name.
       IMAGECONTAINER INTERFACE (inherited from ImageContainer superclass).
       `rf.width` and `rf.height` are the dimensions of the RangeFinder image, in pixels.
       `rf[y]` returns row y of the the current RangeFinder reading
       `rf[y,x]` returns the current reading of the RangeFinder at row y, column x (fast). Note y index comes first!
         At most one of these indices can be a slice, e.g., 0:2.
       `for row in rf` and `for row in rf.by_row` iterate through rows, from top to bottom.
       `for pixel in rf.by_pixel` and `for x, y, pixel in rf.enumerated` go through all pixels, row after row.
       `rf.value` returns a ctypes array containing the RangeFinder's current reading. This array shares memory
         with Webots, so is valid only for the duration of this timestep. If you'll want access to this information
         later, you'll need to copy it. This array is a sequence of rows, where each row is a sequence of floats.
         This allows indexing by rows, then columns [y][x], but does not allow comma multi-indexing like [y,x].
         Iterating this .value iterates through rows.
       `rf.array` returns a view of the current readings as a Numpy array which shares memory with the simulation so is
         extremely fast to create, but is valid only for the current timestep (extremely fast, but requires numpy)
       `rf.copy()` returns a copy of the current readings that (unlike .value and .array) does not share memory with
         Webots' internal representation of the image, so is safe to use on future timesteps. This copy retains the
         same ImageContainer-interface as the RangeFinder itself. (fast, recommended way to copy outside numpy)
       `rf.buffer` copies the current value to a python bytes object (fairly fast, but very inconvenient to use)
       `rf.list` returns the current readings all together in one long 1D python list (slow)
       `rf.nested_list` returns a 2D list of row-like lists of RangeFinder readings (slow)
       `rf.save_image(filename, quality)` saves the current image to file, with quality specify jpeg quality (0-100)
       RANGEFINDER CHARACTERISTICS.
       `rf.fov` and `rf.vertical_fov` return the horizontal and vertical fields of view of the rangefinder, in radians.
       `rf.min_range` and `rf.max_range` return bounds on the range of detectable objects."""

    #--- RangeFinder image dimensions (accessed by ImageContainer superclass) ---

    @property
    def width(self) -> int:
        """Returns the current width of this RangeFinder's image, in pixels."""
        return wb.wb_range_finder_get_width(self.tag)
    @use_docstring_as_deprecation_warning
    def getWidth(self) -> int:
        """DEPRECATED: RangeFinder.getWidth() is deprecated. Please use rangefinder.width instead."""
        return wb.wb_range_finder_get_width(self.tag)

    @property
    def height(self) -> int:
        """Returns the current height of this RangeFinder's image, in pixels."""
        return wb.wb_range_finder_get_height(self.tag)
    @use_docstring_as_deprecation_warning
    def getHeight(self) -> int:
        """DEPRECATED: RangeFinder.getHeight() is deprecated. Please use rangefinder.height instead."""
        return wb.wb_range_finder_get_height(self.tag)

    #--- RangeFinder characteristics ---

    wb.wb_range_finder_get_fov.restype = c_double
    @property
    def fov(self) -> float:
        """Returns the horizontal field of view of this RangeFinder, in radians."""
        return wb.wb_range_finder_get_fov(self.tag)
    @use_docstring_as_deprecation_warning
    def getFov(self) -> float:
        """DEPRECATED: RangeFinder.getFov() is deprecated. Please use rangefinder.fov instead."""
        return wb.wb_range_finder_get_fov(self.tag)

    @property
    def vertical_fov(self) -> float:
        """Computes the vertical field of view of this RangeFinder, in radians, using the formula in Webots docs."""
        return 2 * math.atan(math.tan(self.fov * 0.5) * (self.height / self.width))

    wb.wb_range_finder_get_min_range.restype = c_double
    @property
    def min_range(self) -> float:
        """Returns the minimum possible range of this RangeFinder."""
        return wb.wb_range_finder_get_min_range(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinRange(self) -> float:
        """DEPRECATED: RangeFinder.getMinRange() is deprecated. Please use rangefinder.min_range instead."""
        return wb.wb_range_finder_get_min_range(self.tag)

    wb.wb_range_finder_get_max_range.restype = c_double
    @property
    def max_range(self) -> float:
        """Returns the maximum possible range of this RangeFinder."""
        return wb.wb_range_finder_get_max_range(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxRange(self) -> float:
        """DEPRECATED: RangeFinder.getMaxRange() is deprecated. Please use rangefinder.max_range instead."""
        return wb.wb_range_finder_get_max_range(self.tag)

    # ---RangeFinder reading values---

    wb.wb_range_finder_get_range_image.restype = c_float_p
    @timed_cached_property
    def value(self) -> Sequence[Sequence[float]]:
        width, height = wb.wb_range_finder_get_width(self.tag), wb.wb_range_finder_get_height(self.tag)
        c_arraytype = (c_float*width)*height
        # array_interface = dict(size=(height, width),
        #                        type='<f2')
        # array_type = type(f"RangeArray{width}x{height}", (c_arraytype, RangeFinder.RangeImage), {})
        ptr = wb.wb_range_finder_get_range_image(self.tag)
        return ctypes.cast(ptr, POINTER(c_arraytype))[0]
        # return c_arraytype.from_buffer(wb.wb_range_finder_get_range_image(self.tag))

    # Many conversions, like to lists, bytes or numpy.arrays are inherited from ImageContainer.
    # Here we mostly just need to link old deprecated versions to those.

    @use_docstring_as_deprecation_warning
    def getRangeImage(self, data_type='list'):
        """DEPRECATED: RangeFinder.getRangeImage(datatype) is deprecated. Use rf.list or rf.bytes, or
           `[value for x,y,value in rangefinder]` is equivalent to the old 'list' data_type.
           `bytes(rangefinder.value)` is equivalent to the old 'buffer' data_type, but not very useful.
           `rangefinder.array` more quickly returns a numpy array than the old buffer approach."""
        if data_type == 'buffer': return self.bytes
        if data_type == 'list': return self.list
        WarnOnce(f"Error: RangeFinder data_type cannot be {data_type}! Supported values are 'list' and 'buffer'.\n")

    @use_docstring_as_deprecation_warning
    def getRangeImageArray(self):
        """DEPRECATED: RangeFinder.getRangeImageArray() is deprecated. An equivalent is rangefinder.nested_list.
           Other options may be better."""
        return self.nested_list

    def save_image(self, filename: str, quality: int = 90) -> int:
        """Saves an image file of what this RangeFinder currently sees.  If filename concludes '.jpg' or '.jpeg'
           then quality will determine jpeg quality (0-100, default 90).
           The boolean of the returned int indicates whether the save failed (-1/True = failure, 0/False = success)."""
        return wb.wb_range_finder_save_image(self.tag, filename.encode(), quality)
    @use_docstring_as_deprecation_warning
    def saveImage(self, filename: str, quality: int = 90):
        """DEPRECATED: Webots is moving to conventional Python naming, so RangeFinder.saveImage is now .save_image.
           Also its quality parameter is now optional (defaults to 90)."""
        return wb.wb_range_finder_save_image(self.tag, filename.encode(), quality)

    @staticmethod
    def rangeImageGetDepth(im, width, x, y):
        raise NotImplementedError("RangeFinder.rangeImageGetDepth() is no longer supported. Use rangefinder[x,y].")


# === Devices involving Motors ========

class Brake(Device):
    ROTATIONAL = WB_ROTATIONAL
    LINEAR = WB_LINEAR
    ANGULAR = ROTATIONAL  # extra-deprecated version of ROTATIONAL, for backwards compatibility
    # TODO add deprecation warnings to all of these?  Or just to ANGULAR?

    @property
    def is_rotational(self) -> bool:
        """Will be True if this brake is on any sort of rotational hinge/ball joint, False otherwise."""
        return wb.wb_brake_get_type(self.tag) == WB_ROTATIONAL

    @property
    def is_linear(self) -> bool:
        """Will be True if this brake is on a linear slider joint, False otherwise."""
        return wb.wb_brake_get_type(self.tag) == WB_LINEAR

    def getType(self) -> int:
        """DEPRECATED.  Returns the type of this Brake, which will be Brake.LINEAR for slider joints,
           or Brake.ROTATIONAL for hinge/hinge2/ball joints.  brake.getType() is deprecated and may
           someday be removed.  Please use brake.is_rotational or brake.is_linear instead."""
        WarnOnce("DEPRECATION. brake.getType() is deprecated. Please use brake.is_rotational or brake.is_linear instead.")
        return wb.wb_brake_get_type(self.tag)

    @property
    def damping_constant(self):
        """`brake.damping_constant = c` sets the damping constant for this brake, which together with the damping
           constant for the associated joint, determines how quickly a moving joint will tend to come to rest.
           If this robot is a supervisor and the world module has been imported, then reading `brake.damping_constant`
           will use supervisor powers to read the actual value of this field in the simulated world.  Otherwise,
           the Webots API does not provide any way for ordinary robots to read the current value for this, so trying
           to read this before setting it will raise an AttributeError, but once you have set it, the Python controller
           will remember the last value you set it to, and return that as brake.damping_constant."""
        if self._world is not None:  # If this robot is a supervisor, we use superpowers to look up the actual value
            return self._world.Node(self).dampingConstant
        if 'damping_constant' in self.__dict__: return self.__dict__['damping_constant']
        raise AttributeError("Brake.damping_constant has not yet been set, so it has no value knowable by this robot.")
    @damping_constant.setter
    def damping_constant(self, new_value: float):
        self.__dict__['damping_constant'] = new_value  # Cache this as a non-supervisor's best estimate of current value
        wb.wb_brake_set_damping_constant(self.tag, c_double(new_value))
    @use_docstring_as_deprecation_warning
    def setDampingConstant(self, new_value: float):
        """DEPRECATED: Brake.setDampingConstant(c) is deprecated. Please use brake.damping_constant = c."""
        self.damping_constant = new_value

    @property
    def motor(self) -> "Motor":
        """Returns the Motor associated with this Brake, or None if there is none."""
        motor_tag = wb.wb_brake_get_motor(self.tag)
        return Motor(motor_tag - 1) if motor_tag else None  # device index is tag minus 1
    @use_docstring_as_deprecation_warning
    def getMotor(self) -> "Motor":
        """DEPRECATION. Brake.getMotor() is deprecated. Please use brake.motor instead."""
        return self.motor

    @cached_property
    def position(self) -> "PositionSensor":
        """Returns the PositionSensor associated with this Brake, or None if there is none.
           This sensor will automatically be enabled to provide a reading at each future timestep (alterable
           by `brake.position.sampling = t`) but its immediate reading will be float(nan).
           A PositionSensor works as a surrogate for its own .value, so you can use `if brake.position > 0:`"""
        ps_tag = wb.wb_brake_get_position_sensor(self.tag)
        return PositionSensor(ps_tag - 1) if ps_tag else None  # device index is tag minus 1
    @use_docstring_as_deprecation_warning
    def getPositionSensor(self) -> "PositionSensor":
        """DEPRECATION. Brake.getPositionSensor() is deprecated. Please use brake.position instead."""
        return self.position

# TODO once I become confident that MetaDevice['Motor'] works here, should add similar for all devices
class Motor(Device, metaclass = MetaDevice['Motor']):
    """A Python Motor object is used to control a motor node in the Webots simulation, which may either be a linear
       motor on a slider joint or a rotational motor on a hinge or ball joint.
       `motor = robot.Motor("motorname")` creates a new Motor whose name-field in the simulation is "motorname".
       `motor.position` returns the PositionSensor for this motor (if it has one). PositionSensors serve as surrogates
       for their own .value, so you may use commands like `if motor.position > 0:`
       `motor.target_position = t` sets a target for this motor to gradually approach as the simulation proceeds.
       `motor.target_position = None` stops the motor from automatically approaching any target position, e.g. so that
       you can control the .velocity that it moves at instead.
       `motor.velocity` reads or adjusts the target velocity for the motor to attempt to run at.
       `motor.acceleration` reads or adjusts the amount of acceleration the motor will attempt to use.
       The min and max limits on position and velocity may be read with expressions like `motor.min_position`.
       (Technically, other measures like velocity are as much "target" measures as "target_position" is. However, Webots
       provides no means for reading their *actual* values, so only `position` requires `target_` to disambiguate.)
       `motor.is_linear` will be True if this motor is on a linear slider joint, in which case you can
       read 'motor.max_force' and can read/alter 'motor.force' and `motor.available_force`.
       `motor.is_rotational` will be True if this motor is on a rotating hinge/ball joint, in which case you can
       read 'motor.max_torque' and can read/alter 'motor.torque' and `motor.available_torque`.
       `motor.multiplier` returns this motor's multiplier (e.g. -1 if it is coupled to run opposite another motor).
       `motor.pid = p, i, d` alters the p, i, and d, parameters for the motor's PID controller."""

    # --- Motor.methods involving checking Motor type ---

    @property
    def is_linear(self):
        """Will be True if this Motor is on a linear slider joint, False otherwise."""
        return wb.wb_motor_get_type(self.tag) == WB_LINEAR
    @property
    def is_rotational(self):
        """Will be True if this Motor is on any sort of rotational hinge/ball joint, False otherwise."""
        return wb.wb_motor_get_type(self.tag) == WB_ROTATIONAL
    @use_docstring_as_deprecation_warning
    @cached_property
    def LINEAR(self):
        """DEPRECATION. motor.LINEAR and motor.getType() are deprecated. Please use motor.is_linear instead."""
        return WB_LINEAR
    @use_docstring_as_deprecation_warning
    @cached_property
    def ROTATIONAL(self):
        """DEPRECATION. motor.ROTATIONAL and motor.getType() are deprecated. Please use motor.is_rotational instead."""
        return WB_ROTATIONAL
    @use_docstring_as_deprecation_warning
    @cached_property
    def ANGULAR(self):
        """DEPRECATION. Motor.ANGULAR and Motor.getType() are deprecated. Please use motor.is_rotational instead."""
        return WB_ROTATIONAL
    @use_docstring_as_deprecation_warning
    def getType(self):
        """DEPRECATED.  Returns the type of this Motor, which will be Motor.LINEAR for slider joints,
           or Motor.ROTATIONAL for hinge/hinge2/ball joints.  motor.getType() is deprecated and may
           someday be removed.  Please use motor.is_rotational or motor.is_linear instead."""
        WarnOnce("DEPRECATION. Motor.getType() is deprecated. Use motor.is_rotational or motor.is_linear instead.")
        return wb.wb_motor_get_type(self.tag)

    # --- Motor.methods involving position ---

    @cached_property
    def position(self) -> "PositionSensor":
        """`motor.position` returns the PositionSensor associated with this Motor, or None if there is none.
           Accessing this PositionSensor will automatically enable it to gather future readings each timestep
           (alterable by `motor.position.sampling = t`) but its immediate reading will be `float(nan)`.
           A PositionSensor works as surrogates for its own .value, so you can use `if motor.position > 0:`
           Note that `motor.position` cannot directly be set to a new value, but setting `motor.target_position = t`
           will make a motor gradually attempt to adjust its position to the target as the simulation proceeds."""
        ps_tag = wb.wb_motor_get_position_sensor(self.tag)
        return PositionSensor(ps_tag - 1) if ps_tag else None  # device index is tag minus 1

    @use_docstring_as_deprecation_warning
    def getPositionSensor(self) -> "PositionSensor":
        """DEPRECATION. Motor.getPositionSensor() is deprecated. Please use motor.position instead."""
        return self.position

    wb.wb_motor_get_target_position.restype = c_double
    @property
    def target_position(self) -> float:
        """`motor.target_position` returns the current target_position of this Motor, or None if it has none.
           `motor.target_position = t` sets the motor to gradually approach this new target as the simulation proceeds.
           `motor.target_position = None` stops the motor from automatically approaching any target, so that
           you can control its .velocity instead, e.g. by then setting motor.velocity = v."""
        pos = wb.wb_motor_get_target_position(self.tag)
        return pos if pos != float('inf') else None
    @target_position.setter
    def target_position(self, new_target_position: float):
        if new_target_position is None: new_target_position = float('inf')
        wb.wb_motor_set_position(self.tag, c_double(new_target_position))
    @use_docstring_as_deprecation_warning
    def getTargetPosition(self) -> float:
        """DEPRECATED: Motor.getTargetPosition() is deprecated. Please use motor.target_position instead, though
           note that this returns None rather than float('inf') in the case where a motor has no target position."""
        return wb.wb_motor_get_target_position(self.tag)
    def setPosition(self, new_target_position: float):
        """DEPRECATED: Motor.setPosition(t) is deprecated. Please use motor.target_position = t instead."""
        self.target_position = new_target_position

    wb.wb_motor_get_min_position.restype = c_double
    @property
    def min_position(self) -> float:
        """Returns the current minimum possible position of this Motor."""
        return wb.wb_motor_get_min_position(self.tag)
    @use_docstring_as_deprecation_warning
    def getMinPosition(self) -> float:
        """DEPRECATED: Motor.getMinPosition() is deprecated. Please use motor.min_position instead."""
        return wb.wb_motor_get_min_position(self.tag)

    wb.wb_motor_get_max_position.restype = c_double
    @property
    def max_position(self) -> float:
        """Returns the current maximum possible position of this Motor."""
        return wb.wb_motor_get_max_position(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxPosition(self) -> float:
        """DEPRECATED: Motor.getMaxPosition() is deprecated. Please use motor.max_position instead."""
        return wb.wb_motor_get_max_position(self.tag)

    # --- Motor.methods involving velocity ---

    wb.wb_motor_get_max_velocity.restype = c_double
    @property
    def max_velocity(self) -> float:
        """Returns the current max_velocity of this Motor."""
        return wb.wb_motor_get_max_velocity(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxVelocity(self) -> float:
        """DEPRECATED: Motor.getMaxVelocity() is deprecated. Please use motor.max_velocity instead."""
        return wb.wb_motor_get_max_velocity(self.tag)

    wb.wb_motor_get_velocity.restype = c_double
    @property
    def velocity(self) -> float:
        """`motor.velocity` returns the current target velocity that this Motor is attempting to run at.
           Note that this may differ from its current actual velocity, which you could approximately compute by
           comparing recent readings of `motor.position.value`.
           `motor.velocity = v` sets the target velocity of this Motor."""
        return wb.wb_motor_get_velocity(self.tag)
    @velocity.setter
    def velocity(self, new_velocity: float):
        wb.wb_motor_set_velocity(self.tag, c_double(new_velocity))
    @use_docstring_as_deprecation_warning
    def getVelocity(self) -> float:
        """DEPRECATED: Motor.getVelocity() is deprecated. Please use motor.velocity instead."""
        return wb.wb_motor_get_velocity(self.tag)
    @use_docstring_as_deprecation_warning
    def setVelocity(self, new_velocity: float):
        """DEPRECATED: Motor.setVelocity(v) is deprecated. Please use motor.velocity = v instead."""
        wb.wb_motor_set_velocity(self.tag, c_double(new_velocity))

    # --- Motor.methods involving acceleration ---

    wb.wb_motor_get_acceleration.restype = c_double
    @property
    def acceleration(self) -> float:
        """Returns or adjusts the target acceleration of this Motor. Note that *actual* acceleration will be affected
           by other factors, like resisting forces, and is not directly readable, but could be estimated by using
           three or more .position readings at successive timesteps to estimate two or more instantaneous velocities."""
        return wb.wb_motor_get_acceleration(self.tag)
    @acceleration.setter
    def acceleration(self, new_acceleration: float):
        wb.wb_motor_set_acceleration(self.tag, c_double(new_acceleration))
    @use_docstring_as_deprecation_warning
    def getAcceleration(self) -> float:
        """DEPRECATED: Motor.getAcceleration() is deprecated. Please use motor.acceleration instead."""
        return wb.wb_motor_get_acceleration(self.tag)
    @use_docstring_as_deprecation_warning
    def setAcceleration(self, new_acceleration: float):
        """DEPRECATED: Motor.setAcceleration(a) is deprecated. Please use motor.acceleration = a instead."""
        wb.wb_motor_set_acceleration(self.tag, c_double(new_acceleration))

    # --- Motor.methods involving force ---
    # These are defined on the Motor class since users will often create motors with Motor() rather than LinearMotor()
    # so if we want these to show up for linting purposes, we need it on the Motor class

    wb.wb_motor_get_max_force.restype = c_double
    @property
    def max_force(self) -> float:
        """Returns the current max_force of this Motor."""
        return wb.wb_motor_get_max_force(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxForce(self) -> float:
        """DEPRECATED: Motor.getMaxForce() is deprecated. Please use motor.max_force instead."""
        return wb.wb_motor_get_max_force(self.tag)

    wb.wb_motor_get_available_force.restype = c_double
    @property
    def available_force(self) -> float:
        """Returns or adjusts the current available_force of this Motor."""
        return wb.wb_motor_get_available_force(self.tag)
    @available_force.setter
    def available_force(self, new_available_force: float):
        wb.wb_motor_set_available_force(self.tag, c_double(new_available_force))
    @use_docstring_as_deprecation_warning
    def getAvailableForce(self) -> float:
        """DEPRECATED: Motor.getAvailableForce() is deprecated. Please use motor.available_force instead."""
        return wb.wb_motor_get_available_force(self.tag)
    @use_docstring_as_deprecation_warning
    def setAvailableForce(self, new_available_force: float):
        """DEPRECATED: Motor.setAvailableForce(f) is deprecated. Please use motor.available_force = f instead."""
        wb.wb_motor_set_available_force(self.tag, c_double(new_available_force))

    class ForceSensor(Sensor, SurrogateValue, auto_link_wb_methods=False):
        # this has idiosyncratically named wb_methods, so we define them here, and set auto_link to False above
        _enable = wb.wb_motor_enable_force_feedback
        _disable = wb.wb_motor_disable_force_feedback
        _get_sampling_period = wb.wb_motor_get_force_feedback_sampling_period

        def __init__(self, motor: 'LinearMotor'):
            self.tag = motor.tag
            self.sampling = True  # automatically enable with basic timestep as sampling period

        wb.wb_motor_get_force_feedback.restype = c_double
        @property
        def value(self) -> float:
            """The current force this linear motor is providing, in Newtons."""
            return wb.wb_motor_get_force_feedback(self.tag)

    @cached_property
    def force(self) -> ForceSensor:
        """Available only in LinearMotors (motors on slider joints).  Reads or sets this motor's force.
            Whenever you first refer to motor.force, or if you set motor.force.sampling = True, this sensor will
            automatically be enabled with the simulation's basic timestep as its sampling period, or you can set
            another sampling period in milliseconds. Setting motor.force.sampling = None disables readings,
            which will slightly speed up the simulation again.  When enabled, motor.force.value returns the
            current force that the motor is applying in Newtons.  As a SurrogateValue, motor.force can be used
            in place of motor.force.value for most purposes.  Setting motor.force = new_force manually specifies
            how much force the motor should administer, bypassing the PID controller."""
        if not isinstance(self, LinearMotor):
            raise TypeError("Only Linear Motors (motors on slider joints) have .force sensors.")
        return Motor.ForceSensor(self)  # now future references will retrieve this directly
    @force.set  # .set is equivalent to an @property .setter; needs different name to avoid confusing PyCharm linter
    def force(self, new_force: float):
        wb.wb_motor_set_torque(self.tag, c_double(new_force))
    @use_docstring_as_deprecation_warning
    def setForce(self, new_force):
        """DEPRECATED: Motor.setForce(f) is deprecated.  Please use motor.force = f instead."""
        self.force = new_force

    @use_docstring_as_deprecation_warning
    def enableForceFeedback(self, sampling_period):
        """DEPRECATION: Motor.enableForceFeedback(p) is deprecated. Please use motor.force.sampling = p
           or the first reference to motor.force will automatically enable it with the basic timestep as the period."""
        self.force.sampling = sampling_period
    @use_docstring_as_deprecation_warning
    def disableForceFeedback(self):
        """DEPRECATION: Motor.disableForceFeedback() is deprecated. Please use motor.force.sampling = None"""
        self.force.sampling = None
    @use_docstring_as_deprecation_warning
    def getForceFeedbackSamplingPeriod(self):
        """DEPRECATION: Motor.getForceFeedbackSamplingPeriod() is deprecated. Please use motor.force.sampling"""
        return self.force.sampling
    @use_docstring_as_deprecation_warning
    def getForceFeedback(self):
        """DEPRECATION: Motor.getForceFeedback() is deprecated. Please use motor.force.value or simply motor.force instead"""
        return self.force.value

    # --- Motor.methods involving torque ---
    # Defined on the Motor class since users will often create motors with Motor() rather than RotationalMotor()
    # so if we want these to show up for linting purposes, we need it on the Motor class

    wb.wb_motor_get_max_torque.restype = c_double
    @property
    def max_torque(self) -> float:
        """Returns the current max_torque of this Motor. XXX"""
        return wb.wb_motor_get_max_torque(self.tag)
    @use_docstring_as_deprecation_warning
    def getMaxTorque(self) -> float:
        """DEPRECATED: Motor.getMaxTorque() is deprecated. Please use motor.max_torque instead."""
        return wb.wb_motor_get_max_torque(self.tag)

    wb.wb_motor_get_available_torque.restype = c_double
    @property
    def available_torque(self) -> float:
        """Returns or adjusts the current available_torque of this Motor."""
        return wb.wb_motor_get_available_torque(self.tag)
    @available_torque.setter
    def available_torque(self, new_available_torque: float):
        wb.wb_motor_set_available_torque(self.tag, c_double(new_available_torque))
    @use_docstring_as_deprecation_warning
    def getAvailableTorque(self) -> float:
        """DEPRECATED: Motor.getAvailableTorque() is deprecated. Please use motor.available_torque instead."""
        return wb.wb_motor_get_available_torque(self.tag)
    def setAvailableTorque(self, new_available_torque: float):
        """DEPRECATED: Motor.setAvailableTorque(t) is deprecated. Please use motor.available_torque = t instead."""
        wb.wb_motor_set_available_torque(self.tag, c_double(new_available_torque))

    class TorqueSensor(Sensor, SurrogateValue, auto_link_wb_methods=False):
        # this has idiosyncratically named wb_methods, so we define them here, and set auto_link to False above
        _enable = wb.wb_motor_enable_torque_feedback
        _disable = wb.wb_motor_disable_torque_feedback
        _get_sampling_period = wb.wb_motor_get_torque_feedback_sampling_period

        def __init__(self, motor: 'RotationalMotor'):
            self.tag = motor.tag
            self.sampling = True  # automatically enable with basic timestep as sampling period

        wb.wb_motor_get_torque_feedback.restype = c_double
        @property
        def value(self) -> float:
            """The current torque this rotational motor is providing, in Newton meters."""
            return wb.wb_motor_get_torque_feedback(self.tag)

    @cached_property
    def torque(self) -> TorqueSensor:
        """Available only in RotationalMotors (motors on hinge/ball joints).  Reads or sets this motor's torque.
           Whenever you first refer to motor.torque, or if you set motor.torque.sampling = True, this sensor will
           automatically be enabled with the simulation's basic timestep as its sampling period, or you can set
           another sampling period in milliseconds. Setting motor.torque.sampling = None disables readings,
           which will slightly speed up the simulation again.  When enabled, motor.torque.value returns the current
           torque that the motor is applying in Newton meters.  As a SurrogateValue, motor.torque can be used in
           place of motor.torque.value for most purposes.  Setting motor.torque = new_torque manually specifies
           how much torque the motor should administer, bypassing the PID controller."""
        if not isinstance(self, RotationalMotor):
            raise TypeError("Only RotationalMotors (motors on hinge/ball joints) have .torque sensors.")
        return Motor.TorqueSensor(self)  # future references to motor.torque will directly return this
    @torque.set  # .set is equivalent to an @property .setter; needs different name to avoid confusing PyCharm linter
    def torque(self, new_torque: float):
        wb.wb_motor_set_torque(self.tag, c_double(new_torque))
    @use_docstring_as_deprecation_warning
    def setTorque(self, new_torque):
        """DEPRECATED: Motor.setTorque(t) is deprecated.  Please use motor.torque = t instead."""
        self.torque = new_torque

    @use_docstring_as_deprecation_warning
    def enableTorqueFeedback(self, sampling_period):
        """DEPRECATION: motor.enableTorqueFeedback(p) is deprecated. Please use motor.torque.sampling = p
           or the first reference to motor.torque will automatically enable it with the basic timestep as the period."""
        self.torque.sampling = sampling_period
    @use_docstring_as_deprecation_warning
    def disableTorqueFeedback(self):
        """DEPRECATION: Motor.disableTorqueFeedback() is deprecated. Please use motor.torque.sampling = None"""
        self.torque.sampling = None
    @use_docstring_as_deprecation_warning
    def getTorqueFeedbackSamplingPeriod(self):
        """DEPRECATION: Motor.getTorqueFeedbackSamplingPeriod() is deprecated. Please use motor.torque.sampling"""
        return self.torque.sampling
    @use_docstring_as_deprecation_warning
    def getTorqueFeedback(self):
        """DEPRECATION: Motor.getTorqueFeedback() is deprecated. Please use motor.torque.value or simply motor.torque instead"""
        return self.torque.value

    # --- Motor.methods other ---

    @property
    def pid(self):
        raise AttributeError("Motor PID parameters are settable but not readable.")
    @pid.setter
    def pid(self, pid_as_tuple: Tuple[float, float, float]):
        """`motor.pid = p, i, d` alters the p, i and d parameters of the motor's PID controller, which affects
           how exactly the motor attempts to home in on its target_position."""
        p, i, d = pid_as_tuple
        wb.wb_motor_set_control_pid(self.tag, c_double(p), c_double(i), c_double(d))

    @use_docstring_as_deprecation_warning
    def setControlPID(self, p: float, i: float, d: float):
        """DEPRECATED: Motor.setControlPID(p,i,d) is deprecated.  Use `motor.pid = p, i, d` instead."""
        wb.wb_motor_set_control_pid(self.tag, c_double(p), c_double(i), c_double(d))

    wb.wb_motor_get_multiplier.restype = c_double
    @property
    def multiplier(self) -> float:
        """Returns the current multiplier value of this Motor (e.g. -1 for a motor that is coupled to run in the
           opposite direction of another motor)."""
        return wb.wb_motor_get_multiplier(self.tag)
    @use_docstring_as_deprecation_warning
    def getMultiplier(self) -> float:
        """DEPRECATED: Motor.getMultiplier() is deprecated. Please use motor.multiplier instead."""
        return wb.wb_motor_get_multiplier(self.tag)

    # --- Motor.methods to find associated brake ---

    @cached_property
    def brake(self) -> "Brake":
        """Returns the Brake associated with this Motor, or None if there is none."""
        brake_tag = wb.wb_motor_get_brake(self.tag)
        return Brake(brake_tag - 1) if brake_tag else None  # device index is tag minus 1
    @use_docstring_as_deprecation_warning
    def getBrake(self) -> "Brake":
        """DEPRECATION. Motor.getBrake() is deprecated. Please use motor.brake instead."""
        return self.brake


class RotationalMotor(Motor):
    # defined for parity with Brakes and PositionSensors, though isinstance(this_motor, RotationalMotor) would work too
    # since these don't vary, they could have been assigned as simple attributes, but properties provide docstrings
    @property
    def is_rotational(self):
        """Will be True if this motor is on any sort of rotational hinge/ball joint, False otherwise."""
        return True  # always True for RotationalMotors
    @property
    def is_linear(self):
        """Will be True if this motor is on a linear slider joint, False otherwise."""
        return False  # always False for RotationalMotors


class LinearMotor(Motor):
    @property
    def is_rotational(self):
        """Will be True if this motor is on any sort of rotational hinge/ball joint, False otherwise."""
        return False  # always False for LinearMotors
    @property
    def is_linear(self):
        """Will be True if this motor is on a linear slider joint, False otherwise."""
        return True  # always True for LinearMotors


class PositionSensor(Device, Sensor, SurrogateValue):
    """A Python PositionSensor object is used to monitor the position of a joint in the Webots simulation.
       XXX"""

    ROTATIONAL = WB_ROTATIONAL
    LINEAR = WB_LINEAR
    ANGULAR = ROTATIONAL  # extra-deprecated version of ROTATIONAL, for backwards compatibility
    # TODO add deprecation warnings to all of these?  Or just to ANGULAR?

    @property
    def is_rotational(self) -> bool:
        """Will be True if this PositionSensor is on any sort of rotational hinge/ball joint, False otherwise."""
        return wb.wb_position_sensor_get_type(self.tag) == WB_ROTATIONAL

    @property
    def is_linear(self) -> bool:
        """Will be True if this PositionSensor is on a linear slider joint, False otherwise."""
        return wb.wb_position_sensor_get_type(self.tag) == WB_LINEAR

    def getType(self):
        """DEPRECATED.  Returns the type of this PositionSensor, which will be PositionSensor.LINEAR for slider joints,
           or PositionSensor.ROTATIONAL for hinge/hinge2/ball joints.  PositionSensor.getType() is deprecated and may
           someday be removed.  Please use ps.is_rotational or ps.is_linear instead."""
        WarnOnce("DEPRECATED: PositionSensor.getType() is deprecated. Please use p.is_rotational or p.is_linear")
        return wb.wb_position_sensor_get_type(self.tag)

    wb.wb_position_sensor_get_value.restype = c_double
    @property
    def value(self) -> float:
        """Returns the current value of this PositionSensor."""
        return wb.wb_position_sensor_get_value(self.tag)
    @use_docstring_as_deprecation_warning
    def getValue(self) -> float:
        """DEPRECATED: PositionSensor.getValue() is deprecated. Please use positionsensor.value instead,
           or for most purposes you can use a PositionSensor as a surrogate for its own value."""
        return wb.wb_position_sensor_get_value(self.tag)

    @property
    def motor(self) -> "Motor":
        """Returns the Motor associated with this PositionSensor, or None if there is none."""
        motor_tag = wb.wb_position_sensor_get_motor(self.tag)
        return Motor(motor_tag - 1) if motor_tag else None  # device index is tag minus 1
    @use_docstring_as_deprecation_warning
    def getMotor(self) -> "Motor":
        """DEPRECATION. PositionSensor.getMotor() is deprecated. Please use ps.motor instead."""
        return self.motor

    @property
    def brake(self) -> "Brake":
        """Returns the Brake associated with this PositionSensor, or None if there is none."""
        brake_tag = wb.wb_position_sensor_get_brake(self.tag)
        return Brake(brake_tag - 1) if brake_tag else None  # device index is tag minus 1

    @use_docstring_as_deprecation_warning
    def getBrake(self) -> "Brake":
        """DEPRECATION. PositionSensor.getBrake() is deprecated. Please use ps.brake instead."""
        return self.brake


# === Other Devices ===

class Connector(Device):
    """A Python Connector object is used to control a connector node in the simulation. A connector node can form a
       temporary physical coupling with another nearby connector node of an appropriate model, passive/active type,
       and in a suitable location and orientation.
       `connector = Connector("name")` creates a python Connector to control the connector node with the given name.
       `connector.present` indicates whether there is a compatible connector suitably located to couple.
          (1 = yes, 0 = no, -1 = passive/inapplicable; boolean True means yes, boolean False means no/inapplicable)
       As with sensors, the first reference to connector.present will enable this to produce readings each future
       timestep, alterable with `connector.present.sampling = t`.
       `connector.lock()` or `connector.is_locked = True` makes the connector generally willing to couple with another.
       If a compatible mate is already present, it will attempt to form a coupling. Otherwise, if this connector's
       autoLock setting in the simulation is True, then this connector will attempt to lock once a mate becomes present.
       `connector.unlock()` or `connector.is_locked = False` makes the connector unwilling to be coupled. This would
       break its current coupling, unless `unilateralUnlock` is false, and the other connector is still clinging on.
       `connector.is_locked` returns the connector's willingness to become locked to another connector."""

    class PresenceSensor(Sensor, SurrogateValue, auto_link_wb_methods=False):
        # this has idiosyncratically named wb_methods, so we define them here, and set auto_link to False above
        _enable = wb.wb_connector_enable_presence
        _disable = wb.wb_connector_disable_presence
        _get_sampling_period = wb.wb_connector_get_presence_sampling_period

        def __init__(self, connector: 'Connector'):
            self.tag = connector.tag
            self.sampling = True  # automatically enable with basic timestep as sampling period

        def __bool__(self):
            """This special boolean method returns False in the passive/non-applicable case, (value -1)
               rather than treating that as True as would have been the default."""
            return wb.wb_connector_get_presence(self.tag) == 1

        wb.wb_connector_get_presence.restype = c_int
        @property
        def value(self) -> int:
            """Indicates whether a compatible detector is currently present.
               Value 1 indicates that one is present; 0 that none is; -1 that this is inapplicable because this
               connector is of passive type."""
            return wb.wb_connector_get_presence(self.tag)

    @cached_property
    def present(self) -> PresenceSensor:
        """`connector.present` reads whether a compatible connector is present.
           Whenever you first refer to connector.present, or if you set `connector.present.sampling = True`, presence
           readings will automatically be enabled with the simulation's basic timestep as the sampling period,
           or you can set another sampling period in milliseconds. Setting `connector.present.sampling = None` disables
           readings, which will slightly speed up the simulation again.  When enabled, `connector.present` (a surrogate
           for `connector.present.value`) indicates whether a compatible connector is currently present in an
           appropriate position and orientation to mate with this one.  Value 1 indicates a compatible mate is present;
           0 indicates that none is; -1 that this is inapplicable because this connector is of passive type.
           This has a special boolean method making `if connector.present:` count as True when a compatible mate is
           present, and False otherwise, including the (value -1) case where this connector is passive/inapplicable."""
        return Connector.PresenceSensor(self)  # now future references will retrieve this directly

    @use_docstring_as_deprecation_warning
    def enablePresence(self, samplingPeriod: int):
        """DEPRECATED: Connector.enablePresence() is deprecated. Please use connector.present.sampling = True"""
        wb.wb_connector_enable_presence(self.tag, samplingPeriod)

    @use_docstring_as_deprecation_warning
    def disablePresence(self):
        """DEPRECATED: Connector.disablePresence() is deprecated. Please use connector.present.sampling = None"""
        wb.wb_connector_disable_presence(self.tag)

    @use_docstring_as_deprecation_warning
    def getPresenceSamplingPeriod(self) -> int:
        """DEPRECATED: Connector.getPresenceSamplingPeriod() is deprecated.
           Please use connector.present.sampling instead."""
        return wb.wb_connector_get_presence_sampling_period(self.tag)

    @use_docstring_as_deprecation_warning
    def getPresence(self) -> int:
        """DEPRECATED: Connector.getPresence() is deprecated. Please use connector.present instead."""
        return wb.wb_connector_get_presence(self.tag)

    wb.wb_connector_is_locked.restype = c_bool
    @property
    def is_locked(self) -> bool:
        """Reads/adjusts whether this connector is generally willing to couple with another compatible connector.
           Setting connector.is_locked = True attempts to create a physical coupling if a compatible mate is
           currently present, and makes the connector willing to autolock if a mate becomes present later and this
           connector node has autoLock enabled.
           Setting connector.is_locked = False makes the connector unwilling to autolock and stops maintaining any
           current coupling, though if the node's unilateralUnlock is False, the coupled mate may need to let go too."""
        return wb.wb_connector_is_locked(self.tag)
    @is_locked.setter
    def is_locked(self, new_value: bool):
        if new_value:
            wb.wb_connector_lock(self.tag)
        else:
            wb.wb_connector_unlock(self.tag)
    @use_docstring_as_deprecation_warning
    def isLocked(self) -> bool:
        """DEPRECATED. Connector.isLocked() is deprecated. Please use the property connector.is_locked instead."""
        return wb.wb_connector_is_locked(self.tag)

    def lock(self):
        """Makes this connector generally willing to couple with other connectors.  If a compatible connector is
           currently present, this attempts to form a physical connection to it.  Or if this connector node's autoLock
           field is True, a connection will automatically form when a compatible connector becomes present later.
           Equivalent to setting connector.is_locked = True"""
        wb.wb_connector_lock(self.tag)

    def unlock(self):
        """Makes this connector unwilling to be coupled with other connectors, and attempts to break any current
           physical connection (though if unilateralUnlock is False, the other connector will have to let go too).
           Equivalent to setting connector.is_locked = False"""
        wb.wb_connector_unlock(self.tag)


class LED(Device, SurrogateValue):
    """A Python LED object is used to control a Light Emitting Diode (LED) device node in the Webots simulation.
       `led = robot.LED("name") creates a Python LED device to control the LED with that name.
       `led.value` and `led.value = new_value` return and alter the current value of that LED as an integer.
          For non-gradual LED's value 0 = off, 1 = the first color in the LED node's `color` field, 2 = the second...
          For a monocrhrome LED (gradual with a single color), values range from 0/black to 255 (the given color).
          For an RGB LED (gradual, no preset color) values are a hexcolor integer, like 0xFF00FF.
       `led.color` and `led.color = new_color` provide a more convenient way to read and set RGB LED color, returning
          Color vectors with .r, .g, and .b components, and accepting vector-like new_color arguments."""

    Color = Color  # provide a handle to the Color vector subclass as LED.Color

    @property
    def value(self) -> int:
        """'led.value` returns the current status value of this LED as an integer. Since each LED is a surrogate for
           its own value, you can often just use the device in place of its value.  E.g. `if led:` will trigger when
           the LED is on (non-zero value), and not if it is off/zero.
           Setting `led.value = new_value` sets this LED to the given new_value.
           For a non-gradual LED, the value is the index of the current color from the LED's list of possible colors.
           For a monochromatic (gradual single-colored) LED, the value is the intensity ranging 0..255.
           For an RGB LED (gradual, no preset colors) the value is a 24bit integer representation of its color.
           Note that led.color would return an (often more useful) Color vector representation of this color.
           When setting `led.value = new_value` for an RGB LED, the new_value may be a 24-bit integer like 0xFFFFFF,
           or a vector-like [red, green, blue] triple with color components ranging 0..1."""
        return wb.wb_led_get(self.tag)
    @value.setter
    def value(self, new_value: Union[int, Iterable3f]):
        wb.wb_led_set(self.tag, Color(new_value if not isinstance(new_value, Iterable) else Color(new_value).hexcolor))

    @use_docstring_as_deprecation_warning
    def get(self) -> int:
        """DEPRECATED.  LED.get() is deprecated. Please use `led.value` or `led.color` instead,
           or for most purposes an LED serves as a surrogate for its value."""
        return wb.wb_led_get(self.tag)

    @use_docstring_as_deprecation_warning
    def set(self, *new_value: Union[int, Iterable3f]):
        """DEPRECATED.  LED.set() is deprecated.  Please use led.value = new_value or led.color=new_color instead."""
        if len(new_value) > 1 or isinstance(new_value, Iterable):
            wb.wb_led_set(self.tag, Color(new_value).hexcolor)
        else:
            wb.wb_led_set(self.tag, new_value[0])

    @property
    def color(self) -> Color:
        """Returns or adjusts the current color of an RGB LED (a gradual LED with no preset colors).
           Other (monochrome/non-gradual) LEDs' integer values would be converted to a bluish .color.
           `led.color` returns a Color vector indicating the three components [red, green, blue] of this LED's
           current color, each ranging 0-1. (Contrast: led.value returns a less easily usable 24-bit integer.)
           `led.color = new_color` adjust the RGB LED's color.  The new_color may be given as a vector-like
           [red, green, blue] triple, each ranging 0-1, or as a 24-bit integer like 0xFFFFFF.
           Note: Color vectors allow access to components as .r/.red, .g/.green and .b/.blue, to vector arithmetic for
           adding/averaging colors, and to a .hexcolor property that returns a 24-bit integer representation of it."""
        return Color(wb.wb_led_get(self.tag))
    @color.setter
    def color(self, new_value: Union[int, Iterable3f, Iterable4f]):
        wb.wb_led_set(self.tag, Color(new_value).hexcolor)


class Pen(Device):
    # Default values, will be overwritten in instances
    _color = Color(0x000000)
    _color_hex = 0x000000
    _density = c_double(1)

    def write(self, writing_or_color: Union[bool, int, Iterable3f, Iterable4f, Callable] = True,
                    density: float = None, color: Union[int, Iterable3f, Iterable4f] = None):
        """Alters whether and/or how this pen will write.
           `pen.write(None)` and `pen.write(False)` make the pen stop writing.
           If the first `writing_or_color` argument is not None/False/any then the pen will start/continue writing. So,
           `pen.write()`, `pen.write(True)`, and `pen.write(new_color)` each make the pen start/continue writing.
           Ink color may be changed by passing a color as the first `writing_or_color` arg, or with `color =` keyword.
           Colors may be given as 24-bit integers like `0xFFFFFF`, or more often as vector-like sequences of the
           form `[red, green, blue]` or `[red, green, blue, density]` with each component ranging 0-1.
           Ink density is altered by this vector's 4th component, if included, or by the `density =` keyword.
           NOTE: Webots is currently constrained to alter ink color and density together, and non-supervisor controllers
           typically won't have access to the default values built into the simulation.  So, if you want to adjust
           either color or density, and don't want the other to become Python's default opaque/black, then you'll need
           to set the other at least once, after which the Python controller will automatically remember and re-use
           the last setting whenever that characteristic is not explicitly specified in later commands.
           NOTE: If you want to just alter color/density without altering whether a pen is writing, use
           `pen.set(color=new_color, density=new_density)`"""

        if writing_or_color in (None, False):
            wb.wb_pen_write(self.tag, 0)  # stop writing
        elif writing_or_color is not any:  # `any` means to keep any current setting for whether the pen is writing
            wb.wb_pen_write(self.tag, 1)  # start writing
            if color is None and writing_or_color is not True: color = writing_or_color
        if color is not None or density is not None: self.set(color, density)

    def set(self, color: Union[int, Iterable3f, Iterable4f] = None, density: float = None):
        """Alters this pen's ink color and/or density, but not whether it is writing. (Use `pen.write()` for that.)
           The pen's ink-color may be changed by passing a color as the first argument, or as the `color =` keyword.
           Colors may be specified as 24-bit integers like `0xFFFFFF`, or more often as vector-like sequences
           of the form `[red, green, blue]` or `[red, green, blue, density]` with each component ranging 0-1.
           The pen's density is altered by this vector's 4th component, if included, or by the `density =` keyword.
           NOTE: Webots is currently constrained to alter ink color and density together, and the python controller
           typically won't have access to the default values built into the simulation.  So, if you want to adjust
           either color or density, and don't want the other to become Python's default opaque/black, then you'll need
           to set the other at least once, after which the Python controller will automatically remember and re-use
           the last setting whenever that characteristic is not explicitly specified in later commands."""
        if color is not None:  # if given a color, coerce to Color, cache it, and infer density from alpha channel
            color = self.__dict__['color'] = Color(color)
            if density is None and len(color) == 4: density = color[3]
        else:                  # if not given a color, use superpowers/cache/default to guess what color to use
            color = self.color
        if density is not None:  # if given a density, cache it
            self.__dict__['density'] = density
        else:                    # if not given a density, use superpowers/cache/default to guess what density to use
            density = self.density
        wb.wb_pen_set_ink_color(self.tag, color.hexcolor, c_double(density))

    @use_docstring_as_deprecation_warning
    def setInkColor(self, color: Union[int, Iterable3f, Iterable4f], density: float):
        """DEPRECATED: Pen.setInkColor() is deprecated. Please use pen.set(color=new_color, density=new_density)."""
        self.set(color, density)

    @property
    def color(self) -> Color:
        """`pen.color` returns the best estimate of the pen's current inkColor, though Webots provides no way for
           ordinary robots to directly read this.  If this controller has already set the color, the last set value
           will be returned.  Otherwise, if world has been imported, `pen.color` uses supervisor powers to
           read the pen node's `inkColor` field in the simulation.  Otherwise the default black is returned.
           `pen.color = [red, green, blue]` sets this pen's ink color to the given color, with components ranging 0-1.
           Webots currently requires that density be specified whenever color is altered.  If a 4th alpha color
           component is included, the pen's density will be set to that. Otherwise this will use pen.density to form
           a best guess as to density (again using cache/world/default).
           `pen.color = 0xFFFFFF` sets this pen's color to that 24-bit color, again using the best guess for density."""
        if 'color' in self.__dict__: return self.__dict__['color']
        if self._world is not None:  # If we have supervisor powers, we can read the correct field value
            return Color(self._world.Node(self).inkColor)
        return Color(0)
    @color.setter
    def color(self, new_color):
        self.set(new_color)

    @property
    def density(self) -> float:
        """Reading `pen.density` returns the best estimate of the pen's current inkDensity, though Webots does not make
           this information directly available to ordinary robots. This returns the last explicitly set density if
           it has been set, or, if world has been imported, it uses supervisor powers to read the inkDensity field of
           the pen node, or returns the default opaque 1.
           `pen.density = new_density` sets this pen's ink density, ranging from 0 (transparent) to 1 (opaque).
           NOTE: Webots is currently constrained to alter ink color and density together, so whenever you alter
           density alone, the python controller will use its best estimate of color (again using cache/world/default).
           It is generally advisable to initially set both with a single command like `pen.set(color, density)`,
           `pen.write(color, density)` or `pen.color = [r, g, b, density]`. The Python controller automatically
           remembers and re-uses these settings when you later alter just one."""
        if 'density' in self.__dict__: return self.__dict__['density']
        if self._world is not None:  # If we have supervisor powers, we can read the correct field value
            return self._world.Node(self).inkDensity
        return 1
    @density.setter
    def density(self, new_density):
        self.set(density=new_density)


class Skin(Device):
    """A python Skin object is used to control a skin node in the webots simulation, which allows soft mesh animation
       for example of a human or an animal.
       Much of the skin's structure is set in the simulation file itself.  This controller has access only to the bones.
       Hence, in Python, a Skin Device serves as a list-like container of its component Bones.
         `for bone in skin` iterates through the bones
         `skin[index]` retrieves the bone with that index
         `len(skin)` returns the number of bones
       Each Bone has the following properties:
         `bone.name` reads the bone's name (read-only)
         `bone.translation_from_parent` reads/assigns the bone's position relative to its parent bone (3D vector)
         `bone.translation_from_skin` reads/assigns the bone's position relative to this Skin device (3D vector)
         `bone.rotation_from_parent` reads/assigns the bone's rotation relative to its parent bone (4D vector)
         `bone.rotation_from_skin` reads/assigns the bone's rotation relative to this Skin device (4D vector)"""

    class Bone:
        __slots__ = ('tag', 'index')
        def __init__(self, skin, index):
            self.tag = skin.tag
            self.index = index

        wb.wb_skin_get_bone_name.restype = c_char_p
        @property
        def name(self) -> str:
            """The name of this bone."""
            return wb.wb_skin_get_bone_name(self.tag, self.index).decode()

        wb.wb_skin_get_bone_position.restype = c_void_p
        @property
        def translation_from_parent(self) -> Vec3f:
            """Reads/assigns the bone's position relative to its parent bone (3D vector)"""
            return Vec3f.from_address(wb.wb_skin_get_bone_position(self.tag, self.index, 0))  # 0 = "non-absolute"
        @translation_from_parent.setter
        def translation_from_parent(self, new_value: Iterable3f):
            wb.wb_skin_set_bone_position(self.tag, self.index, Vec3f(new_value), 0)  # 0 = "non-absolute" position

        @property
        def translation_from_skin(self) -> Vec3f:
            """Reads/assigns the bone's position relative to this Skin device (3D vector)"""
            return Vec3f.from_address(wb.wb_skin_get_bone_position(self.tag, self.index, 1))  # 1 = "absolute" position
        @translation_from_skin.setter
        def translation_from_skin(self, new_value: Iterable3f):
            wb.wb_skin_set_bone_position(self.tag, self.index, Vec3f(new_value), 1)  # 1 indicates "absolute" position

        wb.wb_skin_get_bone_orientation.restype = c_void_p
        @property
        def rotation_from_parent(self) -> Vec4f: # TODO eventually make this a Rotation
            """Reads/assigns the bone's rotation relative to its parent bone (4D vector)"""
            return Vec4f.from_address(wb.wb_skin_get_bone_orientation(self.tag, self.index, 0))  # 0 = "non-absolute"
        @rotation_from_parent.setter
        def rotation_from_parent(self, new_value: Iterable4f):
            wb.wb_skin_set_bone_orientation(self.tag, self.index, Vec4f(*new_value), 0)  # 0 = "non-absolute" orientation

        @property
        def rotation_from_skin(self) -> Vec4f:
            """Reads/assigns the bone's rotation relative to this Skin device (4D vector)"""
            return Vec4f.from_address(wb.wb_skin_get_bone_orientation(self.tag, self.index, 1))  # 1 = "absolute" orientation
        @rotation_from_skin.setter
        def rotation_from_skin(self, new_value: Iterable4f):
            wb.wb_skin_set_bone_orientation(self.tag, self.index, Vec4f(*new_value), 1)  # 1 = "absolute" orientation

    # --- skin container interface ---

    def __len__(self) -> int:
        """len(skin) returns the number of bones in this skin."""
        return wb.wb_skin_get_bone_count(self.tag)

    def __getitem__(self, index: int) -> 'Skin.Bone':
        """skin[i] returns the bone in this skin device with index i."""
        return Skin.Bone(self, index)

    def __iter__(self) -> Iterable['Skin.Bone']:
        """Iterating over a skin device iterates over its component bones, lowest index to highest."""
        return (Skin.Bone(self, i) for i in range(len(self)))

    def __reversed__(self) -> Iterable['Skin.Bone']:
        """Iterating over a skin device in reverse iterates over its component bones, highest to lowest."""
        return (Skin.Bone(self, i) for i in reversed(range(len(self))))

    # --- skin deprecated methods ---

    @use_docstring_as_deprecation_warning
    def getBoneCount(self) -> int:
        """DEPRECATED: Skin.getBoneCount() is deprecated. Please use len(skin) instead."""
        return wb.wb_skin_get_bone_count(self.tag)

    @use_docstring_as_deprecation_warning
    def getBoneName(self, index: int) -> str:
        """DEPRECATED: skin.getBoneName(index) is deprecated. Please use skin[index].name instead."""
        return self[index].name

    @use_docstring_as_deprecation_warning
    def getBoneOrientation(self, index: int, absolute: bool) -> Vec4f:
        """DEPRECATED: Skin.getBoneOrientation() is deprecated. Please use skin[bone_index].rotation_from_skin
           or skin[bone_index].rotation_from_parent instead."""
        return wb.wb_skin_get_bone_orientation(self.tag, index, absolute)

    @use_docstring_as_deprecation_warning
    def setBoneOrientation(self, index: int, orientation: Iterable4f, absolute: bool):
        """DEPRECATED: Skin.getBoneOrientation() is deprecated. Please use skin[bone_index].rotation_from_skin = new_r
           or skin[bone_index].rotation_from_parent = new_r instead."""
        return wb.wb_skin_set_bone_orientation(self.tag, index, Vec4f(*orientation), absolute)

    @use_docstring_as_deprecation_warning
    def getBonePosition(self, index: int, absolute: bool) -> Vec3f:
        """DEPRECATED: Skin.getBonePosition() is deprecated. Please use skin[bone_index].translation_from_skin
           or skin[bone_index].translation_from_parent instead."""
        return Vec3f.from_address(wb.wb_skin_get_bone_position(self.tag, index, absolute))

    @use_docstring_as_deprecation_warning
    def setBonePosition(self, index: int, position: Iterable3f, absolute: bool):
        """DEPRECATED: Skin.getBonePosition() is deprecated. Please use skin[bone_index].translation_from_skin = new_p
           or skin[bone_index].translation_from_parent = new_p instead."""
        return wb.wb_skin_set_bone_position(self.tag, index, Vec3f(*position), absolute)


class Speaker(Device):

    # --- speaker playing sound files ---

    def play(self, sound_path: str, volume: float = 1.0, pitch: float = 1.0, balance: float = 0, loop: bool = False):
        """Plays a sound file on this speaker.
           `speaker.play(sound_path)` plays from a single Speaker in mono, whereas
           `(left_speaker & right_speaker).play(sound_path)`, plays from a Speaker.Pair in stereo.
           Otherwise these two .play methods work the same.
           `sound_path` is required, the path to the sound file to be played, often in the controller's own folder.
           Keyword `volume` indicates the fraction of full volume to play (defaults to 1 = full volume).
           Keyword 'pitch' is a multiplier on pitch (default 1), making sounds lower or higher pitched.
           Keyword 'balance' ranges from -1 (left speaker only) through the default 0 (even) to +1 (right speaker only).
           Keyword `loop = False` plays the sound once (the default), `loop = True` plays it repeatedly."""

        return wb.wb_speaker_play_sound(self.tag, self.tag, sound_path.encode(),
                                        c_double(volume), c_double(pitch), c_double(balance), c_bool(loop))

    @staticmethod
    @use_docstring_as_deprecation_warning
    def playSound(left, right, sound_path: str,
                  volume: float = 1.0, pitch: float = 1.0, balance: float = 0, loop: bool = False):
        """DEPRECATED: Speaker.playSound(left_speaker, right_speaker, ...) is deprecated.
           Please use (left_speaker & right_speaker).play(...) for stereo, or speaker.play(...) for mono."""
        Speaker.Pair(left, right).play(sound_path, volume=volume, pitch=pitch, balance=balance, loop=loop)

    def stop_playing(speaker, sound_path: str):
        """`speaker.stop_playing(sound)` makes an individual speaker stop playing the given sound.
           `(left & right).stop_playing(sound)` makes a pair of speakers stop playing that sound."""
        wb.wb_speaker_stop(speaker.tag, sound_path.encode())
    stop = stop_playing  # for backwards compatibility, but I didn't explicitly deprecate it

    def is_playing(self, sound_path: str) -> bool:
        """`speaker.is_playing(sound)` returns True if the given sound file is playing from that speaker
           `(left & right).are_playing(sound)` returns True if the given sound file is playing from both speakers"""
        return wb.wb_speaker_is_sound_playing(self.tag, sound_path.encode())
    @use_docstring_as_deprecation_warning
    def isSoundPlaying(self, sound_path: str) -> bool:
        """DEPRECATED: speaker.isSoundPlaying(sound) is deprecated. Please use speaker.is_playing(sound) instead."""
        return wb.wb_speaker_is_sound_playing(self.tag, sound_path.encode())

    class Pair:
        def __init__(self, *args: Union['Speaker', int, str, Tuple['Speaker','Speaker']]):
            """A Speaker.Pair is created by conjoining `speaker1 & speaker2`, enabling `(left & right).play(sound)`
               An alternative constructor is `Speaker.Pair(id1, id2)` where the id's are names or indices of speakers."""
            if len(args) == 1: args = args[0]  # treat single arg as an iterable of args like [left, right]"""
            self.tags = [arg.tag if isinstance(arg, Speaker) else Speaker(arg).tag for arg in args]

        def play(self, sound_path: str,
                 volume: float = 1.0, pitch: float = 1.0, balance: float = 0, loop: bool = False):
            """Plays the given sound on this pair of speakers.  See Speaker.play for more documentation."""
            wb.wb_speaker_play_sound(*self.tags, sound_path.encode(),
                                     c_double(volume), c_double(pitch), c_double(balance), c_bool(loop))

        def stop_playing(self, sound_path: str):
            """Stops playing the given sound file on this pair of speakers."""
            sound_path_encoded = sound_path.encode()
            for tag in self.tags: wb.wb_speaker_stop(tag, sound_path_encoded)
        stop = stop_playing

        def are_playing(self, sound_path: str) -> bool:
            """Returns True if the given sound file is playing on both of these speakers."""
            sound_path_encoded = sound_path.encode()
            return all(wb.wb_speaker_is_sound_playing(tag, sound_path_encoded) for tag in self.tags)
        is_playing = are_playing

    def __and__(self, other) -> 'Speaker.Pair':
        """Conjoining `(speaker1 & speaker2)` returns a Speaker.Pair with own .play, .stop, and .are_playing methods.
           As a consequence, `(speaker1 & speaker2).play(sound)` will play sound in stereo from these two speakers."""
        return Speaker.Pair(self, other)
    def __rand__(self, other): return Speaker.Pair(other, self)

    # --- speaker text-to-speech ---

    wb.wb_speaker_get_engine.restype = c_char_p
    wb.wb_speaker_set_engine.restype = c_bool
    @property
    def engine(self) -> str:
        """Returns or adjusts the current text-to-speech engine of this Speaker.
           In the case where Webots fails to set this engine, a ValueError is raised."""
        return wb.wb_speaker_get_engine(self.tag).decode()
    @engine.setter
    def engine(self, new_engine: str):
        success = wb.wb_speaker_set_engine(self.tag, new_engine.encode())
        if not success:
           raise ValueError("Webots failed to set the requested text-to-speech engine.")
    @use_docstring_as_deprecation_warning
    def getEngine(self) -> str:
        """DEPRECATED: Speaker.getEngine() is deprecated. Please use speaker.engine instead."""
        return wb.wb_speaker_get_engine(self.tag).decode()
    @use_docstring_as_deprecation_warning
    def setEngine(self, new_engine: str):
        """DEPRECATED: Speaker.setEngine(e) is deprecated. Please use speaker.engine = e instead,
           but note that this raises ValueError, rather than returning False, when setting fails."""
        return wb.wb_speaker_set_engine(self.tag, new_engine.encode())

    wb.wb_speaker_get_language.restype = c_char_p
    wb.wb_speaker_set_language.restype = c_bool
    @property
    def language(self) -> str:
        """Returns or adjusts the current language of this Speaker.
           In the case where Webots fails to set this language, a ValueError is raised."""
        return wb.wb_speaker_get_language(self.tag).decode()
    @language.setter
    def language(self, new_language: str):
        success = wb.wb_speaker_set_language(self.tag, new_language.encode())
        if not success:
            raise ValueError("Webots failed to set the requested language for the current text-to-speech engine.")
    @use_docstring_as_deprecation_warning
    def getLanguage(self) -> str:
        """DEPRECATED: Speaker.getLanguage() is deprecated. Please use speaker.language instead."""
        return wb.wb_speaker_get_language(self.tag).decode()
    def setLanguage(self, language: str):
        """DEPRECATED: Speaker.setLanguage(new_language) is deprecated. Please use speaker.language = new_language,
           but note that this raises ValueError, rather than returning False, when setting fails."""
        return wb.wb_speaker_set_language(self.tag, language.encode())

    def speak(self, text: str, volume: float = 1.0):
        """Makes this speaker pronounce the given text at the given volume (default 1.0)."""
        wb.wb_speaker_speak(self.tag, text.encode(), c_double(volume))

    wb.wb_speaker_is_speaking.restype = c_bool
    @property
    def is_speaking(self) -> bool:
        """Returns True if this speaker is currently speaking text-to-speech."""
        return wb.wb_speaker_is_speaking(self.tag)
    @use_docstring_as_deprecation_warning
    def isSpeaking(self) -> bool:
        """DEPRECATED: Speaker.isSpeaking() is deprecated. Please use speaker.is_speaking instead."""
        return wb.wb_speaker_is_speaking(self.tag)



# === Displays and Brushes ===


class MetaBrush(type):
    """Brushes use Python class inheritance for augmented versions of brushes to dynamically inherit attributes from
       their ancestors.  Within brush subclasses, it makes sense to have the attribute lookup order (aka "method
       resolution order" or "MRO") be fully depth-first, rather than using Python's default C3 algorithm.
       Suppose thick_red inherits its color from red, and dark_red also inherits from red but overwrites its color.
       `new_red = dark_red(thick_red)` is supposed to produce a brush that overrides dark_red's settings with
       thick_red's, and it does so by having new_red have superclasses [thick_red, dark_red].  This is a diamond
       inheritance pattern with two paths from new_red up to red.  Python's default C3 MRO prioritizes the second path
       (dark_red) over the tip of the diamond (red), so would wrongly make new_red inherit dark_red's dark color,
       whereas we want new_red to prioritize any setting that the first (thick_red) path defines, including the color
       it gets from red.  So we need an MRO that goes depth-first (straight up the first path to red), rather than C3's
       both-sides-before-the-tip.  Customizing MRO's requires a metaclass, so here we are.
       MRO's are supposed to be setlike -- no repetitions -- but ordered, so we store the info as a dict, in each
       class's .__mrodict__ attribute, since that is Python's most efficient ordered setlike data structure.
       Subclasses prioritize themselves, then go depth-first through Brush subclasses, finally Brush, object."""

    def mro(cls):
        if object in cls.__bases__:  # Should occur just for the generic Brush class itself
            cls.__mrodict__ = {}  # dict versions always leave out Brush and object; saving them for very last
            return super().mro()
        elif Device in cls.__bases__:  # Should occur just for Display (or perhaps user subclasses thereof)
            # we mostly defer to Python's default mro, but save a dict version in case subclasses need it
            default_mro = super().mro()
            cls.__mrodict__ = {k: 0 for k in default_mro if k not in (Brush, object)}  # leaves Brush, object for last
            return default_mro
        # Otherwise, this should be a dynamically generated subclass of Brush
        d = cls.__mrodict__ = {cls: 0}  # our top priority is this class itself
        for base in cls.__bases__: d.update(base.__mrodict__)  # depth-first through Brush superclasses
        list_version = list(d)
        list_version += [Brush, object]  # Brush and object are always the last priorities
        return list_version

    def update(cls, dic):
        """Works much like dict.update() except updating this class's own __dict__.  (Unfortunately Python class
           __dict__'s are read-only, so we can't just use the ordinary dict.update().)"""
        for k,v in dic.items(): setattr(cls, k, v)

    def __repr__(cls):
        own_settings = cls.own_settings
        all_settings = cls.settings
        inherited_settings = {k:v for k,v in all_settings.items() if k not in own_settings}
        if inherited_settings:
            return f"BrushType:{cls.__name__}(own={own_settings}, inherited={inherited_settings})"
        return f"BrushType:{cls.__name__}({own_settings})"

class Brush(metaclass=MetaBrush):
    # These pseudo-attributes are handled by __getattr__ and __setattr__, declared here for linters
    color: Color
    alpha: float
    opacity: float
    offset: Vector
    thickness: Vector
    hardness: float
    font: str
    fontsize: int
    antialiasing: bool
    weight = 1.0

    layers = None  # will be overriden with [] in first call to __init__

    default_settings = dict(color=Color(1,1,1), alpha=1, opacity=1,
                            offset=Vector(0, 0), thickness=Vector(1, 1), hardness=1,
                            font="Lucida Console", fontsize=8, antialiasing=True)

    def __init__(self, display: 'Display', weight: float=1):
        """A brush contains settings for Display drawing-attributes like color, alpha, opacity, thickness and offset,
           each of which may be set by the corresponding optional keyword argument in brush creation.
           Brushes can be created with `newbrush = display(...)`, or more generally with `newbrush = oldbrush(...)`
           which produces a new brush like oldbrush, except the given new settings take precedence over old ones.
           Each brush has Webots drawing commands as methods, so `brush.oval(center, radii)` will draw an oval
           using this brush's settings, e.g. for color and thickness.
           Brushes have dynamically created brushtypes and use Python class inheritance to govern inheritance of
           brush settings (though every brush has its own `offset`, usually `Vector(0,0)`, so this is never inherited,
           and the method resolution order (mro) for brush subclasses is depth-first unlike Python's default).
           The above-noted `newbrush = oldbrush(...)` creates a new brushtype that is a subtype of oldbrush's.
           The new type stores its explicitly given settings, and inherits other settings from the oldbrush superclass.
           If that superclass' settings later change (e.g. due to commands like `oldbrush.set(...)` or
           `oldbrush.color = [1,0,0]`) those changes will automatically be inherited by newbrush, unless overriden.
           Each brush token is tied to the Display from which it was (directly or indirectly) created. This is the
           display the brush's draw methods like .oval would affect. You can use `display2(brush1)` to produce a
           new brush token of the same brushtype as brush1 that is instead tied to display2.
           Each brush token also has its own weight (typically 1), which serves as a multiplier on its opacity and
           as its weight in weighted sums.  `brush/2` and `0.5*brush` each produce a new token of brush's type,
           but with weight 0.5, and hence half the apparent opacity.
           Old brushes can be combined in 3 ways to make new brushes:
             1. Augmentation. `brush3 = brush1(brush2)` produces a new brush like brush1, except augmented by the
                settings of brush2, which take precedence over those of brush1.  This creates a brush3 class with
                brush2's and brush1's classes as superclasses. Brush3 automatically inherits changes to brushes 1-2.
                In augmentation, you may give multiple brush arguments (with later ones taking precedence) and/or
                specify individual settings with keyword arguments as in brush creation (even higher precedence).
             2. Concatenating. `shadow & red` produces a new multibrush, whose draw commands will be executed twice,
                first employing shadow, then employing red (useful, for example for putting offset drop shadows under
                text or drawings), and later changes to either component are automatically inherited in the multibrush.
             3. Adding/mixing. `1/4*red + 3/4*blue` produces a new indigo brush. Unlike augmentation and concatenation,
                adding/mixing brushes with `+` produces an independent new brush class that will not automatically
                inherit later changes to either component, as recomputing mixes on the fly would have been costly."""
        self.display = display
        self.weight = weight

    def __repr__(self):
        own_settings = self.own_settings
        all_settings = self.settings
        inherited_settings = {k:v for k,v in all_settings.items() if k not in own_settings}
        if 'display' in self.__dict__: own_settings['display'] = self.display
        if 'weight' in self.__dict__: own_settings['weight'] = self.weight
        if inherited_settings:
            return f"{type(self).__name__}(own={own_settings}, inherited={inherited_settings})"
        return f"{type(self).__name__}({own_settings})"

    class OwnSettingsDescriptor:
        """brush.own_settings and BrushType.own_settings each return a dictionary of settings made by this BrushType,
           e.g. {thickness:3, opacity:1}, not including any settings inherited from supertypes or display defaults."""
        # We need to manually construct this descriptor, rather than using an @property, to get it to work for class too
        def __get__(self, instance, owner):
            # TODO double check that Python 3 always does send owner
            my_dic = owner.__dict__
            # TODO not sure if I should screen out None values?
            return {k: my_dic[k] for k in Brush.default_settings if k in my_dic}
    own_settings = OwnSettingsDescriptor()

    class SettingsDescriptor:
        """brush.settings and BrushType.settings each return a dictionary of settings made by this BrushType or
           inherited from supertypes, e.g. {thickness:3, opacity:1}, but does not include the default settings that
           would be borrowed from a Display when this brush is used."""
        # We need to manually construct this descriptor, rather than using an @property, to get it to work for class too
        def __get__(self, instance, owner):
            settings_with_extra_nones = {k: getattr(owner, k, None) for k in Brush.default_settings}
            return {k: v for k, v in settings_with_extra_nones.items() if v is not None}
            # TODO If we shift our requirements to Python 3.8+, the following one-liner using := is more efficient:
            #  self_settings = {k:v for k in Brush.default_settings if (v:=getattr(self, k, None)) is not None}
    settings = SettingsDescriptor()


    def copy(self: Union['Brush', type], display=None, weight=None):
        """This can be called as brush.copy() or as Brush.copy(brush_or_brushtype).  Returns a new token of the same
           brushtype as the given argument (which may be a brush or a brushtype), but with given display (defaults
           to the copied brush's display) or weight (defaults to 1)."""
        brushtype = type(self) if isinstance(self, Brush) else self
        if display is None: display = getattr(self, 'display', None)
        if weight is None: weight = 1  # Some other functions may explicitly pass None so we need to legalize this
        return brushtype(display, weight)

    def copy_type(self: Union['Brush', type], name: str = None):
        """This can be called as brush.copy_type() or as Brush.copy_type(brush_or_brushtype)
           Returns a new dynamically generated brushtype that is a duplicate of the argument's (which may be
           given as a brush or brushtype).
           The new brushtype's .__name__ will be name if given, otherwise, the old class's."""
        if isinstance(self, Brush): self = type(self)
        return type(name or self.__name__, self.__bases__, self.__dict__.copy())

    # --- brush creation, augmentation, and setting

    # __getattr__ is not needed, since brush settings are stored as attributes of brushtype, so inherited by brush
    def __setattr__(self, key, value):
        """This makes brush settings assignable like brush attributes, e.g. brush.opacity = 1.
           Other attributes are set normally.  Note that brush settings are stored in brushtypes,
           so changing a brush's settings also changes those of copies of that brush (e.g. ones with different weight,
           or linked to a different display), and potentially of other brushes that inherit from this one.
           Multiple attributes may be set at once using keywords in `brush.set(...)`"""
        if key in Brush.default_settings:
            if self.layers:
                raise AttributeError(f"Multibrushes (like A&B) do not have attributes like `{key}` to set. "
                                     f"You could instead set this attribute of each component layer.")
            if key == 'color': value = Color(value)
            if key in ('offset', 'thickness'):
                if isinstance(value, (int, float)):
                    value = Vector(value, value)
                else:
                    value = Vector(value)
            setattr(type(self), key, value)
        else:
            super().__setattr__(key, value)

    @staticmethod
    def process_settings(settings: dict) -> dict:
        """Alters the given dictionary of keyword settings to legalize their values, convert to appropriate types,
           and to split 4th color channel to alpha. Settings, thus altered, is returned."""
        color = settings.get('color')
        if color is not None:
            color = Color(color)
            if len(color) == 4:
                settings['color'], settings['alpha'] = Color(color[0:3]), color[3]
            else:
                settings['color'] = color

        offset = settings.get('offset')
        if offset is None:
            settings['offset'] = Vector(0, 0)  # all brushes must have offset, to sum in full_offset
        elif isinstance(offset, Sequence):
            settings['offset'] = Vector(offset)
        else:
            settings['offset'] = Vector(offset, offset)  # offset -2 means [-2,-2]

        thickness = settings.get('thickness')
        if thickness is not None:
            if isinstance(thickness, Sequence):
                settings['thickness'] = Vector(thickness)
            else:
                settings['thickness'] = Vector(thickness, thickness)

        return settings

    def __call__(self, *brushes: 'Brush',
                       name: str = None,
                       display: 'Display' = None,
                       color: Union[int, Iterable3f] = None,
                       alpha: float = None,
                       opacity: float = None,
                       weight: float = None,
                       thickness: Union[float, Iterable2f] = None,
                       offset: Union[int, Tuple[int,int]] = None,
                       font: str = None,
                       fontsize: int = None,
                       antialiasing: bool = None,
                       _settings: dict = None) -> 'Brush':
        """Any brush, including a Display object itself, can be called like a function to return an augmented version
           of that brush, which will inherit its old settings, overriding them with any explicitly given new settings.
           New settings may be given as zero or more Brush arguments or as optional keyword settings.
           E.g. `thin_blue(thick)` would yield a thick blue brush (keeping thin_blue's color, overwriting thickness).
           E.g., `brush4 = brush1(brush2, brush3, opacity=1)` returns a member of a new brush class that explicitly
           sets opacity = 1, and inherits other settings from brush3's class, then brush2's, and finally brush1's.
           As a rule of thumb, items in this call override items to their left, so, e.g., brush2 settings take
           precedence over brush1 settings, but would be superceded by brush3 settings and/or given keywords.
           Later changes to any of these superclasses (e.g. via `brush1.set(...)` or `brush3.alpha = 0.5`) will
           automatically be inherited by brush4, except when superceded by one or more items to the right.
           This "rightmost wins" rule applies to .weight as well, even though that is stored on instances not types.
           However this rule does not apply to .display: `brush1(brush2)` will draw to brush1's display, not brush2's.
           Every brush uses its display's own settings as the default for any setting the brush (and its superclasses)
           doesn't specify. `display(...)` creates a brush that is not actually a Display subclass, but still
           does use its display as the last resort to supply default values for settings.
           If only a brush's weight and/or display are augmented, e.g. '`display(brush)` or `brush(weight=1)` then
           a new token of the existing brushtype will be returned. (display and weight are stored on tokens, not types.)
           See Brush documentation for more about brushes, including how they can be used to initiate draw commands
           like brush.oval(center, radii) or other ways they can be combined to make mixed and multi-brushes."""
        if _settings is not None:  # if given a pre-processed _settings dict, we'll just use it
            settings = _settings
        else:  # otherwise we'll process the given keywords to be our settings
            ## We could filter explicit settings from locals() by screening out Nones and non-setting vars
            # loc = locals()
            # settings = {k: loc[k] for k in Brush.default_settings if loc[k] is not None}
            ## But we'll assume that all locals (but a few we'll pop) are settings worth keeping, so just filter Nones
            settings = {k: v for k, v in locals().items() if k != 'self' and v is not None}
            brushes = settings.pop('brushes', ())
            weight = settings.pop('weight', None)
            if weight is None: weight = brushes[-1].weight if brushes else self.weight  # "rightmost wins"
            display = settings.pop('display', self.display)
            # TODO would probably be more efficient to overwrite this function to just capture given keywords as **settings
            #  For now, I'll stick with the more explicit keywords specifications for linting purposes, though, at least
            #  in Pycharm, it works to first define the function the way you want linters to see it, then override it with
            #  how you want it to actually work.
            settings = Brush.process_settings(settings)

        self_is_display = isinstance(self, Display)

        # If there are no explicit new settings, we may not need to generate a new brushtype.
        if not settings:
            if self_is_display:  # display(brush) creates another token of that type, tied to this display
                if len(brushes)==1:
                    if weight is None: weight = brushes[0].weight
                    return brushes[0].copy(display=self, weight=weight)
            elif not brushes:  # brush() with only token-specific settings like weight= or display=
                if weight is None: weight = self.weight
                return self.copy(display=display, weight=weight)
        # Otherwise we actually will need to generate a new brushtype for the augmented brush

        # If self is a multibrush, e.g., A&B(...), need to distribute augmentations over each of self's layers
        if self.layers:
            new_layers = (my_layer(*brushes, _settings=settings, weight=weight) for my_layer in self.layers)
            return MultiBrush(new_layers, display=self.display)
        # Otherwise, self must be a simple brush.

        # If self is a simple brush being augmented by a multibrush, e.g. A(B&C), we'll distribute its layers
        if any(brush.layers for brush in brushes):
            if len(brushes) != 1:
                raise SyntaxError("When augmenting a brush with a multibrush, only 1 augmenting brush can be given.")
                # TODO It'd be a pain to generalize this to handle multiple multis or brushes with lower/higher priority
            new_types = (type("BrushLayer", (type(layer), type(self)), settings) for layer in brushes[0].layers)
            new_layers = (new_type(self.display, weight=weight) for new_type in new_types)
            return MultiBrush(new_layers, name=name or "MultiBrush", display=self.display)

        # Otherwise, this is just a simple/simple case with no multi-brushes involved at all.
        # The output will inherit from all given brushes and then my own brushtype, except pseudo-inheritance of default
        # settings from displays is handled separately, so display(...) subclasses from given brushes or Brush itself.
        last_superclass_if_any = (self,) if not self_is_display else (Brush,) if not brushes else ()
        chain = itertools.chain(reversed(brushes), last_superclass_if_any)
        superclasses = tuple(type(brush) if isinstance(brush, Brush) else brush for brush in chain)
        name = name or ("CustomBrush" if self_is_display else "AugmentedBrush")
        new_subclass = type(name, superclasses, settings)
        return new_subclass(self.display, weight=weight)

    def set(self, *brushes: 'Brush',
                  display=None,
                  color: Union[int, Iterable3f] = None,
                  alpha: float = None,
                  opacity: float = None,
                  weight: float = None,
                  thickness: Union[float, Iterable2f] = None,
                  offset: Union[int, Tuple[int,int]] = None,
                  font: str = None,
                  fontsize: int = None,
                  antialiasing: bool = None,
                  _settings: dict = None) -> 'Brush':
        """Changing a brush's settings with brush.set(...) is much like producing an augmented version of it
           with brush(...).  Both return a brush that is like the original, but revised to match the given brushes
           and/or keyword arguments. The differences are (1) that brush1.set(...) alters the settings of brush1's
           brushtype itself, and hence of all tokens and descendants of that type, rather than producing a new subtype,
           (2) brush1.set(brush2) simply copies brush2's current settings, but does not create any lasting dynamic
           inheritance relation between them, (3) brush1.set(...) overwrites brush1's own offset component with the
           net offset from the new brushes and settings, which may not be the same offset that augmentation produces,
           since augmentation keeps brush1's original offset in the sum and inherits offset components from
           superclasses in ways that may be redundant with contributions from the augmenting brushes,
           and (4) `A(B&C)` can augment brush A with a single multibrush, yielding a new multibrush, whereas
           `A.set(B&C)` would raise a TypeError, as it can't transform a simple brush into a multibrush.
           In the case where brush1 is a multibrush like `shadow & red`, its layers will be replaced with
           tokens of new lookalike types (to leave the originals unchanged in case they are used elsewhere) and then
           each clone will be set to the new settings, as described above.
           Display objects themselves are brushes, whose default drawing settings may be altered by `display.set(...)`.
           This has two important effects. (1) Calls to display.oval(...) employ the display's own default settings.
           (2) When other brushes don't explicitly specify a relevant setting, like color, their draw commands fall
           back on their display's default settings. If you're sure a setting will always or at least often be used in
           a display (e.g. if it is monochromatic) it can be convenient to set the display to always use that setting.
           However, `display.set(...)` can have unwanted lingering effects, so it is generally encouraged to use
           more encapsulated drawing settings, like brushes or single commands like `display(...).oval(...)`.
           Note that you could create a brush with default settings, e.g. default = display(...some settings...),
           and then augmented versions of it will dynamically inherit settings from it, e.g.
           `default(...more settings...).oval(...)` or `brush = default(...more settings...)`."""

        if _settings is not None:  # if given a pre-processed _settings dict, we'll just use it
            settings = _settings
        else:  # otherwise we'll process the given keywords to be our settings
            ## We could filter explicit settings from locals() by screening out Nones and non-setting vars
            # loc = locals()
            # settings = {k: loc[k] for k in Brush.default_settings if loc[k] is not None}
            ## But we'll assume that all locals (but a few we'll pop) are settings worth keeping, so just filter Nones
            settings = {k: v for k, v in locals().items() if k != 'self' and v is not None}
            brushes = settings.pop('brushes', ())
            weight = settings.pop('weight', None)
            if weight is None: weight = brushes[-1].weight if brushes else self.weight  # "rightmost wins"
            display = settings.pop('display', None)
            # TODO would probably be more efficient to overwrite this function to just capture given keywords as **settings
            #  For now, I'll stick with the more explicit keywords specifications for linting purposes, though, at least
            #  in Pycharm, it works to first define the function the way you want linters to see it, then override it with
            #  how you want it to actually work.
            settings = Brush.process_settings(settings)

        if display: self.display = display

        # Compile the new brushes and keyword settings into a single brush to copy settings from
        compiled_brush = self.display(*brushes, _settings=settings)
        if compiled_brush.layers:
            raise TypeError("Brush.set() accepts only simple brushes as arguments, not multibrushes like A&B.")

        # Create a dictionary just of settings that are explicitly affected by compiled_brush (including superclasses)
        explicit_changes = compiled_brush.settings

        if self.layers:  # For a multibrush, we clone each layer and update each clone to the new settings
            new_types = [type("RevisedLayer", type(l).__bases__, type(l).__dict__.copy()) for l in self.layers]
            for t in new_types: t.update(explicit_changes)
            self.layers = [t(display=self.display, weight=weight) for t in new_types]
        else:  # For a simple brush, we copy new settings right into self's brushtype
            if weight is not None: self.weight = weight
            type(self).update(explicit_changes)

        return self

    # --- brush concatenation, weighting and adding ---

    def __and__(self, other: 'Brush') -> 'MultiBrush':
        """`brush1 & brush2` returns a new MultiBrush tied to the display of the first conjunct.
           Draw commands issued from this multibrush will be executed in that display multiple times, first using
           the settings of the first brush, then using the settings of the second, and so on...
           E.g. `(shadow & red).text("Hello world!")` could draw an offset shadow with red text atop it.
           Setting M=A&B produces a multibrush whose layers are A and B themselves, so future changes to either alter M.
           Concatenations of multibrushes e.g., (A&B)&(C&D) are flattened into a single many-layered multibrush.
           Most other operations are distributed across the layers. See docs for MultiBrush for details."""
        return MultiBrush(self, other)

    def __mul__(self, n):
        """Multiplying brush * n yields a similar brush whose weight is n times that of the original.
           Or if the brush is a multibrush, the new brush's new layers will have n times their original weight.
           Brush weight serves as a multiplier on opacity and as its weight in weighted sums."""
        return type(self)(display=self.display, weight = n * self.weight)
    __rmul__ = __mul__  # brush multiplication is commutative

    def __truediv__(self, n):
        """Dividing a simple brush by n yields a similar brush whose weight is 1/n the original brush's weight.
           Or if the brush is a multibrush, the new brush's new layers' weights will be divided by n instead.
           Brush weight serves as a multiplier on opacity, and as its weight in weighted sums."""
        return type(self)(display=self.display, weight=self.weight/n)
    # we don't define __rdiv__ since 1/brush is not meaningful

    def __add__(self, other):
        """`brush1 + brush2` returns a new brush whose settings are a weighted mix of brush1 and brush2. If just one
        of these brushes affects a setting, its setting will be used.  If both affect some setting, their effects
        will be combined, using a weighted average (with each brushes' .weight as its weight) for numerical settings,
        and giving higher priority to the weightier addend for non-numerical settings like font.
        The resulting brush will have all its settings innately, rather than inheriting from any superclasses.
        The resulting brush's .weight will be the sum of the addends' weight, which allows many-term weighted sums
        like `A/3 + B/3 + C/3` or `(A+B+C)/3` to work out as expected.
        Note that an unweighted sum like A+B returns a brush with weight 2, so it would appear with twice the
        average opacity of the components (capped at 1), and it would have heavier-than-normal weight in further sums.
        For purposes of mixing brushes, it is usually advisable to use weighted sums with total weight 1."""

        # If either brush is a multibrush, we'll vectorize, recursively adding particular layers
        if self.layers:  # Adding multibrush (A&B)+...
            if other.layers:  # (A&B)+(C&D) yields (A+C)&(B+D)
                return MultiBrush(layer1 + layer2 for layer1, layer2 in zip(self.layers, other.layers))
            else:  # Adding (A&B)+C yields (A+C)&(B+C)
                return MultiBrush(layer + other for layer in self.layers)
        if other.layers:  # Adding A+(B&C) yields (A+B)&(A+C)
            return MultiBrush(self + layer for layer in other.layers)

        # Otherwise, both self and other must be simple brushes A+B
        self_settings = self.settings
        other_settings = other.settings

        total_weight = self.weight + other.weight
        self_multiplier = self.weight / total_weight
        other_multiplier = other.weight / total_weight

        # Create a new_settings dict combining both their settings, with higher weight taking precedence
        if self.weight > other.weight:
            new_settings = other_settings.copy()
            new_settings.update(self_settings)
        else:
            new_settings = self_settings.copy()
            new_settings.update(other_settings)

        # For each numerical setting in their intersection, we use a weighted average
        for k in self_settings.keys() & other_settings.keys():
            if k != 'font' and k != "antialiasing":
                new_settings[k] = self_settings[k]*self_multiplier + other_settings[k]*other_multiplier
            if k == 'color':
                new_settings[k] = Color(new_settings[k])
                # TODO this may create fractional offsets and sizes. Should round?

        new_subclass = type("MixedBrush", (Brush,), new_settings)
        return new_subclass(self.display, weight=total_weight)

    __radd__ = __add__  # brush addition is commutative

    # --- brush enactment (preparing to draw with a brush) ---

    def full_offset(self, display=None):
        """Returns the full offset of this brush, factoring in the offsets of all ancestors.
           E.g., brush1(rightward, upward, offset=[-1,-1]).full_offset will be the sum of the offsets contributed by
           the superclasses brush1, rightward, and upward, and the explicit keyword offset given here."""
        if display is None: display = self.display
        return sum((brush.offset for brush in type(self).__mro__
                                 if brush is not object and brush is not Brush), display.offset)

    def enact_layers(self, display=None, text=False) -> Iterable[Vector]:
        """This generator will enact the settings for this brush, and then yield one or more offsets for use in
           drawing commands with this brush.  For a simple 1x1 brush, a single such vector will be yielded, indicating
           the full offset of this brush (factoring in offsets from ancestor brushes).
           For simple brushes with larger thickness, the number of yielded values will be multiplied by
           (width + height - 1) of the brush, to simulate a crosshairs-shape brush by redrawing side-by-side.
           For MultiBrushes like shadow&figure, the number of yielded values will be further multiplied by
           the number of layers/conjuncts (handled in that class)."""
        if display is None: display = self.display
        myclass = type(self)  # all settings are stored in class so can slightly speed lookups by going straight there

        # For each relevant setting, (1) we look up that setting in self/ancestors falling back on display default,
        # (2) we check if that desired setting differs from what was last sent to the wb_API,
        # (3a) if so we send an updated setting, and (3b) remember that we sent it
        color = getattr(myclass, 'color', display.color)
        if isinstance(color, Color): color = color.hexcolor
        if color != display.last_sent_settings.color:
            wb.wb_display_set_color(display.tag, color)
            display.last_sent_settings.color = color

        alpha = getattr(myclass, 'alpha', display.alpha)
        if alpha != display.last_sent_settings.alpha:
            wb.wb_display_set_alpha(display.tag, c_double(alpha))
            display.last_sent_settings.alpha = alpha

        opacity = self.weight * getattr(myclass, 'opacity', display.opacity)
        if opacity != display.last_sent_settings.opacity:
            wb.wb_display_set_opacity(display.tag, c_double(opacity))
            display.last_sent_settings.opacity = opacity

        if text:
            font = getattr(myclass, 'font', display.font)
            fontsize = round(getattr(myclass, 'fontsize', display.fontsize))
            antialiasing = c_bool(getattr(myclass, 'antialiasing', display.antialiasing))
            if (   font != display.last_sent_settings.font or
                   fontsize != display.last_sent_settings.fontsize or
                   antialiasing != display.last_sent_settings.antialiasing):
                wb.wb_display_set_font(display.tag, font.encode(), fontsize, antialiasing)
                display.last_sent_settings.font = font
                display.last_sent_settings.fontsize = fontsize
                display.last_sent_settings.antialiasing = antialiasing

        hardness = getattr(myclass, 'hardness', display.hardness)
        # TODO should hardness always be single number? Or vectorized sometimes? Would complexify match test below.

        thickx, thicky = thicknesses = getattr(myclass, 'thickness', display.thickness)
        even = round((thickx - 1) % 2), round((thicky - 1) % 2)  # 1 for even thicknesses, 0 for odd

        offx, offy = offset = self.full_offset(display)
        vecs = [Vector(offx, offy), Vector(offx, offy)]  # one for each crosshair; will modify and yield repeatedly

        x, y = 0, 1  # constants to allow clearer reference to axes

        # Yield the central pixel offset(s) of the brush
        yield vecs[x]  # yield the center of the brush
        if even[x]:    # yield second center pixel of even-width horizontal crosshair
            vecs[x][x] += 1
            yield vecs[x]
        if even[y]:    # yield second center pixel of even-height vertical crosshair
            vecs[y][y] += 1
            yield vecs[y]

        matching = ((thickx-1)//2 == (thicky-1)//2)  # True if x and y have same thickness of periphery
        if matching: thicknesses = (thicknesses[0],)  # We draw matching peripheries together for fewer opacity changes
        # For thicker brushes, we'll yield peripheral offsets on the crosshairs
        for axis, thickness in enumerate(thicknesses):  # x = axis 0, y = axis 1
            if thickness > 2:
                half = int((thickness-1)/2)  # the number of peripheral points to add on each side
                softening = opacity * (1-hardness) / (half*half)  # full opacity at center, hardness at periphery
                for dist in range(1, half+1):
                    if softening:
                        fade = opacity - softening * dist * dist
                        wb.wb_display_set_opacity(display.tag, c_double(fade))
                        display.last_sent_settings.opacity = fade
                    vecs[axis][axis]=offset[axis] + dist + even[axis]  # adjust for fat center on + side of even widths
                    yield vecs[axis]  # yield positive side of the periphery at this distance
                    vecs[axis][axis]=offset[axis]-dist
                    yield vecs[axis]  # yield negative side of the periphery at this distance

                    if matching:  # if the vertical crosshairs match the horizontal can do them now with same opacity
                        vecs[y][y] = offy + dist + even[y]  # adjust for fat center on + side of even widths
                        yield vecs[y]  # yield positive side of the matching vertical periphery at this distance
                        vecs[y][y] = offy - dist
                        yield vecs[y]  # yield negative side of the matching vertical periphery at this distance

    # --- brush drawing methods ---

    @staticmethod
    def generate_points_from(args: Sequence) -> Iterable[Tuple[float, float]]:
        """This generator yields a series of points given in args, where args will have been captured as *args
           for some drawing command, and those args, to which the points may have been given (a) as separate args,
           where each arg is either (a1) a vector-like [x,y] pair, or (a2) an x-value to be paired with the y-value
           given as the next arg; or (b) as a single iterable (e.g. a list) of such items."""
        if len(args) == 1: args = tuple(args[0])  # treat single iterable arg (b) as an (a-like) tuple of args
        it = iter(args)
        for p in it:
            # yield p if it is already a vector-like [x,y], otherwise pair it together with the next arg
            yield p if isinstance(p, Sequence) else (p, next(it))

    @staticmethod
    def generate_successive_pairs_from(items: Iterable) -> Iterable[tuple]:
        """This generator yields a series of successive neighboring pairs from the given sequence of items.
           E.g., if items is [a,b,c], this will first yield (a,b), then (b,c)."""
        last_item = unavailable = object()
        for next_item in items:
            if last_item is not unavailable: yield last_item, next_item
            last_item = next_item

    # TODO should I clamp values within the display?
    # TODO should pixel and line drawings allow an endcap setting?

    def pixel(self, *args: Union[Iterable2f, Iterable[Iterable2f], float, Iterable[float]]):
        """Uses this brush to draw a series of individual pixels at the given point(s). Points may be given as:
             (a1) separate vector-like [x,y] pairs, e.g. `pixel([x1,y1], [x2,y2])`,
             (a2) separate numbers that will be paired, e.g. `pixel( x1,y1, x2,y2 )`,
             (b1) as a single iterable of vector-like pairs, e.g. `pixel( [[x1,y1], [x2,y2]] )`, or
             (b2) as a single iterable of numbers to be paired, e.g., `pixel( [x1,y1, x2,y2] )`
           If this brush has thickness larger than [1,1], then each "pixel" will appear like crosshairs."""
        points = tuple(self.generate_points_from(args))
        for offx, offy in self.enact_layers():  # using this brush's settings to draw at various offsets in its display
            for x, y in points:
                wb.wb_display_draw_pixel(self.display.tag, round(x+offx), round(y+offy))

    def line(self, *args: Union[Iterable2f, Iterable[Iterable2f], float, Iterable[float]]):
        """Uses this brush to draw a line from each given point to the next.  Points may be given as:
             (a1) separate vector-like [x,y] pairs, e.g. `line([x1,y1], [x2,y2])`,
             (a2) separate numbers that will be paired, e.g. `line( x1,y1, x2,y2 )`,
             (b1) as a single iterable of vector-like pairs, e.g. `line( [[x1,y1], [x2,y2]] )`, or
             (b2) as a single iterable of numbers to be paired, e.g., `line( [x1,y1, x2,y2] )`.
           Note: if this brush has thickness larger than [1,1] then additional lines will be drawn nearby
           to simulate a larger brushsize."""
        points = tuple(self.generate_points_from(args))
        for offx, offy in self.enact_layers():  # make this brush's settings be the current drawing settings for this display
            for (x1,y1), (x2,y2) in self.generate_successive_pairs_from(points):
                wb.wb_display_draw_line(self.display.tag, round(offx+x1), round(offy+y1),
                                                          round(offx+x2), round(offy+y2))

    def polygon(self, *vertices: Union[Iterable2f, Iterable[Iterable2f], float, Iterable[float]], filled=False):
        """Uses this brush's settings to draw a polygon with the given vertices. Vertices may be given as:
             (a1) separate vector-like [x,y] pairs, e.g. `polygon([x1,y1], [x2,y2], [x3,y3])`,
             (a2) separate numbers that will be paired, e.g. `polygon( x1,y1, x2,y2, x3,y3 )`,
             (b1) as a single iterable of vector-like pairs, e.g. `polygon( [[x1,y1], [x2,y2], [x3,y3]] )`, or
             (b2) as a single iterable of numbers to be paired, e.g., `polygon( [x1,y1, x2,y2, x3,y3] )`
           If keyword filled = True the polygon will be filled.  Note: if this brush has thickness larger than [1,1],
           then multiple polygons will be drawn near each other to simulate a larger brushsize."""
        points = tuple(self.generate_points_from(vertices))
        length = len(points)
        CIntList = c_int * length
        px_list, py_list = zip(*points)
        wb_function = wb.wb_display_fill_polygon if filled else wb.wb_display_draw_polygon
        for offx, offy in self.enact_layers():  # use this brush's settings
            x_list = CIntList(*(round(offx + px) for px in px_list))
            y_list = CIntList(*(round(offy + py) for py in py_list))
            wb_function(self.display.tag, x_list, y_list, length)

    def rectangle(self, corner: Union[Iterable2f,float], size: Union[Iterable2f, float], *args: float, filled=False):
        """Uses this brush's settings to draw a rectangle with the given corner (a vector-like [x,y] pair),
           and the given size, which may be given as a single number (for a square), or a vector-like [width,height]
           pair (for a rectangle).  These may also be passed as 3 separate args (cx, cy, square_size), or as
           4 separate args (cx, cy, width, height).
           If keyword filled=True the rectangle will be filled.  If this brush has thickness larger than [1,1] then
           multiple rectangles will be drawn near each other to simulate a larger brushsize."""
        if len(args) == 1:    # rectangle(cx,cy, square_size)
            cx,cy, width,height = corner,size, args[0],args[0]
        elif len(args) == 2:  # rectangle(cx,cy, width,height)
            cx,cy, width,height = corner,size, args[0],args[1]
        else:                 # rectangle([cx,cy], size)
            cx,cy = corner
            width, height = size if isinstance(size, Sequence) else (size, size)
        wb_function = wb.wb_display_fill_rectangle if filled else wb.wb_display_draw_rectangle
        for offx, offy in self.enact_layers():  # make this brush's settings be the current drawing settings for this display
            wb_function(self.display.tag, round(cx+offx), round(cy+offy), round(width), round(height))

    def clear(self, *color_components:Union[float, Sequence[float]]):
        """A convenient shortcut for drawing a filled rectangle across this brush's entire display using this brush's
           settings, augmented with the given color, if given. Common usages: `display.clear(0,0,0)` or `brush.clear()`.
           If this brush has an offset or semi-opacity, the results may seem odd."""
        if len(color_components)==1: color_components = color_components[0]  # unpack single argument
        brush = self if color_components==() else self(color=color_components)
        brush.rectangle((0,0), self.display.size, filled=True)

    def oval(self, center: Sequence[float], rad: Union[float,Sequence[float]], *args: float, filled=False):
        """Uses this brush's settings to draw an oval with the given center (a vector-like [x,y] pair), and the given
           radii, which may be given as a single number (for a circle), or a vector-like [rx,ry] pair (for an ellipse).
           These may also be passed as 3 separate args (cx, cy, circle_radius), or as 4 separate args (cx, cy, rx, ry).
           If keyword filled=True the oval will be filled.  Note: if this brush has thickness larger than [1,1] then
           multiple ovals will be drawn near each other to simulate a larger brushsize."""
        if len(args) == 1:    # oval(cx,cy, circle_radius)
            cx,cy, rx,ry = center,rad, args[0],args[0]
        elif len(args) == 2:  # oval(cx,cy, rx,ry)
            cx,cy, rx,ry = center,rad, args[0],args[1]
        else:               # oval([cx,cy], rad)
            cx, cy = center
            rx, ry = rad if isinstance(rad, Sequence) else (rad, rad)
        wb_function = wb.wb_display_fill_oval if filled else wb.wb_display_draw_oval
        for offx, offy in self.enact_layers():  # use this brush's settings for its display
            wb_function(self.display.tag, round(offx+cx), round(offy+cy), round(rx), round(ry))

    def text(self, txt: str, *xy: Union[float, Iterable2f]):
        """Using this brush's settings, including its text settings like font, fontsize, and antialiasing, writes
           the given text string at location xy in the linked display.  This location may be given as a vector-like
           [x,y] pair, or as two separate args, x and y."""
        x, y = xy if len(xy) == 2 else xy[0]
        for offx, offy in self.enact_layers(text=True):  # use this brush's settings, incl font, as drawing settings
            wb.wb_display_draw_text(self.display.tag, txt.encode(), round(offx + x), round(offy+y))

class MultiBrush(Brush):
    def __init__(self, *brushes: Union['Brush', Iterable['Brush']],
                       display: 'Display' = None, weight: float = None, name: str = None):
        """A MultiBrush is most often created with brush1 & brush2.  Draw commands issued from a
           MultiBrush will be executed multiple times in the associated display, first using
           the settings of the first brush, then using the settings of the second, and so on...
           E.g. `(shadow & red).text("Hello world!")` could draw an offset shadow with red text atop it.
           Setting M=A&B produces a multibrush whose layers are A and B themselves, so future changes to either alter M.
           Concatenations of multibrushes e.g., (A&B)&(C&D) are flattened into a single many-layered multibrush.
           (So if you set C = A&B; then E=C&D; E will end up having conjuncts [A,B,D], but not C itself.)
           Most other operations on a multibrush are distributed over the conjuncts.
           Draw commands.  `(A&B).oval(...)` is equivalent to `A.oval(...); B.oval(...)`.
           Augmenting. `b1(b2)` generally returns a brush like b1, except with its settings overridden by b2.
              (A&B)(C) is equivalent to A(C)&B(C).  I.e. C augments each layer in the multibrush.
              A(B&C) is equivalent to A(B)&A(C).  I.e. each layer in the multibrush separately augments A.
              (A&B)(C&D) is equivalent to A(C&D) & B(C&D), which is equivalent to A(C)&A(D) & B(C)&B(D).
           Weighting. `0.5*b` generally returns a brush like b but with weight 0.5, so it appears with half-opacity,
           and has half-weight in weighted sums.  (A&B)/2 is equivalent to (A/2) & (B/2).
           Addition.  `b1+b2` generally returns a brush whose settings are a weighted sum of component settings.
              (A&B)+C is equivalent to (A+C) & (B+C)
              (A&B)+(C&D) is equivalent to ((A&B)+C) & ((A&B)+D) which is equivalent to (A+C)&(A+D)&(B+C)&(B+D).
           """
        super().__init__(display = display if display is not None else brushes[0].display)
        self.name = name or "MultiBrush"
        if len(brushes) == 1: brushes = brushes[0]  # treat single arg as iterable
        self.display = display if display is not None else brushes[0].display
        clumps = (brush.layers if brush.layers else (brush,) for brush in brushes)
        if weight is None:
            self.layers = [brush for brush in itertools.chain.from_iterable(clumps)]
        else:
            self.layers = [brush.copy(display=self.display, weight=weight)
                           for brush in itertools.chain.from_iterable(clumps)]

    def __repr__(self):
        return f"{self.name}({len(self.layers)} layers) for {self.display}"

    def __len__(self):
        return len(self.layers)

    def __iter__(self):
        if self.layers: yield from self.layers

    def __mul__(self, n):
        """Multiplying a multibrush by n produces a similar multibrush whose layers' weights are n times as large.
           Brush weight serves as a multiplier on opacity, and as its weight in weighted sums."""
        return MultiBrush(layer*n for layer in self.layers)
    __rmul__ = __mul__  # brush multiplication is commutative

    def __truediv__(self, n):
        """Dividing a multibrush by n produces a similar multibrush whose layers' weights are 1/n times as large.
           Brush weight serves as a multiplier on opacity, and as its weight in weighted sums."""
        return MultiBrush(layer/n for layer in self.layers)

    def copy(self, display=None, weight=None) -> 'MultiBrush':
        return MultiBrush(self.layers, display=display, weight=weight)

    def enact_layers(self, display=None, text=False) -> Iterable[Vector]:
        """This generator will yield vector offsets for use in drawing commands.  For multibrushes, this yields from
           the component layers' enactments, in succession. Any layer that is a simple 1x1 brush will yield one offset.
           Thicker brushes will yield more offsets to simulate thickness via side-by-side redrawings."""
        if display is None: display = self.display
        for layer in self.layers:
            yield from layer.enact_layers(display=display, text=text)

class MetaBrushDevice(MetaBrush, MetaDevice):
    # To get the Display class to be a subclass of both Brush and Device, we need a unified metaclass for it;
    # fortunately there are no real conficts between these metaclasses
    pass

class Display(Brush, Device, metaclass=MetaBrushDevice):
    """A Python Display object is used to control a display node in the webots simulation, whose image may be altered
       by this controller, and may be viewed in a pop-up window and/or upon the surface of some simulated object,
       like a viewscreen or chameleon skin.
       Many display.commands employ various brush settings that affect how the drawing will occur.
           color: like other python Webots colors, this is usually as a sequence [red, green, blue], each 0..1,
                  though you may also specify hex colors as python integers (0-16777215) or hex (up to 0xFFFFFF).
                  If a 4th alpha parameter is given, it will be used as the alpha.
           alpha:  0..1 The transparency..opacity drawn parts of the display will have. Lower values will reveal
                   more of the background behind the display.
           opacity:  The default extent 0..1 to which drawing will affect what is already present on the display.
           weight:  For most brushes this is 1.  weight * opacity is the actual extent (capped at 1) to which
                    drawing with this brush will affect what is already present on the display.  Weight also
                    determines a brush's relative contribution when added to another brush.
           offset: A 2D sequence [x,y] indicating the number of display pixels to shift drawing.
           thickness:  A 2D sequence [x,y] indicating brush width and height.  Larger brushes are simulated by
                       repeatedly drawing/writing with different offsets. Each brush has the shape of a plus-sign,
                       centered around the given coordinates for drawing.  If you don't want drawn pixels/endpoints
                       to look like +, you may want to draw over them with something like an oval.
                       TODO or it might make sense eventually to offer an endcap option.
           hardness: A brush with hardness 1 has its full opacity thoughout its thickness (like a permanent marker).
                     Lower hardnesses fade out to the given hardness at their periphery (like an airbrush).
           font: The name of the font to use (default "Lucida Console")
           fontsize: in pixels (default 8)
           antialiasing: True indicates that the font edges will be smoothed; False not
       Each draw command will be initiated from a Brush object that stores a collection of brush settings.
       Simple common draw commands call display(...) to create a Brush with keyword settings, and immediately
       use a method of that brush to draw using those settings. E.g., the following draws a red pixel at x=2, y=1:
           `display(color=[1,0,0]).pixel(2, 1)`
       Brush objects can be assigned to variables, e.g. with `thick_red = display(color[1,0,0], thickness=[3,3])`.
       After this, draw commands like `thick_red.oval(center, radii)` will draw re-using that brush's settings.
       Each brush is tied to a particular Display object, the one its methods like .oval() would draw to.
       `display2(brush1)` returns a brush tied to display2, rather than whatever display brush1 was tied to.
       When using brushes in multiple displays, a common pattern will be `display2(brush).oval(...)`
       Brushes may be combined in three ways:
          brush1(brush2)   returns an augmented version of brush1 with brush2 settings taking precedence.
                           This accepts zero or more brush args, with later ones taking higher precedence.
                           And it accepts keyword args, like `opacity=1` to augment particular settings.
          brush1 + brush2  adds the settings of the two brushes, adjusted by their weight.  It is often useful
                           to use this along with multipliers to produce a weighted average, e.g., red/4 + 3/4*blue.
          brush1 & brush2  returns a multibrush whose draw commands will be run twice, first with the
                           settings of brush1, then with the settings of brush2.
                           E.g. `(shadow & red).text(...)` could draw an offset shadow with red text atop it.
       Brush settings may be read/changed like python properties, e.g. brushname.color returns the current color,
       or brushname.color = newcolor changes the color.  Multiple settings can be changed at once with keyword
       arguments: brushname.set(color = ..., opacity = ...).  Also brush1.set(brush2) copies brush2's
       settings to brush1.
       The display device itself is a Brush, so its settings could be adjusted in the ways just described,
       e.g. with display.set(opacity=1), display.set(brush2), or display.opacity=1. The display's own settings
       will be used (1) for draw commands issued from the display itself, e.g., display.oval(...), and (2) as
       default settings for draw commands issued by brushes within that display.
       Changing a display's default settings can be useful if those settings will rarely change, e.g. in a
       monochromatic display.  However changes to these settings will linger until changed again, which may
       bring about unwanted effects.  So it is often advisable to instead use brushes to temporarily draw
       with particular settings, with no risk of unwanted lingering effects."""

    # Since each display is a Brush, it automatically inherits draw commands like display.ellipse from Brush.
    Brush = Brush  # Now the Brush class is accessible as Display.Brush

    RGB  = WB_IMAGE_RGB
    RGBA = WB_IMAGE_RGBA
    ARGB = WB_IMAGE_ARGB
    BGRA = WB_IMAGE_BGRA
    ABGR = WB_IMAGE_ABGR
    initialized = False  # will be changed on first call to __init__

    def __init__(self, name_or_index: Union[str, int] = None):

        # Device() uses __new__ to find/construct devices, and each time this display gets found again, its __init__
        # will be called again.  So we take care to do one-time initialization steps just once.
        if not self.initialized:
            Brush.__init__(self, display=self)
            self.last_sent_settings = self(_settings=Brush.default_settings.copy())
            self.set(self.last_sent_settings)
            self.initialized = True

    def __repr__(self):
        return f'Display("{self.name}")'

    def full_offset(self, display=None):
        """Overwrites the more complicated Brush.full_offset, which computes how far left/right and up/down to offset
           drawn items. For displays, full_offset just is offset, since there are no brush superclasses to factor in."""
        return self.offset

    # --- display deprecated adjusting drawing color and font ---

    @use_docstring_as_deprecation_warning
    def setColor(self, color):
        """DEPRECATED: Display.setColor() is deprecated. Please use other options like display.color = newcolor."""
        self.color = color

    @use_docstring_as_deprecation_warning
    def setAlpha(self, alpha: float):
        """DEPRECATED: Display.setAlpha() is deprecated. Please use other options like display.alpha = newvalue."""
        self.alpha = alpha

    @use_docstring_as_deprecation_warning
    def setOpacity(self, opacity):
        """DEPRECATED: Display.setOpacity() is deprecated. Please use other options like display.opacity = newopacity."""
        self.opacity = opacity

    @use_docstring_as_deprecation_warning
    def setFont(self, font: str, size: int, antiAliasing: bool):
        """DEPRECATED: Display.setFont() is deprecated. Please use other options
           like display.set() with keywords font, fontsize and/or antialiasing."""
        self.font, self.fontsize, self.antialiasing = font, size, bool(antiAliasing)

    # --- display dimensions ---

    @property
    def size(self) -> Vector:
        """Returns the dimensions of this display as a [width, height] vector."""
        return Vector(wb.wb_display_get_width(self.tag), wb.wb_display_get_height(self.tag))

    @property
    def width(self) -> int:
        """Returns the current width of this Display, in pixels."""
        return wb.wb_display_get_width(self.tag)
    @use_docstring_as_deprecation_warning
    def getWidth(self) -> int:
        """DEPRECATED: Display.getWidth() is deprecated. Please use display.width instead."""
        return wb.wb_display_get_width(self.tag)

    @property
    def height(self) -> int:
        """Returns the current height of this Display, in pixels."""
        return wb.wb_display_get_height(self.tag)
    @use_docstring_as_deprecation_warning
    def getHeight(self) -> int:
        """DEPRECATED: Display.getHeight() is deprecated. Please use display.height instead."""
        return wb.wb_display_get_height(self.tag)

    # --- attaching camera to displays ---

    def attach_camera(self, camera: Camera):
        wb.wb_display_attach_camera(self.tag, camera.tag)
    @use_docstring_as_deprecation_warning
    def attachCamera(self, camera: Camera):
        """DEPRECATED: Webot is shifting to conventional python naming, so attachCamera is now attach_camera."""
        wb.wb_display_attach_camera(self.tag, camera.tag)

    def detach_camera(self):
        wb.wb_display_detach_camera(self.tag)
    @use_docstring_as_deprecation_warning
    def detachCamera(self):
        """DEPRECATED: Webot is shifting to conventional python naming, so detachCamera is now detach_camera."""
        wb.wb_display_detach_camera(self.tag)

    # --- deprecated display draw commands (unfilled) ---

    @use_docstring_as_deprecation_warning
    def drawPixel(self, x1: int, y1: int):
        """DEPRECATED. Display.drawPixel() is deprecated. Please use display.pixel() or brush.pixel() instead."""
        self.pixel(x1, y1)

    @use_docstring_as_deprecation_warning
    def drawLine(self, x1: int, y1: int, x2: int, y2: int):
        """DEPRECATED. Display.drawLine() is deprecated. Please use display.line() or brush.line() instead."""
        self.line(x1,y1, x2,y2)

    @use_docstring_as_deprecation_warning
    def drawRectangle(self, x: int, y: int, width: int, height: int):
        """DEPRECATED. Display.drawRectangle() is deprecated. Please use display.rectangle() or brush.rectangle()."""
        self.rectangle(x,y, width,height)

    @use_docstring_as_deprecation_warning
    def drawOval(self, cx: int, cy: int, rx: int, ry: int):
        """DEPRECATED. Display.drawOval() is deprecated. Please use display.oval() or brush.oval()."""
        self.oval(self.tag, cx,cy, rx,ry)

    @use_docstring_as_deprecation_warning
    def drawPolygon(self, x_list:List[int], y_list:List[int]):
        """DEPRECATED. Display.drawPolygon(x_list, y_list) is deprecated.
           Please use display.polygon([x1,y1],[x2,y2],...) or display.polygon(zip(x_list, y_list))
           or their brush.polygon(...) analogs instead."""
        self.polygon(zip(x_list, y_list))

    @use_docstring_as_deprecation_warning
    def drawText(self, txt: str, x: int, y: int):
        """DEPRECATED. Display.drawText() is deprecated. Please use display.text() or brush.text()."""
        self.text(txt, x, y)

    # --- deprecated display filled drawing commands ---

    @use_docstring_as_deprecation_warning
    def fillRectangle(self, x: int, y: int, width: int, height: int):
        """DEPRECATED. Display.fillRectangle() is deprecated.
           Please use display.rectangle(x,y,width,height, filled=True) or its brush.rectangle(...) analog."""
        self.rectangle(x,y, width,height, filled=True)

    @use_docstring_as_deprecation_warning
    def fillOval(self, cx: int, cy: int, rx: int, ry: int):
        """DEPRECATED. Display.fillOval() is deprecated.
           Please use display.oval(x,y,rx,ry, filled=True) or its brush.oval(...) analog."""
        self.oval(self.tag, cx,cy, rx,ry, filled=True)

    @use_docstring_as_deprecation_warning
    def fillPolygon(self, x_list:List[int], y_list:List[int]):
        """DEPRECATED. Display.fillPolygon(x_list, y_list) is deprecated.
           Please use display.polygon(x1,y1, x2,y2, ..., filled=True) or
           display.polygon(zip(x_list, y_list), filled=True) or their brush.polygon(...) analogs."""
        self.polygon(zip(x_list, y_list), filled=True)

    # --- display copy/pasting images ---

    class ImageRef(c_void_p):
        display: 'Display'  # the display that this image will paste to by default; set by Display.Image()

        # To avoid confusion with nearby uses of `self` meaning this Display, we use `img` in place of `self` here
        def __repr__(img): return f"ImageRef({img.value})"

        def paste(img, xy:Sequence[int] = (0, 0), blend:bool = True, display:'Display' = None):
            """Pastes this image onto display, if given, or this image's own .display (the one it was created from)
               at the given xy position.  If blend is True, pixels will be blended with existing values."""
            if display is None: display = img.display
            wb.wb_display_image_paste(display.tag, img, int(xy[0]), int(xy[1]), c_bool(blend))

        def paste_once(img, xy:Sequence[int] = (0, 0), blend:bool = True, display:'Display' = None):
            """A convenience combination of image.paste() and image.delete(), useful for images that will be used
               just once and needn't be retained after. This enables convenient shorthand like
               `display.Image(myarray).paste_once(xy)` with no need to create a local name for the image nor to
               delete it in a separate line."""
            img.paste(xy, blend, display)
            img.delete()

        def save(img, filename: str):
            return wb.wb_display_image_save(img.display.tag, img, filename.encode())

        def delete(img):
            return wb.wb_display_image_delete(img.display.tag, img)
    # Now that the ImageRef class is defined, we can declare restypes for wb_methods that return ImageRef's
    wb.wb_display_image_new.restype = ImageRef
    wb.wb_display_image_copy.restype = ImageRef
    wb.wb_display_image_load.restype = ImageRef

    class ImageCreator:
        def __init__(self, display:'Display'):
            self.display = display  # the display that this image will paste to by default; set by Display.Image()

        # To avoid confusion with nearby uses of self, we use `creator` in place of `self` here
        # These are classmethods, as they will typically be called via `display.Image.foo(...)` to create an instance
        def new(creator, data: Union[Sequence[float], Sequence[Iterable4f], Sequence[Sequence[Iterable4f]]],
                         format: int = WB_IMAGE_BGRA, width: int = None, height: int = None, range=255) -> 'ImageRef':
            """Passes into Webots the data for a new image that can then be pasted onto a Display, and returns
               a Display.Image object whose methods like .paste and .save will use the imported image data.
               This will be fastest if the given data is a NumPy array, opencv Mat, PIL, or other object
               with Python's buffer protocol, with each color component occupying one byte.
               TODO  Alternatively the data may be given as a sequence or nested sequence to be traversed depth-first.
               If height and width are not given, they will be automatically inferred from the buffer protocol,
               or from the first and second dimensions of nested sequences, or a TypeError will be raised.
               Width and height can be automatically inferred from the buffer protocol or nested sequence structure.
               `format` should be something like Display.BGRA (Webots' default and fastest) indicating what order
               the red, green, blue and (optional) alpha components have in each pixel.
               `range` indicates the maximum value of each color component in the given data, either 1 for colors
               represented as floats ranging 0..1, 255 for bytes (the fastest format) TODO or allow 2**24 and 2**32?
               """
            if isinstance(data, bytes):
                # TODO use missing ones of width, height to infer the others
                # TODO create copy in ctypes
                # TODO send it in
                raise NotImplementedError
            # If missing width or height, first try to see if this object supports Python's buffer protocol
            if width is None or height is None:
                try:
                    mview = memoryview(data)
                except TypeError:  # sadly, it must not support the buffer protocol
                    mview = None

                if mview is not None:
                    if mview.ndim == 3:
                        height, width, comps = mview.shape
                    elif mview.ndim == 2:  # Ambiguous between long 1D list of 1D pixels, and 2D array of 0D ints
                        pass  # TODO could probably infer it in some cases
                    else:
                        pass  # TODO could probably infer it in some cases

            if width is None or height is None:
                l1 = len(data)

            # Now that we have width and height settled, can create a pointer to the data

            # TODO should probably confirm that the buffer contains bytes rather than some other data?

            # First we'll try to use Python's buffer protocol to share memory with a buffer (e.g. numpy array)
            try:
                ptr = c_ubyte.from_buffer(data)
            except TypeError:  # oops, it must not have the buffer protocol after all
                ptr = None

            if ptr is not None:  # we successfully got a pointer via the buffer protocol
                img = wb.wb_display_image_new(creator.display.tag, int(width), int(height), ctypes.byref(ptr), format)
                img.display = creator.display  # Let this image know what display it was created from
                return img

            # Otherwise, we need to construct the data ourselves

            raise NotImplementedError  # TODO extraction of data from sequences is not finished

            def traverse(clump: Iterable) -> Iterable[float]:
                """Simple function to recursively unpack all the floats out of a nested structure, assuming that each
                   clump is non-empty, and that the type of its first element is representative of the rest of the clump."""
                if isinstance(clump[0], (int, float)):
                    yield from clump
                else:
                    for subclump in clump:
                        yield from subclump

            data = tuple(traverse(data))
            n = len(data)
            # TODO for type RGB, does the data come in triples or quadruples?
            if width is None and height is None: pass  # TODO XXX

            if width is None: pass  # TODO XXX
            if isinstance(data, list):
                return wb.wb_display_image_new(len(data), len(data[0]), data, format)
            elif width is None or height is None:
                raise TypeError('imageNew : width and height must be given if data is not a list')
            else:
                return wb.wb_display_image_new(width, height, data, format)
        __call__ = new

        @classmethod
        def copy(creator, *corner_and_size: Union[int, Sequence[int]])  -> 'ImageRef':
            """`display.Image.copy() creates an image by copying from a region of this display."""
            corner, size = creator.display.generate_points_from(corner_and_size)
            img = wb.wb_display_image_copy(creator.device.tag, int(corner.x), int(corner.y), int(size.x), int(size.y))
            img.display = creator.display  # Let this image know what display created it
            return img

        @classmethod
        def load(creator, filename: str)  -> 'ImageRef':
            """`display.Image.load(filename) loads a new image from a file."""
            img = wb.wb_display_image_load(creator.display.tag, filename.encode())
            img.display = creator.display  # Let this image know what display created it
            return img

    @cached_property
    def Image(self) -> ImageCreator:
        """Returns a Display.ImageCreator object tied to this particular Display by default. This ImageCreator
           can be used to create new Display.Image objects which can then be pasted onto a display.
           `display.Image(data, format, width, height, range)` or equivalently `display.Image.new(...)` can be
           used to import a raw image into Webots, e.g. from a Numpy array or nested list of color values.
           `display.Image.copy() creates an image by copying from a region of this display.
           `display.Image.load(filename) loads a new image from a file.
           Whichever way such an image is created:
           `img.paste(xy, blend, [display])` pastes that image back as location `xy` in the given `display` (which
            defaults to being the display from which the image was created) perhaps blending with existing pixels.
           `img.save(filename)` saves this image to a file.
           `img.delete()` tells webots to free up the memory it had been using to store this image."""
        return Display.ImageCreator(self)

    @use_docstring_as_deprecation_warning
    def imageNew(self, data: Union[Sequence[float],Sequence[Iterable4f],Sequence[Sequence[Iterable4f]]],
                       format: int = RGBA, width: int = None, height: int = None) -> ImageRef:
        """DEPRECATED. Display.imageNew() is deprecated. Please use display.Image() instead."""
        return self.image_new(data, format, width, height)

    @use_docstring_as_deprecation_warning
    def imageLoad(self, filename: str) -> ImageRef:
        """DEPRECATED. Display.imageLoad(filename) is deprecated. Please use display.Image.load(filename) instead."""
        return self.Image.load(filename)

    @use_docstring_as_deprecation_warning
    def imageCopy(self, x: int, y: int, width: int, height: int) -> ImageRef:
        """DEPRECATED. Display.imageCopy() is deprecated. Please use display.Image.copy() instead."""
        wb.wb_display_image_copy(self.tag, x, y, width, height)

    @use_docstring_as_deprecation_warning
    def imagePaste(self, ir: ImageRef, x: int, y: int, blend: bool = False):
        """DEPRECATED. Display.imagePaste(img, x, y, blend) is deprecated. Please use img.paste((x, y), blend, display)
           though blend will default to False, and display will default to the display used to create the image."""
        """DEPRECATED: Display.imagePaste(ir, x, y, blend) is deprecated. Use display.image_paste(ir, (x, y), blend)"""
        ir.paste((x, y), blend, self)

    @use_docstring_as_deprecation_warning
    def imageSave(self, ir: ImageRef, filename: str):
        """DEPRECATED. Display.imageSave(img, filename) is deprecated. Please use img.save(filename) instead."""
        return ir.save(filename)

    @use_docstring_as_deprecation_warning
    def imageDelete(self, ir: ImageRef):
        """DEPRECATED. Display.imageDelete(img) is deprecated. Please use img.delete() instead."""
        return wb.wb_display_image_delete(self.tag, ir)


# === Emitter, Receiver and Whitelisted Unpickler ===

class Unpickler(pickle.Unpickler):
    """A custom version of Python's Unpickler, restricted to use only a small whitelist of classes and functions.
       This avoids security concerns that arise for Python's default unpickler, which gives pickled messages free
       reign to prompt the unpickler to call other functions, including ones that might execute arbitrary code or
       system commands.  The Unpickler class maintains its own whitelist as Unpickler.whitelist, or an alternative
       whitelist may be passed with the whitelist keyword argument in Unpickler creation.
       Unpickler.Whitelist is a custom dictionary subclass making it easier to maintain whitelists.
       By default, Coder objects use a whitelist that lets this unpickler reconstruct only standard Python datatypes
       like int, float, str, dict, list, and tuple, and commonly used Webots datatypes like Vectors and Colors.
       Users may whitelist additional classes/functions with whitelist(fn) or by decorating the
       function/class definition with @whitelist, and may customize pickling in standard Python ways.
       If you'll ever receive pickles from untrusted sources, you should never whitelist any function or class
       that you wouldn't want to let be called with any possible arguments. For classes, beware that unpickling may
       call the class's __new__, __setstate__, __append__ or __extend__ methods, so you shouldn't whitelist any class
       where you'd be unwilling to let the emitter call these with any arguments."""

    def __init__(self, packet, *args, whitelist: dict = None, **kwargs):
        if isinstance(packet, bytes):
            packet = io.BytesIO(packet)
        if whitelist is not None: self.whitelist = whitelist  # otherwise will inherit the default class whitelist
        super().__init__(packet, *args, **kwargs)

    def find_class(self, module, name):
        """Overwrites the default unpickler.find_class to allow only whitelisted classes and functions"""
        known_fn = self._whitelist.get((module, name), None)
        if known_fn is not None:
            return known_fn
        # Otherwise, it wasn't on the whitelist, so...
        raise pickle.UnpicklingError(f"Permission to use {module}.{name} is denied. "
                                     f"You may consider adding it to the whitelist.")

    class Whitelist(dict):
        """An Unpickler.Whitelist is a specialized dictionary mapping information that pickle would use to identify a
           whitelisted class or function (its __module__ and __name__) to that class/function.
           A Whitelist is callable like a function (and hence is usable as a decorator), which adds the given arg(s) to
           the whitelist of classes and functions that this unpickler will be willing to employ.
           If you'll ever be unpickling messages from untrusted sources, you should be careful to never whitelist any
           function or class that you wouldn't want to have called with any possible arguments.  For classes, beware
           that unpickling may call the class's __new__, __setstate__, __append__ or __extend__ methods, so again you
           shouldn't whitelist any class where you'd be unwilling to have these called with any arguments."""
        def __new__(cls, *args: Callable, default_webots=True):
            self = super().__new__(cls)
            self(*args)  # whitelist the given args
            # TODO set up an overarching whitelist, so classes can @whitelist themselves; then draw from that here
            if default_webots: # add in default webots whitelistees like Vector
                self(Vector, Color)  # TODO others???
            return self

        def __call__(self, *args) -> Callable:
            for arg in args:
                module_name = module, name = arg.__module__, arg.__name__
                existing_entry = self.get(module_name, None)
                if existing_entry is not None and existing_entry is not arg:
                    raise NameError(f"Attempting to whitelist {arg}, when the whitelist already "
                                    f"has a different entry with {module=} and {name=}.")
                self[module_name] = arg
            # to work as a decorator, this must return the result of decoration (in this case unmodified)
            return args[0] if args else None

    whitelist = Whitelist()  # create a token Whitelist to use as the default one for unpickling
whitelist = Unpickler.whitelist


class Coder:
    """This class provides functionality for serializing objects used by Emitter devices, Receiver devices,
       robot.encode and robot.decode.  This includes a safely restricted version of Python's pickle
       that can convey copies of almost any Python object, including all the ones json handles, more faithful
       transmission of dicts and tuples, transmission of common webots types like Vectors and Colors, and is
       easily extendable to transmit other user-created class instances. This version of pickle is restricted to a
       small whitelist of classes and functions that it can reconstruct. This avoids security concerns for Python's
       default unpickling, which gave pickled messages free reign to call other functions and cause unwanted effects.
       """
    # Note: backwards incompatibility: this used to be the constant -1, rather than the python built-in all
    BROADCAST = CHANNEL_BROADCAST = all
    _encoding = "pickle"  # can be 'pickle', 'json', 'str' or None
    whitelist = Unpickler.whitelist  # maps (module, name):constructor
    KNOWN_ENCODINGS = {pickle, 'pickle',
                       str, 'str',
                       json, 'json',
                       None, 'None', 'none', bytes, 'bytes'}

    @property
    def encoding(self):
        """Reads or alters the encoding scheme that this encoder/decoder will use.
         `dev.encoding` indicates the sort of encoding will be used.  The same .encoding should be used for both
         encoding (e.g. Emitters) and decoding (e.g. Receivers). Possible values:
         `dev.encoding = None` or `dev.encoding = bytes` can handle only python bytes objects
         `dev.encoding = str` can handle bytes objects or strings (converted with python str.encode and bytes.decode).
         `dev.encoding = 'json'` uses the Python's json, which can handle basic Python datatypes like strings,
         numbers, and lists, and (with some alteration) tuples and dicts.
         `dev.encoding = 'pickle'` is the default. It uses a safely restricted version of Python's pickle. This
         can convey copies of almost any Python object, including all the ones json handles, more faithful transmission
         of dicts and tuples, transmission of common webots types like Vectors and Colors, and is easily extendable
         to transmit other user-created class instances. This version of pickle is restricted to a small whitelist of
         classes and functions that it can reconstruct. This avoids security concerns for Python's default unpickling."""
        return self._encoding
    @encoding.setter
    def encoding(self, new_encoding):
        if new_encoding not in self.KNOWN_ENCODINGS:
            WarnOnce(f"{self} does not recognize encoding '{new_encoding}'.  Use 'pickle','json','str' or None.")
        self._encoding = new_encoding

    def encode(self, data: Any) -> bytes:
        """Encodes the given data to bytes using the current emitter.encoding settings."""
        if self._encoding in (pickle, 'pickle') and (not isinstance(data, bytes) or data.startswith(b'\x80')):
            return pickle.dumps(data)
        if self._encoding in (str, 'str') and isinstance(data, str):  # use str.encode
            return data.encode()
        elif self._encoding in (json, 'json'):
            return json.dumps(data).encode()  # final encode converts from json string to bytes
        elif not isinstance(data, bytes):
            raise TypeError(f"{self}: Given data {data} is not bytes, and no applicable emitter.encoding has been set.")
        return data

    def decode(self, packet: bytes) -> any:
        """Decodes the given packet using this receiver's current .encoding setting.  If decoding the message causes
           an error, e.g. because the message was malformed, a warning will be printed and the raw packet will be
           returned as a python bytes object.  Typically, if you aren't expecting to receive a bytes object, you can use
           isinstance(packet, bytes) to distinguish successfully-decoded packets from undecodable ones left as bytes."""
        try:
            if self._encoding in (pickle, 'pickle') and packet.startswith(b'\x80'):
                return Unpickler(packet, whitelist=self.whitelist).load()
            if self._encoding in (str, 'str', 'string'): return packet.decode()
            if self._encoding in (json, 'json'): return json.loads(packet.decode())
        except (pickle.PickleError, json.JSONDecodeError) as err:
            WarnOnce(str(err))
        # otherwise... if we hit an error or if self._encoding likely in {None, 'None', 'none', bytes, 'bytes'}:
        return packet

    # --- Coder channel management ---

    # most Coders will be emitter/receiver devices with the following attributes used in managing channels
    get_channel_fn: Callable
    set_channel_fn: Callable
    tag: int

    @property
    def channel(self) -> int:
        """`device.channel` reads the current channel for this emitter/receiver (either all or some integer)
           `device.channel = c` sets the channel of this emitter/receiver to integer c.
           `device.channel = all` sets this emitter/receiver to use all available channels."""
        channel = self.get_channel_fn(self.tag)
        return channel if channel != -1 else all
    @channel.setter
    def channel(self, new_channel: Union[int, Callable]):
        if new_channel is all: new_channel = -1
        self.set_channel_fn(self.tag, new_channel)

    @use_docstring_as_deprecation_warning
    def getChannel(self) -> int:
        """DEPRECATED: Emitter.getChannel() and Reciever.getChannel() are deprecated.  Use dev.channel instead."""
        return self.channel
    @use_docstring_as_deprecation_warning
    def setChannel(self, new_channel: int):
        """DEPRECATED: Emitter.setChannel(c) and Reciever.setChannel(c) are deprecated.  Use dev.channel = c instead."""
        self.channel = new_channel


class Emitter(Coder, Device):
    """The Emitter device is used to model radio, serial or infra-red emitters.  Emitters send data to receivers,
       but cannot receive information themselves.  In Python, the emitter can be called like a function.
         `emitter(packet)` sends this packet to any receiver in range listening on an appropriate channel.
         `emitter.channel` indicates the emitter's current channel (integer or all)
         `emitter.channel = c` changes to channel c (integer), which must be among this emitter's allowed channels
         `emitter.channel = all` sends on all allowed channels at once.
         `emitter.range` indicates the range that the emitter will currently send messages.
         `emitter.range = new_range` alters this range.
         `emitter.encoding` indicates how this emitter will encode packets it sends, which determines what sorts of
         packets it can send. It should match the `receiver.encoding` of the corresponding receiver. Possible values:
         `emitter.encoding = None` can handle only python bytes objects
         `emitter.encoding = str` can handle bytes objects or strings (converted to bytes by python str.encode).
         `emitter.encoding = 'json'` uses the popular json standard to encode packets, which can handle basic Python
         datatypes like strings, numbers, lists, and (with some infidelity) dicts and tuples.
         `emitter.encoding = 'pickle'` is the default. It uses a safely restricted version of Python's pickle. This
         can convey copies of almost any Python object, including all the ones json handles, more faithful transmission
         of dicts and tuples, transmission of common webots types like Vectors and Colors, and is easily extendable
         to transmit other user-created class instances. This version of pickle is restricted to a small whitelist of
         classes and functions that it can reconstruct. This avoids security concerns for Python's default unpickling.
       """
    get_channel_fn = wb.wb_emitter_get_channel  # Used by Coder.channel
    set_channel_fn = wb.wb_emitter_set_channel

    def __call__(self, data: Any) -> int:
        """An emitter can be called like a function `emitter(data)` to broadcast the data on the current channel.
        What type(s) of data can be sent depends upon the emitter's .encoding.  Returns 1 if the packet is
        successfully added to the sending queue, 0 if that queue was full."""
        data = self.encode(data)  # self.encode() is inherited from Coder
        return wb.wb_emitter_send(self.tag, data, len(data))
    send = __call__

    @property
    def buffer_size(self) -> int:
        """Returns the current buffer_size of this Emitter. XXX"""
        return wb.wb_emitter_get_buffer_size(self.tag)
    @use_docstring_as_deprecation_warning
    def getBufferSize(self) -> int:
        """DEPRECATED: Emitter.getBufferSize() is deprecated. Please use emitter.buffer_size instead."""
        return wb.wb_emitter_get_buffer_size(self.tag)

    wb.wb_emitter_get_range.restype = c_double
    @property
    def range(self) -> float:
        """Returns or adjusts the current range of this Emitter."""
        return wb.wb_emitter_get_range(self.tag)
    @range.setter
    def range(self, range: float):
        wb.wb_emitter_set_range(self.tag, c_double(range))
    @use_docstring_as_deprecation_warning
    def getRange(self) -> float:
        """DEPRECATED: Emitter.getRange() is deprecated. Please use emitter.range instead."""
        return wb.wb_emitter_get_range(self.tag)
    @use_docstring_as_deprecation_warning
    def setRange(self, range: float):
        """DEPRECATED: Emitter.setRange() is deprecated. Please use emitter.range = new_range instead."""
        wb.wb_emitter_set_range(self.tag, c_double(range))


class Receiver(Coder, Device, Sensor):
    """The Receiver device is used to model radio, serial or infra-red receivers.  Receivers receive data sent from
       Emitters, but cannot send information themselves.  In Python, the receiver functions like an iterator.
         `for packet in receiver` iterates through any packets in the receiver's queue.  Packets are removed from
            the queue as they are iterated over, so be sure to store a copy if you'll want a packet later.
            By default (see below), Webots uses a safely restricted version of Python's pickle to faithfully transmit
            a wide variety of possible python objects from emitter to receiver, so the received packet will
            automatically be decoded to have a datatype corresponding to whatever the emitter sent.
            If decoding a packet causes an error, e.g. because the packet was malformed, a warning will be printed and
            the raw packet will be returned as a python bytes object. If you aren't expecting to receive a bytes object,
            isinstance(packet, bytes) can distinguish successfully-decoded packets from undecodable ones left as bytes.
         `len(receiver)` says how many packets are currently in the queue.
         `if receiver` tests whether there are any packets in the queue (equivalent to `if(len(receiver))`)
         `next(receiver [, default])` works as it does for other python iterators, removing and returning the next
            packet in the queue, automatically decoding it. If the queue is empty, the optional `default` will be
            returned if one was given; otherwise a StopIteration error will be raised.
         `receiver.pop([default])` is like the above, except does not automatically decode the packet.
         `receiver.peek([default])` is like the above, but neither removes nor automatically decodes the packet.
         `receiver.decode(raw_packet)` uses the receiver's encoding setting to decode a raw packet, eg. from pop/peek.
         `receiver.strength` and `receiver.peek_strength` are the signal strengths for the last-retrieved
            and next-to-be-retrieved packets (each depends on the sending Emitter's range and distance)
         `receiver.direction` and `receiver.peek_direction` are vectors pointing to the emitters that sent the
            last-retrieved and next-to-be-retrieved packets
         `receiver.sampling` works as it does for all other Sensors, reading/adjusting whether and how often the
         receiver checks for new messages.  By default, this is set to the basic timestep when you create the Receiver.
         `receiver.channel` indicates the receiver's current channel (integer or all)
         `receiver.channel = c` changes to channel c (integer or all)
         `receiver.encoding` indicates the sort of encoding the receiver expects packets to have. It should match
         the `emitter.encoding` of the corresponding emitter.  Possible values:
         `receiver.encoding = None` or `receiver.encoding = bytes` can handle only python bytes objects
         `receiver.encoding = str` can handle bytes objects or strings (converted from bytes by python bytes.decode).
         `receiver.encoding = 'json'` uses the popular json standard to encode packets, which can handle basic Python
         datatypes like strings, numbers, and lists, and (with some alteration) tuples and dicts.
         `receiver.encoding = 'pickle'` is the default. It uses a safely restricted version of Python's pickle. This
         can convey copies of almost any Python object, including all the ones json handles, more faithful transmission
         of dicts and tuples, transmission of common webots types like Vectors and Colors, and is easily extendable
         to transmit other user-created class instances. This version of pickle is restricted to a small whitelist of
         classes and functions that it can reconstruct. This avoids security concerns for Python's default unpickling,
         which gave pickled messages free reign to call other functions and bring about unwanted effects.
         Still, if you don't trust another robot's controller not to be malicious, you probably shouldn't even run that
         controller at all, as that already would give it ample opportunity to cause harm.
         Also, even though the whitelist prevents the worst things that default unpickling could do, there are still
         many ways a non-cooperative emitter could send nuisance messages, e.g., malformed ones that would raise errors
         (unless you catch them with try-except blocks) or bulky messages that would waste processing time and memory.
         If you don't trust emitters to be cooperative, it is usually advisable to turn off automatic decoding, and
         instead parse messages manually and skeptically.
         `Receiver.whitelist(c)` adds class or function c to the whitelist of functions that unpickling may use.
       Note: the receiver behaves like Python iterators for files, and unlike many Python iterators, in that it can
       and should still be used even after it has raised StopIteration to indicate that it is currently out of packets.
       """
    OMITTED = object()  # used to distinguish truly omitted default values from actually-legit values like None
    get_channel_fn = wb.wb_receiver_get_channel  # Used by Coder.channel
    set_channel_fn = wb.wb_receiver_set_channel

    # --- Receiver peeking at top packet ---

    wb.wb_receiver_get_data.restype = POINTER(c_char)
    def peek(self, default=OMITTED, check_if_empty=True) -> bytes:
        """Returns the next packet in the receiver's queue as a bytes string, without any automatic decoding, and
           without removing that packet from the queue.
           If no packets are available, default will be returned, if given, or a StopIteration error is raised."""
        if check_if_empty and not wb.wb_receiver_get_queue_length(self.tag):
            if default is not self.OMITTED: return default
            raise StopIteration("No packets in reciever queue.")
        c_type = c_char * wb.wb_receiver_get_data_size(self.tag)
        return bytes(c_type.from_address(wb.wb_receiver_get_data(self.tag)))
    @use_docstring_as_deprecation_warning
    def getDataSize(self) -> int:
        """DEPRECATED. Receiver.getDataSize() is deprecated.
           Methods that retrieve packets automatically get the appropriate amount of data.
           len(receiver.peek()) is equivalent."""
        return wb.wb_receiver_get_data_size(self.tag)
    @use_docstring_as_deprecation_warning
    def getData(self) -> str:
        """DEPRECATED: Receiver.getData() is deprecated. Use `for packet in receiver` or another alternative."""
        return self.peek().decode()

    # --- Receiver iteration ---

    def pop(self, default=OMITTED) -> bytes:
        """Removes and returns the next packet in the queue as a raw bytes string, without any automatic decoding.
           If no packets are available, default will be returned, if given, or a StopIteration error is raised."""

        # self.strength and self.direction reflect popped packet; .peek_strength/.peek_direction will reflect the next
        if not wb.wb_receiver_get_queue_length(self.tag):
            if default is not self.OMITTED: return default
            raise StopIteration("No packets in reciever queue.")
        self.strength = wb.wb_receiver_get_signal_strength(self.tag)
        self.direction = wb.wb_receiver_get_emitter_direction(self.tag)
        packet = self.peek(check_if_empty=False)  # we just checked if empty a moment ago, so no need to repeat
        wb.wb_receiver_next_packet(self.tag)  # remove this packet from head of queue
        return packet
    @use_docstring_as_deprecation_warning
    def nextPacket(self):
        """DEPRECATED: Receiver.nextPacket() is deprecated. Receiver.pop() has same effect,
           but `for packet in receiver` may be more useful."""
        self.pop()

    def __iter__(self):
        """Each Receiver works as a stop-and-go iterator, yielding packets from the queue until the queue is empty.
           After a receiver stops iterating, later attempts to iterate from it (at later timesteps) may yield new
           packets if new packets have arrived in the interim.  Each packet is automatically decoded in accord
           with the receiver's current .encoding setting."""
        return self  # python will now view this receiver as its own iterator, using its __next__ method

    def __next__(self) -> any:
        """This is automatically called by next(receiver [, default]) or in iterating over the receiver.
           This returns the next packet from the receiver's queue, automatically decoded in accord with the receiver's
           .encoding setting, or raises StopIteration if the queue is empty. After a receiver stops iterating, later
           attempts to iterate from it may yield new packets if any have arrived in the interim."""
        return self.decode(self.pop())  # decode is inherited from Coder; pop raise StopIteration if empty

    # --- Receiver length and boolean ---

    def __bool__(self) -> bool:
        """A receiver has boolean value True when it has packets in its queue, False otherwise."""
        return bool(wb.wb_receiver_get_queue_length(self.tag))

    def __len__(self) -> int:
        """len(receiver) returns the number of packets remaining in this receiver's queue"""
        return wb.wb_receiver_get_queue_length(self.tag)
    @use_docstring_as_deprecation_warning
    def getQueueLength(self) -> int:
        """DEPRECATED: Receiver.getQueueLength() is deprecated. len(receiver) is equivalent,
           but `for packet in receiver` may be more useful."""
        return wb.wb_receiver_get_queue_length(self.tag)

    # --- Receiver strength ---

    wb.wb_receiver_get_signal_strength.restype = c_double
    @descriptor(prioritized=False)  # for linting/docstring; preempted by instances' own .strength, once it is set
    def strength(self) -> float:
        """Returns the signal strength of the most recently retrieved packet, which depends on the emitter's range and
           distance. In contrast, receiver.peek_strength returns the strength for the next packet to be retrieved."""
        return 0
    @property
    def peek_strength(self) -> float:
        """Returns the signal strength of the next packet that will be retrieved, which depends on emitter's range and
           distance. In contrast, receiver.strength returns the strength for the most recently retrieved packet."""
        return wb.wb_receiver_get_signal_strength(self.tag)
    @use_docstring_as_deprecation_warning
    def getSignalStrength(self) -> float:
        """DEPRECATED: Receiver.getSignalStrength() is deprecated. Use receiver.strength or receiver.peek_strength."""
        return self.peek_strength

    # --- Receiver direction ---

    wb.wb_receiver_get_emitter_direction.restype = c_void_p
    @cached_property
    def direction(self) -> Vec3f:
        """Returns a vector pointing to the emitter that sent the most recently retrieved packet.
           In contrast, receiver.peek_direction returns the direction for the next packet to be retrieved."""
        return Vec3f(0, 1, 0)
    def peek_direction(self) -> Vec3f:
        """Returns a vector pointing towards the emitter that sent the next packet that will be retrieved.
           In constrast, receiver.direction returns the direction for the most recently retrieved packet."""
        return Vec3f.from_address(wb.wb_receiver_get_emitter_direction(self.tag))
    @use_docstring_as_deprecation_warning
    def getEmitterDirection(self) -> Vec3f:
        """DEPRECATED: Receiver.getEmitterDirection() is deprecated. Use receiver.direction or .peek_direction."""
        return Vec3f.from_address(wb.wb_receiver_get_emitter_direction(self.tag))
    # TODO confirm that this is the right temporal offset

