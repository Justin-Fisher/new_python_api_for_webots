"""This is the main interface for ordinary python controllers to control Webots robots in the "New Python API".
   This functionality can be imported via `import robot`.  The `robot` module is manually morphed to be an instance
   of the `RobotModule` class, enabling it to have properties like `robot.time`."""

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

# TODO Many of these imports are redundant with importing * from devices, though I may eventually opt not to do that

import sys
from ctypes import c_char_p, c_double, c_bool, c_void_p, POINTER as c_pointer
from types import ModuleType
import functools
from typing import Callable, Union, Tuple, Sequence

import settings
import core_api
from core_api import * # imports webots constants, the ctypes library wb, the robot.step function, timed_cached_property
from webots_warnings import Warn, WarnOnce, use_docstring_as_deprecation_warning
from descriptors import descriptor, cached_property  # more options than python's @property
from surrogate_values import SurrogateValue, surrogate_attribute
from vectors import Vector
from devices import *

# === Claim control of robot (allows wb.functions to work) ===

if settings.initialize_robot_on_import:
    init(again = False) # initialize controller's control over a robot (if not done already, e.g. by importing world)

# === Robot Input pseudo-Devices ===

class WbUserInputEvent(int):
    """This class is used mostly for typing to represent classes of Events that can be awaited with
       robot.waitForUserInputEvent. Simple event types are stored as Robot attributes .EVENT_QUIT, EVENT_NO_EVENT,
        .EVENT_MOUSE_CLICK, .EVENT_MOUSE_MOVE, .EVENT_KEYBOARD, .EVENT_JOYSTICK_BUTTON,
        .EVENT_JOYSTICK_AXIS, .EVENT_JOYSTICK_POV.  Disjunctive event types may combined with
         bitwise or, e.g. robot.EVENT_MOUSE_MOVE | robot.EVENT_MOUSE_CLICK."""

# Trying to satisfy multiple desiderata:
# (0) Don't generate pseudodevices unnecessarily - instead wait until demanded
# (1) Create just one pseudodevice of each type (e.g. just one Keyboard)
# (2) Make robot.keyboard the primary recommended/documented way of interacting with the device
# (3) Enable pseudodevices in a smooth way, ideally automatically
# (4) Continue providing something akin to the traditional getKeyboard(), a way of referring to a keyboard object
# (5) Allow reference to the class as, e.g. robot.Keyboard() or robot.Keyboard() and optional construction using that
# (6) Allow for reasonably easy user subclassing of devices
# (7) Allow smooth interfacing with robot.step() ???

# robot.Keyboard refers to the class (meets 5) which can be used as a constructor (meeting 4-6) which will
# return robot.keyboard if it is already a Keyboard (meeting 1/uniqueness) and otherwise construct a new Keyboard
# and store it as robot.keyboard (meeting 2). robot.keyboard is initially set to be a placeholder InstallOnDemand
# object that, upon first reference will replace itself with a newly constructed Keyboard (meeting 0-2),
# which will automatically enable itself (meeting 3), and this will be the recommended way of accessing the
# keyboard (meeting 3).  Upon creation, the keyboard will produce a dummy reading until it has enough time
# to generate an actual reading, enabling it to work sensibly even without any prior enabling (meeting 3).

class InstallOnDemand:
    """A placeholder class attribute that, upon first access, will replace itself with a RobotPseudoDevice
       like a Joystick, Keyboard, Mouse or Battery.  Note that the recommended way to begin using such
       a device is with a command like robot.keyboard.sampling = True """
    def __init__(self, PseudoDeviceClass: type):
        self._class = PseudoDeviceClass # remember what class of pseudodevice we'll need to create
    def __set_name__(self, owner: type, name: str):
        self._owner = owner # have pseudodevice class remember what possesses this placeholder (usually Robot)
        self._name = name   # remember this placeholder's name (usually _class.lower() )
    def __get__(self, instance, cls=None):
        """This will be called whenever anyone tries to access robot.name.
           This will construct an object of the appropriate class, which will store itself
           as as robot.name, overwriting this installer, so that future references to
           robot.name will go directly to this freshly installed pseudodevice."""
        dev = self._class(sampling = True)
        globals()[self._name] = dev   # for RobotPseudoSensors this is also done by their __init__
        return dev

class RobotPseudoSensor(Sensor, auto_link_wb_methods = False):
    """This abstract class contains various unique sensor-like objects that don't correspond to device nodes within
       the simulation, like the joystick, keyboard, mouse, and battery.  These are made available as
       attributes of the Robot class itself, e.g. as robot.keyboard  Those attributes are initially filled by
       an InstallOnDemand placeholder that will create relevant RobotPseudoSensor on demand.
       This class is responsible for storing the newly created RobotPseudoSensor in the relevant robot.attribute
       (replacing the placeholder), for initializing the pseudo-sensor's sampling period (defaulting to the
       simulation's basic timestep), and for providing a useful enough __repr__."""
    @property
    def _robot_attribute(self) -> str: return type(self).__name__.lower() # robot.attribute this will be accessible as

    def __init__(self, sampling = True):
        self.sampling = sampling
        globals()[ self._robot_attribute ] = self

    def __repr__(self): return self._robot_attribute # e.g. 'keyboard'


class Battery(RobotPseudoSensor, SurrogateValue):
    """The battery is not implemented in Webots as a separate node in the scene tree, but in Python we interface
       with it much as we would with other sensors."""
    _api_name = "robot_battery_sensor"  # Now Sensor superclass can find associated enable/disable/period methods

    def __init__(self, sampling = True):
        """The battery is not implemented in Webots as a separate node in the scene tree, but in Python we interface
           with it much as we would with other sensors.  Any reference to robot.battery will enable battery sensor
           readings using the simulation's basic timestep as the sampling period, or this may be adjusted with
           robot.battery.sampling .  The battery sensor's value is accessible via robot.battery.value
           though, as a SurrogateValue, robot.battery itself can usually play the same role as this .value."""
        super().__init__(self, sampling = sampling)
        # TODO initialize dummy .value in case this gets read too soon

    wb.wb_robot_battery_sensor_get_value.restype = c_double
    @property
    def value(self) -> float:
        """The fraction of charge remaining in the robot battery."""
        return wb.wb_robot_battery_sensor_get_value()

import functools
def _key_boolean(fn:Callable) -> Callable:
    @functools.wraps(fn)
    def wrapped_fn(self:'Key', other:Union['Key',str]) -> 'Key':
        return fn(self, ord(other) if isinstance(other, str) else other)
    return wrapped_fn
def _key_arithmetic(fn:Callable) -> Callable:
    @functools.wraps(fn)
    def wrapped_fn(self:'Key', other:Union['Key',str]) -> 'Key':
        return Key( fn(self, ord(other) if isinstance(other, str) else other) )
    return wrapped_fn
class Key(int):
    """A Key represents a particular keystroke or combination of keys like CONTROL+ALT+DELETE.
       These are stored as integers, typically the ord() of the character on the key, with values like
       keyboard.SHIFT, .CONTROL, or .ALT added to indicate combined keys.
       key.SHIFT will be truth-like if this key-combination involves the SHIFT key. Similarly for .CONTROL and .ALT.
       key.KEY returns the basic key from this key-combination, excluding SHIFT, CONTROL and ALT.
       This class has overloaded operations allowing comparison and arithmetic combination with
       single-character strings, so e.g. `keyboard.CONTROL + 'A'` is equivalent to `keyboard.CONTROL + ord('A')`
       and `key == 'A'` is equivalent to `key == ord('A')`.
       TODO These comparisons are case-insensitive, and also take various symbols to be equivalent to numerals that
        they share a key with (on the swiss keyboard upon which Webots is based).  So, even though Webots represents
        SHIFT-A as keyboard.SHIFT+ord('a') and SHIFT-1 as keyboard.SHIFT+ord('!'), TODO or is it '+'???
        these will count as being equal to
        keyboard.SHIFT+'A' and keyboard.SHIFT+'1' respectively.
       """
    __eq__ = _key_boolean(int.__eq__)
    __gt__ = _key_boolean(int.__gt__)
    __lt__ = _key_boolean(int.__lt__)
    __ge__ = _key_boolean(int.__ge__)
    __le__ = _key_boolean(int.__le__)
    __add__ = _key_arithmetic(int.__add__)
    __sub__ = _key_arithmetic(int.__sub__)
    __and__ = _key_arithmetic(int.__and__)
    __or__ = _key_arithmetic(int.__or__)
    __xor__ = _key_arithmetic(int.__xor__)
    __radd__ = _key_arithmetic(int.__radd__)
    __rsub__ = _key_arithmetic(int.__rsub__)
    __rand__ = _key_arithmetic(int.__rand__)
    __ror__ = _key_arithmetic(int.__ror__)
    __rxor__ = _key_arithmetic(int.__rxor__)

    @property
    def SHIFT(self):
        """Equals keyboard.SHIFT if this keystroke involved the SHIFT key, and 0 otherwise."""
        return self & WB_KEYBOARD_SHIFT

    @property
    def CONTROL(self):
        """Equals keyboard.CONTROL if this keystroke involved the CONTROL key, and 0 otherwise."""
        return self & WB_KEYBOARD_CONTROL

    @property
    def ALT(self):
        """Equals keyboard.ALT if this keystroke involved the ALT key, and 0 otherwise."""
        return self & WB_KEYBOARD_ALT

    @property
    def KEY(self):
        """Equals the basic component of this key, excluding SHIFT, CONTROL and ALT"""
        return self & WB_KEYBOARD_KEY

    def __repr__(self):
        return(  ("CONTROL-" if self.CONTROL else "")
               + ("ALT-" if self.ALT else "")
               + ("SHIFT-" if self.SHIFT else "")
               + f"'{chr(self.KEY)}'"  )

    def __hash__(self):
        """Each Key hashes to where its basic character (ignoring CTRL/ALT/SHIFT) hashes.
           This means you could look up a Key in a set or dictionary of characters, but not of ints."""
        return hash(chr(self & WB_KEYBOARD_KEY))


class ButtonTracker:
    """This abstract class contains (pseudo-)devices like Joystick and Keyboard that have pressable buttons/keys,
       yielded from the C-API one at a time by self._depressed_buttons, defined in each class.
       This will automatically store the set of currently-depressed buttons as self.value, and
       sets of just-pressed and just-released buttons as self.pressed and self.released,
       automatically refreshing when accessed again at a later simulation time."""

    _depressed_buttons:Callable # defined in each subclass

    @timed_cached_property  # this will automatically update itself whenever accessed at a later simulation time
    def value(self) -> Set[Key]:
        """The set of currently depressed keys/buttons in this keyboard/joystick. For most purposes, you can use
           this device itself as a surrogate for this value, e.g. `for depressed_key in keyboard`.
           Note that keyboard.pressed is the set of recently-pressed keys, and keyboard.released is the set of
           recently-released keys.  Similarly for joystick.pressed and joystick.released."""
        self.old_value = self.__dict__.get('value', set())
        return set(self._depressed_buttons) # @timed_cached_property will cache this as self.value, timestamped

    @timed_cached_property  # will automatically update itself and self.value whenever accessed at later sim time
    def pressed(self) -> Set[Key]:
        """The set of keys/buttons that just became pressed this timestep."""
        new_value = self.value # will automatically refresh self.value and self.old_value if needed
        return new_value - self.old_value

    @timed_cached_property  # will automatically update itself and self.value whenever accessed at later sim time
    def released(self) -> Set[Key]:
        """The set of keys/buttons that just became released this timestep."""
        new_value = self.value # will automatically refresh self.value and self.old_value if needed
        return self.old_value - new_value

    # Even though we read all buttons at once, this will yield each button exactly once; used by
    # keyboard.getKey() and joystick.getButton() for backwards compatibility
    @timed_cached_property  # this will automatically update itself whenever accessed at a later simulation time
    def _iter_cached_buttons(self):
        """Each timestep, this will return a new iterator that will yield this timestep's depressed buttons once."""
        return iter(self.value)  # this access of self.value will refresh its own cache, if necessary


class Joystick(ButtonTracker, RobotPseudoSensor):
    """robot.joystick is used to detect inputs from a joystick.  Upon first access, joystick readings will automatically
       be enabled with the basic timestep as the sampling period, alterable by setting joystick.sampling = n_msec.
       Setting joystick.sampling to None stops generating readings, or to True again uses the basic timestep.
       Upon first access, a joystick will become associated with this controller, if one is available.
       joystick.is_connected will be True if a joystick is successfully connected in this way.
       joystick.pressed returns the set of buttons that have just become pressed this timestep.
       joystick.released returns the set of buttons that have just been released this timestep.
       joystick.value returns the set of buttons that are currently depressed (and may have been for some time).
       For most purposes joystick itself serves as a surrogate for this .value, e.g. `for depressed_button in joystick`
       """
    def __init__(self, sampling = True):
        super().__init__( self, sampling = sampling )
        # TODO initialize dummy .value in case this gets read too soon

    @property
    def _depressed_buttons(self) -> Key:
        """Yields currently depressed buttons, one at a time, straight from the Webots C-API, stopping
           when the C-API returns -1. Users shouldn't use this directly, as doing so would interfere with
           automatically computing joystick.value, joystick.pressed and joystick.released."""
        while True:
            button = wb.wb_joystick_get_pressed_button()
            if button == -1: return
            yield Key(button)

    @use_docstring_as_deprecation_warning
    def getPressedButton(self)->int:
        """DEPRECATED: Joystick.getPressedButton() is deprecated.  `joystick.value` is the set of currently depressed
           buttons.  `joystick.pressed` and `joystick.released` are sets of just-pressed and just-released buttons."""
        return next( self._iter_cached_buttons, -1 )

    wb.wb_joystick_is_connected.restype = c_bool
    def is_connected(self)->bool:
        return wb.wb_joystick_is_connected()
    @use_docstring_as_deprecation_warning
    def isConnected(self)->bool:
        """DEPRECATED. Joystick.isConnected() is deprecated. Please use robot.joystick.is_connected"""
        return wb.wb_joystick_is_connected()

    # TODO these need to be (auto-)converted to properties
    def getModel(self) -> str:
        return wb.wb_joystick_get_model()

    "axis_count"

    def getNumberOfAxes(self)->int:
        return wb.wb_joystick_get_number_of_axes()

    "axis"

    def getAxisValue(self, axis:int)->int:
        return wb.wb_joystick_get_axis_value(axis)

    "POV_count"

    def getNumberOfPovs(self)->int:
        return wb.wb_joystick_get_number_of_povs()

    "POV"

    def getPovValue(self, pov)->int:
        return wb.wb_joystick_get_pov_value(pov)

    "force"

    def setConstantForce(self, level:int):
        wb.wb_joystick_set_constant_force(level)

    "force_duration"

    def setConstantForceDuration(self, duration:float):
        wb.wb_joystick_set_constant_force_duration(c_double(duration) )

    "force_axis"

    def setForceAxis(self, axis:int):
        wb.wb_joystick_set_force_axis(axis)

    "autocentering"

    def setAutoCenteringGain(self, gain:float):
        wb.wb_joystick_set_auto_centering_gain(c_double(gain) )

    "resistance"

    def setResistanceGain(self, gain:float):
        wb.wb_joystick_set_resistance_gain(c_double(gain) )


class Keyboard(ButtonTracker, RobotPseudoSensor, SurrogateValue):
    """robot.keyboard is used to detect user keystrokes.  Upon first access, keyboard readings will automatically
       be enabled with the basic timestep as the sampling period, alterable by setting keyboard.sampling = n_msec.
       Setting keyboard.sampling to None stops generating readings, or to True again uses the basic timestep.
       keyboard.pressed returns the set of keys that have just become pressed this timestep.
       keyboard.released returns the set of keys that have just been released this timestep.
       keyboard.value returns the set of keys that are currently depressed (and perhaps have been for some time).
       For most purposes keyboard itself serves as a surrogate for this .value, e.g. `for depressed_key in keyboard`.
       Keys are returned as Key objects, which are like ints, but with overloaded comparison and arithmetic methods,
       making, e.g., `key=='A'` be shorthand for `key==ord('A')`
       key.CONTROL will be truth-like if CONTROL was part of this key-combination.  Similarly for .SHIFT and .ALT.
       key.KEY returns the basic component of key, excluding CONTROL, SHIFT and ALT.
       Various special keys equal other constants, like keyboard.UP or keyboard.HOME."""
    END = Key( WB_KEYBOARD_END )
    HOME = Key( WB_KEYBOARD_HOME )
    LEFT = Key( WB_KEYBOARD_LEFT )
    UP = Key( WB_KEYBOARD_UP )
    RIGHT = Key( WB_KEYBOARD_RIGHT )
    DOWN = Key( WB_KEYBOARD_DOWN )
    PAGEUP = Key( WB_KEYBOARD_PAGEUP )
    PAGEDOWN = Key( WB_KEYBOARD_PAGEDOWN )
    NUMPAD_HOME = Key( WB_KEYBOARD_NUMPAD_HOME )
    NUMPAD_LEFT = Key( WB_KEYBOARD_NUMPAD_LEFT )
    NUMPAD_UP = Key( WB_KEYBOARD_NUMPAD_UP )
    NUMPAD_RIGHT = Key( WB_KEYBOARD_NUMPAD_RIGHT )
    NUMPAD_DOWN = Key( WB_KEYBOARD_NUMPAD_DOWN )
    NUMPAD_END = Key( WB_KEYBOARD_NUMPAD_END )
    KEY = Key( WB_KEYBOARD_KEY )
    SHIFT = Key( WB_KEYBOARD_SHIFT )
    CONTROL = Key( WB_KEYBOARD_CONTROL )
    ALT = Key( WB_KEYBOARD_ALT )

    @property
    def _depressed_buttons(self) -> Key:
        """Yields currently depressed keys, one at a time, straight from the Webots C-API, stopping
           when the C-API returns -1. Users shouldn't use this directly, as doing so would interfere with
           automatically computing keyboard.value, keyboard.pressed and keyboard.released."""
        while True:
            k = wb.wb_keyboard_get_key()
            if k == -1: return
            yield Key(k)

    @use_docstring_as_deprecation_warning
    def getKey(self)->int:
        """DEPRECATED: Keyboard.getKey() is deprecated. `keyboard.value` is the set of currently depressed keys.
           `keyboard.pressed` and `keyboard.released` are sets of just-pressed and just-released keys."""
        return next( self._iter_cached_buttons, -1 )


class Mouse(RobotPseudoSensor):
    """robot.mouse provides information about the current position of the mouse.  Upon first reference to robot.mouse
       this will automatically be enabled to produce new readings each basic timestep, or this may be adjusted in
       the same manner as other sensors with robot.mouse.sampling = new_period.
       mouse.left, .middle, and .right indicate whether the respective mouse button is pressed.
       mouse.uv is a 2D vector indicating the fractions across and down the simulation window the mouse cursor is.
       mouse.xyx is a 3D vector indicating the 3D global coordinates of the cursor.  This computation is fairly
                 expensive so must first be enabled with mouse.xyz.sampling = True, and typically should be
                 disabled again by setting this to None when not needed.
       These coordinates are also accessable as mouse.u, .v, .x, .y, and .z"""

    # TODO XXX *** ??? can the mouse update between simulation steps??? If so, the caching I use here would be bad!!!

    class State( ctypes.Structure ):
        # declare fields for c_types conversion from C-API
        _fields_ = [('left',   c_bool ), ('middle', c_bool ), ('right',  c_bool ),
                    ('u', c_double ), ('v', c_double ),
                    ('x', c_double ), ('y', c_double ), ('z', c_double ) ]

        # declare fields for Python linters
        left: bool
        middle: bool
        right: bool
        u: float
        v: float
        x: float
        y: float
        z: float

        def __repr__(self): return f"Mouse.State( {self.u:.3f}, {self.v:.3f} )"

    _value: State = State( *([0]*8) ) # will store a cached version of the .value
    last_cache_time = -1.0            # will store the latest simulation time when the cached ._value was uptodate

    wb.wb_mouse_get_state.restype = c_pointer(State)
    @timed_cached_property
    def value(self) -> 'Mouse.State':
        """A MouseState object that represents current left/right/middle, u/v, and x/y/z attributes of the mouse.
           It is generally preferable to access these directly, e.g., as mouse.left, rather than as mouse.value.left.
           The position values are also compiled into Vectors mouse.uv and mouse.xyz"""
        return Mouse.State.from_buffer_copy(wb.wb_mouse_get_state())
    @use_docstring_as_deprecation_warning
    def getState(self) -> 'Mouse':
        """DEPRECATED: Mouse.getState() is deprecated. State attributes are now accessible directly as mouse.left,
           mouse.right, mouse.middle, mouse.uv, and mouse.xyz"""
        return self

    # Rather than just making this a SurrogateValue, we define surrogate attributes for type-hints and docstrings
    @surrogate_attribute # this descriptor makes mouse.left return mouse.value.left; we provide type-hint and docstring
    def left  (self) -> bool:  "Indicates whether left mouse button is pressed."
    @surrogate_attribute
    def middle(self) -> bool:  "Indicates whether middle mouse button is pressed."
    @surrogate_attribute
    def right (self) -> bool:  "Indicates whether right mouse button is pressed."
    @surrogate_attribute
    def u     (self) -> float: "The fraction of the way across the simulation window the mouse cursor is."
    @surrogate_attribute
    def v     (self) -> float: "The fraction of the way down the simulation window the mouse cursor is."
    @surrogate_attribute
    def x     (self) -> float: "Global x coordinate of the mouse cursor (if you've set mouse.xyz.sampling = True)"
    @surrogate_attribute
    def y     (self) -> float: "Global y coordinate of the mouse cursor (if you've set mouse.xyz.sampling = True)"
    @surrogate_attribute
    def z     (self) -> float: "Global z coordinate of the mouse cursor (if you've set mouse.xyz.sampling = True)"

    @property
    def uv(self)->Vector:
        """A vector representing the mouse 2D position in as fractions 0..1 across and down the simulation window."""
        return Vector(self.value.u, self.value.v)

    # --- mouse xyz 3D position ---

    class XYZ_Sensor(SecondarySensor, VectorValue):
        _enable = wb.wb_mouse_enable_3d_position
        _disable = wb.wb_mouse_disable_3d_position
        _get_sampling =  wb.wb_mouse_is_3d_position_enabled

        @timed_cached_property
        def value(self):
            return Vector(self.value.x, self.value.y, self.value.z)

    @cached_property
    def xyz(self) -> XYZ_Sensor:
        """robot.mouse.xyz returns a Mouse.XYZ_Sensor object that allows access to the mouse's 3D position.
           Upon first reference to robot.mouse.xyz this will automatically be enabled to produce readings whenever
           the mouse itself does, readable and adjustable via robot.mouse.sampling, either True or None.
           robot.mouse.xyz.value returns the current reading, but for most purposes you can use mouse.xyz as a
           surrogate for this value."""
        xyz_sensor = Mouse.XYZ_Sensor()
        xyz_sensor.sampling = True
        return xyz_sensor

    @use_docstring_as_deprecation_warning
    def enable3dPosition(self):
        """DEPRECATED: Mouse.enable3dPosition() is deprecated.  Please use mouse.xyz.sampling = True"""
        mouse.xyz.sampling = True
    @use_docstring_as_deprecation_warning
    def disable3dPosition(self):
        """DEPRECATED: Mouse.disable3dPosition() is deprecated.  Please use mouse.xyz.sampling = None"""
        mouse.xyz.sampling = None
    @use_docstring_as_deprecation_warning
    def is3dPositionEnabled(self)->bool:
        """DEPRECATED: Mouse.is3dPositionEnabled() is deprecated.  Please use mouse.xyz.sampling"""
        return mouse.xyz.sampling


# === RobotModule class (needed to enable robot.properties to work) ===

class RobotModule(ModuleType):
    """Python controllers should begin with `import robot` to import the robot module.
       That module will be made to behave as though it is an instance of the RobotModule class defined here, mostly
       because we need a class to enable dynamic robot.properties, like robot.time .
       The robot module's __class__ is dynamically altered to be RobotModule."""

    MODE_SIMULATION = WB_MODE_SIMULATION
    MODE_CROSS_COMPILATION = WB_MODE_CROSS_COMPILATION
    MODE_REMOTE_CONTROL = WB_MODE_REMOTE_CONTROL

    EVENT_QUIT = WB_EVENT_QUIT
    EVENT_NO_EVENT = WB_EVENT_NO_EVENT
    EVENT_MOUSE_CLICK = WB_EVENT_MOUSE_CLICK
    EVENT_MOUSE_MOVE = WB_EVENT_MOUSE_MOVE
    EVENT_KEYBOARD = WB_EVENT_KEYBOARD
    EVENT_JOYSTICK_BUTTON = WB_EVENT_JOYSTICK_BUTTON
    EVENT_JOYSTICK_AXIS = WB_EVENT_JOYSTICK_AXIS
    EVENT_JOYSTICK_POV = WB_EVENT_JOYSTICK_POV

    _instance: 'RobotModule' # will store the single instance of this class, the module named 'robot'

    # --- robot time information ---

    @property
    def time(self)->float:
        """Returns the current simulation time, in seconds"""
        # Unfortunately when pytest starts a new test mid-simulation, core_api.time reverts to its initial value, so to
        # avoid that we make the first call to robot.time manually check the time, then replace itself with faster version
        type(self).time = RobotModule.quick_time
        return core_api.check_the_time()

    @property
    def quick_time(self)->float:
        """Returns the current simulation time, in seconds"""
        return core_api.time

    wb.wb_robot_get_basic_time_step.restype = c_double
    @property
    def timestep_ms(self) -> float:
        """Returns the value of the basicTimeStep field of the WorldInfo node, the number of milliseconds the
           simulation advances in each step of the physics simulation."""
        return wb.wb_robot_get_basic_time_step()
    @property
    def timestep_sec(self) -> float:
        """Returns the value of the basicTimeStep field of the WorldInfo node, expressed in seconds rather than ms.
           This is the fraction of a second the simulation advances in each step of the physics simulation."""
        return 0.001 * wb.wb_robot_get_basic_time_step()
    @use_docstring_as_deprecation_warning
    def getBasicTimeStep(self)->int:
        """DEPRECATED. robot.getBasicTimeStep is deprecated. Use robot.timestep_ms or robot.timestep_sec instead."""
        return wb.wb_robot_get_basic_time_step()

    @property
    def is_synchronized(self) -> bool:
        """Returns the boolean value corresponding to the synchronization field of the Robot node.
           If True, the simulation will await this controller to call robot.step() before it advances to next time step,
           so if this controller runs slowly, the simulation will run slowly.
           If False, the simulation will advance on its own without waiting for this controller, so if this
           controller runs slowly, the world will have change more between each time robot.step() is called."""
        return wb.wb_robot_get_synchronization()
    @use_docstring_as_deprecation_warning
    def getSynchronization(self)->bool:
        """DEPRECATION.  robot.getSynchronization is deprecated. Please use robot.is_synchronized instead."""
        return wb.wb_robot_get_synchronization()

    # --- robot information about itself ---

    wb.wb_robot_get_name.restype = c_char_p
    @property
    def name(self) -> str:
        """Returns the value of the name field for this robot node in the scene tree."""
        return wb.wb_robot_get_name().decode()
    @use_docstring_as_deprecation_warning
    def getName(self)->str:
        """DEPRECATED. robot.getName is deprecated. Please use robot.name instead."""
        return wb.wb_robot_get_name().decode()

    wb.wb_robot_get_model.restype = c_char_p
    @property
    def model(self) -> str:
        """Returns the value of the model field for this robot node in the scene tree."""
        return wb.wb_robot_get_model().decode()
    @use_docstring_as_deprecation_warning
    def getModel(self)->str:
        """DEPRECATED. robot.getModel is deprecated. Please use robot.model instead."""
        return wb.wb_robot_get_model().decode()

    @property
    def is_supervisor(self) -> bool:
        """Returns True if this robot is of type supervisor, and False otherwise.  Supervisor robots have access
           to additional methods that ordinary robots lack."""
        return wb.wb_robot_get_supervisor()
    @staticmethod
    @use_docstring_as_deprecation_warning
    def getSupervisor():
        """DEPRECATION.  robot.getSupervisor() is deprecated. Please use robot.is_supervisor instead."""
        return wb.wb_robot_get_supervisor()


    wb.wb_robot_get_custom_data.restype = c_char_p
    @property
    def custom_data(self)->str:
        """The contents of the robot's customData field.  This may be read with robot.custom_data, or altered
           with robot.custom_data = new_data"""
        return wb.wb_robot_get_custom_data().decode()
    @custom_data.setter
    def custom_data(new_data: str):
        """Changes the value of this robot's customData field."""
        wb.wb_robot_set_custom_data( new_data.encode() )

    @use_docstring_as_deprecation_warning
    def getCustomData(self)->str:
        """DEPRECATION. robot.getCustomData is deprecated. Please use robot.custom_data instead."""
        return wb.wb_robot_get_custom_data().decode()
    @use_docstring_as_deprecation_warning
    def setCustomData( self, new_data: str ):
        """DEPRECATION: robot.setCustomData is deprecated. Please use robot.custom_data = new_string."""
        self.custom_data = new_data.encode()

    @property
    def mode(self) -> int:
        """Returns or changes the current operating mode for the controller.
           The returned mode will be robot.MODE_SIMULATION, robot.MODE_CROSS_COMPILATION, or robot.MODE_REMOTE_CONTROL.
           Setting `robot.mode = mode, arg` will change the robot's mode, where the first given value will be
           the mode to shift to (one of the three listed above), and the second is a string that will be
           passed to the wbr_start function of the remote control plugin.  If just the mode value is given, the arg
           will default to the empty string."""
        return wb.wb_robot_get_mode()
    @mode.setter
    def mode(self, mode_comma_string:Tuple[int,str]):
        if isinstance( mode_comma_string, Sequence ):
            mode, arg = mode_comma_string
        else:
            mode, arg = mode_comma_string, ""
        wb.wb_robot_set_mode(mode, arg.encode())

    @staticmethod
    @use_docstring_as_deprecation_warning
    def getMode() -> int:
        """DEPRECATION.  robot.getMode() is deprecated. Please use robot.mode instead."""
        return wb.wb_robot_get_mode()
    @staticmethod
    @use_docstring_as_deprecation_warning
    def setMode(mode:int, arg:str):
        """DEPRECATED: robot.setMode(mode,arg) is deprecated.  Please use `robot.mode = mode, arg`. """
        return wb.wb_robot_set_mode(mode, arg.encode() )

    # TODO this is more computationally intensive than what we would usually use a property for. Leave as function?
    @staticmethod
    def get_urdf(prefix:str = '' ) -> str:
        """Allows a robot controller to export URDF, an XML format for representing a robot model.
           A prefix for URDF link and joint names can be specified by prefix (useful in multi-robot systems to
           distinguish different robots).  The function is particularly useful for ROS applications in which
           URDF is widely used to describe robot models.  The exported URDF may be incomplete -- see docs."""
        return wb.wb_robot_get_urdf(prefix)
    @use_docstring_as_deprecation_warning
    def getUrdf(prefix:str = '' ) -> str:
        """DEPRECATION. Webots is shifting to conventional Python naming. Please use robot.get_urdf rather than .getUrdf()."""
        return wb.wb_robot_get_urdf(prefix)

    # --- robot world information ---

    wb.wb_robot_get_project_path.restype = c_char_p
    @property
    def project_path(self) -> str:
        """Returns the full path of the current project, that is the directory which contains the worlds and
           controllers subdirectories (among others) of the current simulation world."""
        return wb.wb_robot_get_project_path().decode()
    @use_docstring_as_deprecation_warning
    def getProjectPath(self)->str:
        """DEPRECATED. robot.getProjectPath() is deprecated. Please use robot.project_path instead."""
        return wb.wb_robot_get_project_path().decode()

    wb.wb_robot_get_world_path.restype = c_char_p
    @property
    def world_path(self) -> str:
        """Returns the full path of the current opened world, including the filename and its .wbt extension."""
        return wb.wb_robot_get_world_path().decode()
    @use_docstring_as_deprecation_warning
    def getWorldPath(self)->str:
        """DEPRECATED. robot.getWorldPath() is deprecated. Please use robot.world_path instead."""
        return wb.wb_robot_get_world_path().decode()

    # --- robot window control ---

    #TODO give these pythonic snake_case_names?
    #TODO The C-API also allows for sending/receiving integers, but the swig controller.py did not support this
    #       could add some type checking to allow a unified method that can send either
    @staticmethod
    def wwiSendText(text:str):
        """Send a message to a JavaScript function running in the HTML robot window.
           The message is received using the webots.window("<robot window name>").receive method of
           the Webots JavaScript API."""
        return wb.wb_robot_wwi_send_textXXX(text)
    @staticmethod
    def wwiReceiveText()->str:
        """Receive a message sent from a JavaScript function running in the HTML robot window.
           The message is sent using the webots.window("<robot window name>").send method of
           the Webots JavaScript API."""
        return wb.wb_robot_wwi_receive_textXXX()

    # --- robot user input ---

    # TODO XXX figure out how to pass WbUserInputEvent through the C-API
    @staticmethod
    def wait_for_user_input_event(event_type:WbUserInputEvent, timeout: int)->WbUserInputEvent:
        """Suspends the simulation until
           (a) an event of the specified event_type occurs (returns a specific type of event),
           (b) timeout milliseconds have elapsed (returns EVENT_NO_EVENT), or
           (c) the simulation is terminated (returns .EVENT_QUIT).
           Simple event types are stored as Robot attributes .EVENT_QUIT, EVENT_NO_EVENT,
            .EVENT_MOUSE_CLICK, .EVENT_MOUSE_MOVE, .EVENT_KEYBOARD, .EVENT_JOYSTICK_BUTTON,
            .EVENT_JOYSTICK_AXIS, .EVENT_JOYSTICK_POV.  Disjunctive event types may combined with
             bitwise or, e.g. robot.EVENT_MOUSE_MOVE | robot.EVENT_MOUSE_CLICK."""
        return wb.wb_robot_wait_for_user_input_event(event_type, timeout)

    @use_docstring_as_deprecation_warning
    def waitForUserInputEvent(event_type:WbUserInputEvent, timeout: int)->WbUserInputEvent:
        """DEPRECATION: Python is shifting to conventional python naming.
           Please change waitForUserInputEvent to wait_for_user_input_event."""
        return wb.wb_robot_wait_for_user_input_event(event_type, timeout)

    # --- Pseudo-sensors: battery, joystick, keyboard, and mouse ----------------

    # Create placeholders that will replace themselves with the relevant pseudo-sensor upon first access
    battery: Battery = InstallOnDemand(Battery)
    joystick: Joystick = InstallOnDemand(Joystick)
    keyboard: Keyboard = InstallOnDemand(Keyboard)
    mouse: Mouse = InstallOnDemand(Mouse)

    # Deprecate old robot.methods for interacting with pseudo-sensors
    @use_docstring_as_deprecation_warning
    def getJoystick(self):
      """DEPRECATION. robot.getJoystick() is deprecated. You may refer to it as robot.joystick,
         and enable it with robot.joystick.sampling = True."""
      return RobotModule._instance.joystick
    @use_docstring_as_deprecation_warning
    def getKeyboard(self):
      """DEPRECATION. robot.getKeyboard() is deprecated. You may refer to it as robot.keyboard,
         and enable it with robot.keyboard.sampling = True."""
      return RobotModule._instance.keyboard
    @use_docstring_as_deprecation_warning
    def getMouse(self):
      """DEPRECATION. robot.getMouse() is deprecated. You may refer to it as robot.mouse,
         and enable it with robot.mouse.sampling = True."""
      return RobotModule._instance.mouse

    @use_docstring_as_deprecation_warning
    def batterySensorEnable(self, sampling_period ):
        """DEPRECATION. robot.batterySensorEnable is deprecated. Please use robot.battery.sampling = new_period
           or any reference to robot.battery will automatically enable it with the simulation's basic timestep."""
        return wb.wb_robot_battery_sensor_enable( sampling_period )
    @use_docstring_as_deprecation_warning
    def batterySensorDisable(self):
        """DEPRECATION. robot.batterySensorDisable is deprecated. Please use robot.battery.sampling = None"""
        return wb.wb_robot_battery_sensor_disable()
    @use_docstring_as_deprecation_warning
    def batterySensorGetSamplingPeriod(self):
        """DEPRECATION. robot.batterySensorGetSamplingPeriod is deprecated. Please use robot.battery.sampling"""
        return wb.wb_robot_battery_sensor_get_sampling_period()
    @use_docstring_as_deprecation_warning
    def batterySensorGetValue(self):
        """DEPRECATION. robot.batterySensorGetValue is deprecated. Please use robot.battery.value,
            or for most purposes, robot.battery can be used in place of its own .value"""
        return wb.wb_robot_battery_sensor_get_value()

    # Motion = Motion  # TODO allow creation of Motion objects via robot.Motion

    # --- handling Robot Devices ------------------------------------------------------

    @staticmethod
    def getDevice(name_or_index: Union[str,int] )->Device:
        """DEPRECATED. Returns a Device object with the given name or index, or None if there is none.
           This object will automatically be converted to the relevant device subclass, e.g. DistanceSensor.
           This function is deprecated and may eventually be removed. Use robot.Device(name_or_index) instead, or
           better yet, use a specific subclass like robot.Motor(name), which will provide helpful type-hinting and
           documentation for users working in a Python editing environment that supports such things, like Pycharm."""
        WarnOnce(f"robot.getDevice(name) is deprecated. Please use robot.Device(name_or_index) instead, "
                 f"or better yet, some specific subclass like robot.Motor(name).")
        return Device(name_or_index)

    def getDeviceByIndex(self, index: Union[str,int] )->Device:
        """DEPRECATED. robot.getDeviceByIndex(i) is deprecated.  Please use robot.devices[i] instead.
           Or use specific subclasses like robot.Motor(name_or_index), which will provide helpful type-hinting and
           documentation for users working in a Python editing environment that supports such things, like Pycharm."""
        WarnOnce(f"robot.getDeviceByIndex(i) is deprecated. Please use robot.devices[i] instead, "
                 f"or better yet, some specific subclass like robot.Motor(name_or_index).")
        return RobotModule._instance.devices[ index ]

    @use_docstring_as_deprecation_warning
    def getNumberOfDevices(self):
        """DEPRECATED. robot.getNumberOfDevices() is deprecated.  Please use len(robot.devices) instead."""
        return wb.wb_robot_get_number_of_devices()



# === Re-declare RobotModule properties for linters ===

# Pycharm's linter doesn't automatically recognize module class properties, so these are declared again here on the
# module instance to make them and their documentation show up for the linter.  We set this equal to the property
# to get the docstring, but declare the returned type from that property so linters will use it.
# At runtime, the class-level property definitions always take priority over mere instance attributes like these.
time:float = RobotModule.time
name:str = RobotModule.name
model:str = RobotModule.model
keyboard: Keyboard
joystick: Joystick
mouse: Mouse
battery: Battery

# === Morph robot module to be member of RobotModule class ===

RobotModule._instance = sys.modules[__name__] # Store a reference to this module, the sole RobotModule instance
RobotModule._instance.__class__ = RobotModule # This module will now behave as though it is a Robot, making robot.time work
