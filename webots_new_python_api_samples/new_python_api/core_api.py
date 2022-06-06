"""This part of the "new Python API" for Webots is responsible for initiating the ctypes connection to the Webots C-API,
   and for some low-level interaction with that API, including the .step() function and .time property that
   are in common to both world and robot modules."""

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

import ctypes
import functools
import os as _os
from typing import Callable

import settings
from WB_CONSTANTS import * # import all Webots constants at global scope

# Import the Webots C API
# TODO this path will need to be changed to work on other computers!!!
# wb = ctypes.CDLL(r"D:\Program Files\Webots\lib\controller\Controller")
wb = ctypes.cdll.LoadLibrary(_os.path.join(_os.environ['WEBOTS_HOME'], 'lib', 'controller', 'Controller.dll'))

# === Initializing the robot/supervisor ===

def init(again = True):
    """Initializes this Python controller's control over a robot.
       This is automatically called when importing world or robot, so users typically needn't call this explicitly.
       #TODO make automatic initialization depend on some optional setting
       In the case of an "internal" controller (one specifically named in the robot's controller field) Webots starts
       the controller and waits for it to claim its robot by calling this.
       For "external" controllers (ones whose robots have "<extern>" as the value in their controller field)
       the controller must be started externally, and will lay claim to a robot by calling this.  See Webots
       docs for information on how a robot is chosen in cases where multiple robots expect external controllers.
       If again is false, this will make initialization moves only if such moves have not been attempted already,
       e.g. this prevents initialization from occurring twice if both robot and world are imported.
       If again is true, this will make intitialization moves again, even if this has been done before,
       potentially useful e.g., for an external controller to exert control over a new or reset robot.
       """
    #TODO test what problems reinitializing might have if unneeded, and what benefits it might have if ever needed
    if init.has_been_initialized and not again: return
    wb.wb_robot_init()
    init.has_been_initialized = True
init.has_been_initialized = False

# === Handling robot.time / world.time ===

wb.wb_robot_get_time.restype = ctypes.c_double
wb.wb_robot_get_basic_time_step.restype = ctypes.c_double

# TODO Unfortunately when using pytest to start up tests, this does not immediately give the correct time!
time = wb.wb_robot_get_time() # is automatically updated by step()

def check_the_time()->float:
    """Returns the current simulation time in seconds, and updates core_api.time.
       core_api.time is set when the core_api is loaded, and automatically updated each time step() is called,
       so it is usually right. The main known exception is that, when using pytest to generate tests with an
       external controller, it reverts back to the initial time at the start of each test."""
    global time
    time = wb.wb_robot_get_time()  # update cached time to assist with other caching
    return time

class timed_cached_property:
    """A timed_cached_property works much like a cached_property (a low priority descriptor that stores its return
       value as the corresponding instance-attribute so future attempts to retrieve this value will get this cached
       value directly without retriggering the property getter), except it checks whether the simulation time has
       advanced past the time of the last caching, and if so, refreshes the stale cache."""
    # Note: Some objects may veto updating by setting a higher, or even infinite, `_{__name__}_last_update_time`

    def __init__(self, getter ):
        self.__doc__ = getter.__doc__
        self.__name__ = getter.__name__
        functools.update_wrapper(self, getter)
        self.update_name = f"_{self.__name__}_last_update_time"
        self._getter = getter

    def __repr__(self): return f"timed_cached_property( {self.__name__} )"

    def __get__(self, instance, cls=None):
        if instance is None: return self # C.f returns descriptor itself
        if time <= instance.__dict__.get(self.update_name, -1):
            return instance.__dict__[self.__name__]
        value = self._getter(instance)
        instance.__dict__[self.__name__] = value # cache value f(i) as i.f
        instance.__dict__[self.update_name ] = time
        return value

    # We need to define a setter to keep high priority for the getter to check if the cache has become stale
    def __set__(self, instance, value):
        """By default a timed_cached_property raises an AttributeError if someone tries to set instance.attribute."""
        raise AttributeError(f"Can't set protected attribute {instance}.{self.__name__}")

    def setter(self, setter: callable)->'timed_cached_property':
        """This decorator allows for defining a setter for this descriptor, much like Python's @property .setter"""
        @functools.wraps(self._getter) # copy name and docstring of our decorated getter
        def _simple_set(self, instance, value):
            setter(instance, value) # wrap the decorated setter in one that accepts the needed self arg too
        self.__set__ = _simple_set
        return self
    set = setter # an alias for setter to avoid current linting bugs

# === robot.step / world.step ===

class StepReturnValue(int):
    """The return value of robot.step and world.step will be coerced to be of this class.  Unlike an ordinary integer,
       this will have boolean True if the integer differs from settings.false_step_return_value which defaults to -1,
       making these step() functions return a True-like value whenever it is fine to continue, and a False-like
       value when it is time to exit, thus making it quite sensible to head a main loop with `while robot.step():`.
       Changing this setting to 0 would revert to the older behavior which lumps together as "True" both exit
       conditions (-1) and some unremarkable asynchronous timing discrepancies (positive values)."""
    def __bool__(self):
        return self != settings.false_step_return_value

def step(duration:int = True, exit = SystemExit) -> StepReturnValue:
    """This function is available as robot.step() and world.step().
       This makes the Webots simulation advance for duration simulated milliseconds (or the default duration = True
       makes it advance for a single basic timestep), after which the flow of control will return to this
       controller to determine what to do next.  For supervisor controllers, calling step(0) enables some supervisor
       commands to take immediate effect without advancing the simulation time.
       This returns an integer which roughly represents the discrepancy (in milliseconds) between the requested
       duration and how much simulation time has actually passed. For synchronous controllers and fast-acting
       asynchronous controllers, this discrepancy should always be 0, but for slower asynchronous ones it may be
       higher -- see Webots docs for details.  Sometimes, when step() is called, Webots will determine that your
       controller should stop, e.g., because the user hit the reset button, or because a supervisor controller issued
       commands to restart your robot's controller. In such cases, if exit = SystemExit (the default) this will use
       Python's `raise SystemExit` to cause your controller to exit immediately unless you capture this by encasing
       the call to step() somewhere within a `try:` block with an `except SystemExit:` clause.
       If you set exit to any other value, Webots will instead return the value -1 to indicate that it is time to exit.
       If you do prevent your controller from automatically exiting in either of these ways, your controller should
       perform whatever last rites you'd like it to do (e.g. printing some values or saving them to file) before
       stopping itself, e.g. by reaching the end of the program or by using `raise SystemExit`. It should also do this
       quickly, because after about 1 second, Webots will stop waiting and kill the process itself."""
    global time
    if duration is True: duration = wb.wb_robot_get_basic_time_step()
    value = StepReturnValue(wb.wb_robot_step(int(duration)))
    time = wb.wb_robot_get_time()  # update cached time to assist with other caching
    if value == -1 and exit is SystemExit:
        raise SystemExit("This controller was stopped by Webots.")
    return value

# === Console coloration constants ===

# TODO could make sense to make this its own little module, to allow from console import *
class Console:
    """This class is not expected to be instantiated, and serves only as a container for various .attributes.
       In print commands, these strings change the coloration of text printed to the Python/Webots console.
       Example: print(f"{Console.RED}HELLO {Console.YELLOW_BACK}WORLD!{Console.RESET}"""

    RESET = u'\u001b[0m'

    BOLD = u'\u001b[1m'
    UNDERLINE = u'\u001b[4m'

    BLACK_BACK = BLACK_BACKGROUND = u'\u001b[40m'
    RED_BACK = RED_BACKGROUND = u'\u001b[41m'
    GREEN_BACK = GREEN_BACKGROUND = u'\u001b[42m'
    YELLOW_BACK = YELLOW_BACKGROUND = u'\u001b[43m'
    BLUE_BACK = BLUE_BACKGROUND = u'\u001b[44m'
    MAGENTA_BACK = MAGENTA_BACKGROUND = u'\u001b[45m'
    CYAN_BACK = CYAN_BACKGROUND = u'\u001b[46m'
    WHITE_BACK = WHITE_BACKGROUND = u'\u001b[47m'

    BLACK = BLACK_FOREGROUND = u'\u001b[30m'
    RED = RED_FOREGROUND = u'\u001b[31m'
    GREEN = GREEN_FOREGROUND = u'\u001b[32m'
    YELLOW = YELLOW_FOREGROUND = u'\u001b[33m'
    BLUE = BLUE_FOREGROUND = u'\u001b[34m'
    MAGENTA = MAGENTA_FOREGROUND = u'\u001b[35m'
    CYAN = CYAN_FOREGROUND = u'\u001b[36m'
    WHITE = WHITE_FOREGROUND = u'\u001b[37m'

    CLEAR = CLEAR_SCREEN = u'\u001b[2J'

    @staticmethod
    def clear(): print(u'\u001b[2J')
