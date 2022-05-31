"""world.py contains the main user interface for Webots supervisor functionality in the "New Python API", much of
   which is implemented in nodes.py.  This functionality may be imported in python controllers via `import world`.
   The `world` module is manually morphed to become of class WorldModule which itself is a subclass of Node, so `world`
   inherits the full suite of Node functionality, and `world` serves as the root node in the "proxy scene tree".
   The "proxy scene tree" is a treelike structure of Python Nodes and Fields that is isomorphic to the corresponding
   structures within the Webots simulation's own scene tree. For most purposes this proxy tree behaves as though it
   were the actual scene tree in the simulation, so e.g. reading `world.ROBOT1.translation` reads the translation field
   of that robot, and setting world.ROBOT1.translation = [0,0,0] alters that field in the simulation.
   The proxy tree also treats descendants as Node.attributes, so you may be able to refer to the same node
   as both world.ROBOT1.Camera, and as world.Camera, if there is no other Camera situated before it in the tree.
   It is somewhat expensive to fetch up-to-date information about the scene tree through the Webots C-API
   (and, at the time of this writing, this becomes even more expensive the more you use it).
   So the proxy scene tree delays such requests as much as possible, until they are demanded.
   When initially imported, by default, world.py fetches the overall structure of the simulation via a single `export`,
   and parses that to populate the proxy scene tree with its overall structure of nodes and limited further information.
   Since each Node and Field may be arrived at in different ways that provide more or less information, these have
   many (cached_)properties that will expend the effort to fill in missing information on demand, and will cache it in
   cases where it is immutable, or where triggers have been set up fix/clear the cache when it changes.
   Proxy MFNode, SFNode, and `name` Fields maintain an uptodate cache of their contents (that's what the proxy tree is),
   coordinating this with their analogs in the simulation (though beware changes made by other supervisors!).
   All other fields of "real" Nodes do not maintain a cached value, and instead always "see through" to the corresponding
   fields in the simulation. In merely planned (aka "non-real") Nodes, all fields maintain a local cache of their values,
   since they have no analog in the simulation to "see through" to."""

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




import itertools
import sys
from types import ModuleType
from typing import Callable, Iterable, Union as _Union, Dict as _Dict, List as _List, Any as _Any
import ctypes
from ctypes import c_double, c_bool, c_char_p, c_void_p, POINTER as c_pointer

import core_api # so we can refer to core_api.time
from core_api import wb, step, check_the_time, init, Console as console, timed_cached_property
import WB_CONSTANTS as _wb_constants
from vectors import Vector, Color, Rotation, Vec2f, Vec3f, Vec4f, Vec2f_p, Vec3f_p, Vec4f_p, Iterable2f, Iterable3f, Iterable4f

import nodes as _nodes_module
from surrogate_values import SurrogateValue as _SurrogateValue
from nodes import Node, catalog, plan, FieldsBearer as _FieldsBearer
from nodes import wbNodeRef, wbFieldRef # TODO not sure I need these?
import robot as _robot
import settings

from webots_warnings import Warn, WarnOnce, use_docstring_as_deprecation_warning
from descriptors import descriptor, cached_property

_omitted = object()  # placeholder value to indicate keyword arguments that were omitted

# === Claim control over a robot (allows wb.functions to work) ===

if settings.initialize_robot_on_import:
    init(again = False) # initialize controller's control over a robot (if not done already, e.g. by importing robot)

# === Initialize world module's node-like attributes ===

# world module will morph to be a Node, but isn't created by Node.__new__, so we manually do equivalent initialization
wb.wb_supervisor_node_get_root.restype = wbNodeRef
root = sys.modules[__name__] # type: 'WorldModule' #`world.root` is an alias for `world` itself
_nodes_module.world = root                         # Nodes module needs access to world as default Node() search_area
_robot.Device._world = root                        # Some devices need access to world for convenient lookup of fields
_as_parameter_ = wb.wb_supervisor_node_get_root()  # wbNodeRef of root, used by ctypes functions if world passed as arg
is_real = True                                     # world is a real node in the simulation, not a merely planned node
id = wb.wb_supervisor_node_get_id(_as_parameter_)  # world.id is the id of the root (is this always 0?)
fields = _FieldsBearer( root )                     # world.fields is a _FieldsBearer for the root's fields
parent = parent_field = None                       # world is the root, so is not in a field of a parent
nickname = 'world'                                 # root's nickname is 'world', used in __repr__ and paths
__path__ = None                                    # ensures that Python will consider this a module not a package
is_fully_cataloged = False                         # indicates whether everything within has been read into proxy tree
is_inside_proto = False                            # the root is not descended inside a proto
is_hiding = False                                  # the root is not hiding inside an unexposed proto field or USE node
USE = ''                                           # the root does not USE any other node

catalog[id] = root                                 # store reference to world/root in catalog of all nodes

#TODO not sure if this is needed
def __bool__(world):
    print(f"Just FYI, bool(world) was called for some reason!?")
    return True

# === World label handling ===

class MetaLabel(type):
    """This metaclass is needed to enable Label[index] to return the Label with that index."""
    instances: dict  # Label.instances will map indices to Labels
    def __getitem__(cls, index):
        return cls.instances[index]

class Label(_SurrogateValue, metaclass = MetaLabel):
    """`label = world.Label(...)` creates a new Label object to control a text message projected on the Webots display.
       Label attributes are typically set by keyword in `Label()` creation, and/or in calling `label.update()`:
       `value`: the current value to be displayed on screen for this label (when visible). Whenever the label is
       displayed, `str(label)` is called to determine what string to display.  By default, this simply returns
       `str(label.value)` but Label instances/subclasses could overwrite __str__ to add formatting, units, etc...
       This allows that `label.value` could be used as a storage place for non-string info, like an integer counter.
       Each Label is a surrogate for its own .value, so, e.g. if you use label.value as an integer counter,
       then `label + 1` is equivalent to `label.value + 1`).
       `pos`: the vector-like (x, y) position of this label, in fractions across and down the display window
       `size`: indicates the vertical size of the label, as a fraction of the display window. (The label expands itself
       to the right to accommodate its value).
       `color`: the color this label will be printed in, either vector-like (1, 0, 0) or hexcolor integer 0xFF0000.
       `transparency`: would be inferred from 4th alpha component of vector-like `color` and is stored there
       `font`: the font to be used TODO see Webots docs for options
       `is_visible`: when false-like the label will not be displayed, regardless of its .value
       `index`: Each label is tracked in Webots by an integer index. A specific index may be given in label creation,
       but more commonly you can leave it unspecified and one will be auto-created for each new Label,
       starting at 0 and counting up. Label[index] retrieves the current label with that index, or raises IndexError.
       `label.update()` accepts the same arguments as Label(), updating the label's .attributes and its onscreen
       display accordingly, remembering and re-using the old settings for anything not explicitly changed.
       `label.hide()` and `label.show()` are convenience ways of updating `label.is_visible`
       `label.delete()` hides this label and frees its index to be re-used."""
    instances = {}        # Label.instances will map each index to the corresponding Label
    filled_to_index = -1  # Will mark the highest index for which we're sure no lower index is unused
    default_font = "Impact"  # May be overwritten in subclasses
    # TODO create an AcceptableColor type to use in circumstances like this
    def __init__(self, value: _Any = "", pos:Iterable2f=(0.0, 0.0), size=0.1,
                 color: _Union[Iterable3f, Iterable4f, int] = 0xFFBB00,
                 transparency=0.0, font: str = "default", is_visible=True, index: int = None,
                 shadow = None, shadow_index:int = None, shadow_offset=(0.002, 0.002), shadow_color=(0,0,0,0.8)):
        # We'll just do invariant initialization here, then pass vary-able stuff to update()
        self.shadow = None
        if shadow or shadow_index is not None:
            self.shadow = True
            self.shadow_index = self.claim_index(shadow_index)
        self.claim_index(index)
        if font == "default": font = self.default_font
        self.update(value=value, pos=pos, size=size, color=color, transparency=transparency,
                    font=font, is_visible=is_visible, shadow_offset=shadow_offset, shadow_color=shadow_color)

    def claim_index(self, index:int)->int:
        if index is None:
            index = next(i for i in itertools.count(Label.filled_to_index+1) if i not in Label.instances)
            Label.filled_to_index = index
        self.index = index
        Label.instances[index] = self
        return index

    def update(self, value: _Any = _omitted, pos: Iterable2f = _omitted, size:float = _omitted,
               color:_Union[Iterable3f, Iterable4f, int] = _omitted, transparency: float = _omitted,
               font: str = _omitted, is_visible: bool = _omitted,
               shadow_offset = _omitted, shadow_color = _omitted ) -> _Any:
        """Updates this label's .attributes as specified in given (keyword) arguments, and updates the onscreen
           display of this label. Returns this label itself (which is a surrogate for its value)."""
        if value is not _omitted: self.value = value
        if pos is not _omitted:  self.pos = Vector(pos)
        if size is not _omitted: self.size = size
        if color is not _omitted:
            self.color = Color(color)
            # TODO double-check whether alpha channel is generally construed as opacity or transparency?!
            if len(self.color) == 3: self.color.append(1)
        if transparency is not _omitted: self.color.alpha = 1 - transparency
        if font is not _omitted: self.font = font
        if is_visible is not _omitted: self.is_visible = is_visible
        if shadow_offset is not _omitted: self.shadow_offset = Vector(shadow_offset)
        if shadow_color is not _omitted: self.shadow_color = Color(shadow_color)

        self.text = b"" if not self.is_visible else str(self).encode()

        if self.shadow is not None:
            wb.wb_supervisor_set_label(self.shadow_index, self.text,
                                       c_double(self.pos.x + self.shadow_offset.x),
                                       c_double(self.pos.y + self.shadow_offset.y),
                                       c_double(self.size),
                                       self.shadow_color.hexcolor, c_double(1 - self.shadow_color.alpha),
                                       self.font.encode())

        wb.wb_supervisor_set_label(self.index, self.text,
                                   c_double(self.pos.x), c_double(self.pos.y), c_double(self.size),
                                   self.color.hexcolor, c_double(1 - self.color.alpha), self.font.encode())
        return self

    def __repr__(self):
        return f"Label[{self.index}]"

    def hide(self, *args, **kwargs):
        """Hides this label. Additional args and kwargs will be passed to label.update()."""
        kwargs['is_visible'] = False
        self.update(*args, **kwargs)

    def show(self, *args, **kwargs):
        """Makes this label visible. Additional args and kwargs will be passed to label.update()."""
        kwargs['is_visible'] = True
        self.update(*args, **kwargs)

    def delete(self):
        """Hides this label and frees its integer index for re-use."""
        self.update(is_visible=False)
        del Label.instances[self.index]
        Label.filled_to_index = min(Label.filled_to_index, self.index - 1)
        if self.shadow:
            del Label.instances[self.shadow_index]
            Label.filled_to_index = min(Label.filled_to_index, self.shadow_index - 1)


class TimerLabel(Label):
    """A subclass of Label to allow for onscreen timers that measure elapsed simulation time.
       TimerLabels have all the standard Label attributes and the following ones, also settable by keyword
       in TimerLabel() creation and/or in other methods like .update().
       `duration` is the number of seconds the timer counts down from; or if zero/None, it will count up from 0.
       `paused` determines whether the timer will actively run when .update() is called.
       `value` the current number of seconds the timer will display, auto-computed by .update() if not explicitly given.
       Like other Labels, a TimerLabel is a surrogate for its .value, so `if timer:` will be True while there is any
       time on the timer, and `timer < 3` will be True when there are less than 3 seconds on the timer.
       `mytimer.update()` will update the onscreen display to match the current simulation time.
       `mytimer.pause()` and `.unpause()` are convenient ways to update whether it is paused.
       TimerLabel.__str__ converts the .value to a string, formatted like '1:00', though this could be overwritten
       in an instance/subclass to produce alternate formatting, as in the FuseTimerLabel subclass."""
    default_font = "Impact"
    def __init__(self, duration: float = None,
                 pos: Iterable2f = (0.0, 0.0), size=0.1,
                 color: _Union[Iterable3f, Iterable4f, int] = 0xFF1111, transparency=0.0,
                 font: str = "default", is_visible=True, paused = False,
                 value: float = _omitted, index: int = None,
                 shadow = None, shadow_index:int = None, shadow_offset=(0.002, 0.002), shadow_color=(0,0,0,0.8)):

        # We'll just do timer-specific initialization, then pass the buck up to the superclass
        self.duration = duration
        self.paused = paused
        self.time_elapsed_before_pausing = 0
        self.start_time = core_api.time
        super().__init__(value=value, pos=pos, size=size, color=color,
                         transparency=transparency, font=font,
                         is_visible=is_visible, index=index,
                         shadow=shadow, shadow_index=shadow_index,
                         shadow_offset=shadow_offset, shadow_color=shadow_color)  # let the superclass do most of the work

    def update(self, duration: float = _omitted, pos: Iterable2f = _omitted, size: float = _omitted,
               color: _Union[Iterable3f, Iterable4f, int] = _omitted, transparency: float = _omitted,
               font: str = _omitted, is_visible: bool = _omitted,
               paused: bool = _omitted, value: float = _omitted,
               shadow_offset = _omitted, shadow_color = _omitted):
        """This accepts many standard label arguments in case you want to update any of them, but it also works
           fine with no parameters, in which case it will just update this timer to fit the current time.
           Returns this TimerLabel itself, which is a surrogate for its value."""
        if duration is not _omitted: self.duration = duration
        if paused is not _omitted:
            should_be_paused = paused  # to make the following easier for humans to read
            if should_be_paused and not self.paused:
                self.time_elapsed_before_pausing += core_api.time - self.start_time
            elif self.paused and not should_be_paused:
                self.start_time = core_api.time
            self.paused = should_be_paused
        # If no value was explicitly given, we automatically compute the number of seconds to display on the timer
        if value is _omitted:
            value = self.time_elapsed_before_pausing
            if not self.paused: value += core_api.time - self.start_time
            if self.duration: value = max(0.0, self.duration - value)
        # Any remaining arguments will be handled by superclass
        super().update(value=value, pos=pos, size=size, color=color, transparency=transparency,
                       font=font, is_visible=is_visible, shadow_offset=shadow_offset, shadow_color=shadow_color)
        return self

    def reset(self, **kwargs):
        """Resets this timer to its starting value (its .duration for countdown timers, 0 for countup timers).
           Accepts all the standard Label/TimerLabel keyword arguments, which will be passed on to update()."""
        self.start_time = core_api.time
        self.time_elapsed_before_pausing = 0
        self.update(**kwargs)

    def pause(self, **kwargs):
        """Causes this timer to stop running, while retaining its current time.
           Accepts all the standard Label/TimerLabel keyword arguments, which will be passed on to update()."""
        kwargs['paused'] = True
        self.update(**kwargs)

    def unpause(self, **kwargs):
        """Causes this timer to stop running, while retaining its current time.
           Accepts all the standard Label/TimerLabel keyword arguments, which will be passed on to update()."""
        kwargs['paused'] = False
        self.update(**kwargs)

    def __str__(self):
        """Creates a formatted string representation of the timer, like '1:00'."""
        n = int(self.value)
        return f"{n // 60}:{str(n % 60).zfill(2)}"


class FuseTimerLabel(TimerLabel):
    """A TimerLabel with custom __str__ to display a '-----' fuse that shortens as the timer approaches 0.
       Altering mytimer.hyphens_per_second alters the rate at which the fuse is consumed, and hence the visible
       length of a fuse that represents a particular number of seconds."""
    default_font = "Arial"
    hyphens_per_second = 1

    def __str__(self):
        """Fuse timers have a string representation like '---------'."""
        return '-' * int(self.value * self.hyphens_per_second)


# === WorldModule class (needed to enable world.properties to work) ===

class SimulationMode(str):
    """A SimulationMode is a string, either 'PAUSE', 'PLAY' or 'FAST' with a specially methods allowing comparisons
       to be case-insensitive and allowing SimulationModes to count as being equal to the corresponding Webots integer
       constants. A SimulationMode has boolean False if paused, and True otherwise."""
    constants_to_strings = {_wb_constants.WB_SUPERVISOR_SIMULATION_MODE_PAUSE: 'PAUSE',
                            _wb_constants.WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME: 'PLAY',
                            _wb_constants.WB_SUPERVISOR_SIMULATION_MODE_FAST: 'FAST'}
    strings_to_constants = {s:c for c,s in constants_to_strings.items()}

    def __new__(cls, string_or_constant:_Union[str,int]):
        if isinstance(string_or_constant, int):
            constant = string_or_constant
            string = cls.constants_to_strings[constant]
        else:
            string = string_or_constant.upper()
            constant = cls.strings_to_constants[string]
        self = super().__new__(cls, string)
        self.constant = constant
        return self

    def __eq__(self, other):
        """SimulationMode equality comparison is case-insensitive, and also counts them as equal to the corresponding
           Webots constants."""
        if isinstance(other,int): return self.constant == other
        if isinstance(other, str): return super().__eq__( other.upper() )
        return super().__eq__(other)

    def __lt__(self, other):
        """Comparisons are done using Webots constants, so PAUSE < PLAY < FAST"""
        if isinstance(other,str): return self.constant < self.strings_to_constants[ other.upper() ]
        return self.constant < other

    def __le__(self, other): return self==other or self<other

    def __bool__(self):
        """A SimulationMode counts as False when paused, and True otherwise."""
        return self != 'PAUSE'



#TODO perhaps needs some path initialization?
class WorldModule(Node, ModuleType):
    """Python supervisor controllers should use `import world` to import supervisor functionality in the world module.
       That module will be made to behave as though it is an instance of the WorldModule class defined here, mostly
       because we need a class to enable dynamic world.properties, like world.time and world.mode.
       The world module's __class__ is dynamically altered to be WorldModule."""

    def __init__(self, *args):
        # This won't be called to actually initialize world, since world won't morph to this class until later.
        # However, this would (annoyingly!) be automatically called whenever Node.__new__ returns world.
        # So we need it to do nothing in that case, rather than calling Python's ModuleType.__init__.
        pass

    def __repr__(self): return 'world' # otherwise ModuleType.__repr__ takes precedence, which is ugly

    # world-involving properties shared with robot, made available directly from world for convenience
    time = _robot.RobotModule.time
    project_path = _robot.RobotModule.project_path
    world_path = _robot.RobotModule.world_path
    timestep_ms = _robot.RobotModule.timestep_ms
    timestep_sec = _robot.RobotModule.timestep_sec

    wb.wb_supervisor_node_get_self.restype = wbNodeRef
    @property
    def self(self)->Node:
        """world.self returns the Robot Node in the scene tree that this supervisor is controlling."""
        return Node( wb.wb_supervisor_node_get_self() )

    wb.wb_supervisor_node_get_selected.restype = wbNodeRef
    @property
    def selected(self)->Node:
        """world.selected returns the currently selected Node in the scene tree, or None if none is selected."""
        return Node( wb.wb_supervisor_node_get_selected() )

    @property
    def mode(self)->SimulationMode:
        """world.mode returns the current SimulationMode string, either 'PAUSE', 'PLAY', or 'FAST'.
           This mode may also be changed, e.g. by setting world.mode = 'PAUSE'."""
        return SimulationMode( wb.wb_supervisor_simulation_get_mode() )
    @mode.setter
    def mode(self, new_mode:_Union[str, int]):
        if isinstance(new_mode,str): new_mode = SimulationMode.strings_to_constants[new_mode.upper()]
        wb.wb_supervisor_simulation_set_mode( int(new_mode) )

    #TODO this is just for console testing convenience, can remove
    @property
    def re(self):
        """For convenience from console, world.re will reload the world module"""
        from importlib import reload
        reload(root)
        return root

# === Re-declare WorldModule properties for linters ===

# Pycharm's linter doesn't automatically recognize module class properties, so these are declared again here on the
# module instance to make them and their documentation show up for the linter. We set this equal to the property
# to get the docstring, but declare the returned type from that property so linters will use it.
# At runtime, class-level property definitions always take priority over mere instance attributes like these.

time: float = WorldModule.time
project_path: str = WorldModule.project_path
world_path: str = WorldModule.world_path
timestep_ms: int = WorldModule.timestep_ms
timestep_sec: float = WorldModule.timestep_sec
self: Node = WorldModule.self
selected: Node = WorldModule.selected
mode: SimulationMode = WorldModule.mode

# === world.functions for controlling simulation ===

# Currently provided just in case some legacy code really needs to perfectly preserve old getFromDef calls
wb.wb_supervisor_node_get_from_def.restype = wbNodeRef
def getFromDef(name: str) -> 'Node':
    """DEPRECATED. Returns a node with this DEF-name, using the Webots API's `wb_supervisor_node_get_from_def`.
       A '.'-separated path of names can be given, e.g., "ROBOT1.LEFT_SHOULDER.LEFT_ARM.LEFT_HAND"; the first of which
       may be at any level, but then successive names cannot skip any further levels.
       For most purposes it is preferable to use `world.NAME` as this is similarly fast, but is briefer, allows
       level-skipping within '.'-paths, (so e.g. `world.ROBOT1.LEFT_HAND` is fine), can provide matches to name-fields
       or to NodeTypes as well as DEF-names, and can be initiated from any node e.g. as node.DESCENDANT_NAME."""
    WarnOnce("DEPRECATED. world.getFromDef('DEFNAME') is deprecated. Use world.DEFNAME or world.Node('DEFNAME').")
    return Node( wb.wb_supervisor_node_get_from_def(name.encode()) )

def quit(status:int):
    """Tells Webots to quit at its earliest convenience and to return the given status.
       After calling this, a controller should also call robot.clean_up() or world.clean_up()"""
    wb.wb_supervisor_simulation_quit(status)

def reset():
    wb.wb_supervisor_simulation_reset()

def reset_physics():
    wb.wb_supervisor_simulation_reset_physics()

def reload():
    """Tells Webots to stop the current simulation, reload the current world, and then restart controllers,
       so in particular this controller will stop running."""
    return wb.wb_supervisor_world_reload()

def load(filename:str):
    """Tells Webots to stop the current simulation, load the world with the given filename, probably ending '.wbt',
       and then restart controllers, so in particular this controller will stop running."""
    return wb.wb_supervisor_world_load( filename.encode() )

#TODO check if None passes through c-types correctly???
wb.wb_supervisor_world_save.restype = c_bool
def save(filename:str = None)->bool:
    """Saves the current world to the given filename, or silently overwrites the current file if no filename given.
       Returns True if the save operation was successful."""
    return wb.wb_supervisor_world_save( filename.encode() )

def save_image(filename:str, quality:int):
    """Saves a jpeg image of the current simulation window to the given filename (silently overwriting any existing
       file of that name!) with the designated jpeg quality, ranging 1-100."""
    return wb.wb_supervisor_export_image(filename.encode(), quality)


# === manually morph this module into a WorldModule (and hence Node) instance ===

root.__class__ = WorldModule # This module will now behave as though it is an instance, making world.properties work

#TODO unfortunately all the Node.methods that world inherits also need separate declaration to show up in linters!
# And even more unfortunately, those that are cached properties probably cannot be set to informative defaults
# A possible (clumsy ugly) solution is for the user to type declare that world is of type world.WorldModule

all_descendants: Iterable[Node] = WorldModule.all_descendants
descendants: Iterable[Node] = WorldModule.descendants
forget_descendants: Callable[[],None] = root.forget_descendants
refresh_descendants: Callable[[],None] = root.refresh_descendants


# === pre-parse the scene tree from a Webots export ===

#TODO make this conditional upon a settings.setting
root.catalog_descendants()


