## New Python API for Webots
Justin C. Fisher

### Apache License.

Copyright 2022. Justin Fisher.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

### Warning and thanks.

This new Python API is still partly under construction and likely 
still contains many bugs. Thanks in advance for any help you can 
give in finding and quashing them. And thanks also for any other 
helpful feedback or suggestions you might give. -Justin Fisher

### Contents

The enclosed Webots project includes an early version of the 
new python api (in the folder `new_python_api`) and a number 
of sample worlds and controllers that show the new api in action 
and could serve as a model for making your own.

This readme file contains an overview of the new API, instructions 
for using the early-access version of it, some tips for users of  
the old API regarding what has changed, and some known issues. 

### Overview.

Webots has historically provided a simple Python API, allowing Python 
programs to control individual robots or the simulated world. This 
Python API was a thin wrapper over a C++ API, which itself was 
a wrapper over Webots’ core C API.  These nested layers of 
API-wrapping were inefficient. Furthermore, this API was not 
very “pythonic” and did not provide many of the conveniences that 
help to make development in Python be fast, intuitive, and easy to 
learn.  This new Python API more efficiently interfaces directly 
with the Webots C API and provides a more intuitive, easily usable, 
and “pythonic” interface for controlling Webots robots and simulations.

In qualitative terms, the old API felt like you were awkwardly 
using Python to call C and C++ functions, whereas the new API 
feels much simpler, much easier, and like it was fully intended 
for Python.  Here is a representative (but far from comprehensive) 
list of examples:  

* Unlike the old API, the new API contains helpful Python type 
annotations and docstrings.
* Webots employs many vectors, e.g., for 3D positions, 4D rotations, 
and RGB colors.  The old API typically treated these as lists or 
integers (24-bit colors).  In the new API these are Vector objects, 
with conveniently addressable components (e.g. `vector.x` or 
`color.red`) and overloaded vector arithmetic operations, akin 
to (and interoperable with) Numpy arrays.  The new API also provides
easy interfacing between high-resolution Webots sensors 
(like cameras and Lidar) and Numpy arrays, to make it much more 
convenient to use Webots with popular python packages 
like Numpy, Scipy, or OpenCV.
* The old API often required that all function parameters be given 
explicitly in every call, whereas the new API gives many parameters 
commonly used default values, allowing them often to be omitted, 
and keyword arguments to be used where needed. 
* Most attributes are now accessible (and alterable, when applicable)
by pythonic properties like `motor.velocity`.  
* Many devices now have python methods like `__bool__` 
overloaded in intuitive ways.  E.g., you can now use `if bumper` to 
detect if a bumper has been pressed, rather than the 
old `if bumper.getValue()`.
* Pythonic container-like interfaces are now provided.  You may now 
use `for target in radar` to iterate through the various targets a 
radar device has detected or `for packet in receiver` to iterate 
through communication packets that a receiver device has received 
(and it now automatically handles a wide variety of python objects, 
not just strings).
* The old API required supervisor controllers to use a wide variety 
of separate functions to traverse and interact with the simulation’s
scene tree, including different functions for different VRML 
datatypes (like `SFVec3f` or `MFInt32`). The new API automatically 
invisibly handles these datatypes and translates intuitive python 
syntax (like dot-notation and square-bracket indexing) to the Webots 
equivalents.  E.g., you can now move a particular crate 1 meter in 
the x direction using a command like 
`world.CRATES[3].translation += [1,0,0]`. Under the old API, this 
would have required numerous function calls 
(calling `getNodeFromDef` to find the CRATES node, `getMFNode` to 
find the child with index 3, `getSFField` to find its translation 
field, and `getSFVec3f` to retrieve that field’s value, then some 
list manipulation to alter the x-component of that value, and 
finally a call to `setSFVec3f` to set the new value).  

The new API is mostly backwards-compatible with the old Python 
Webots API, and provides an option to display deprecation warnings 
with helpful advice for changing to the new API.

### Early Access Installation 
This of course requires Python.  I've tried not to use any features from 
later than Python 3.6, but have done most of the testing in Python 3.9,
so would generally recommend Python 3.9 or higher. 

This of course also requires Webots.  A few recent bugfixes are relevant, 
especially if you want to make heavy use of the new supervisor/`world`, 
so it may be best to use a recent nightly build of Webots.  But for most 
purposes, the most recent stable build should probably be fine.

Eventually the relevant files for the new python API will be situated 
within the Webots installation and will automatically be available for 
import, much as the old python `controller` was.  To test an 
early-access version of this new controller, you needn't do a 
special Webots installation.  Instead you can use an ordinary Webots 
installation and just situate the relevant files for the new Python API
somewhere where your Python controller can import them.

The enclosed Webots project situates the relevant files within 
a `new_python_api` folder at the top level alongside the `worlds` 
folder, and then each particular python controller is accompanied 
by a `runtime.ini` indicating that it may import
from `../../new_python_api`, `:`-separated from the paths for any 
other folders you'd like the controller to also be able to import 
from.  E.g., the following `runtime.ini` also allows controllers 
to import from the project's `libraries` folder.

Here is the relevant `runtime.ini` line:
```
PYTHONPATH = $(PYTHONPATH):../../libraries:../../new_python_api
```

To test python controllers within a copy of this project, all you'll 
need is a copy of that `runtime.ini` alongside your controller.  

To test the new API within another project, you'll need to put
a copy of the contents of `new_python_api` somewhere your 
controller can find it, e.g. by copying this folder to the top 
level of your project alongside your `worlds` folder, and ensuring 
that your own `runtime.ini` includes that folder, or simply pasting 
those contents directly alongside your controller in its own folder 
(fine for simple trials, but won't scale well to multiple controllers.)

### Recommended Interactive Development Environment (IDE).

It is strongly encouraged to use a sophisticated IDE like PyCharm, 
as it will be able to give you type-hinting, auto-completion, and 
and mouse-over doc-strings.  You will probably need to go to project 
settings and ensure that PyCharm knows to look for the new api 
files wherever you opted to situate them.

The new python API includes thorough inline documentation
in its source code.  Eventually the Webots online docs' Python tabs 
will be updated to include this information too. But for early access, 
you'll likely want to rely a lot upon mouseover 
doc-strings, auto-completion hints, perusing the heavily-commented 
source files, looking at included sample controllers, and/or asking me.  

### Importing the new controller.

For ordinary non-supervisor robot controllers, the import is simply:

```python
import robot
```

After this the `robot` object will work quite similarly to the `robot`  
that users of the old API often made by first importing `Robot` from 
`controller` and then setting `robot = Robot()`.  (Eventually, the 
new API will include versions of `controller` and `Robot` that work 
like this for backwards-compatibility, but early-access version does 
not to avoid potential conflicts with the `controller` that will 
be present in current Webots installations.)

The `robot` module gives you access to robot devices, and other 
sorts of robot functionality.  E.g., the following simple controller
would make a motor attempt to oscillate its position.

```python
import math, robot
motor1 = robot.Motor("motor1")
while robot.step():
    motor1.target_position = math.sin(robot.time)
```

If you want to use supervisor functionality, that is now located 
entirely within the `world` module.

```python
import world
world: world.WorldModule  # this explicit type declaration helps PyCharm give better hints
```

(The second line is optional, but will help PyCharm's linter recognize 
that `world` inherits many `Node` methods and properties via `WorldModule`. 
I will eventually take manual steps to make more of these appear 
automatically which will make this explicit declaration less useful.)

Note that `world` provides only supervisor functionality, so 
if you want a controller to both exercise god-like supervisor 
powers over the world, and mere robot-like control over its 
own devices, you will need to import both `world` and `robot`.

The `world` module allows you to interact with the Webots scene 
tree as though it were composed of ordinary Python objects, 
without needing to bother with calling any functions to find nodes 
or fields, nor to hassle with VRML types like `SFVec3f` or `MFInt32` 
when interacting with the fields.

For comparison, here are two lines from Webots' sample 
`supervisor_draw_trail`, as it would appear in the old
Python controller.

```python
root_children_field = supervisor.getField(supervisor.getRoot(), "children")
root_children_field.importMFNodeFromString(-1, trail_string)
```
And here is how that looks written in the new controller:
```python
world.children.append(trail_string)
```
The `world` module also provides access to other supervisor 
functionality, and to some functionality shared with `robot` 
that is also likely to be used by supervisors, 
like `.step()` and `.time`.

```python
world.mode = "PLAY"
while world.step():
    if world.time > 10: break
world.save("myfile")
```

### Cheat-sheet for differences from the old API

#### General `robot` / `world`.

`object.getFoo()` has typically been changed to `object.foo` 

`object.setFoo(new_value)` has typically been changed to 
`object.foo = new_value`.  Take care with your spelling, as it is 
dangerously easy to silently create some new attribute rather than 
the one you intended to set. 

Any method name written in `camelCase` is now probably rewritten 
in Pythonic `snake_case` and/or given a more convenient name.

The old versions (like `.getFoo`, `.setFoo` and `.camelCase`) 
will generally still work, but will print a one-time (per run) 
warning about being deprecated, together with a suggested translation. 
(There'll also soon be an option in `settings` to hide such warning spam.)

So, in general, you could just follow the docs for the old API 
and get something that will probably work, with deprecation 
warnings pointing you towards newer, and typically better, 
ways of doing things.

The `settings` module contains some (and eventually will contain more) 
settings, many of which let you turn off a new feature for backwards 
compatibility. To alter settings, you would typically 
`import settings` and then assign `settings.setting_name = new_value`.  
Depending on the setting, you may need to do this before importing 
`robot` or `world`.

Anything that returned a vector-like list of floats now returns a 
some form of `Vector` object, whose components may be accessed via
`.x`, `.y` and `.z` (or for Colors, things like `.red`).  Vector 
objects support vector-arithmetic, like `2*v1 + v2` and provide 
convenience functions like `v1.magnitude` or `v2.unit_vector`.
If you ever want to create a vector, you can import from `vectors`
or use `robot.Vector` or `robot.Color`.

Most old functions that had many arguments that you had to list in 
the right order now have sensible default values, and let 
you use keywords to specify just what arguments you need to have 
differ from defaults. A huge offender was the old 
supervisor `setLabel` which you'll find to have been tremendously 
improved in `world.Label`.

In the spirit of Python duck-typing, most of the new API's functions 
aren't very picky regarding what sort of arguments you pass them.  
E.g. functions that expect a color will accept a `Color` vector, 
or a tuple/list of RGB(A) values ranging 0-1, or a hex integer 
like 0xFF00FF.  In general, if you think it would make good sense 
for a function to accept an argument, there's a good chance it can, 
and you can check the type-hinting for confirmation.

#### Robot Sensors.

All sensors (and anything else with a sensor-like sampling period) 
are now automatically enabled upon first access with the simulation's
basic timestep as their sampling period.  You may alter this with 
`sensor.sampling = new_period` or disable a sensor with 
`sensor.sampling = None`.

Most sensors' values can be accessed via `sensor.value` (with the
main exceptions being sensors that can provide a variety of 
different values).  However, most sensors are now "surrogates" 
for their own `.value`, meaning that you can typically use the sensor 
itself in commands where you would have used its `.value`.  
E.g. you can use `if light_sensor > 50` or `if bumper` or 
`estimated_velocity += accelerometer * robot.timestep_sec` 
or `gps.x` or `for target in radar`.  The main reason to use 
`.value` is if you want to store a snapshot of the sensor's
current value for later comparison, because at later timesteps, 
the sensor itself will have forgotten its old value and become 
a surrogate for its new value.  (For high bandwidth sensors like 
Cameras, RangeFinders and Lidar, .value shares memory with the 
simulation to allow for faster reading, but this means the 
.value will be valid only for this timestep. For these, use 
the device's .copy() method to store a lasting value, if you need it.)

#### Supervisors / `world`

For supervisors, `world.DEFNAME` will find the node with that 
DEF-name descended from the root `world`.  You can similarly 
initiate such a search from any other node, and it will recursively 
seek downwards, skipping levels when appropriate (unlike Webots' 
built-in dot-pathing which requires that you explicitly mention every 
successive level after jumping to the first). You may also search
for NodeTypes, e.g. `world.ROBOT1.Camera` or device names, e.g. 
`world.ROBOT1.camera`.  ALL-CAPS DEF-names should not collide 
with other names, but other names could.  You can also use 
`world.Node(identifier)` to find nodes, where the identifier 
could be a DEF-name, NodeType, device name, Device object, or 
unique ID integer.

Nodes' fields are accessible as `node.fieldname`. 
Referring to an SF (single) field will automatically return its
value. E.g. `world.ROBOT1.translation` returns a 3D Vector.  
SF fields are also settable, e.g. `world.ROBOT1.translation = (0,0,0)`.
Referring to an MF (multi) field will return a container object 
that lets you interact with that field as though it were 
a Python list.  E.g., `world.children[0]` returns the first child 
of the root `world`, `for node in world.children` iterates over 
all these children, and `world.children.append(new_node)` 
would import a new node at the end of this list.  If you treat 
MF Fields like python lists, they'll probably do exactly what you expect.
(If accessing fields via `node.fieldname` ever doesn't work, e.g. 
due to some name collision, you may also access, and iterate 
over fields, using `node.fields` or `node.proto_fields`)

The new API makes it quite easy to traverse fields or the tree, e.g. 
with `for f in node.fields` or `for node in world.descendants`.

The `world` module generally does a good job at caching a 
"proxy scene tree" to avoid slow repeat-lookups through the C-API. 
It's often advisable to just access things through this proxy tree, 
rather than storing your own local references to node or especially
fields.  The `world` module's caching is good enough that there's 
generally not much speed to be gained by storing your 
own local references. Personal convenience may still justify 
creating a local reference to a stable node that you'll often use, e.g. 
by setting `bot2 = world.ROBOT2`. However, (unlike the old API) 
there usually is no point in storing local references to most fields 
(i.e. usually better to use `bot2.translation` than to store 
`trans_field = bot2.fields.translation` and then refer to 
`trans_field.value`). If you'll do much deleting of nodes from the 
scene tree, then storing your own local references to nodes/fields 
will risk accidentally using one that has become outdated, which may 
crash Webots, whereas if you refer to things through the proxy scene 
tree, it automatically updates its references
(in response to changes that this supervisor itself has made -- 
if other supervisors make changes, things get a lot
messier, and you may run into some bugs deep in Webots that make 
keeping track of such changes practically impossible).

You can import a node to an SF field simply by assigning it a 
value.  E.g., `joint.endPoint = "wheel.wbo"` or 
`joint.endPoint = my_VRML_string`.  For an MF field, you would 
add a node much as you would for a python list, with `.append`, 
`.insert` or slicing, e.g. `n.children[0:2]=[string1,string2,string3]`. 
In addition to accepting .wbo filenames and VRML strings, these 
import commands will also accept existing nodes to be copied, 
or you can use `world.plan` to create planned versions of nodes
using python syntax rather than VRML, which makes it easy to 
incorporate dynamic variables into plans, or to dynamically alter 
plans, e.g., between successive imports.  
Here's a simple example from the sample `supervisor_draw_trail.py`.

```python
plan = world.plan  # for easier repeated reference
trail_plan = plan.IndexedLineSet(DEF = "TRAIL_LINE_SET",
                                 material = plan.Material(diffuseColor=TRAIL_COLOR, emissiveColor=TRAIL_COLOR),
                                 coord = plan.Coordinate(point=[initial_pos] * (TRAIL_LENGTH+1)),
                                 coordIndex = [0] * (TRAIL_LENGTH+2)
                                )
world.children.append(trail_plan)
```
### Known Issues.

#### World / Supervisor.

The `world` module is quite complete and well tested.  

Pycharm's linter isn't good at recognizing that `world` will 
inherit methods from `WorldModule` and hence from `Node`. 
I have jury-rigged a way to make some commonly used 
ones automatically appear in the linter, but have not yet done this for 
all of them, and this work-around won't show docstrings for all 
properties.  I encourage early-access users to explicitly declare 
`world: 'world.WorldModule'` after you import it to get better linting.  

Pycharm's linter will complain about many uses of the proxy scene 
tree.  E.g. it won't know that `world.ROBOT1` will dynamically 
create a Node representing the scene tree node with DEF-name `ROBOT1`, 
so will instead red-underline this and fail to give helpful 
linting advice.  A partial work-around is to explicitly type-declare 
what type of object you expect such references to produce.
(Commonly used fields, like `children` and `translation` have already 
been type-declared for you.)

#### Robot module.

The `robot` module is near-complete and largely works, but is
not yet thoroughly tested.

Basic functionality like motors and simple sensors generally 
work (though likely with a few bugs).

High bandwidth devices are not entirely implemented. Ordinary 
camera and rangefinder and lidar images and pointclouds should 
work.  I expect that camera recognition objects won't work 
entirely. Coming soon!

`robot.keyboard` is fairly well tested. I haven't had a joystick 
to test but it shares core functionality with keyboard, so hopefully will 
work.  `robot.mouse` is untested and likely doesn't work entirely.

If you use `robot.Device()` (or the deprecated `robot.getDevice()`)
to create devices, Pycharm's linter won't help you much for them, 
since it won't know which type of Device will be returned.
It's generally better to use more specific constructors like 
`robot.Motor()`.  If you detect devices dynamically, e.g. with 
`robot.devices[0]` you will also need an explicit type declaration 
to get full linting help.

