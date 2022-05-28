"""This file contains various settings that affect how the "new Python API" for Webots will run; many of which
   involve an option to turn off new functionality for backwards compatibility.
   Typical usage would be to have your controller `import settings` then change `settings.foo = new_value`"""

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

# === core settings ===

initialize_robot_on_import = True  # Must be set before importing robot/world, or it'll be too late

false_step_return_value = -1  # Specifies what return value from robot.step and world.step would have a "False" boolean
                              # value. By default this is -1, making this return value count as "True" whenever it is
                              # fine to continue, and "False" when the controller should stop. In earlier versions of
                              # Webots, the return value would be "True" if non-zero which may not be so useful.

# === world module settings ===

use_nodes_are_hidden = 0.5  # 0 = always visible, 1 = always hidden, 0.5 = effectively hidden til accessed, then visible

use_node_children_are_hidden = True  # falsish = visible to things that see parent, trueish = always hidden

negative_insertion_indices_are_pythonic = True  # When True/1, mf_field.insert(-1, v) puts v before the old last item
                                                # and you would use mf_field.append(v) to put v last, as in Python.
                                                # When False/0, this instead puts v last (as in other Webots languages)
