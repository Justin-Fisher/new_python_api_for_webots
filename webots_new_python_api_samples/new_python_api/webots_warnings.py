"""A fairly crude system for printing warning messages without spamming them.  Perhaps should be replaced by
   use of the Python warnings module."""

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

import sys
from functools import wraps

# ---- functions to allow generating warnings without spamming them

def Warn( new_warning:str, category:str = None ):
    sys.stderr.write(f"WARNING: {new_warning}\n")

class WarnOnce:
    past_warnings = set()

    def __init__(self, new_warning:str, category:str = None ):
        """The first time a warning in this category (or with this exact string, if category is not given) occurs,
           this writes new_warning to the standard error channel, sys.stderr ."""
        if category == None: category = new_warning
        if category not in WarnOnce.past_warnings:
            sys.stderr.write(f"WARNING: {new_warning}\n")
            WarnOnce.past_warnings.add(category)

# Decorator to print a function's docstring as a deprecation warning
def use_docstring_as_deprecation_warning(fn):
    @wraps(fn) # copy the name and docstring of the decorated fn
    def new_fn(*args, **kwargs):
        WarnOnce(fn.__doc__)
        return fn(*args, **kwargs)
    return new_fn

