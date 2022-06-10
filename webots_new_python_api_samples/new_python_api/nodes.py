"""nodes.py contains data structures and methods for supervisors to deal with nodes and fields in the scene tree via the
world module, and for the construction of planned nodes for potential importation into the scene tree.
The "proxy scene tree" is a treelike structure of Python Nodes and Fields that is isomorphic to the corresponding
structures within the Webots simulation's own scene tree. For most purposes this proxy tree behaves as though it
were the actual scene tree in the simulation, so e.g. reading world.ROBOT1.translation reads the translation field
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
import textwrap
from typing import Union, Callable, Iterable, Dict, List, Tuple, Any, Sequence, Iterator, Container, Mapping, \
    Collection, overload
from pathlib import Path as _pathlib_Path  # an acceptable way of designating a .wbo file to import as a Node
import ctypes
from ctypes import c_double, c_bool, c_char_p, c_void_p, c_int, POINTER
import re

import settings
from core_api import wb, Console, timed_cached_property
import WB_CONSTANTS as _WB
from dictset import DictSet
from vectors import GenericVector, Vector, Color, Rotation, Vec2f, Vec3f, Vec4f, Vec2f_p, Vec3f_p, Vec4f_p, Iterable2f, \
    Iterable3f, Iterable4f, VectorValue
from webots_warnings import Warn, WarnOnce, use_docstring_as_deprecation_warning
from descriptors import descriptor, cached_property, documented_attribute
from devices import Device
import nodetypes


# --------LOGGING----------------------------------------------------

VERBOSE = False    # print a lot of debug log items to console?
HIGHLIGHT = False  # while parsing VRML, print highlighted versions of the text as you go?

def logprint(*args, **kwargs):
    """print a green message to console log.  Can be toggled by changing VERBOSE"""
    if VERBOSE: print(Console.GREEN, *args, Console.RESET, **kwargs)

# ------------ wbNodeRef/wbFieldRef pointers ---------------------------------------------

class wbNodeRef(c_void_p):
    """A simple subclass of ctypes c_void_p pointers used to store wbNodeRef pointers,
       providing methods to enable these to work interchangeably with full-fledged Nodes in some circumstances.
       wbNodeRefs are used under the hood by the Webots C API, but not directly by ordinary Python users.
       A few time-sensitive loops, like Node.__new__'s traversing the scene tree seeking particular nodes, employ
       faster wbNodeRefs rather than creating full-fledged Nodes."""
    def __repr__(self):
        if self.id != -1: return f"wbNodeRef(#{self.id})"
        return f"wbNodeRef({self.value})"

    @cached_property
    def id(self):
        return wb.wb_supervisor_node_get_id(self)

    def __hash__(self):
        return hash(self.value)

    def __eq__(self, other):
        """A wbNodeRef is equal to another wbNodeRef or Node if they point to same memory address."""
        if isinstance(other, wbNodeRef): return self.value == other.value
        if isinstance(other, Node): return self.value == Node._as_paramater_.value
        return super().__eq__(other)

    # TODO not sure we really want/need this? Just allows appending ._as_parameter_ to wbNodeRefs to better quack like Nodes
    #  For now I'll comment it out and see if anything complains!
    # @cached_property
    # def _as_parameter_(self): return self

class wbFieldRef(c_void_p):
    """A simple subclass of ctypes c_void_p pointers used to store wbFieldRef pointers. When setting a restype
       for wbFieldRef-returning wb_functions, we need a subclass of c_void_p or ctypes won't actually convert
       the value to a pointer.  In addition we define some conveniences for debugging."""
    def __repr__(self): return f"wbFieldRef('{self.name}':{self.type})"

    def __hash__(self): return hash(self.value)

    @cached_property
    def name(self): return wb.wb_supervisor_field_get_name(self).decode()

    @cached_property
    def type(self): return wb.wb_supervisor_field_get_type_name(self).decode()

# ===== Bind =====

# TODO may not be needed any more, so may be removable
def _bind(func, object, name=None):
    """Bind an instance of function func to object, using the given name or func's
    own __name__ if name is None or not given.  After this, object.name(...) will be
    equivalent to func(object, ...).  If name is given as "" no bound function will
    be stored on the object.  A bound function instance will be returned regardless,
    and calling it(...) will be equivalent to calling func(object, ...)."""
    if name is None: name = func.__name__  # if no name given, use func's own __name__
    bound_func = func.__get__(object)  # bound_func(...) is equiv to func(object,...)
    if name: setattr(object, name, bound_func)  # store this as object.name
    return bound_func

# === Catalog ===

class NodesCatalog(dict):
    """A NodesCatalog is a dict subclass that maps known integer node ID's to the corresponding Node (or returns None
       for unknown integers) and that maps each string nomiker (DEF-name, name, or nodetype) to a list of Node objects
       with that nomiker.  In the case where an unknown nomiker is accessed, this automatically creates and returns
       a [] entry for that nomiker."""
    def __missing__(self, key):
        if isinstance(key, int): return None
        empty = self[key] = []
        return empty

catalog = NodesCatalog()  #type: NodesCatalog[Union[int, str], Union['Node', List['Node']]]

# === Class Node ===

# Declare restypes for various node-getting wb_functions employed below
wb.wb_supervisor_node_get_root.restype = wbNodeRef
wb.wb_supervisor_node_get_self.restype = wbNodeRef
wb.wb_supervisor_node_get_from_id.restype = wbNodeRef
wb.wb_supervisor_node_get_from_device.restype = wbNodeRef

world: 'Node'  # will be set by world module if that is imported, as it must be to use Nodes to impact scene tree

class Node:
    # Declare common fields for linters # TODO round this list out!
    # Group field:
    children: 'MField[Node]'    # 'MFNode'
    # Transform fields:
    translation: Vec3f
    rotation: Vec4f
    scale: Vec3f
    translationStep: float
    rotationStep: float
    # Solid fields:
    name: str
    model: str
    description: str
    contactMaterial: str
    immersionProperties: 'MFNode'
    boundingObject: 'Node'
    physics: 'Node'
    locked: bool
    radarCrossSection: float
    recognitionColors: 'MFColor'
    linearVelocity: Vec3f
    angularVelocity: Vec3f

    endPoint: 'Node'

    # --- Node finder/constructor ---

    def __new__(cls, identifier: Union[str, int, wbNodeRef, Device],
                     search_area: Union['Node', Iterable['Node']] = None,
                     field: 'Field' = None,
                     visibility: Union[int, Callable] = 1) -> Union['Node', None]:
        """A Node object is a Python representation of a node in the Webots scene tree (or of a planned node that could
           be imported into the scene tree -- TODO see the plan module). A Node allows convenient access to fields, e.g., as
           node.rotation or node.children, and to descendant nodes, e.g. as node.endPoint or node.DESCENDANT_DEF_NAME.
           New scene tree nodes should typically be acquired with `world.name` or `world.waypoint.name`. These will
           search the scene tree for a Node whose DEF-name, name field, or NodeType matches name, as described below.
           Nodes are cached for fast re-use. (In fact, under default settings, the world module automatically pre-caches
           the full structure of the scene tree from a single export, faster than fetching many parts separately.)
           The main reasons to use the explicit Node() constructor would be to find a node using an identifier that is
           an integer id, a wbNodeRef pointer, a Device, a `name` field with illegal characters for Python names, or
           a name that collides with the name of some other field or method. E.g. `world.Node` already means this class,
           so if you name a node 'Node' you couldn't access it as `world.Node` but could as `world.Node('Node')`.
           Such collisions will be rare, especially for ALL-CAPS DEF-names and for standard field names.
           TODO which will currently collide only for 'DEF', 'USE' and '_WB'. (Is that all???)
           This constructor attempts to retrieve/construct a Node matching identifier, which may be an integer node ID,
           a Device, or a string matching the DEF-name, name field, or NodeType of a node in search_area.
           E.g., a Camera with DEF-name 'CAMERA' and name 'camera' may be sought as 'CAMERA', 'Camera', or 'camera'.
           TODO Currently: In the default case where we parse an exported version of the scene tree to speed node access,
            getting a node by `name`-field works only for nodes with non-default names, since the export omits all
            fields with default values. Such nodes are still gettable by DEF, NodeType and/or Device object.
           The identifier may be a descending '.'-separated path of nodes to seek in succession, e.g. 'ROBOT2.CAMERA'.
           The search_area defaults to the whole world, or may be given. The search_area can be given as an iterable of
           Nodes to be checked in that order for matches. Otherwise search_area should be a single ancestor
           Node, and TODO search_area.all_descendants will be searched. That search is mostly depth-first, except immediate
           matches are sought at each level before delving deeper (prioritizing SFNode fields over MFNode).
           TODO At each level, nodes within MFNode fields like 'children' are searched in order, but the order in
            which the fields themselves are searched may be difficult to predict (due to limitations in what information
            about fields can be quickly fetched from the simulation).
           The optional visibility parameter determines whether nodes may be returned that are "hidden" within a
           non-exposed field of a PROTO and/or by being or being within a USE node.
           If visibility is the default 1, this will search all immediate children (of the search area, and of each
           waypoint in a '.'-path) including ones "hidden" by the starting node(s), but it will not search more distant
           descendants hidden by an intermediary. With this default setting, to access a hidden descendant, you must
           explicitly path to its parent and search within it, since hidden nodes will not be visible from further away.
           If visibility is 0, then this won't return any hidden nodes, not even immediate children.
           If visibility is all, then the search would include all hidden nodes, regardless of distance.
           If no matching scene tree node is found, this returns None.
           When a match is found, this returns the existing Node representing it, or constructs a new one.
           The field parameter, if given, should be the field of the parent Node that contains this node, and in this
           case, search_area should be that parent Node (or None)."""
        # print(f"{Console.BLUE}Now seeking scene tree node within {search_area} matching {identifier=}")
        # if isinstance(search_area, Node):
        #     print(f"{Console.BLUE}  This search area has wbNodeRef.value {search_area._as_parameter_.value}{Console.RESET}")
        if isinstance(identifier, str):  # Node('name') looks in search_area for DEF/name/NodeType that matches name
            if '.' in identifier:  # If identifier is a path, like 'ROBOT.HEAD', we recursively visit waypoints
                waypoint, remaining_path = identifier.split('.', 1)
                waypoint_node = Node(waypoint, search_area, field=field, visibility=visibility)
                if waypoint_node is None: return None
                return Node(remaining_path, search_area=waypoint_node, visibility=visibility)
            if search_area is None: search_area = world
            if isinstance(search_area, Node):
                if search_area.is_fully_cataloged:
                    # When in a fully cataloged search area, we can look up the list of known nodes with right name,
                    # restrict attention to those visible in the search area, and return the minimal one, or None
                    match = min((n for n in catalog[identifier]
                                   if search_area.__contains__(n, visibility)), default=None)
                    if match is not None: return match
                    # If we needn't bother looking for hidden children, then a non-match means no match
                    # TODO need to think carefully about USE-node handling
                    #  When parsed, USE-nodes have no known type/name/DEF, and we don't really want to fetch these,
                    #  So they'll effectively be hidden from catalog-based searches.
                    #  To retain consistency, that suggests they should still be unavailable after a type/name is fetched.
                    #  That opens the question of whether they should be "hidden" or completely unfindable
                    #  If merely hidden, then in high visibility circumstances, we'd need to check them again.
                    #  And doing that requires gathering a few hidden rejects from the list of ordinary visible children
                    #  But this is an unfortunate layer of complexity to add just for a few USE nodes
                    #  Could consider just making USE-nodes unfindable, period, to simplify these searches ***
                    #   Instead the only way to fetch them would be through their parent_field, as it historically was
                    #   This would require checking USE status prior to cataloging (or perhaps upon being returned)
                    #  Or could live with the slight inconsistency of making USE nodes be "hiding in plain sight"
                    #   without any distinguishable features until their name/type is fetched
                    if not visibility: return None
                    # Otherwise, still need to search any uncataloged hidden children and their descendants
                    nodes_to_search = [child for child in search_area.hidden_children if not child.is_fully_cataloged]
                else:                            
                    # Otherwise, we construe this Node search_area as meaning to search its (visible-enough) children
                    nodes_to_search = search_area.all_children if visibility else search_area.visible_children
                # We've just used up one layer of visibility, so unless it was all/unlimited, can decrement
                if visibility is not all: visibility = max(0, visibility - 1)
            else:  # otherwise, search_area should be an iterable
                nodes_to_search = tuple(search_area)  # spiel it into a tuple to allow re-use
                search_area = None                    # but stop calling it search_area, since can't mark it as cataloged
            # First check if there is any of the nodes at this level is an immediate match:
            for n in nodes_to_search:
                if not n.USE and (n.DEF == identifier or n.type == identifier or n.name == identifier): return n
            # If none matched, then recursively delve deeper into each node seeking a match
            for n in nodes_to_search:
                matching_descendant = Node(identifier, n, visibility=visibility)
                if matching_descendant: return matching_descendant
            # If we still have no match, we've now fully charted this area (yay!), but must report the search failed.
            if search_area is not None: search_area.__dict__['is_fully_cataloged'] = search_area.is_real
            return None

        # Otherwise, we were given a non-string identifier that we'll need to convert to a ref and then to a Node
        id = parent = node_type = None  # will overwrite if we happen across a value, to save unnecessary lookup
        if isinstance(identifier, wbNodeRef):  # Node(wbNodeRef) returns a Node for that pointer
            if not identifier.value: return None  # Webots sometimes returns Null wbNodeRefs for failed searches
            ref = identifier
        elif isinstance(identifier, Device):  # Node(device_object) returns a node for that device
            ref = wb.wb_supervisor_node_get_from_device(identifier.tag)
        elif isinstance(identifier, int):   # Node(127) returns the node with id 127
            id = identifier
            ref = wb.wb_supervisor_node_get_from_id(id)
        else:
            raise TypeError(f"Node(identifier) requires an identifier that is either a string name, an integer ID, "
                            f"a Device, or a wbNodeRef pointer, not {identifier}.")

        # First check catalog to see if we already have constructed a Node with the right unique id
        is_inside_proto = field and (field.hides_children or field.node.is_inside_proto)
        if id is None and not is_inside_proto: id = wb.wb_supervisor_node_get_id(ref)
        if id is not None:
            node = catalog[id]
            if node is not None: return node

        # TODO once we have an id-including version of export, the following would be redundant with the above
        # Next, check if this is a known but unexplored Node in the proxy tree (usu. created by parsing an export)
        if id is not None:  # We can't do this check for id-less PROTO subnodes
            # Since type is always known (for non-USE), and id is expensive to acquire, type is a worthwhile pre-screen
            # TODO think through how this will interact with USE nodes. Would this unhide them?
            node_type = wb.wb_supervisor_node_get_type_name(ref).decode()
            # If we don't already know what field to search in, then we must ask Webots who the parent is (often recursing
            # upwards!) and then search through all of its fields for a match.
            if field is None: parent = Node(wb.wb_supervisor_node_get_parent_node(ref))
            # We take advantage of the fact that field (if given) and parent would each have a .known_children property
            node = next((n for n in (field or parent).known_children
                           if (n.USE or n.type == node_type) and n.id == id), None)
            if node is not None: return node

        # Finally, if we still haven't found a node with the right id, need to create one
        node = super().__new__(cls) # create a new Node without any attributes yet
        # For each node.attribute, we alter its __dict__ directly, to bypass the slow custom __setattr__
        node.__dict__['is_real'] = True        # all nodes constructed this way should be part of scene tree
        node.__dict__['_as_parameter_'] = ref  # ctypes will now construe node as its wbNodeRef
        node.__dict__['fields'] = FieldsBearer(node)
        node.__dict__['is_inside_proto'] = is_inside_proto
        node.__dict__['id'] = id
        if field is not None: node.__dict__['parent_field'] = field
        if parent is not None: node.__dict__['parent'] = parent
        node.__dict__['_incoming_references'] = set()  # set of (a,n) pairs of ancestors with names for this node
        node.__dict__['is_fully_cataloged'] = False    # will become true once all non-hidden descendants are cataloged
        if node_type is None: node_type = wb.wb_supervisor_node_get_type_name(node._as_parameter_).decode()
        node.__dict__['type'] = node_type
        node.__dict__['DEF'] = wb.wb_supervisor_node_get_def(node._as_parameter_).decode()

        # Store entries in catalog for this node to enable fast lookup
        if id is not None: catalog[id] = node
        for nomiker in node.nomikers:  # nomikers include DEF/name/type (if non-empty)
            catalog[nomiker].append(node)

        return node

    # --- Node basic attributes ---

    @cached_property  # Node.__new__ will cache this directly for known nodes; this will locate parsed nodes on demand
    def _as_parameter_(self) -> wbNodeRef:
        """Returns the wbNodeRef for this node (and caches this for faster re-use).
           For nodes initially found by their wbNodeRef, that wbNodeRef will be cached and this wouldn't be called.
           So this will be called primarily for nodes that were initially created by parsing an exported version
           of the scene tree.  #TODO this will be simplified if we get an id-including version of export.
           Currently, exports tell us only the node's parent, parent_field name and (for MF fields) index in that field,
           so this first needs to find a way to refer to that parent (stepping recursively up the family tree until
           reaching an already-explored Node or the root world), finds the relevant field, and then looks within that
           field (at the appropriate index) to fetch what is presumed to be this Node's wbNodeRef."""
        # print(f"Attempting to look up wbNodeRef for {self}")
        if self.parent_field.is_SF:  # accessing this may trigger a recursive traversal up toward the root
            ref = wb.wb_supervisor_field_get_sf_node(self.parent_field._as_parameter_)
        else:
            index = self.parent_field.cached_value.index(self) # find self in parent field, hopefully already cached
            ref = wb.wb_supervisor_field_get_mf_node(self.parent_field._as_parameter_, index)
        self.__dict__['_as_paramater_'] = ref
        self.__dict__.pop('nickname', None)  # decache, as this may now become more informative
        if 'id' not in self.__dict__:  # now that this node's id is easily available, we can catalog it by id
            # It isn't necessary to check if self.is_inside_proto, since __new__ should have set self.id=None already
            id = self.__dict__['id'] = wb.wb_supervisor_node_get_id(ref)
            catalog[id] = self
        return ref

    @cached_property  # defined for linting purposes; superceded by Node.__new__ setting this directly
    def id(self) -> int:
        """Returns the unique ID number for this node, which reflects order of creation in the Webots scene tree.
           Returns None for merely planned nodes and for nodes inside the non-exposed parts of protos."""
        if not self.is_real: return None
        return wb.wb_supervisor_node_get_id(self._as_parameter_)
    @use_docstring_as_deprecation_warning
    def getId(self) -> int:
        """DEPRECATED: Node.getID() is deprecated.  Use node.id instead."""
        return wb.wb_supervisor_node_get_id(self._as_parameter_)

    wb.wb_supervisor_node_get_type_name.restype = c_char_p
    @descriptor  # takes lower priority than cached value, which is typically stored in Node creation
    def type(self) -> str:
        """Returns the type of this node, as a string, e.g. 'Solid', or '' if that type is not immediately available
           (typically for USE Nodes that are merely planned or that were parsed from an export of the scene tree
           where that node has not yet been accessed individually through the C-API)."""
        ref = self.__dict__.get('_as_parameter_')
        if ref is None: return ''  # tentatively return '', but do not cache this
        t = self.__dict__['type'] = wb.wb_supervisor_node_get_type_name(self._as_parameter_).decode()
        catalog[t].append(self)  # catalog this new nomiker #TODO check that node is not hidden first?
        return t
        # TODO consider whether I really want to defer fetching type when not immediately available?
        #  The initial motivation was largely to avoid looking up parsed USE nodes
        #  But now (or at least soon from now!) USE nodes should be handled in a way that avoids requesting type
        #   except for when doing so is required by settings.
        #  Arguably better to provide type on demand rather than saying "la la la can't hear you!" as it currently does!
        #
        # TODO may need same deferral for USE-nodes' .name as that won't be immediately available either
        #
        # TODO There is some inconsistency involving finding USE-nodes from ancestors. When a USE-node is parsed from
        #  an export, we currently don't get any information besides the DEF-name of the target, so in particular,
        #  we won't know its name or type, and it usually wouldn't be worth going out of our way to fetch that info...
        #  But after accessing (and cataloging) its name or type, then it would be accessible
        #
        # TODO Need to get clear on whether USE nodes themselves are "hidden", or if just their descendants are.
        #  If USE-nodes themselves are hidden, then that would already disqualify them from being found from afar,
        #  so it wouldn't matter that their lack of cataloged nomikers would as well.  But we need to be sure that
        #  searches from the parent would not just use the catalog, and would actually trigger .name and .type properties
        #  And we need to be sure that incremental searches obey the same visibility restrictions.
    @use_docstring_as_deprecation_warning
    def getTypeName(self) -> str:
        """DEPRECATED: Node.getTypeName is deprecated. Please use node.type instead."""
        return self.type
    def getType(self) -> str:
        """Returns this Node's type as an integer constant. It is encouraged to use node.type instead,
           which returns a more easily interpretable string, like 'Solid'."""
        return wb.wb_supervisor_node_get_type(self._as_parameter_)

    wb.wb_supervisor_node_get_base_type_name.restype = c_char_p
    @cached_property
    def base_type(self) -> str:
        """Returns the base type of this node, as a string, e.g. 'Solid'."""
        return wb.wb_supervisor_node_get_base_type_name(self._as_parameter_).decode()
    @use_docstring_as_deprecation_warning
    def getBaseTypeName(self):
        """DEPRECATED: Node.getBaseTypeName is deprecated. Please use node.base_type instead."""
        return wb.wb_supervisor_node_get_base_type_name(self._as_parameter_).decode()

    # --- Node naming ---

    wb.wb_supervisor_node_get_def.restype = c_char_p
    @cached_property  # defined for linting purposes; superceded by Node.__new__ setting this directly
    def DEF(self) -> str:
        """Returns the DEF name for this node, cached for faster repeat access, since DEF names are immutable."""
        return wb.wb_supervisor_node_get_def(self._as_parameter_).decode()
    def getDef(self) -> str:
        """DEPRECATED: Node.getDef() is deprecated.  Use node.DEF instead."""
        return self.DEF

    @cached_property
    def USE(self) -> str:
        """If self is a USE node, returns the DEF-name of the used source node.  Otherwise returns ''.
           By default, this would be pre-cached when an initial export of the scene-tree is parsed.
           This may be called in atypical cases where a node has been accessed by the C-API rather than pre-parsing.
           In this case, our only way to tell if this node is a USE-node is to look at an export of it,
           which is somewhat slow, but fortunately this shouldn't happen much, and we'll cache the answer."""
        if not self.is_real: return ''                              # Planned USE nodes would have .USE explicitly set
        if self.parent_field and self.parent_field.is_MF: return '' # USE nodes can dwell only in SF fields
        s = self.encode()  # .encode() is like .export(), but with the output left as a python bytes string
        if not s.startswith(b'USE '): return ''
        return s[4:].decode()

    @cached_property
    def name(self) -> str:
        """Returns this node's name field content, if it has one, or '' otherwise, cached for fast repeat access.
           A node's name may be changed by setting node.name = new_name, which updates (1) its name field in the
           simulation, (2) its node.name cache, (3) the nickname used in string representations of the node,
           (4) world.catalog which maps each name to a list of nodes with that name for fast lookup of nodes by name,
           and (5) whether ancestors think of ancestor.old_name as a way of referring to this node.
           If (1) the nodes's name field is changed in other ways (e.g. by another supervisor, or by directly setting
           node.fields.name.value = new_value), (2-5) will behave as though the node still has its old name.
           You may access the current value of the name field with n.fields.name.value and to refresh all of (1-5) to
           that current value, set node.name = node.fields.name.value"""
        # We bypass using self.fields.name for speed and to avoid potential circularity
        if not self.is_real: return ''
        name_field = wb.wb_supervisor_node_get_field(self._as_parameter_, b"name")
        # TODO in nested protos reading this field can cause an OSError, even when the node and field's ref's both look fine!
        return wb.wb_supervisor_field_get_sf_string(name_field).decode() if name_field else ''
    @name.set
    def name(self, new_name: str):
        # TODO should this behave differently for merely planned nodes?
        if isinstance(new_name, Field): return  # TODO not sure if this is needed to avoid getting name field stored as node.name?
        old_name = self.__dict__.get('name')  # remember our cached old name (or None if hadn't been cached yet)
        self.fields.name.set_value(new_name)  # (1) update the name field in the scene tree
        self.__dict__['name'] = new_name      # (2a) update locally cached name
        self.__dict__.pop('nomikers', None)   # (2b) decache as this will now have changed; will recompute self on demand
        self.__dict__.pop('nickname', None)   # (3) decache as this may now change; will recompute itself on demand
        entry = catalog[old_name]                # (4a) remove catalog[old_name] reference(s) to self
        while self in entry: entry.remove(self)  #
        catalog[new_name].append(self)           # (4b) create catalog[new_name] reference to self
        # (5) Forget cases where an ancestor refers to this node as ancestor.old_name
        references_by_old_name = {(ancestor, name) for ancestor, name in self._incoming_references if name == old_name}
        for ancestor, name in references_by_old_name:
            actual_value = ancestor.__dict__.pop(name, self)  # pops existing value if there is one, or self if not
            # if the ancestor had somehow come to refer to something other than self, put that value back (oops!)
            if actual_value is not self: ancestor.__dict__[name] = actual_value
        self._incoming_references -= references_by_old_name # No need to still remember the references we just wiped

    @cached_property
    def nomikers(self) -> Tuple[str, ...]:
        """Returns a sequence of nomikers that this node is searchable by, including whichever ones of its
           DEF-name, name field, and node type are non-empty."""
        if self.USE: return tuple(n for n in (self.DEF, self.name) if n)
        return tuple(n for n in (self.DEF, self.name, self.type) if n)

    @cached_property
    def nickname(self):
        """Returns and caches an informative name for this node, e.g. ROBOT1, Robot['Marvin'], or Robot[#8].
           This will fetch information from the C-API only for nodes whose wbNodeRef has already been fetched."""
        explored = '_as_parameter_' in self.__dict__
        if (explored or 'USE' in self.__dict__) and self.USE: return f"USE {self.USE}"
        if (explored or 'DEF' in self.__dict__) and self.DEF: return self.DEF
        if (explored or 'name' in self.__dict__) and self.name: return f'{self.type}["{self.name}"]'
        if explored or 'id' in self.__dict__: return f"{self.type}[#{self.id}]"
        if self.is_real: return f"{self.type}[unexplored]"
        return f"{self.type}[planned]"

    def __repr__(self): return self.nickname

    # --- Node ancestry and comparison ---

    wb.wb_supervisor_node_get_parent_node.restype = wbNodeRef
    @cached_property
    def parent(self) -> 'Node':
        """Returns the parent of this node in the scene tree: the node with a SFNode/MFNode field containing this node."""
        if 'parent_field' in self.__dict__: return self.parent_field.node
        if not self.is_real: return None
        return Node(wb.wb_supervisor_node_get_parent_node(self._as_parameter_))
    @use_docstring_as_deprecation_warning
    def getParentNode(self) -> 'Node':
        """DEPRECATED: Node.getParentNode() is deprecated. Please use node.parent instead."""
        return self.parent

    @cached_property
    def parent_field(self) -> 'Field':
        """Returns the field containing this node in this node's parent, or None."""
        # By default, scene tree structure is drawn from initial export, so most nodes' parent_fields are pre-cached.
        # This will be called for nodes that are found in other ways, e.g. by id number, or by world.self
        if self.parent is None: return None
        return next((f for f in self.parent.fields_that_may_contain_children if self in f.children), None)

    @property
    def parent_field_index(self) -> int:
        """Returns the index of self within self.parent_field"""
        return self.parent_field.cached_value.index(self)

    @cached_property
    def lineage(self) -> Tuple['Node', ...]:
        """An ancestral list of nodes starting with the root, and descending to this node."""
        if self.parent is None: return (self,)
        return self.parent.lineage + (self,)

    @property
    def local_address(self) -> str:
        """For parentless nodes, this simply returns the node's nickname, like 'world'.
           Otherwise this returns something like `children[0]`: a string indicating the parent_field.name, and when
           that parent_field is MF, a [bracketed] indication of this node's index in that parent field."""
        if self.parent is None: return self.nickname
        if self.parent_field.seems_to_be_MF:
            return f"{self.parent_field.name}[{self.parent_field_index}]"
        return self.parent_field.name

    @property
    def full_address(self) -> str:
        """A string similar to `world.children[12].endPoint.children[0].geometry` that begins with a nickname
           for this node's root, and then proceeds through a dot-connected path of fields leading from that root
           down to this node, including [bracketed] indices for the MF fields. This full_address could change if
           earlier fields are inserted or deleted from any of those MF fields."""
        return '.'.join(n.local_address for n in self.lineage)

    @cached_property
    def _depth(self) -> int:
        """The number of ancestors this node has."""
        return len(self.lineage) - 1

    def __contains__(ancestor, descendant: 'Node', visibility: Union[int, Callable] = all) -> bool:
        """Returns True if descendant is visible from ancestor, as constrained by visibility, which indicates which
           descendants will still be visible even if "hidden" by being within a USE node or non-exposed PROTO field.
           If visibility is the default `all`, then all descendants are taken to visible from their ancestors,
           so Python will evaluate `descendant in ancestor` as True for any descendant of ancestor, even hidden ones.
           If visibility is 1, then immediate children are always visible even if "hidden", but more distant descendants
           hidden by an intermediary PROTO/USE node are not. This setting is commonly used in searching for descendants,
           and makes "hidden" descendants accessible only by searches initiated in the parent that hides them.
           If visibility is 0, then no hidden nodes are visible, even immediate children."""
        # First we reject anything that is not a descendant of ancestor
        if not isinstance(descendant, Node): return False                     # descendant must be a node,
        if descendant._depth <= ancestor._depth: return False                 # that is deeper than ancestor,
        if descendant.lineage[ancestor._depth] is not ancestor: return False  # and has ancestor in its lineage.
        # If all descendants count as visible, then we're done!
        if visibility is all: return True
        # Otherwise, we return True iff no intermediary beyond the visibility range blocks ancestor's view
        return not any( True for i in range(ancestor._depth+1+(visibility or 0), descendant._depth+1)
                        if descendant.lineage[i].is_hiding )

    def __lt__(A, B) -> bool:
        """Node A is "less than" Node B iff A would be tagged first in a "standard traversal" of all Nodes.
           A "standard traversal" begins by tagging and exploring the root. Whenever it explores node N, it
           first tags all of N's children (as generated by N.all_children) and then explores each of those children.
           This is the same order as root.all_descendants, though this is more efficient than full traversal."""
        if not isinstance(B, Node): return NotImplemented
        # Let An and Bn be the first pair of nodes in A's and B's respective lineages that diverge (or both None).
        An, Bn = next( ((An,Bn) for An,Bn in zip(A.lineage, B.lineage) if An is not Bn), (None,None) )
        if An is Bn: return A._depth < B._depth # If A and B are in same lineage, then A < B iff A is less deep
        # If A xor B is an immediate child of their common ancestor G, then A < B iff A was that immediate child
        if (An is A) ^ (Bn is B): return An is A
        # Otherwise, A < B iff An < Bn, i.e., iff An arises before Bn in G.all_children.
        return An is next(child for child in An.parent.all_children if child is An or child is Bn)
        # TODO the outcome for siblings and cousins depends upon the ordering of their common ancestor's .all_children,
        #  which currently, in fully_cataloged parts of the tree, depends upon its known_children ordering, which
        #  unfortunately is difficult to predict and may differ from the index-ordering used in not fully_cataloged parts
        #  This could eventually be made more consistent by learning what fields each NodeType has, or `export`ing them

        # TODO some traversals prioritize exposed over hidden. Such prioritization that is present in Node.all_children
        #  will be inherited here.  But is that the only such prioritazation I do?  E.g. should hidden children really be
        #  initially tagged prior to the exploration of exposed ones?

    def __le__(A, B) -> bool: return A is B or A < B

    def __bool__(self): return True
    # TODO may eventually want nodes to stop returning True once they've been removed

    # --- Node iterating children ---

    @staticmethod
    def children_in(fields: Iterable['Field']) -> Iterable['Node']:
        """Yields successive children from the given fields, which may be a mix of SFNode, MFNode and/or others."""
        return itertools.chain.from_iterable(f.children for f in fields if f.can_contain_children)

    def decache_children(self):
        """Decaches any lists of children of this node.  Automatically called by methods that add/delete children."""
        for property_name in Node._cached_children_property.names:
            if property_name in self.__dict__: del self.__dict__[property_name]
        # TODO: if I do similar caching for lists of descendants, this should trigger decaching those too, up the lineage

    class _cached_children_property:
        """An @_cached_children_property is like a descriptors.cached_property (or Python 3.8 functools.cached_property)
           but also stores a record of this property in Node._cached_children_property.names to allow easy decaching
           of lists of children whenever a child is added or removed."""
        names = []
        def __new__(cls, getter):
            cls.names.append(getter.__name__)
            return cached_property(getter)

    @_cached_children_property
    def known_children(self) -> List['Node']:
        """Returns a list of immediate children of this node that are already known, but does not construct new Nodes.
           Visible fields are prioritized over hidden PROTO fields.  Otherwise, Node-containing fields are traversed
           in their order of creation, which may be difficult to predict/remember.
           This will be cached for faster repeat access and automatically decached whenever a child is added/removed.
           TODO as with all_children, this could be made more predictable if we knew what fields each node type has."""
        return list(self.children_in(self.known_fields))

    @_cached_children_property
    def hidden_children(self) -> List['Node']:
        """Returns a list of immediate children of the given node that are "hidden" due to being in a non-exposed
           PROTO field or (depending on settings) USE node, and so (by default) will be hidden from searches from
           ancestors of this node.
           This will be cached for faster repeat access and automatically decached whenever a child is added/removed."""
        # TODO are there concerns that modifying a USE target thereby also modifies its USE shadow?  If so,
        #  that'd pose challenges for any attempt to cache the shadow's structure!
        print(f"Creating list of hidden_children for {self}")
        all_children = self.children_in(self.fields_that_may_contain_children)
        if settings.use_node_children_are_hidden and self.USE: return list(all_children)
        return [child for child in all_children if child.is_hiding]

    @_cached_children_property
    def visible_children(self) -> List['Node']:
        """Returns a list of children of this Node that will be visible to searches from this node's ancestors -- i.e.
           children that are not hidden inside non-exposed proto fields nor inside a USE shadow node. This constructs
           new Nodes where needed.
           This will be cached for faster repeat access and automatically decached whenever a child is added/removed."""
        if settings.use_node_children_are_hidden and self.USE: return []
        if not settings.use_nodes_are_hidden:
            return list(self.children_in(self.fields._that_may_contain_children))
        return list(child for child in self.children_in(self.fields._that_may_contain_children) if not child.is_hiding)

    @_cached_children_property
    def all_children(self) -> List['Node']:
        """Returns a list of immediate children within any of this node's fields (not just the field named 'children',
           which is accessible as node.children, when present), newly constructing Nodes for them, if necessary.
           Exposed fields of proto nodes are prioritized over non-exposed fields.  Otherwise,
           Node-containing fields are traversed in their order of creation, which may be difficult to predict/remember.
           This will be cached for faster repeat access, and automatically decached whenever a child is added/removed.
           TODO By default, the proxy tree will initially parse an export of the full scene tree, which will create
            python Fields for any fields that don't contain default values, and then will create additional Fields
            on demand. If we implement a way for nodetypes to know which fields they're supposed to have, this could be
            made to be more predictable, e.g. by traversing the known fields in their expected order."""
        return list(self.children_in(self.fields_that_may_contain_children))

    # --- Node handling descendants ---

    def get_node(self, name: str, visibility: Union[int, Callable] = 1) -> 'Node':
        """This recursively seeks out, at any level, a descendant of this node whose DEF-name, name-field or NodeType
           is name, equivalent to calling `world.Node( name, search_area=this_node, visibility=visibility )`.
           Returns None if no match found.  Usually it is advisable to simpy use `this_node.name` which typically has
           the same effect. The main reasons to use this are to match a `name` field that's not a legal Python name due
           to spaces or punctuation, or to disambiguate from look-alike names for fields or methods.
           The optional visibility parameter works as in world.Node(), for which this is a convenience caller."""
        return Node(name, search_area=self, visibility=visibility)

    def get_proto_node(self, name: str) -> 'Node':
        """This seeks out, at any level, a descendant within the non-exposed field of this PROTO node whose DEF-name,
           name-field or NodeType is name.  Returns None if no match found.
           Usually it is advisable to simpy use `this_node.name` which typically has the same effect, and caches the
           node for faster re-use. The main reasons to call this are to match a `name` field that's not a legal Python
           name due to spaces or punctuation, or to disambiguate from like-named fields or methods."""
        return Node(name, search_area=self.children_in(self.proto_fields), visibility=all)

    wb.wb_supervisor_node_get_from_proto_def.restype = wbNodeRef
    def getFromProtoDef(self, name: str) -> 'Node':
        """Returns a descendant node with this DEF-name hidden within the non-exposed part of this PROTO node, or None.
           In most circumstances node.name would also provide access to this node, so long as there is no
           higher-priority target with the same name, e.g. a Node.method or a non-hidden descendant.
           node.get_proto_node( name ) avoids such collisions with other sorts of objects.  Both of these alternatives
           match name-fields and NodeTypes in addition to DEF-names."""
        return Node(wb.wb_supervisor_node_get_from_proto_def(self._as_parameter_, name.encode()))

    def catalog_descendants(self):
        """Catalogs all descendants of this Node by having Webots export a VRML string representation of the
           corresponding node in the simulation, and parsing that representation into this proxy Node's fields,
           including all descendants. This is significantly faster than using separate C-API calls to traverse the tree.
           TODO This also avoids needlessly accessing node's fields, which is good because, at the time of this writing,
            accessing more fields causes the Webots C-API to become slower in subsequent ops involving other fields.
           Cataloging descendants makes it much faster to find nodes by DEF/type/name, and/or to iterate through the
           tree, e.g. with node.all_descendants.  By default, the entire world is cataloged upon `import world`, and
           this catalog is automatically updated in response to changes made by this supervisor. So there is rarely
           any need to explicitly recatalog anything, except in cases where that setting was turned off, or where
           you suspect another supervisor may have added or removed nodes to the tree, making the catalog out of date.
           If descendants are not pre-cataloged, the Proxy Scene Tree would incrementally catalog them on demand, but
           in a way that will likely end up having been slower if you do much searching or traversal of the tree.
           TODO: This does not currently make any moves to evacuate old contents, so may be safest to use when
            no such contents have yet been identified, or after calling node.forget_descendants().
            Ideally the C-API will eventually offer a version of node.export that includes nodes' unique id's,
            which would enable refreshing the catalog without needing to forget all of the old contents first."""
        VRML_Parser(self.export(), is_real=True).parse_node(self)

    # TODO various versions of descendant iteration could benefit from being cached, akin to @cached_children_property
    #  However currently they're implemented as iterators, which allow the prospect of stopping partway through
    #  Shifting to caching would either require pre-building the whole sequence or some hybrid approach that
    #   caches the sequence as it is incrementally built.  Triggered decaching would unfortunately cascade upward,
    #   so in general there are fewer benefits to caching when more is unstable.
    #  There might also be some advantage to making the various component lists be stable rather than rebuilt,
    #   so that ancestors could build structures that embed descendants' lists, and count upon changes to those lists
    #   to automatically percolate up without any need to decache all the way up.  The resulting nested lists would
    #   be a bit harder to iterate than a simple list, but probably still much faster than nested function calls.

    def traverse(self, start = 'all_children', recursive = 'visible_children') -> Iterable['Node']:
        """Yields successive descendants of self, starting with ones listed in self.start, and proceeding
           through the ones listed in descendant.recursive for each descendant thus reached.
           This traversal is mostly depth-first except all relevant immediate children are yielded at each layer
           before any is delved into.
           If start and recursive both involve all/visible children, then each node is marked as fully_cataloged
           after its descendants have been traversed.
           This traversal has been optimized to avoid slow recursive function calls and nested iterables."""
        if start is all: start = 'all_children'
        if recursive is None: recursive = 'visible_children'
        higher_layers = []
        higher_indices = []
        higher_lengths = []
        current_layer = getattr(self, start)
        yield from current_layer
        current_index = 0
        depth = 0
        max_depth = last_push_depth = -1
        current_length = len(current_layer)
        while True:
            while current_index >= current_length: # while finished with current layer, pop up to next unfinished layer
                if depth <= 0: return              # or if all layers are finished, can return entirely
                depth -= 1
                last_push_depth = depth            # any lower pushes are no longer relevant
                current_layer = higher_layers[depth]
                current_index = higher_indices[depth]
                current_length = higher_lengths[depth]
            next_layer = getattr(current_layer[current_index], recursive)
            current_index += 1  # after we deal with next_layer we'll be ready to look in the next node on this layer
            if next_layer:  # if there are subnodes in next_layer, push current state to stack while we handle them
                yield from next_layer
                if last_push_depth == depth:    # if we've already pushed at this depth, all we need to update is index
                    higher_indices[depth] = current_index
                else:                      # otherwise we need to push the current state
                    if depth > max_depth:  # If we've never reached this depth before, need to append to our stack
                        higher_layers.append(current_layer)
                        higher_indices.append(current_index)
                        higher_lengths.append(current_length)
                        max_depth += 1
                    else:                  # Otherwise, we can just overwrite the old stacked value at this depth
                        higher_layers[depth] = current_layer
                        higher_indices[depth] = current_index
                        higher_lengths[depth] = current_length
                    last_push_depth = depth
                # and now prepare to traverse next_layer
                current_layer = next_layer
                current_index = 0
                current_length = len(next_layer)
                depth += 1
                # loop back up to start traversing next_layer

    @property
    def all_descendants(self: 'Node') -> Iterable['Node']:
        """Yields successive descendants of the given node using a recursive search, including not just ones in ordinary
           visible fields like `children` or `endPoint`, but also ones "hidden" inside non-exposed PROTO fields or USE
           shadow nodes. New Nodes will be created where needed.
           This search is mostly depth-first, except all immediate children of a node are yielded before delving deeper.
           Each node's hidden PROTO fields are traversed after its ordinary exposed fields."""
        return self.traverse('all_children', 'all_children')
        # TODO old way with recursive calls and nested generators, replaced with significantly faster traverse() call
        # all_children = self.all_children
        # yield from all_children
        # for child in all_children:
        #     yield from child.all_descendants
        # self.__dict__['is_fully_cataloged'] = self.is_real

    @property
    def visible_descendants(self: 'Node') -> Iterable['Node']:
        """Yields successive "visible" descendants of the given node using a recursive search, but does not include
           any "hidden" descendants (inside USE nodes or non-exposed PROTO fields), not even immediate children.
           New Nodes will be created as needed. This search is mostly depth-first, except all immediate children of a
           node are yielded before delving deeper."""
        return self.traverse('visible_children', 'visible_children')
        # TODO old way with recursive calls and nested generators, replaced with significantly faster traverse() call
        # visible_children = self.visible_children
        # yield from visible_children
        # for child in visible_children:
        #     yield from child.visible_descendants
        # self.__dict__['is_fully_cataloged'] = self.is_real

    @property
    def known_descendants(self: Union['Node', wbNodeRef]) -> Iterable['Node']:
        """Yields successive known descendants of the given node using a recursive search, including ones in a `children`
           field or in other SFNode/MFNode fields like `endPoint`. No new Nodes will be created.
           This search is mostly depth-first, except all known children of a node are yielded before delving deeper.
           Exposed proto fields will be traversed, but non-exposed ones will not be."""
        return self.traverse('known_children', 'known_children')
        # TODO old way with recursive calls and nested generators, replaced with significantly faster traverse() call
        # known_children = self.known_children
        # yield from known_children
        # for child in known_children:
        #     yield from child.known_descendants

    @property
    def descendants(self: 'Node') -> Iterable['Node']:
        """Yields successive descendants of the given node using a recursive search, including hidden children of this
           node itself, but not hidden further descendants.  This search is mostly depth-first, except all relevant
           immediate children of a node are yielded before delving deeper. This yields the same nodes as the ones that
           would be iterated over by `for descendant in node:` and, by default, are searched by `node.get_node()`"""
        return self.traverse('all_children', 'visible_children')
        # TODO old way with recursive calls and nested generators, replaced with significantly faster traverse() call
        # relevant_children = self.all_children
        # yield from relevant_children
        # for child in relevant_children:
        #     yield from child.visible_descendants
        # self.__dict__['is_fully_cataloged'] = self.is_real

    def __iter__(self):
        """Iterating `for descendant in node` yields successive descendants of the given node using a recursive search,
           including hidden children of this node itself, but not hidden further descendants.
           This search is mostly depth-first, except all relevant immediate children of a node are yielded before
           delving deeper. This yields the same nodes as the ones (by default) searched by `node.get_node()`"""
        return self.traverse('all_chidren', 'visible_children')

    # --- Node field and proto-field handling ---

    wb.wb_supervisor_node_is_proto.restype = c_bool
    @cached_property
    def is_proto(self) -> bool:
        """Returns True if this Node is a PROTO node."""
        if not self.is_real: return False  # TODO figure out best way to handle merely-planned PROTOs???
        return wb.wb_supervisor_node_is_proto(self._as_parameter_)
    @use_docstring_as_deprecation_warning
    def isProto(self) -> bool:
        """DEPRECATED: Node.isProto() is deprecated.  Use node.is_proto instead"""
        return wb.wb_supervisor_node_is_proto(self._as_parameter_)

    @cached_property
    def is_inside_proto(self):
        """Returns True iff this node is descended from a non-exposed proto field."""
        if not self.is_real: return False
        return self.parent.is_inside_proto or self.parent_field.hides_children

    @descriptor  # low priority so will defer to cached value; this manually self-caches when stable value is known
    def is_hiding(self) -> bool:
        """A Node is "hiding" if it is directly inside a non-exposed field of a PROTO, or
           perhaps (depending upon settings) if itself and/or its parent is a USE node.
           By default "hidden" nodes are invisible to searches from ancestors of their parent, so interacting with a
           hidden node usually requires explicitly referencing its parent (TODO or another part of its meganode?)
           and then seeking the hidden node inside of it.
           The returned value will be cached except when unstable (for planned nodes that have no parent yet).
           TENTATIVELY: Default settings leave USE nodes practically hidden (because minimal info about them is present
           in an exported version of the scene tree) until info about them is explicitly accessed, after which they
           are visible.  Default settings make the children of USE nodes be hidden. """
        if settings.use_nodes_are_hidden == 1 and self.USE:
            self.__dict__['is_hiding'] = True
            return True
        # (If the settings say USE-nodes are half-hidden we won't claim to be hiding but we'll also likely not show
        #  up in early catalog-based searches due to there not being any cataloged information about us yet.)
        # The only other things that could hide self involve the parent, so if self has no parent, it can't hide
        if self.parent is None:
            return False # but don't cache, in case this planned node later gets embedded in a field that hides it
        # The parent can hide this node by keeping it in an unexposed field, or by being a USE node (given settings)
        self.__dict__['is_hiding'] = (self.parent_field.hides_children or
                                     (settings.use_node_children_are_hidden and self.parent.USE))
        return self.is_hiding

    # TODO since this may end up iterating through children anyway, probably better to fold this in with .hidden_children
    def may_have_hidden_children(self) -> bool:
        """Returns True iff this Node may have children that would be hidden from searches from this node's ancestors
           and so may have been omitted from a catalog-based search -- i.e., iff this node is a USE node or a PROTO node."""
        if self.is_proto: return True
        if settings.use_node_children_are_hidden and self.USE: return True
        if settings.use_nodes_are_hidden:
            return any(1 for f in self.fields_that_may_contain_children
                       if not f.seems_to_be_MF and isinstance(f.cached_value, Node) and f.cached_value.USE)
        return False

    @cached_property  # Defined for linting purposes; superceded by __new__ setting instance.fields
    def fields(self) -> 'FieldsBearer':
        """For each Node, `node.fields.fieldname` or `node.fields['field_name']` will return the Field of this node
           with that name, or raises an AttributeError.
           Note: `node.fieldname` typically lets you read/alter this field more directly, so is often preferable.
           You may also use `node.fields[index]`, `len(node.fields)` and `for field in node.fields`."""
        return FieldsBearer(self)

    @cached_property  # Since this is rarely used, we'll generate only on demand
    def proto_fields(self) -> 'ProtoFieldsBearer':
        """For each proto Node, `node.proto_fields.fieldname` or `node.proto_fields['field_name']` will return the
           hidden Field of this proto node with that name, or raises an AttributeError.
           Note: `node.fieldname` typically lets you read/alter this hidden field more directly, so long as no
           higher-priority target (like a Node.method or exposed descendant node) shares that name.
           This supports `node.proto_fields[index]`, `len(node.proto_fields)` and `for field in node.proto_fields`."""
        return ProtoFieldsBearer(self)

    @property
    def all_fields(self) -> Iterable['Field']:
        """Yields all fields from self.fields and then self.proto_fields, fetching them through the C-API as needed.
           Fields are returned by their index-ordering in Webots."""
        # TODO double-check how proto hidden fields appear in exports."""
        if not self.is_proto: return self.fields
        return itertools.chain(self.fields, self.proto_fields)

    @property
    def known_fields(self) -> Iterable['Field']:
        """Yields known fields from this nodes .fields and then its .proto_fields, including ones that have been
           previously found, and ones that are known to exist from having pre-parsed a VRML version of this node,
           TODO or from this Node's type,
           but will not seek any new fields from the C-API.
           Ordinary fields (and then hidden proto fields) are currently yielded in the order they were created,
           which may be somewhat difficult to remember/predict. This may change in future updates.
           TODO A more flexible version of node.export, and/or a way to access default fields for each nodetype,
            would allow a more consistent predictable ordering.
           #TODO double-check how proto hidden fields appear in exports."""
        if not self.is_proto: return self.fields._that_are_known
        return itertools.chain(self.fields._that_are_known, self.proto_fields._that_are_known)

    @property
    def fields_that_may_contain_children(self) -> Iterable['Field']:
        """Yields all of this node's fields that may contain children, first from self.fields then self.proto_fields.
           In the (typical) case where all relevant fields are already known, fields will be quickly skimmed from
           self's known fields, TODO with its currently-suboptimal ordering;
           otherwise fields will be slowly fetched through the C-API as needed by their index-ordering in Webots."""
        if not self.is_proto: return self.fields._that_may_contain_children
        return itertools.chain(self.fields._that_may_contain_children, self.proto_fields._that_may_contain_children)

    # --- Deprecated Node field-getting functions ---

    @use_docstring_as_deprecation_warning
    def getField(self, field_name) -> 'Field':
        """DEPRECATED: Node.getField('field_name') is deprecated. A direct equivalent is node.fields['field_name'].
           But it's usually better to simply use node.field_name to directly set/retrieve the value of an SF field,
           or to get a container-like handler for an MF field."""
        return self.fields[field_name]

    @use_docstring_as_deprecation_warning
    def getProtoField(self, field_name) -> 'Field':
        """DEPRECATED: Node.getProtoField(field_name) is deprecated. An equivalent is node.proto_fields[field_name].
           But it's usually better to simply use node.field_name to directly set/retrieve the value of an SF field,
           or to get a container-like handler for an MF field."""
        return self.proto_fields[field_name]

    @use_docstring_as_deprecation_warning
    def getNumberOfFields(self) -> int:
        """DEPRECATED: Node.getNumberOfFields() is deprecated. Use len(node.fields) instead."""
        return wb.wb_supervisor_node_get_number_of_fields(self._as_parameter_)

    @use_docstring_as_deprecation_warning
    def getProtoNumberOfFields(self) -> int:
        """DEPRECATED: Node.getProtoNumberOfFields() is deprecated. Use len(node.proto_fields) instead."""
        return wb.wb_supervisor_node_get_proto_number_of_fields(self._as_parameter_)

    wb.wb_supervisor_node_get_field_by_index.restype = wbFieldRef
    @use_docstring_as_deprecation_warning
    def getFieldByIndex(self, index: int) -> 'Field':
        """DEPRECATED: Node.getFieldByIndex() is deprecated. Use node.fields[index] instead."""
        return self.fields[index]

    wb.wb_supervisor_node_get_proto_field_by_index.restype = wbFieldRef
    @use_docstring_as_deprecation_warning
    def getProtoFieldByIndex(self, index: int) -> 'Field':
        """DEPRECATED: Node.getProtoFieldByIndex() is deprecated. Use node.proto_fields[index] instead."""
        return self.proto_fields[index]

    # --- Node __getattr__ ---

    """Old explanation:  For SF fields, the main things we need are to get/set their Webots values
           when the field is gotten/set.  This could be implemented with __get__ and __set__,
           but those are operable only on *class* attributes, but we need to have
           different fields for different nodes, and don't want to proliferate classes
           to match our number of nodes.  So instead, we'll let these be handled by
           the __getattr__ and __setattr__ methods that we had to define for Node anyway.
           So we simply register this field in node.fields, and Node.__getattr__ and .__setattr__
           will redirect to field.value.  This returns the current value from that field.

           For MF fields, the main things we need are (1) a setter for the whole field
           to handle cases like node.field = valueList -- the field will be added to node.fields
           and node.__setattr__ will use this field's .value setter, and (2) to make node.field
           return an MFField object with a container interface including __getitem__ and __setitem__.
           These methods must be defined as class methods of the *field's* class, unlike
           __get__ methods, which would have needed to have been defined in each *node's* class,
           so we have no need to proliferate node classes.  The new MFField will be stored as an
           ordinary attribute of node, so that it will be speedily gotten by node.field .
           (Again set operations will be intercepted by node's __setattr__.)
           For many purposes, this MFField will operate as a surrogate for the contained value.
           """

    wb.wb_supervisor_field_get_type_name.restype = c_char_p
    def __getattr__(self, attr: str):
        """This is called when someone attempts to access node.attr when it doesn't currently possess that attribute.
           We attempt to resolve this as a reference to one of this node's associated scene tree fields, e.g.
           world.ROBOT.translation, or to one of its descendant Nodes, e.g. world.ROBOT.CAMERA.
           When this search finds a match, a Python Field or Node object will be created, stored as self.fields.attr or
           self.attr to speed repeat access, and it (or its value) is returned. Failed searches raise AttributeError.
           Note: there is some risk of name collisions, e.g. between Python node.attributes, Node class methods, Webots
           scene tree fields, and/or the DEF/type/name of descendant nodes. This employs a sensible prioritization
           for resolving these conflicts, described below. To help avoid such name collisions, it is encouraged to
           follow the Webots convention of using ALL-CAPS DEF-names for nodes, and referring to nodes by DEF-names.
           This should avoid all node-name collisions but 'DEF', and all field-name collisions but unusual protos.
           If collisions do occur, node.get_node(name) or node.fields.name can access particular nodes/fields.
           Here is the current prioritization for seeking values to return (potentially subject to slight change
           in future versions):
             (1) existing attributes of this object (would be returned without this being called)
               (1a) this includes previously found descendants sought directly from this node
               (1b) this includes Field objects for previously accessed MF fields (but not SF)
             (2) existing attributes/methods of the Node's class(es) (would be returned without this being called)
               (2c) TODO this unfortunately includes any defined Node.CONSTANTS, which I used to give lower priority
             (3) previously found SF fields stored in self.fields; field.value will be returned
             (4) previously unfound fields of this scene-tree node. (For SF fields, field.value is again returned.)
               (4a) If this node is a PROTO, this includes its exposed fields, but not its non-exposed fields.
             (5) descendants of this node with given DEF-name, name, or NodeType (eg, ROBOT.CAMERA, .camera or .Camera)
                 These descendants are sought mostly depth-first, except looking for immediate matches in each node.
               (5a) this includes descendants in the "exposed" fields of PROTOs at this or any level
             (6) if PROTO: non-exposed proto fields (again returning field.value for SF fields)
             (7) if PROTO: descendants within the non-exposed part of this PROTO, again matching 3 ways, as in (5)
               TODO I think this is probably currently folded into (5). If we want this separate, may need to adjust (5)
             (8) fields of a closely-linked compatriot node (e.g. shape, appearance and geometry are linked in this way,
                 and if the shape is the 0th child of a Transform or Solid, it and its Physics node are linked as well)
             (9) non-callable attributes of this Webots node (numerous Webots constants)
               TODO If I want to demote Node.CONSTANTS in this way, I'd need to store them elsewhere...  Worth it?
            """
        # TODO think about how nodes within unexposed proto fields will fit into less-than comparisons

        # TODO Collapsing ProxyNode and Node had the unwanted effect of moving several priorities higher.
        #  One option is to live with that, though this will make some DEF-names harder to access, especially any
        #   that collide with Node.CONSTANTS, though I could probably limit how many of those I define.
        #  Another option would be to store the various old Node.methods and/or .CONSTANTS somewhere other than in Node
        #   or a superclass thereof, and continue to look these up with low priority (akin to seeking fields in other place)
        #  Yet another option might be to just use a (perhaps-custom) class-inheritance MRO, with metaclasses to help ensure lookups occur at right level

        # TODO figure out what keeps calling world.__len__ (and sometimes other nodes), probably some PyCharm autocomplete thing
        if attr == '__len__':
            raise AttributeError("Nodes do not have length")

        logprint(f"Seeking node attribute {self}.{attr}")

        # Priorities #1-2. existing attributes of this object/class (would have been returned without this being called)
        # Priorities #3-4. known SF fields, or previously unknown fields of this Webots node
        # When getting an attribute from a merely-planned node, we search only the fields known in its __dict__
        # TODO do I still need this inline if?
        field = (self.fields if self.is_real else self.fields.__dict__).get(attr, None)
        if field is not None:
            if field.seems_to_be_MF: return field  # getting MF field returns that Field to mediate access to elements
            # Otherwise it must be SF, so we "see through" to its value (node.name returns its name, not its name Field)
            return field.get_value()

        # Priority #5. The first descendant of this node whose DEF-name, name, or NodeType matches attr
        # TODO consider whether we want to take steps to exclude unexposed Proto fields from this search?
        #  If proto fields are included, may want to exclude them from being cached as self.attr?
        descendant = Node(attr, search_area = self)
        if descendant:
            if self.is_real:  # in real nodes, but not merely planned ones, we store shortcut down to descendant
                self.__dict__[attr] = descendant
                descendant._incoming_references.add( (self, attr) )  # let descendant know self has name for it, for cleanup
            return descendant

        # Priorities #6-7: Non-exposed fields / descendants of this *PROTO* node.
        if self.is_real and self.is_proto:
            # Priority #6. Non-exposed fields of this PROTO node.
            field = self.proto_fields.get(attr, None)
            if field is not None:
                if field.is_MF: return field  # getting MF field returns that Field to mediate access to elements
                # Otherwise it must be SF, so we "see through" to its value (node.name returns its name, not its name Field)
                return field.get_value()

            # Priority #7. For proto nodes, the first matching descendant within the non-exposed part of the proto.
            # TODO I suspect this will be redundant with #5, currently
            hidden_descendant = Node(attr, search_area=self.children_in(self.proto_fields))
            # TODO if we want to enforce read-only-ness deep within proto, we'd need to mark such nodes, perhaps here
            # For ordinary nodes, this would be cached under self.__dict__[attr]. We refrain due to proto volatility.
            if hidden_descendant: return hidden_descendant

        # Priority #8: fields of a closely-linked compatriot node
        # (e.g. shape, appearance and geometry are linked in this way,
        # and if the shape is the 0th child of a Transform or Solid, it and its Physics node are linked as well)

        # TODO if we want this

        # Priority #9: Non-callable attributes of contained node (numerous Webots constants)
        # TODO dynamically look them up elsewhere, bind them to this node, cache on this node, return

        # If we got here, that means our search failed...
        raise AttributeError(f"Node {self} has no match for '{attr}' "
                             f"among its instance/class attributes, scene tree fields or descendants.")

    # --- Node __setattr__ ---

    def __setattr__(self, attr: str, value: any):
        """This is called whenever someone attempts to set node.attr = new_value.  We attempt to construe unfamiliar
           attributes as references to fields of the associated scene tree node. If successful a Field will be created,
           and its own .value setter method will be used to alter the value of that field in the simulation,
           automatically selecting appropriate Webots functions for this field's VRML datatype.
           Found Fields are cached in node.fields to speed repeated access. As noted with __getattr__, there
           is some risk of name collisions, esp. between Node.methods and proto nodes with unusual field names.
           This uses a sensible prioritization to handle such collisions, described below, or setting
           node.fields.fieldname.value = new_value may bypass the name collision.  Here is the current prioritization
           for seeking things to set (potentially subject to slight revision in future versions):
           (0) any property/descriptor defined in the node's classes will use its __set__ method, if present
           (1) attributes with existing entries in node.fields will employ that field's own .value setter
           (2) existing attributes of this object will be overriden, including attributes added in node-construction
           (3) fields of the associated scene-tree node will create a new Field and use its .value setter, as in (1)
           (3a) merely-planned Nodes currently auto-create a field for *every* unknown attribute name
             # TODO this might change if we someday get a way to access the known fields by NodeType
           (4) otherwise a new node.attribute will be created with the given value in the Python Node."""
        logprint(f"Now attempting to set {self}.{attr} to a new value.")
        # Priority #0. Use __set__ method of a descriptor defined in the node's classes
        descriptor = getattr(type(self), attr, None)
        if descriptor:
            descriptor_set_method = getattr(descriptor, '__set__', None)  # should be bound to this descriptor
            if descriptor_set_method:
                descriptor_set_method(self, value)
                return
        # Priority #1. Use value setter for an already-known field
        known_field = self.fields.__dict__.get(attr)  # find field if previously known, or None otherwise
        if known_field:
            known_field.set_value(value)
        # Priority #2: overwrite existing attributes of this node
        elif attr in self.__dict__:
            self.__dict__[attr] = value
        else:
            # Priority #3:  seek/create previously unknown field
            new_field = self.fields.set(attr, value)  # will create, set, and return field, or None if no such field
            if new_field is None:
                # Priority #4: create this as a brand new node attribute
                self.__dict__[attr] = value

    # --- Node physical measures ---

    wb.wb_supervisor_node_get_position.restype = c_void_p
    @property
    def position(self) -> Vec3f:
        """Returns the current global position of this node, an xyz vector."""
        return Vec3f.from_address(wb.wb_supervisor_node_get_position(self._as_parameter_))
    @use_docstring_as_deprecation_warning
    def getPosition(self) -> Vec3f:
        """DEPRECATED. Node.getPosition() is deprecated.  Use node.position instead."""
        return Vec3f.from_address(wb.wb_supervisor_node_get_position(self._as_parameter_))

    # TODO this returns a 3x3 vector -- don't yet have a data type for that!!!
    def getOrientation(self):
        return wb.wb_supervisor_node_get_orientation(self._as_parameter_)

    wb.wb_supervisor_node_get_center_of_mass.restype = c_void_p
    @property
    def center_of_mass(self) -> Vec3f:
        """Returns the global position of this node's center of mass, an xyz vector."""
        return Vec3f.from_address(wb.wb_supervisor_node_get_center_of_mass(self._as_parameter_))
    def getCenterOfMass(self) -> Vec3f:
        """DEPRECATED. Node.getCenterOfMass() is deprecated.  Use node.center_of_mass instead."""
        return Vec3f.from_address(wb.wb_supervisor_node_get_center_of_mass(self._as_parameter_))

    wb.wb_supervisor_node_get_static_balance.restype = c_bool
    @property
    def is_statically_balanced(self) -> bool:
        """Returns True if this node is statically balanced, i.e., if, from the perspective of gravity, its center of
           mass is surrounded by its contact points."""
        return wb.wb_supervisor_node_get_static_balance(self._as_parameter_)
    def getStaticBalance(self) -> bool:
        """DEPRECATED. Node.getStaticBalance() is deprecated.  Use node.is_statically_balanced instead."""
        return wb.wb_supervisor_node_get_static_balance(self._as_parameter_)

    # --- Node.velocity ---
    # Velocity is tricky because the C-API packs linear and angular together, whereas we treat them separately

    wb.wb_supervisor_node_get_velocity.restype = Vec3f_p
    @timed_cached_property  # gets read just once each timestep, and cached til next timestep
    def velocity(self) -> List[Vec3f]:
        """node.velocity returns a list of two Vec3f's, so it often makes sense to have python unpack these, e.g.,
           using linear_vel, angular_vel = node.velocity, or you may access them separately.
           The first value is this node's linear velocity along global axes, also available as node.linear_velocity.
           The second value is its angular velocity around global axes, also available as node.angular_velocity.
           Setting node.velocity = new_value alters this node's velocity.  New_value may be
             (1) a single float (typically 0) which will be broadcast to all 6 components,
             (2) a pair of values which will be assigned to linear and angular velocities respectively,
                 where these values may each be a triple of floats or a single float to broadcast, or
             (6) a sextuple of floats, which will be assigned to the linear and angular components in succession."""
        return wb.wb_supervisor_node_get_velocity(self._as_parameter_)[:2]
    @velocity.set
    def velocity(self, value: Union[float, Iterable]):
        if isinstance(value, float):  # node.velocity = 0  # sets all 6 components to 0
            value = (value,value,value), (value,value,value)  # broadcast single float to all 2x3 slots
            # array = Array6f(value,value,value, value,value,value)
            # for c in range(3): self.velocity[0][c] = self.velocity[1][c] = value
        else:
            value = tuple(value)  # spiel iterator into tuple, so we can ask its length
            if len(value) == 6:  # node.velocity = lx,ly,lz, ax,ay,az
                value = value[:3], value[3:]  # split 6 given components into pair of triples (lx,ly,lz),(ax,ay,az)
            else:   # node.velocity = linear,angular
                value = [(v,v,v) if isinstance(v, float) else v for v in value]  # broadcast single floats into triples
                # array = Array6f( *value[0], *value[1] )
        # Now value should have form (lx,ly,lz),(ax,zy,az)
        # We need to remember these, in case both node.angular_velocity and .linear_velocity are set in same timestep
        if 'velocity' in self.__dict__:
            for i in range(2): self.__dict__['velocity'][i][:] = value[i]  # update existing cache with uptodate values
        else:
            self.__dict__['velocity'] = [Vec3f(v) for v in value]  # create new cache if one didn't exist (unlikely)
        wb.wb_supervisor_node_set_velocity(self._as_parameter_, (c_double*6)(*value[0], *value[1]))

    @property
    def linear_velocity(self) -> Vec3f:
        """Returns this node's linear velocity along global axes, an xyz vector.
           Setting node.linear_velocity = new_value alters the node's linear velocity, while leaving its angular
           velocity at its current value*. To alter both, use node.velocity = new_linear, new_angular.
           *Note: the "current value" of the angular velocity may be what you've altered it to, either by setting
           self.angular_velocity = new_value, or by reading self.angular_velocity and altering its components."""
        return self.velocity[0]
    @linear_velocity.setter
    def linear_velocity(self, new_linear_velocity):
        self.velocity = new_linear_velocity, self.velocity[1]

    @property
    def angular_velocity(self) -> Vec3f:
        """Returns this node's angular velocity, an xyz vector indicating its rate of turn around the 3 global axes.
           Setting node.angular_velocity = new_value alters the node's angular velocity, while leaving its linear
           velocity at its current value*. To alter both, use node.velocity = new_linear, new_angular.
           *Note: the "current value" of the linear velocity may be what you've altered it to, either by setting
           self.linear_velocity = new_value, or by reading self.linear_velocity and altering its components."""
        return self.velocity[1]
    @angular_velocity.setter
    def angular_velocity(self, new_angular_velocity):
        self.velocity = self.velocity[0], new_angular_velocity

    @use_docstring_as_deprecation_warning
    def getVelocity(self) -> List[float]:
        """DEPRECATED. Node.getVelocity() is deprecated. A near-equivalent is linear, angular = node.velocity,
           but node.linear_velocity and/or node.angular_velocity may be more useful."""
        linear, angular = self.velocity
        return [*linear, *angular] # new list that splats out linear components, then angular components, all together
    @use_docstring_as_deprecation_warning
    def setVelocity(self, new_velocity):
        """DEPRECATED. Node.setVelocity(new_velocity) is deprecated. Use node.velocity = new_linear, new_angular or
           node.velocity = new_combined_velocity"""
        self.velocity = new_velocity

    # --- Node administering force and torque ---

    def add_force(self, force: Iterable3f, offset: Iterable3f = None, relative: bool = False):
        """Administers the given force to this node, at a point offset from the node's center of gravity by the given
           offset, expressed in the node's local coordinates, or at its center of gravity if offset is not given.
           If relative is false (the default) the force will be expressed in global coordinates, if true it will be
           expressed in the node's local coordinates."""
        if offset:
            wb.wb_supervisor_node_add_force_with_offset(self._as_parameter_, Vec3f(force), Vec3f(offset), c_bool(relative))
        else:
            wb.wb_supervisor_node_add_force(self._as_parameter_, Vec3f(force), c_bool(relative))
    @use_docstring_as_deprecation_warning
    def addForce(self, force: Iterable3f, relative: bool):
        """DEPRECATED: Node.addForce(force, rel) is deprecated. An equivalent is node.add_force(force, relative=rel).
           The keyword `relative` is needed, since add_force expects its second arg to be an optional offset."""
        wb.wb_supervisor_node_add_force(self._as_parameter_, Vec3f(force), c_bool(relative))
    @use_docstring_as_deprecation_warning
    def addForceWithOffset(self, force: Iterable3f, offset: Iterable3f, relative: bool):
        """DEPRECATED: Node.addForceWithOffset is deprecated. Use node.add_force(force, offset, relative)."""
        wb.wb_supervisor_node_add_force_with_offset(self._as_parameter_, Vec3f(force), Vec3f(offset), c_bool(relative))

    def add_torque(self, torque: Iterable3f, relative: bool = False):
        """Administers the given torque to this node, around its center of gravity. If relative is false (the default)
           the torque will be expressed in global coordinates, otherwise the node's local coordinates."""
        wb.wb_supervisor_node_add_torque(self._as_parameter_, Vec3f(torque), c_bool(relative))
    @use_docstring_as_deprecation_warning
    def addTorque(self, torque: Iterable3f, relative: bool):
        """DEPRECATED: Webots is shifting to conventional Python naming. Change node.addTorque to node.add_torque."""
        wb.wb_supervisor_node_add_torque(self._as_parameter_, Vec3f(torque), c_bool(relative))

    # --- node joint positions ---

    @property
    def position1(self) -> float:
        """For joint nodes, joint.position1 gets/sets the joint's position on its first degree of freedom."""
        return self.fields.position.get_value()
    @position1.setter
    def position1(self, value):
        wb.wb_supervisor_node_set_joint_position(self._as_parameter_, c_double(value), 1)

    @property
    def position2(self) -> float:
        """For hinge2 and ball-joint nodes,
           joint.position2 gets/sets the joint's position on its second degree of freedom."""
        return self.fields.position2.get_value()
    @position2.setter
    def position2(self, value):
        wb.wb_supervisor_node_set_joint_position(self._as_parameter_, c_double(value), 2)

    @property
    def position3(self) -> float:
        """For balljoint nodes, joint.position3 gets/sets the joint's position on its third degree of freedom."""
        return self.fields.position3.get_value()
    @position3.setter
    def position3(self, value):
        wb.wb_supervisor_node_set_joint_position(self._as_parameter_, c_double(value), 3)

    @use_docstring_as_deprecation_warning
    def setJointPosition(self, new_position: float, index: int = 1):
        """DEPRECATED. Node.setJointPosition(new_pos, index) is deprecated.
           Use node.position1 = new_pos. Similarly for .position2 or .position3. These properties are also readable."""
        wb.wb_supervisor_node_set_joint_position(self._as_parameter_, c_double(new_position), index)

    # --- Node Contact Points ---

    class Contact(Sequence['Node.Contact.Point']):
        """A container interface for a node's Contact.Points. Accessed via Node.contact_points."""

        @documented_attribute
        def include_descendants(self) -> bool:
            """Reading or setting `node.contact_points.include_descendants` reads or alters whether contacts involving
               this node's descendants are counted among this node's own .contact_points. Typically True or False."""
            return True

        def __init__(self, node: 'Node'):
            self.node = node
            self._as_parameter_ = self.node._as_parameter_

        def __call__(self,  sampling:Union[int, bool, None] = None, include_descendants=True) -> 'Contact':
            """`n.contact_points(sampling=False, include_descendants=True)` returns the same container-like Contact
               object as n.contact_points does, and is a convenience method for initializing its .include_descendants
               and .sampling. These settings will remain until altered, so after they are set as you like, you may
               simply employ n.contact_points itself."""
            if self.include_descendants != include_descendants:
                self.include_descendants = include_descendants
                # remove cached .value, since it would change after altering whether to include d's
                if 'value' in self.__dict__: del self.__dict__['value']
            if sampling != self.sampling: self.sampling = sampling
            return self

        def __repr__(self):
            return f"{self.node}.contact_points[0:{len(self)}]"

        def __len__(self) -> int:
            return len(self.value)

        class Point(ctypes.Structure, VectorValue):
            """A Node.Contact.Point indicates a point of contact between this node and another.
               `point.value` returns the coordinates of the point of contact.  The point itself serves as a
                surrogate for this .value, so you can do standard vector arithmetic with points, or use helper
                functions like point.x, point.angle, and point.distance.
               `point.node` returns the Node that is being contacted.
               `point.node_id` returns the unique id of that node."""

            # Declare fields for c_types conversion from C-API
            _fields_ = [('value', Vec3f),
                        ('node_id', c_int)]

            # Re-declare attributes for Python linters
            value: Vec3f
            node_id: int

            def __repr__(self): return f"Contact.Point({self.x}, {self.y}, {self.z})"

            @property
            def node(self):
                """Returns the Node in which the contact is occuring (this node or one of its descendants)."""
                return Node(self.node_id)

            @use_docstring_as_deprecation_warning
            def get_point(self):
                """DEPRECATED. Node.ContactPoint.get_point() and Node.ContactPoint.point are deprecated. Use
                   pt.value instead, or for most purposes a ContactPoint serves as a surrogate for its own .value."""
                return self.value
            point = property(get_point)

            @use_docstring_as_deprecation_warning
            def get_node_id(self):
                """DEPRECATED. Node.ContactPoint.get_node_id() is deprecated. Use point.node_id instead.
                   Or use point.node if you want the node itself and not just its id."""
                return self.node_id
            # The old node_id property still works in the new API

        _count = c_int()                   # create a destination for get_contact_points to store its count into
        _count_ptr = ctypes.byref(_count)  # and a pointer to this destination for us to pass to the C API

        wb.wb_supervisor_node_get_contact_points.restype = POINTER(Point)
        @timed_cached_property
        def value(self) -> Sequence[Point]:
            """Returns a ctypes array containing this node's Contact.Points. This and its constituent Points share
               memory with the Webots simulation, so are very fast to access, but are valid only for the current timestep.
               This is available as .value for anyone willing to risk accidentally preserving an invalid reference
               to shared memory to gain a bit of speed. Most users should avoid `node.contact_points.value` and instead
               use `for p in node.contact_points` or `node.contact_points[i]` which return stable copies of points,
               and hence avoid any worries about accidentally preserving a reference to expired shared memory."""
            list_ptr = wb.wb_supervisor_node_get_contact_points(self.node._as_parameter_,
                                                                c_bool(self.include_descendants),
                                                                self._count_ptr)
            if not list_ptr: return []
            c_arraytype = self._count.value * Node.Contact.Point
            return ctypes.cast(list_ptr, POINTER(c_arraytype))[0]

        @overload  # contact_points[index] returns a copy of a Point
        def __getitem__(self, item: int) -> Point: pass
        @overload  # contact_points[slice] returns a list of copied Points
        def __getitem__(self, item: slice) -> List[Point]: pass
        def __getitem__(self, item):
            if isinstance(item, int):  # contact_points[index] returns a copy of that pixel
                return Node.Contact.Point.from_buffer_copy(self.value[item])
            else:                      # contact_points[slice] returns a list of copied pixels
                return [Node.Contact.Point.from_buffer_copy(p) for p in self.value[item]]

        wb.wb_robot_get_basic_time_step.restype = c_double
        @property
        def sampling(self):
            """`n.contact_points.sampling` (aka "tracking") is quite similar to `sensor.sampling` though enabling this
               beforehand is not a requirement for reading contact_points, but does serve to speed access to them.
               Unlike `sensor.sampling` this will not automatically be enabled, but, like `sensor.sampling`, you may
               enable tracking to occur periodically with `n.contact_points.sampling = period_ms` or can set it to
               `True` to use the basic timestep as the tracking period, or to `None` to disable automatic tracking
               (slightly speeding the simulation, if, and *only* if, you won't use `contact_points` much afterward)."""
            return self.__dict__.get('sampling', None)
        @sampling.setter
        def sampling(self, new_period:Union[int, bool, None]):
            if new_period is True: new_period = wb.wb_robot_get_basic_time_step()
            self.__dict__['sampling'] = new_period  # remember this value so that future read attempts will read it
            if new_period:
                wb.wb_supervisor_field_enable_sf_tracking(self._as_parameter_, new_period)
            else:
                wb.wb_supervisor_field_disable_sf_tracking(self._as_parameter_)
        tracking = sampling

        def disableContactPointsTracking(self, *args):
            return wb.wb_supervisor_node_disable_contact_points_trackingXXX(self._as_parameter_, *args)

    @cached_property
    def contact_points(self) -> Contact:
        """`n.contact_points` returns a sequence-like object whose contents are node n's Contact.Points.
           `n.contact_points(sampling=None, include_descendants=True)` also returns this same object, and is a
           convenience method for initializing its .sampling and .include_descendants, described below.
           `n.contact_points.include_descendants` reads or alters whether contacts involving node n's descendants
           will be counted among n's own .contact_points (default is True).
           `len(n.contact_points)` returns how many contact points this node currently has. (This does not actually
           actually copy any contact points from the simulation, so is quite fast.)
           `n.contact_points[i]` or `n.contact_points[i:j]` index or slice the contact points.
           `for point in n.contact_points` iterates through the contact points.
           Each point is a vector-like surrogate for its xyz coordinates, and point.node is the node being contacted.
           `n.contact_points.sampling` is quite similar to `sensor.sampling` though enabling this beforehand is not
           a requirement for reading contact_points, but does serve to speed access to them.  Unlike `sensor.sampling`
           this will not automatically be enabled, but, like `sensor.sampling`, you may enable tracking to occur
           periodically with `n.contact_points.sampling = period_ms` or can set it to `True` to use the basic timestep
           as the tracking period, or to `None` to disable automatic tracking (slightly speeding up the simulation,
           if, and *only* if, you won't use `contact_points` much afterwards)."""
        return Node.Contact(self)

    # --- Node.contact_points deprecated methods ---

    @use_docstring_as_deprecation_warning
    def enableContactPointsTracking(self, period=True, include_descendants=True):
        """DEPRECATED. Node.enableContactPointsTracking() is deprecated.
           Please use n.contact_points(period, include_descendants) instead."""
        self.contact_points(sampling=period, include_descendants=include_descendants)

    @use_docstring_as_deprecation_warning
    def disableContactPointsTracking(self, include_descendants=True):
        """DEPRECATED. Node.disableContactPointsTracking() is deprecated. Use n.contact_points.sampling = None."""
        self.contact_points(sampling=None, include_descendants=include_descendants)

    @use_docstring_as_deprecation_warning
    def getContactPoint(self, index) -> Contact.Point:
        """DEPRECATED. Node.getContactPoint(i) is deprecated.  Please use node.contact_points[i] instead."""
        return self.contact_points[index]

    @use_docstring_as_deprecation_warning
    def getContactPointNode(self, index) -> int:
        """DEPRECATED. Node.getContactPointNode(i) is deprecated.  Please use node.contact_points[i].node_id instead.
           Or use .node rather than .node_id if you want the node itself, rather than just its id."""
        return self.contact_points[index].node_id

    @use_docstring_as_deprecation_warning
    def getNumberOfContactPoints(self, include_descendants=False) -> int:
        """DEPRECATED. Node.getNumberOfContactPoints() is deprecated.  Please use len(node.contact_points) instead."""
        return len(self.contact_points(include_descendants=include_descendants))

    @use_docstring_as_deprecation_warning
    def getContactPoints(self, includeDescendants=False) -> Contact:
        """DEPRECATED. Node.getContactPoints() is deprecated.  Please use node.contact_points instead."""
        return self.contact_points

    @use_docstring_as_deprecation_warning
    def getContactPointFromList(self, points: Sequence[Contact.Point], index:int) -> Contact.Point:
        """DEPRECATED. Node.getContactPointFromList() is deprecated. You probably want node.contact_points[i]"""
        return points[index]

    # --- Node poses ---

    def enablePoseTracking(self, *args):
        return wb.wb_supervisor_node_enable_pose_tracking(self._as_parameter_, *args)

    def disablePoseTracking(self, *args):
        return wb.wb_supervisor_node_disable_pose_tracking(self._as_parameter_, *args)

    # TODO datatype???
    def getPose(self, *args):
        return wb.wb_supervisor_node_get_pose(self._as_parameter_, *args)

    # --- Node state saving and loading ---

    def save_state(self, state_name: str):
        """Saves the state of this node under the heading state_name. This can be reloaded with node.load_state()."""
        wb.wb_supervisor_node_save_state(self._as_parameter_, state_name.encode())
    @use_docstring_as_deprecation_warning
    def saveState(self, state_name: str):
        """DEPRECATED: Webots is shifting to conventional Python naming. Change node.saveState to node.save_state."""
        wb.wb_supervisor_node_save_state(self._as_parameter_, state_name.encode())

    def load_state(self, state_name: str):
        """Sets this node's state to match the one previously saved under heading state_name with node.save_state()."""
        wb.wb_supervisor_node_load_state(self._as_parameter_, state_name.encode())
    @use_docstring_as_deprecation_warning
    def loadState(self, state_name: str):
        """DEPRECATED: Webots is shifting to conventional Python naming. Change node.loadState to node.load_state."""
        wb.wb_supervisor_node_load_state(self._as_parameter_, state_name.encode())

    # --- Node exporting ---

    wb.wb_supervisor_node_export_string.restype = c_char_p
    def copy(self):
        """Returns new a planned Node that copies the characteristics of this node, including its descendants."""
        raise NotImplementedError
        # TODO***

    # TODO also have export_plan option?  Probably redundant with copy.

    def encode(self, field:'Field'=None) -> bytes:
        """Returns a VRML specification of this node as a Python byte-string. Analogous to str.encode()"""
        if self.is_real: return wb.wb_supervisor_node_export_string(self._as_parameter_)
        return self.export(field=field).encode()
    def export(self, indent=0, field:'Field'=None) -> 'VRML_String':
        """Returns a VRML string specification of this node, which can then be re-imported into the scene tree,
           e.g. by setting an SFNode field equal to it, or inserting it into an MF-Field.
           For most purposes, users needn't call this directly, and instead should use plan(node) to create an easily
           introspectible and modifiable version of that node, or simply import an existing node to a new location,
           e.g. with `node1.children.append(node2)`, to automatically export node2 and import a copy of it.
           For "real" nodes in the Webots scene-tree, the C-API's export command is used; for merely planned Nodes
           a VRML string will be constructed specifying the planned Node's DEF, type, and fields.
           The optional indent parameter indicates how much new lines should currently be indented (used in recursive
           calls to export subnodes with an appropriate amount of indentation).
           The optional field parameter should indicate what field this VRML_String would be imported into, used for
           planned meganodes that will unpack themselves when imported to certain fields."""
        if self.is_real:  # If this is a "real" node in the scene tree, we have webots export it
            s = wb.wb_supervisor_node_export_string(self._as_parameter_).decode()
            if indent > 0: s = textwrap.indent(s, ' '*indent).lstrip()
            return VRML_String(s, add_quotes = None)
        # Otherwise this is a merely planned Node, so we'll need to construct a VRML string ourselves
        # DictSets are a form of dict that allows set-arithmetic akin to Python sets, while retaining mapped values
        given_fields = DictSet({f.name: f.cached_value for f in self.known_fields}) # maps field names to values

        # When imported into a non-geometry field, geometry Nodes like Cones must assemble a whole "meganode" and
        # distribute all given fields (like baseColor or rotation) to the appropriate compatriot Nodes
        if field is not None and field.name != 'geometry' and self.type in nodetypes.GEOMETRY_TYPES:
            # usable_fields maps each NodeType like 'Solid' to the subDictSet of given_fields usable in that NodeType
            # These subDictSets will be disjoint, aside from potential overlap of 'textureTransform' or 'name'
            usable_fields = {cls_name: given_fields & cls_fields
                             for cls_name, cls_fields in nodetypes.GEOMETRY_COMPATRIOT_FIELDS.items() }
            # We tentatively construe a NodeType as "needed" iff it uses any given_field besides 'name' or
            #  'textureTransform', which are handled separately due to being usable in multiple NodeTypes
            needed_others = {cls_name for cls_name, fields in usable_fields.items()
                                      if len(fields) - ('name' in fields) - ('textureTransform' in fields)}

            # Tentatively determine whether we'll need a Transform or Solid (or None) as overarching container
            # We check if any Solid/Transform fields are needed, and if so, whether any are solely Solid fields
            container_type = (None if 'Solid' not in needed_others
                              else 'Solid' if any(f for f in usable_fields['Solid']
                                                    if f != 'name' and f not in usable_fields['Transform'])
                              else 'Transform')

            # If a Physics node is needed, it will need a Solid to contain it within its physics field
            if 'Physics' in needed_others:
                container_type = 'Solid'
                usable_fields['Solid']['physics'] = plan.Physics(**usable_fields['Physics'])

            # We always must create a Shape node to contain the geometry, even if no given_field strictly needs it
            shape_node = plan.Shape(**usable_fields['Shape'])

            # If an Appearance/PBRAppearance node is needed, it will also reside within shape_node
            appearance_type = next((a for a in ('PBRAppearance', 'Appearance') if a in needed_others), None)
            if appearance_type:
                shape_node.appearance = PlannedNode(appearance_type, **usable_fields[appearance_type])

            # The geometry node gets any leftover fields not used by others (and Meshes need to get a copy of name too)
            geometry_fields = given_fields.difference(*usable_fields.values())
            if self.name:
                if self.type == 'Mesh':  # Mesh is the only geometry with a 'name' field
                    geometry_fields['name'] = self.name
                else:  # Otherwise, the only other plausible interpretation of a given 'name' is for Solid
                    container_type = 'Solid'
            shape_node.geometry = PlannedNode(self.type, DEF=self.DEF, **geometry_fields)

            # Finally construct the larger Solid/Transform container if needed, and export the largest container we made
            if container_type:
                container_fields = usable_fields[container_type]
                container_fields.setdefault('children', []).insert(0, shape_node)
                container_node = PlannedNode(container_type, **container_fields)
                return container_node.export(indent=indent, field=field)
            else:
                return shape_node.export(indent=indent, field=field)

        # If a planned Solid specifies its Physics fields directly, we spin those off into a new planned Physics node
        if self.type == 'Solid':
            given_fields = DictSet({f.name: f.cached_value for f in self.known_fields}) # maps field names to values
            physics_fields = given_fields & nodetypes.GEOMETRY_COMPATRIOT_FIELDS['physics']
            if physics_fields:
                given_fields -= physics_fields
                given_fields['physics'] = plan.Physics(**physics_fields)
                # after this reshuffling we can continue to unparse this Solid as we would any other planned Node

        # If we got here, this is a non-geometry Planned Node, so we need to "unparse" it to a VRML string
        if self.USE: return VRML_String(f"USE {self.USE}", add_quotes=None)
        DEF = f"DEF {self.DEF} " if self.DEF else ""
        attributes = (f"{f.name} {VRML_String(f.value, indent + 2, field=f)}" for f in self.known_fields)
        # If there are few fields, and none is a children field, then keep on one line
        # TODO are there other cases where this is short enough to make a one-liner?
        if len(given_fields) <= 3 and 'children' not in given_fields:
            return VRML_String(f"{DEF}{self.type}{{{', '.join(attributes)}}}", add_quotes=None)
        # otherwise, we put this node on multiple indented lines
        sep = f"\n{(indent + 2) * ' '}"
        attribute_string = sep.join(attributes)
        return VRML_String(f"{DEF}{self.type}{{{sep}{attribute_string}}}\n{indent * ' '}", add_quotes=None)

    @use_docstring_as_deprecation_warning
    def exportString(self) -> str:
        """DEPRECATED: Node.exportString() is deprecated. Use node.export() instead."""
        return self.export()

    # --- Node resetting, restarting and removal ---

    def become_forgotten(self, forget_descendants_too=True, remove_from_catalog=True, forget_parent_field=True):
        """Removes incoming references to this Node from its ancestors, usually either in preparation for this Node to
           be removed from the scene tree, or because it is suspected that another supervisor may remove this node.
           Keeping stale references to deleted nodes risks sending later Node commands to a C memory pointer that has
           since come to have some other content, which could cause crashes or other bugs.
           Most soon-to-be-stale references will automatically be cleansed whenever this supervisor itself causes them
           to become stale, so unaccompanied supervisors typically needn't bother themselves with any of this.
           However, users may need to take special care to avoid using old references that *another* supervisor may have
           caused to become stale. The simplest approach in such cases is to call node.refresh_descendants(), which
           (relatively) quickly refreshes a branch of the scene tree by parsing an exported representation of it.
           (Alternatively, some mix of node.become_forgotten(), node.forget_descendants(), or field.forget_descendants()
           can forget suspicious parts of the Proxy Scene Tree to be slowly repopulated on demand, and/or
           node.catalog_descendants() may repopulate a branch more quickly.)
           In all cases, direct references to this node of the form ancestor.name will be removed, as will the parent
           node's cached lists children.
           If forget_descendants_too is true, the descendants of this node will be forgotten as well.
           If remove_from_catalog is true, this node will be removed from the catalog of all nodes.
           If forget_parent_field is true, the cached_value of this node's parent_field will be cleared. (When false,
           the caller itself should typically surgically alter the parent_field to remove its reference to this node.)
           If all of those are true, the net effect will be that this node and its descendants will be fully forgotten
           from the Proxy Scene Tree so would not be returned again by any Proxy Scene Tree functionality -- which would
           instead produce new uptodate copies on demand -- and, unless you've created other lasting references to them,
           these nodes will then be gathered by Python garbage collection.
           If a user has stored other references to this node, e.g. by setting `n = world.NODENAME`, this
           cannot cleanse those references, and reuse of them could cause problems. So, users should take great care in
           storing local references to nodes that may get removed! It often is advisable to always refer to such nodes
           from a stable ancestor like world, so the proxy scene tree can automatically refresh stale references."""

        if forget_descendants_too: self.forget_descendants()

        # Remove all cached direct references to this node of the form ancestor.name
        for ancestor, name in self._incoming_references:
            actual_value = ancestor.__dict__.pop(name, self)  # pops existing value if there is one, or self if not
            # if the ancestor had somehow come to refer to something other than self, put that value back (oops!)
            if actual_value is not self: ancestor.__dict__[name]=actual_value
        self._incoming_references.clear()  # Now that incoming references have been wiped, can forget they were incoming

        if remove_from_catalog:
            catalog.pop(self.id, None)
            for name in self.nomikers:
                entry = catalog[name]
                if entry:
                    entry[:] = (node for node in entry if node is not self)
                else:  # TODO For bug-catching, can remove eventually
                    WarnOnce(f"Somehow Node {self} was created without a catalog entry for its nomiker {name}")

        if self.parent_field is not None:
            self.parent.decache_children()
            if forget_parent_field: self.parent_field.__dict__.pop('cached_value', None)

    def forget_descendants(self):
        """Causes Python's Proxy Scene Tree to forget all descendants of this node, but does not alter the simulation's
           own scene tree in any way. The Proxy Scene Tree will slowly repopulate itself again on demand, or a whole
           branch may be repopulated relatively quickly using node.catalog_descendants(). Forgetting a part of the
           Proxy Scene Tree may be useful in cases where it is suspected that another supervisor has added or deleted
           nodes from the actual scene tree, to avoid the risk of using an out-of-date Proxy.
           TODO However, at the time of this writing, Webots has bugs that interfere with one supervisor's being able
            to keep track of changes made by another, so things may not work as you would expect.  It may help to
            use node.catalog_descendants() to repopulate the scene tree as that should bypass the bugs and reveal the
            correct current structure of a branch, though bugs may still prevent interaction with some nodes in it."""
        for f in self.known_fields:
            if f.can_contain_children: f.forget_descendants()
        self.__dict__['is_fully_cataloged'] = False

    def refresh_descendants(self):
        """Forgets all currently known descendants of this node (using node.forget_descendants), and then
           parses an export from the simulation to (relatively) quickly repopulate this branch of the Proxy Scene Tree,
           (using node.catalog_descendants). Doing this is usually pointless, except in cases where another supervisor
           has altered the structure of the scene tree, so the current Proxy Scene Tree is out of date.
           TODO Once we have a version of export that includes node id's, this can be made more efficient.
            For now, this crudely *removes* and *replaces* all exiting Nodes from this branch of the scene tree,
            so you should not try to re-use references to those nodes that you might have saved elsewhere.
            Instead it is advisable to always refer to such nodes from world or some other stable ancestor within the
            Proxy Scene tree so that the Proxy Scene Tree can automatically deliver the correct current descendant.
            (It automatically employs a cache to make repeated references to descendants quite quick anyway).
           TODO At the time of this writing, Webots has bugs that interfere with one supervisor's being able
            to keep track of changes made by another, so things may not work as you would expect. This should reveal the
            correct current structure of a branch, though bugs may still prevent interaction with some nodes in it."""
        self.forget_descendants()
        self.catalog_descendants()

    def remove(self):
        """Removes this node from the Webots scene tree. Also removes any references to this Node from ancestors in
           Python's proxy scene tree, since attempts to re-use stale references would not work as expected, and could
           corrupt whatever new contents end up occupying the old memory slot of this node."""
        self.become_forgotten(forget_parent_field=False)  # cleanse all incoming references to self, except parent_field
        field_value = self.parent_field.__dict__.get('cached_value', None)
        if field_value is self:
            self.parent_field.cached_value = None
        elif isinstance(field_value, list):
            field_value.remove(self)
        if self.is_real: wb.wb_supervisor_node_remove(self._as_parameter_)  # have webots remove from scene tree

    def resetPhysics(self):
        wb.wb_supervisor_node_reset_physics(self._as_parameter_)

    def restartController(self):
        wb.wb_supervisor_node_restart_controller(self._as_parameter_)

    # --- Node visibility ---

    def moveViewpoint(self):
        return wb.wb_supervisor_node_move_viewpoint(self._as_parameter_)

    def setVisibility(self, _from, visible):
        return wb.wb_supervisor_node_set_visibility(self._as_parameter_, _from, visible)

def USE(target: Union[str, Node] = None, parent_field: 'Field' = None) -> Node:
    """Returns a USE Node that represents a planned `USE DEFNAME` entry. When this planned USE Node is imported into
       the scene tree as the value for an SFNode field, that field will re-use the closest preceding node in the
       tree with DEFNAME. The target node may be given as a string DEF-name or as (planned) Node, whose own DEF-name
       will be used. Most USE nodes will be created either by (1) calling USE(target) to create a merely planned USE
       Node for import into the scene tree, (2) by parsing an exported scene tree USE node, which by default is done
       for all nodes when the world module is first imported, or (3) by fetching a shadow node from the scene tree via
       the C-API, and then discovering that it is actually a USE node, by accessing its .USE property.
       If n is an ordinary non-USE Node, then n.USE will be '', whereas if n is a USE Node, then n.USE will be
       the DEF-name of the used target, so the boolean of n.USE indicates whether n is a USE Node.
       If n is a "real" USE Node (not merely planned), then n will also serve as a "shadow node" with attributes and
       fields mimicking those of its used target, except lacking the target's DEF-name.
       TODO Decide how to do this: Descendants of this shadow node are "hidden", so by default they will not be
        returned in searches for nodes, except for searches initiated from this USE node itself.
       If n is a merely planned USE Node, created with a (real or planned) Node as its target, then the USE Node will
       be a shadow copy of that Node (though, when imported, the shadow's fields are irrelevant and all that matters is
       the DEF-name specified by n.USE, and beware that Webots will seek a target with that DEF-name in the scene tree,
       which may not match the one you'd used in planning).
       If n is a merely planned USE Node with a mere DEF-name as its target, then it will not serve as a "shadow node"
       as there is often no straightforward way to determine which Node with that DEF-name will end up being used."""
    if isinstance(target, Node):
        node = plan(Node)       # Make a shadow copy of the target node for use in planning
        node.USE = node.DEF     # When imported, this USE node will become 'USE DEFNAME'
        node.DEF = ''           # In Webots, shadow copies do not inherit the DEF-name from the source
        return node
    # Otherwise, all we were given was a DEF-name
    return PlannedNode(parent_field=parent_field, USE=target)



# TODO obsolete:
class OLD_USE(Node):
    """A USE Node represents a (real or planned) `USE DEFNAME` entry in the scene tree. When stored as a value in an
       SFNode field, this causes that field to re-use the closest preceding node in the tree with DEFNAME.
       Most USE nodes will be created either by plan.USE(target) as a merely planned Node for import into the
       scene tree or by parsing an exported scene tree node.  If u is a USE Node created from parsing an export,
       then u.USE returns a "shadow node" that is much like the target node being used, except lacking its DEF-name.
       After first access, but not before, this "shadow node" will be included in searches or traversals of the tree.
       Note: USE nodes in the scene tree are visible as such only when you have parsed an exported version of the
       scene tree (as is done automatically with default settings, or manually with node.refresh_descendants()
       or node.catalog_descendants() ).  When traversing the tree incrementally with C-API calls (e.g. if you turn
       the setting to automatically catalog the tree off, or if you manually `forget` a part of the tree) the
       "shadow node" is returned directly in the place where the USE node would have been, and "shadow nodes" will
       always be included in searches (since the Webots C-API simply returns the "shadow node" as the content of the
       parent field, and does not provide any reliably fast way to distinguish a "shadow node" from an ordinary Node).

       TODO: There are unfortunate asymmetries between how USE nodes work when the tree is pre-cataloged from an
        export versus how they work when traversing the tree with separate API calls.  The main options are:
        (0) Live with these asymmetries. Most users will probably just leave the default pre-catalog setting so will
            never encounter the other behavior.  And these differences rarely matter: if you refer to nodes mostly
            by DEF-name, it'd be rare to accidentally retrieve a DEF-name lurking inside a shadow node, and most
            traversals of the tree would skim past most nodes anyway, so likely would skim past shadows.
        (1) Change the pre-catalog approach to match the incremental approach, which would mean having SFNode fields
            with USE contents get their shadow-node as their child, perhaps with some sort of is_USE flag.
            Since the incremental approach traverses these, we'd need the pre-catalog appraoch to mark such parts of
            the tree as not-fully-cataloged, and manually traverse them too.
        (2) Change the piecewise approach to match the pre-catalog approach. This would require that the piecewise
            approach have some way to tell which nodes are shadow/use nodes.  At the moment, the only way I know
            to determine that is by generating an export of the node. Mercifully, most SFNode contents are fairly
            small, so this might not be terrible, though a few, especially endPoint fields, can have fairly hefty
            contents.  If we eventually get a more flexible version of export, it may be possible to do a very
            lightweight export (e.g. ignoring non child-containing fields, and no recursion) to keep this efficient.
        (3) There could be some sort of compromise.  E.g. could make it a setting to choose between (1) and (2).
        ...
        Relatedly, I'm not totally happy with USE nodes being distinct.  There's a lot to be said for some approach
        that makes USE just be an attribute.  For incrementally acquired Nodes it is easy to tell their NodeType, but
        fairly expensive to tell if they are a USE Node



       # TODO: Decide whether to somehow replace USE Nodes with fetched copies

       # TODO figure out why USE nodes have a .base_type; I would have expected this to cause an error
    """
    is_real = False          # USE nodes have no directly corresponding real node addressable in the scene tree

    def __new__(cls, target: Union[str, Node], parent_field: 'Field' = None):  # override Node.__new__'s Node-seeking behavior
        return object.__new__(cls)

    def __init__(self, target: Union[str, Node] = None, parent_field: 'Field' = None):
        # TODO double check what happens if Node.__new__finds this again
        if target is None: return  # do nothing when Node.__new__ refinds this node, and so triggers __init__ again
        if isinstance(target, Node): target = target.DEF
        self.__dict__['USE_target'] = target
        self.__dict__['type'] = 'USE'
        self.__dict__['DEF'] = ''
        self.__dict__['fields'] = FieldsBearer(self)
        if parent_field is not None:
            self.__dict__['parent_field'] = parent_field
            self.__dict__['parent'] = parent_field.node
        self.__dict__['_incoming_references'] = set()  # set of (a,n) pairs of ancestors with names for this node
        self.__dict__['is_fully_cataloged'] = True

    def export(self):
        return VRML_String(f"USE {self.USE_target}", add_quotes=False)

    @cached_property
    def nickname(self) -> str: return f"USE {self.USE_target}"



    # TODO old version, can remove soon:
    # @cached_property
    # def USE(self) -> 'Node':
    #     """If self is a USE node parsed from an export of a scene tree node, initially accessing self.USE creates a
    #        new self.fields.USE Field which behaves much like self.parent's SFNode field that contains self would have
    #        behaved if self had not been parsed into it.  In particular, self.fields.USE.value will be a Node
    #        that closely resembles the target node being used. This node will be cached as self.USE and returned.
    #        Prior to first access of self.USE, this node and its descendants would not have been included in searches
    #        and traversals. After first access, this node (but not its descendants) will be cataloged, and so could
    #        potentially be found by a search within an ancestor for a node with its name/NodeType (though this node
    #        will not inherit the DEF-name of the used target so won't match any search for DEF-name).
    #        After first access, this node, and its descendants, will also be included in traversals of (this part of)
    #        the Proxy Scene tree, e.g. using ancestor.all_descendants or world.all_descendants.
    #        TODO might want to hide these USE-shadow-nodes from traversals and searches?"""
    #     if not self.parent.is_real:
    #         raise AttributeError("Merely planned USE nodes are unable to detect what node will be used.")
    #         # TODO I might be able to reverse-engineer the search for prior nodes, if I had a reversed iterator
    #     field = Field(self, self.parent_field._as_parameter_, name="USE", field_type="SFNode")
    #     self.fields.USE = field
    #     return field.get_value()


# === FieldsBearer classes ===

wb.wb_supervisor_field_get_sf_node.restype = wbNodeRef
wb.wb_supervisor_field_get_mf_node.restype = wbNodeRef

wb.wb_supervisor_node_get_field.restype = wbFieldRef
wb.wb_supervisor_node_get_field_by_index.restype = wbFieldRef
wb.wb_supervisor_field_get_sf_string.restype = c_char_p
wb.wb_supervisor_node_get_proto_field.restype = wbFieldRef
wb.wb_supervisor_node_get_proto_field_by_index.restype = wbFieldRef
class FieldsBearer:
    """For each Node, node.fields will be a FieldsBearer enabling node.fields.fieldname to return the Field of this
       node with name fieldname. In the case where no such field is known already, this attempts to find a field with
       that name in the scene tree and to cache and return a Field object for it, or raises an AttributeError.
       This also enables `node.fields[index]`, `len(node.fields)` and `for field in node.fields`.
       Known fields are stored in the FieldsBearer's own __dict__, e.g. as node.fields.rotation for faster re-use."""
    # Define wb interface functions; overriden in ProtoFieldsBearer subclass
    _hides_children = False
    _cache_found_fields = True
    _wb_get_field = wb.wb_supervisor_node_get_field
    _wb_count_fields = wb.wb_supervisor_node_get_number_of_fields
    _wb_get_field_by_index = wb.wb_supervisor_node_get_field_by_index

    _fraction_of_fields_known = 0.0 # default value, becomes 0.5 when child-containing fields known, 1 when all are known

    def __init__(self, node: 'Node'):
        self._containing_node = node

    def __repr__(self):
        return f"{self._containing_node}.fields"

    @cached_property
    def _cached_length(self):
        return self._wb_count_fields(self._containing_node._as_parameter_)
    def __len__(self) -> int:
        """`len(node.fields)` returns the number of fields this node has in the Webots scene tree.
           These fields may be indexed with `node.fields[0]` and iterated with `for field in node.fields`."""
        if not self._containing_node.is_real:
            return sum(1 for f in self._that_are_known)
        return self._cached_length

    @cached_property
    def _as_list(self):
        return [None]*self._cached_length

    @property
    def _that_are_known(self) -> Iterable['Field']:
        """Yields the known fields in this FieldsBearer, TODO currently in their order of creation.
           This name has leading _ to avoid collisions with field names.
           It is phrased so that node.fields._that_are_known will be clearly understandable."""
        return (field for key, field in self.__dict__.items() if isinstance(key, str) and isinstance(field, Field))

    @property
    def _that_may_contain_children(self) -> Iterable['Field']:
        """Yields all of this FieldBearer's fields that may contain children.
           In the (typical) case where all relevant fields are already known, fields will be quickly skimmed from
           these known fields, TODO currently in the order that they are created, but hopefully that may someday change.
           Otherwise fields will be slowly fetched through the C-API as needed by their index-ordering in Webots.
           This name has leading _ to avoid collisions with field names.
           It is phrased so that node.fields._that_may_contain_children will be clearly understandable."""
        fields_to_search = self._that_are_known if self._fraction_of_fields_known > 0 else self
        return (f for f in fields_to_search if f.can_contain_children)

    def __iter__(self):
        if not self._containing_node.is_real:
            yield from self._that_are_known
        else: # if this is a real node, and we want to include unknown fields
            if self._fraction_of_fields_known == 1:
                yield from self._as_list
            else:
                for i in range(self._cached_length): yield self[i]
                self._fraction_of_fields_known = 1

    # TODO consider mangling names of methods since we don't want to collide with proto field names
    def get(self, name_or_index: Union[str,int,slice], default: Any = Exception) -> 'Field':  # or List[Field] for slices
        """node.fields.name or node.fields[name_or_index] or node.fields.get(name_or_index) each return the field of
           this node with the given name/index.  If found, a Field object is created, stored in this FieldsBearer's
           own __dict__ as node.fields.fieldname (and also in the containing node's .__dict__ if this is an MF field).
           If no match is found, default is returned if given, the main advantage of calling this via node.fields.get(),
           or an appropriate error is raised: AttributeError for a missing name, or IndexError for a missing index.
           For merely planned Nodes, fields cannot currently be gotten by index."""
        if isinstance(name_or_index, str):
            # The following check would be redundant in cases where this is called as __getattr__,
            # but not in cases where it is called as get() or __getitem__
            # TODO so if I ever split up these functionalities, could remove this in the __getattr__ case
            existing_field = self.__dict__.get(name_or_index)  # check if we already know this field by name
            if existing_field is not None: return existing_field
            field_ref = (self._containing_node.is_real and
                         self._wb_get_field(self._containing_node._as_parameter_, name_or_index.encode()))
            if not field_ref:
                # TODO If we someday have list of field default values by NodeType could auto-construct planned fields
                if default is not Exception: return default
                raise AttributeError(f"Unable to find '{name_or_index}' field of {self._containing_node}.")
            name, index = name_or_index, None

        elif isinstance(name_or_index, int):
            if not self._containing_node.is_real:
                if default is not Exception: return default
                raise NotImplementedError(f"Accessing fields by index is not allowed for merely planned Nodes.")
                # TODO If we someday have access to a list of all fields by NodeType this could be computed
            existing_field = self._as_list[name_or_index]  # list is right size, so raises IndexError if out of bounds
            if existing_field is not None: return existing_field
            index = name_or_index
            if index < 0: index = index + self._wb_count_fields(self._containing_node._as_parameter_)
            # TODO I think the following check is redundant with the error that would have been raised 4 lines up
            # if index>=self._wb_count_fields( self._containing_node._as_parameter_ ):
            #     raise IndexError(f"Node {self._containing_node} has only {len(self)} fields.")
            field_ref = self._wb_get_field_by_index(self._containing_node._as_parameter_, index)
            if not field_ref:
                if default is not Exception: return default
                raise IndexError(f"Unable to find a field for {self._containing_node} with index {index}.")
            name = wb.wb_supervisor_field_get_name(field_ref).decode() or None
            if name is not None:
                existing_field = self.__dict__.get(name, None)  # check if we already knew this field by name only
                if existing_field is not None:
                    existing_field.index = index
                    self._as_list[index] = existing_field
                    return existing_field
        elif isinstance(name_or_index, slice):  # handles expressions like node.fields[0:2]
            return [self[i] for i in range(*name_or_index.indices(len(self)))]
        else:
            raise IndexError(f"In node.fields.get(i) and node.fields[i], the index i must be a string, int, or slice.")
        # If we reach here, field_ref, and name and/or index will have been set above; we need to locate/construct it
        # TODO: consider moving some of the following to the Field constructor; then may make sense to split the
        #  above into separate __getattr__, __getitem__ and/or get methods
        field = self.__dict__.get(field_ref.value)  # see if a field for this wbFieldRef has already been created
        if field is None:  # if not, create a new one
            field = Field(self._containing_node, field_ref, name, index=index)
            self.__dict__[field_ref.value] = field  # now we can look up this field by its wbFieldRef
        if index is not None: self._as_list[index] = field
        if name is not None:
            self.__dict__[name] = field  # now node.fields.name will directly return this field
            # field.seems_to_be_MF can miss some singleton-containing fields, but not caching them isn't terrible
            if self._cache_found_fields and field.seems_to_be_MF:
                field.node.__dict__[name] = field # now node.name will return this MF_field
        if self._hides_children: field.hides_children = True
        return field
    __getitem__ = __getattr__ = get  # node.fields.name, node.fields['name'] and node.get('name') are processed alike

    def set(self, fieldname:str, value:Any=NotImplemented, cached_value:Any=NotImplemented) -> 'Field':
        """Attempts to set this FieldsBearer's Field of the given name to the value (if given).
           If no such Field exists, it will be created (for merely planned nodes) or fetched (for real nodes).
           The relevant field will be returned, or None is returned if no such field could be found or created.
           For most purposes it is better to use node.fieldname = value, or node.fields.fieldname.value = value."""
        logprint(f"Attempting to set {self._containing_node}.{fieldname} to a new_value.")
        field = self.get(fieldname, None)
        if field is None:
            if self._containing_node.is_real: return None
            self.__dict__[fieldname] = field = Field(self._containing_node, field_ref=None, name=fieldname)
        if value is not NotImplemented: field.set_value(value)
        if cached_value is not NotImplemented: field.set_cached_value(cached_value)
        return field

    def _create(self, fieldname:str, cached_value:Any=NotImplemented) -> 'Field':
        """Attempts to find or create a field of the given name, and to update its cached_value, if given.
           If a Field of this name is already known, it will be used.  Otherwise a new one will be created.
           In either case, no C-API calls will be made yet (though if self._containing_node.is_real then further
           use of this field may trigger such API-calls). The relevant field will be returned."""
        # print(f"Attempting to create Field {self._containing_node}.{fieldname}.")
        field = self.__dict__.get(fieldname, None)
        if field is None:
            self.__dict__[fieldname] = field = Field(self._containing_node, field_ref=None, name=fieldname)
        if cached_value is not NotImplemented: field.set_cached_value(cached_value)
        return field


    def __contains__(self, item:Union[str, 'Field']):
        """A string is "in" node.fields if that string is a fieldname for one of that node's fields.
           A Field is "in" node.fields if it represents one of that node's fields."""
        if isinstance(item, str): return self.get(item, False)
        if isinstance(item, Field): return item in self.__dict__.values()
        return False

class ProtoFieldsBearer(FieldsBearer):
    """For each Node, node.proto_fields will be a ProtoFieldsBearer enabling node.proto_fields.fieldname to return
       the non-exposed Field of this proto node with name fieldname, or raises an AttributeError.
       This also enables `node.proto_fields[index]`, `len(node.proto_fields)` and `for field in node.proto_fields`."""
    def __repr__(self): return f"{self._containing_node}.proto_fields"
    # Most functionality is inherited from superclass; we mostly just need to change which wb_functions are called
    _hides_children = True
    _cache_found_fields = False  # procedural proto_fields are unstable, so should not be cached
    _wb_get_field = wb.wb_supervisor_node_get_proto_field
    _wb_count_fields = wb.wb_supervisor_node_get_proto_number_of_fields
    _wb_get_field_by_index = wb.wb_supervisor_node_get_proto_field_by_index

# ------------Field------------

class Field:
    """A Field object is a Python representations of a particular field of a particular node in the Webots scene tree.
       Direct usage of SF Fields is discouraged, in favor of commands like `current_value = world.DEFNAME.fieldname`
       or `world.DEFNAME.fieldname = new_value`, which will implicitly employ a Field to mediate interaction
       with the corresponding field in the scene tree.  For MF fields, Field objects provide a container-like
       interface to interact with the contents of the Webots field.
       Field Objects have the traditional Webots methods to access and alter different datatypes of field values, like
       field.getSFFloat or field.setMFVec3f.  However it is generally preferable to use field.value which automatically
       selects the relevant getter/setter.  Even more preferable is to access fields using world.DEFNAME.fieldname."""

    # --- Field CONSTANTS, common to SF and MF ---

    # TODO may want to present type constants in Python as strings, following DistanceSensor model
    #  Since Webots already provides getTypeName, could just use it.
    NO_FIELD = _WB.WB_NO_FIELD
    SF_BOOL = _WB.WB_SF_BOOL
    SF_INT32 = _WB.WB_SF_INT32
    SF_FLOAT = _WB.WB_SF_FLOAT
    SF_VEC2F = _WB.WB_SF_VEC2F
    SF_VEC3F = _WB.WB_SF_VEC3F
    SF_ROTATION = _WB.WB_SF_ROTATION
    SF_COLOR = _WB.WB_SF_COLOR
    SF_STRING = _WB.WB_SF_STRING
    SF_NODE = _WB.WB_SF_NODE
    MF = _WB.WB_MF
    MF_BOOL = _WB.WB_MF_BOOL
    MF_INT32 = _WB.WB_MF_INT32
    MF_FLOAT = _WB.WB_MF_FLOAT
    MF_VEC2F = _WB.WB_MF_VEC2F
    MF_VEC3F = _WB.WB_MF_VEC3F
    MF_ROTATION = _WB.WB_MF_ROTATION
    MF_COLOR = _WB.WB_MF_COLOR
    MF_STRING = _WB.WB_MF_STRING
    MF_NODE = _WB.WB_MF_NODE

    # --- Field initialization ---
    hides_children = False  # default value, will be overridden for hidden proto fields

    def __init__(self, node: Node, field_ref: wbFieldRef = None, name: str = None, field_type: str = None, index: int = None):
        self.node = node
        if field_ref is not None: self._as_parameter_ = field_ref
        if field_type is not None:
            self.type = field_type
            is_SF = self.is_SF = 'SF' in field_type
            self.is_MF = not is_SF
        if name is not None: self.name = name
        if index is not None: self.index = index

    def __repr__(self): return f"{self.node}.{self.name}"

    @cached_property
    def _as_parameter_(self) -> wbFieldRef:
        """Returns the wbFieldRef referring to this field within the C-API. At the time of writing, Webots field ops get
           slower the more fieldrefs are fetched through the C-API, so we refrain from fetching this until needed."""
        # TODO *** this would need to use a different function for hidden proto fields!
        #  Somehow I need a way to distinguish these, e.g. by having each field remember its _FieldsBearer and using its
        #  _wb_get_field function, or by setting a hidden/exposed flag on each field
        return wb.wb_supervisor_node_get_field(self.node._as_parameter_, self.name.encode())

    # TODO The following was an aborted attempt to make iterating SF fields yield their single values
    #  TODO decide whether I want to pursue that.  This could obviate most need for field.children,
    #   but with some wrinkles involving handling fields with null values, which some SFNode fields will have.
    # @cached_property
    # def __iter__(self) -> Callable[['Field'],Iterable]:
    #     if self.uses_cached_value: return self.get_cached_value
    #     if self.is_MF: return self.iter_MF_values
    #     return self.iter_SF_value
    # def iter_cached_value(self) -> Iterator:
    #     """For planned/node-containing fields that rely on their cached_value to be current,
    #        if that cached_value is a list, iterating the field iterates through the list, and
    #        otherwise iterating the field yields just its value.
    #        For example `for child in endpoint_sf_field` will iterate over just the single child."""
    #     if isinstance(self.cached_value, list): return iter(self.cached_value)
    #     return iter( (self.cached_value,) )
    # def iter_MF_values(self) -> Iterable:
    #     for i in range(len(self)): yield self.get_item(i)
    # def iter_SF_value(self) -> Iterable:
    #     yield self.get_value()

    # --- Field.value handling ---

    @cached_property
    def cached_value(self):
        """Returns a known value this field once had, or its current value if no past value is known.
           For fields of parsed and planned nodes, field.cached_value will have been directly set to the initial
           value from the parse/plan, so this getter would not be called. For other fields, this getter will retrieve
           the current value from the C-API upon first demand and cache it. Reading a field does not update its
           cached_value, so most fields' cached_values are initial values that may easily become out of date.
           However, the cached_values of Node-containing fields (SFNode/MFNode) are automatically maintained to be
           current at all times (aside from changes in scene tree structure brought about by other controllers)."""
        # for nodes that don't use their cached_value as their value, we can look up their real value from C-API
        if not self.uses_cached_value: return self.get_value()
        # for nodes that use cached_value, can't just call get_value, as that would circularly call this back!
        if self.type == 'SFNode': return self.fetchSFNode()
        if self.type == 'MFNode':
            return [self.fetchMFNode(i) for i in range(wb.wb_supervisor_field_get_count(self._as_parameter_))]
        # TODO if at some point we set up planned Nodes to act as though their unspecified fields have their default
        #  values (or if we're saving C_API calls on a field we know was recently parsed), we would need to look up
        #  that default value here
        raise AttributeError(f"Field {self} has no cached_value, and no way is known to retrieve its current value.")

    @cached_property
    def uses_cached_value(self) -> bool:
        """This returns False for a typical field that serves as a window that directly sees through to a field in
           the webots scene tree.  This returns True for atypical fields that maintain a locally cached version of
           their value, perhaps also updating a corresponding scene tree field in parallel (for SFNode/MFNode fields)
           or perhaps not (for fields in non-real/merely-planned nodes)."""
        return not self.node.is_real or self.can_contain_children

    @cached_property
    def get_value(self) -> Callable:
        """field.get_value() returns the current value of the associated field in the Webots scene tree.
           This is used by the field.value property, and, for single (SF) fields, is
           used by Node.__getattr__ when it returns a value for `node.sf_field_name`.
           For single-fields (SF) the returned value will be a single item of that field's datatype.
           For multi-fields (MF) the returned value will be a list of the contained items,
           and `node.this_mf_field` will return the Field object itself.
           This cached_property returns an appropriate getter function, tailored to this field's datatype, and
           bound to this field instance. This getter will be cached so that future references to
           this_field.get_value() will call it directly."""
        # For mere planned nodes and nodes containing further parts of the proxy tree, value is kept as cached_value
        # so that the proxy scene tree can be pre-parsed and traversed without slowly fetching info from the C-API
        if self.uses_cached_value: return self.get_cached_value
        # Otherwise this must be a real field that isn't known to currently contain a node
        if self.is_MF:  # accessing self.is_MF typically forces fetching of self's ref and type from the Webots C-API
            return self.get_MF_value
        # For SF fields, we choose a value getter tailored to this datatype, already bound to this Field
        return getattr(self, f"get{self.type}")  # getting from self will get version bound to self
    def get_cached_value(self):
        """The current value of fields containing nodes and of merely planned fields is field.cached_value."""
        return self.cached_value
    def get_MF_value(self) -> list:
        """The current value of an MF Field is a list of its contents"""
        return list(self)

    @cached_property
    def set_value(self) -> Callable:
        """Alters the current value of the associated field in the Webots scene tree to new_value.
           This is used by the self.value property and by Node.__setattr__ to process `node.field_name = new_value`.
           For multi-fields (MF) this replaces the contents of this MF field with those listed in new_value.
           A useful special case is mf_field.value = [], which deletes all elements, so it matches
           the empty list [], equivalent to mf_field.clear().
           For MF fields this function is equivalent to (and is implemented by) mf_field[:]=it, which replaces the
           "full slice" containing all the field's elements with values iterated from it."""
        # For mere planned nodes cached_value is stored as a field attribute
        if not self.node.is_real:
            return self.set_cached_value
        if self.is_MF:  # accessing self.is_MF may force fetching of self's ref and type from the Webots C-API
            return self.set_MF_value
        # For SF fields, we choose a value setter tailored to this datatype, already bound to this Field
        return getattr(self, f"set{self.type}")
    def set_MF_value(self, new_value: Iterable):
        """Version of Field.set_value specifically for MF fields."""
        self[:] = new_value
    def set_cached_value(self, new_value):
        """Adjusts the cached_value of this field, which for merely planned fields is the only value it has.
           If the new value is/contains Nodes that already have another parent, those Nodes are copied.
           This field will claim parentage of its contained nodes."""
        # When this field and/or the new_value involve Nodes, we'll pass this on to a specialist that can
        # decache the old occupant(s) and appropriately copy+claim the new one(s)
        new_value_is_Node = isinstance(new_value, Node)
        new_value_is_list = isinstance(new_value, list)
        new_value_is_list_of_Nodes = (new_value_is_list and len(new_value) > 0 and isinstance(new_value[0], Node))
        if self.can_contain_children or new_value_is_Node or new_value_is_list_of_Nodes:
            self_seems_to_be_MF = self.seems_to_be_MF  # remember this as forgetting descendants may obscure it
            self.forget_descendants()  # forget any current occupants of this field, and decache parent's child-lists
            if new_value_is_list or self_seems_to_be_MF:  #  if this field seems to be MFNode
                if not new_value_is_list: new_value = [new_value]
                new_value = [self.claim_a_version_of(n) for n in new_value]
            else:  #  if this field seems to be an SFNode field, need to forget current occupant and claim new child
                new_value = self.claim_a_version_of(new_value)
        if isinstance(new_value, tuple):  # convert tuples of numbers to Vectors for convenience
            if not any(1 for component in new_value if not isinstance(component, (int, float))):
                new_value = Vector(new_value)
        self.cached_value = new_value

    @property
    def value(self):
        """Reading `field.value` returns the current value of the associated field in the Webots scene tree.
           Setting `field.value = new_value` alters that value.
           These are also implicitly employed by Node.__getattr__ for SF fields to process `node.sf_field_name`
           and by Node.__setattr__ (for both SF and MF fields) to process `node.field_name = new_value`.
           For MF fields the new value should be a list-like iterable of values to replace the field's current
           values, and node.mf_field_name returns the Field object itself."""
        return self.get_value()
    @value.setter
    def value(self, new_value):
        self.set_value(new_value)

    # --- generic Field methods, common to SF and MF ---

    @cached_property  # Defined for linting purposes, but superceded by instances' own .is_SF set in __init__
    def is_SF(self) -> bool:
        """Returns true if this is any sort of SF field, i.e. a field containing a single value."""
        return 'SF' in self.type

    @cached_property  # Defined for linting purposes, but superceded by instances' own .is_MF set in __init__
    def is_MF(self) -> bool:
        """Returns true if this is any sort of MF field, i.e. a list-like field containing 0 or more values."""
        return 'MF' in self.type

    @descriptor  # low-priority descriptor would defer to field.seems_to_be_MF if defined, but doesn't auto-cache that
    def seems_to_be_MF(self) -> bool:
        """Returns True if it seems like this field is MF, either because it is known to be MF, or because its
           cached_value is a non-vector list. However, it can be difficult to tell without C-API calls (that are slow,
           and at the time of this writing cause all future field-ops to become slower) whether a planned/parsed
           field is MF, since these fields' types aren't explicitly specified, and VRML allows that MF fields
           containing singleton items may be specified without brackets. So this may miss some singleton-containing
           MF fields. This provides a quick and often-correct guess for use in cases where that's good enough,
           and caches this answer when it is known to stably be correct."""
        # TODO If we ever have access to FieldTypes by NodeType or export, that'd provide another route to stable answer
        # If our wbFieldRef or type is already known, computing the correct answer is quite cheap
        if '_as_parameter_' in self.__dict__ or 'type' in self.__dict__:
            stable_answer = 'MF' in self.type
        else:  # Otherwise, we inspect the cached_value: non-lists and vectors are ambiguous; non-vector lists are MF
            cached_value = self.__dict__.get('cached_value')
            if not isinstance(cached_value, list) or isinstance(cached_value, Vector): return False  # without caching
            stable_answer = True
        # Now that we know the correct answer, cache it to speed future calls to seems_to_be_MF, is_MF, and is_SF
        self.__dict__['is_MF'] = self.__dict__['seems_to_be_MF'] = stable_answer
        self.__dict__['is_SF'] = not stable_answer
        return stable_answer

    wb.wb_supervisor_field_get_name.restype = c_char_p
    @cached_property  # most fields have this cached upon creation; this would trigger for fields gotten by index
    def name(self) -> str:
        """Returns the name of this Field in the containing scene tree node, e.g. 'rotation'."""
        return wb.wb_supervisor_field_get_name(self._as_parameter_).decode()
    @use_docstring_as_deprecation_warning
    def getName(self) -> str:
        """DEPRECATED: Field.getName() is deprecated. Please use field.name instead."""
        return wb.wb_supervisor_field_get_name(self._as_parameter_).decode()

    @property
    def full_address(self) -> str:
        """Returns something like "world.children[12].children[6].fieldname" beginning with the full_address of the
           parent node, and concluding with this field's own name"""
        return f"{self.node.full_address}.{self.name}"

    wb.wb_supervisor_field_get_type_name.restype = c_char_p
    @cached_property
    def type(self) -> str:
        """Returns the type of this field, as a string, e.g. 'SFVec3f'."""
        t = wb.wb_supervisor_field_get_type_name(self._as_parameter_).decode()
        old_assessment = self.__dict__.get('can_contain_children', 0)
        new_assessment = self.__dict__['can_contain_children'] = 'Node' in t
        if new_assessment > old_assessment:        # if this is a newly discovered potential container for children
            self.node.decache_children()           # then decache any old list of children we'd tried to make
        return t
    @use_docstring_as_deprecation_warning
    def getTypeName(self):
        """DEPRECATED: Field.getTypeName() is deprecated. Please use field.type instead."""
        return wb.wb_supervisor_field_get_type_name(self._as_parameter_)
    def getType(self):
        """Returns an integer constant indicating the type of this field.
           Consider using field.type to get a more informative string instead."""
        return wb.wb_supervisor_field_get_type(self._as_parameter_)

    @descriptor  # takes lower priority than its cached value, which may be cached by this or by Field.type
    def can_contain_children(self) -> bool:
        """Returns True if this field known to be of an appropriate type to contain nodes, and False otherwise.
           This response will be cached when known to be stably correct (e.g., due to the field already
           containing a node, having a name like 'children' that is known to contain nodes, or having a cached_value
           that *isn't* a node), whereas for empty fields False is returned for now, but is not cached.
           Field.type will directly cache the correct value for once that type is fetched through the C API."""
        # TODO nodetype could also be used to infer this, if I can somehow get a list of fields in nodetypes
        x = self.__dict__.get('cached_value')
        if isinstance(x, Node):  # could be SFNode field, or MFNode with just one entry
            self.__dict__['can_contain_children'] = True
            return True
        # Fields named 'children' and fields containing a list of at least one Node *can* contain children
        name = self.__dict__.get('name')
        if name == 'children' or isinstance(x, list) and len(x) > 0 and isinstance(x[0], Node):
            self.type = 'MFNode'
            self.__dict__['can_contain_children'] = True
            return True
        if x is not None and x != []:  # A field currently filled with non-Node(s) can never contain children
            self.__dict__['can_contain_children'] = False
        return False  # For both empty and filled non-node fields our (perhaps tentative) answer is no

    _ImportableAsNode = Union[str, Node, _pathlib_Path]  # type hint for node-importing arguments

    def claim_a_version_of(self, child:_ImportableAsNode) -> _ImportableAsNode:
        """If child is a merely planned Node with no parent it itself is "claimed" by this field (by setting its
           parent_field to be this field) and returned. Otherwise, a new copy of it is claimed and returned.
           Other representations of importable nodes (VRML strings, filenames, or pathlib.Paths) are returned as is.
           Used when assigning an existing Node as the value of a merely-planned SF/MFNode field, to allow re-use
           of merely planned Nodes without their parentage becoming confused."""
        if not isinstance(child, Node): return child
        # TODO think about what all should trigger copying a given node before claiming it
        if child.__dict__.get('parent') is not None:
            child = plan(child)  # TODO it's possible that it would be more efficient to use Python's copy.copy()
        child.__dict__['parent_field'] = self
        child.__dict__['parent'] = self.node
        return child

    # TODO probably not needed, since claiming is now folded into claim_a_version_of
    def claim_children(self):
        """This field sets itself to be the parent_field of each of its children.
           Automatically called when merely-planned SF/MFNode fields are initialized or gain nodes."""
        for child in self.children:  # make all children recognize self.node/self as their parent/parent_field
            child.__dict__.setdefault('parent', self.node)
            child.__dict__.setdefault('parent_field', self)

    # TODO some efficiency might be gained by caching this, and setting up appropriate triggers to decache
    @property
    def children(self) -> Sequence['Node']:
        """Returns a sequence containing the node or nodes immediately contained within this field, constructing new
           Nodes when necessary. (This works for all node-containing fields, including ones not named 'children'.)"""
        if not self.can_contain_children: return ()
        x = self.cached_value                # automatically fetches current value if this was not pre_cached
        if x is None: return ()              # field with None value returns empty sequence of children
        if isinstance(x, Node): return (x,)  # single node gets wrapped in tuple
        return x                             # otherwise should already be MF list

    @property
    def known_children(self) -> Sequence['Node']:
        """Returns a sequence containing the known node or nodes immediately contained within this field, but
           will not construct any new Nodes."""
        if not self.can_contain_children: return ()
        x = self.__dict__.get('cached_value', None)  # get existing cached_value, or None otherwise
        if x is None: return ()                      # field with no/None value returns empty sequence of children
        if isinstance(x, Node): return (x,)          # single node gets wrapped in tuple
        return x                                     # otherwise should already be MF list

    @property
    def count(self) -> int:
        """For MF fields, both len(mf_field) and mf_field.count return the number of items in the associated field in
           the Webots scene tree. For SF fields, sf_field.count returns -1 and len(sf_field) raises a ValueError."""
        return wb.wb_supervisor_field_get_count(self._as_parameter_)
    def __len__(self) -> int:
        """For MF fields, both len(mf_field) and mf_field.count return the number of items in the associated field in
           the Webots scene tree. For SF fields, sf_field.count returns -1 and len(sf_field) raises a TypeError."""
        if self.uses_cached_value: return len(self.cached_value)
        if self.is_SF:
            print(f"***For some reason something inquired about len({self})")
            raise TypeError(f"{self.name} field is an SF field, so has no length.")
        return wb.wb_supervisor_field_get_count(self._as_parameter_)
    @use_docstring_as_deprecation_warning
    def getCount(self) -> int:
        """DEPRECATED: Field.getCount() is deprecated. Please use field.count or len(mf_field) instead."""
        return wb.wb_supervisor_field_get_count(self._as_parameter_)

    def __bool__(self):
        """All existing fields count as True."""
        return True

    def forget_descendants(self, forget_own_cached_value=True):
        """Causes Python's Proxy Scene Tree to forget all descendants of this field, but does not alter the simulation's
           own scene tree in any way. If forget_own_cached_value is false, this does not remove this field's own
           cached_value, useful when the caller has any final need to access that value.
           The Proxy Scene Tree will slowly repopulate itself again on demand, or a whole
           branch may be repopulated relatively quickly using node.catalog_descendants(). Forgetting a part of the
           Proxy Scene Tree may be useful in cases where it is suspected that another supervisor has added or deleted
           nodes from the actual scene tree, to avoid the risk of using an out-of-date Proxy.
           TODO However, at the time of this writing, Webots has bugs that interfere with one supervisor's being able
            to keep track of changes made by another, so things may not work as you would expect.  It may help to
            use node.catalog_descendants() to repopulate the scene tree as that should bypass the bugs and reveal the
            correct current structure of a branch, though bugs may still prevent interaction with some nodes in it."""
        for c in self.known_children:
            c.become_forgotten(forget_parent_field=False)  # can't decache parent_field (i.e. self) while iterating
        self.node.decache_children()  # decache containing node's list(s) of children as this may need to change
        if forget_own_cached_value:
            self.__dict__.pop('cached_value', None)  # *now* we can forget self's cached_value

    def remove(self, index: int = None):
        """For SF fields, this removes the current value of the associated Webots scene tree field.
           For MF fields, this removes the element at the given index in the associated Webots scene tree field.
           NOTE: Unlike Python's list.remove, this specifies the item to removed by index, not by value!
           If the removed element is an existing Node, its ancestors' references to it will be
           cleansed to avoid bugs that could arise when re-using stale node references.
           For merely planned Nodes that have no analog in the scene tree, the item is removed from the plan rather
           than from the scene tree."""
        if self.uses_cached_value:  # if this is a merely planned field, or a SFNode/MFNode field containing other nodes
            cached_value = self.__dict__.get('cached_value', None)
            if index is None:  # removing existing SF cached_value
                if self.node.is_real: wb.wb_supervisor_field_remove_sf(self._as_parameter_)  # remove from sim
                if isinstance(cached_value, Node):
                    cached_value.become_forgotten()  # forget all ancestral references to this, including parent_field's
                    world.step(0)  # allow the scene tree change to take effect
            else:  # removing item from existing MF cached_value
                old_item = cached_value[index]
                if self.node.is_real: wb.wb_supervisor_field_remove_mf(self._as_parameter_, index)  # remove from sim
                if isinstance(old_item, Node):
                    old_item.become_forgotten(forget_parent_field=False)  # we surgically handle parent_field ourselves
                    world.step(0)  # allow the scene tree change to take effect
                cached_value.pop(index)  # remove the indexed item from MF cached_value
            return
        # Otherwise this is a "real" non-Node-containing field in the scene tree, so we call upon Webots to remove it
        if self.is_SF:
            wb.wb_supervisor_field_remove_sf(self._as_parameter_)
            # TODO I think I've had trouble removing occupants like USE FOO.  Poke into this?
        else:
            wb.wb_supervisor_field_remove_mf(self._as_parameter_, index)

    @use_docstring_as_deprecation_warning
    def removeMF(self, index: int = None):
        """DEPRECATED. Field.removeMF() and Field.removeSF() are deprecated. Use field.remove() instead."""
        self.remove(index)
    removeSF = removeMF

    # --- generic SF methods ---

    # Since SF and MF are merged in generic Field, MF Field objects will be cluttered with these methods too.
    # Since we don't yet have a great way of type-hinting which Node.attributes will be fields at all, much less
    # which will be SF and which will be MF, this clutter won't affect much.

    @property
    def sampling(self) -> int:
        """NOTE: automatic sampling (aka "tracking") is available only for SF fields, not MF fields!
           Setting sf_field.sampling = True causes Webots to automatically stream up-to-date values for this
           SF field every timestep, making reading this field be slightly faster than it otherwise would have been.
           Alternative sampling periods, in milliseconds, may be given.  Or setting sf_field.sampling = None, stops
           this automatic streaming, making reading of this field be its ordinary slightly-slower speed again.
           Reading sf_field.sampling returns the last value it was set to (initially None)."""
        return self.__dict__.get('sampling', None)
    @sampling.setter
    def sampling(self, new_period: Union[int, bool, None]):
        if new_period is True: new_period = wb.wb_robot_get_basic_time_step()
        self.__dict__['sampling'] = new_period  # remember this value so that future read attempts will read it
        if new_period:
            wb.wb_supervisor_field_enable_sf_tracking(self._as_parameter_, new_period)
        else:
            wb.wb_supervisor_field_disable_sf_tracking(self._as_parameter_)
    @use_docstring_as_deprecation_warning
    def enableSFTracking(self, samplingPeriod):
        """DEPRECATED. Field.enableSFTracking() is deprecated.
           Use field.sampling = True to automatically sample every timestep, or field.sampling = samplingPeriod."""
        self.sampling = samplingPeriod
    @use_docstring_as_deprecation_warning
    def disableSFTracking(self):
        """DEPRECATED. Field.disableSFTracking() is deprecated. Use field.sampling = None."""
        self.sampling = None

    # --- SF basic get methods (used by Field.get_value) ---

    wb.wb_supervisor_field_get_sf_bool.restype = c_bool
    def getSFBool(self) -> bool:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return wb.wb_supervisor_field_get_sf_bool(self._as_parameter_)

    def getSFInt32(self) -> int:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return wb.wb_supervisor_field_get_sf_int32(self._as_parameter_)

    wb.wb_supervisor_field_get_sf_float.restype = c_double
    def getSFFloat(self) -> float:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return wb.wb_supervisor_field_get_sf_float(self._as_parameter_)

    wb.wb_supervisor_field_get_sf_vec2f.restype = c_void_p
    def getSFVec2f(self):
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return Vec2f.from_address(wb.wb_supervisor_field_get_sf_vec2f(self._as_parameter_))

    wb.wb_supervisor_field_get_sf_vec3f.restype = c_void_p
    def getSFVec3f(self) -> Vec3f:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return Vec3f.from_address(wb.wb_supervisor_field_get_sf_vec3f(self._as_parameter_))

    wb.wb_supervisor_field_get_sf_rotation.restype = c_void_p
    def getSFRotation(self) -> Rotation:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return Rotation(Vec4f.from_address(wb.wb_supervisor_field_get_sf_rotation(self._as_parameter_)))

    wb.wb_supervisor_field_get_sf_color.restype = c_void_p
    def getSFColor(self) -> Color:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return Color(Vec3f.from_address(wb.wb_supervisor_field_get_sf_color(self._as_parameter_)))

    wb.wb_supervisor_field_get_sf_string.restype = c_char_p
    def getSFString(self) -> str:
        """Used by Field.get_value() and Field.value property to get the value of this specific type of field."""
        return wb.wb_supervisor_field_get_sf_string(self._as_parameter_).decode()

    def getSFNode(self) -> Union[Node, None]:
        """Used by Field.get_value() and Field.value property to get the value of this SFNode field.
           SFNode values are cached in the proxy scene tree and kept up to date as this supervisor alters the tree."""
        return self.cached_value

    wb.wb_supervisor_field_get_sf_node.restype = wbNodeRef
    def fetchSFNode(self) -> Union[Node, None]:
        """Uses the C-API to fetch the current content of this SFNode field from the Webots simulation.
           Returns a Node object representing the webots scene tree node that is the value of this field, or None.
           SFNode field values are cached, so this is used only by low-level functions that maintain the cache.
           TODO decide about when, if ever, I want to cache SFNodes as parent.fieldname.  May want to add this to
            parsing/planning, and make sure to update upon changes to scene tree
           This also stores a reference to that child Node in the parent Node containing this field to speed
           repeated access to this child."""
        self.node.decache_children()  # decache node's list(s) of children, as this is about to change
        # print(f"Fetching SFNode within field {self} whose wbFieldRef is {self._as_parameter_} with .value {self._as_parameter_.value}")
        try:
            child_ref = wb.wb_supervisor_field_get_sf_node(self._as_parameter_)
        except OSError:  # Webots' get_sf_node is currently bugged to sometimes produce this error inside nested protos
            WarnOnce(f"OSError when attempting to call wb_supervisor_field_get_sf_node on {self.full_address}")
            child_ref = None
        if not child_ref:
            # print(f"...This attempt to fetch an SFNode has failed since the C-API returned a null parameter.")
            self.node.__dict__.pop(self.name, None)  # remove containing proxy scene tree entry for this field, if any
            return None
        child = self.__dict__['cached_value'] = Node(child_ref, self.node, field=self)  # make Node, store cached_value
        self.node.__dict__[self.name] = child  # also cache as parent.sf_field_name for faster access from parent
        child._incoming_references.add( (self.node, self.name) )  # let child know parent has name for it, for cleanup
        return child

    # --- SF basic set methods (used by Field.set_value) ---

    def setSFInt32(self, value: int):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_int32(self._as_parameter_, int(value))

    def setSFFloat(self, value: float):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_float(self._as_parameter_, c_double(value))

    def setSFVec2f(self, value: Iterable2f):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_vec2f(self._as_parameter_, Vec2f(value))

    def setSFVec3f(self, value: Iterable3f):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_vec3f(self._as_parameter_, Vec3f(value))

    # TODO Allow and type-hint other Rotation formats?
    def setSFRotation(self, value: Iterable4f):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_rotation(self._as_parameter_, Vec4f(value))

    def setSFColor(self, value: Iterable3f):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_color(self._as_parameter_, Vec3f(value))

    def setSFString(self, value: str):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_string(self._as_parameter_, value.encode())

    def setSFBool(self, value):
        """Used by Field.set_value() and Field.value property to set the value of this specific type of field."""
        wb.wb_supervisor_field_set_sf_bool(self._as_parameter_, c_bool(value))

    # --- SF methods for manipulating nodes ---
    def setSFNode(self, value: _ImportableAsNode):
        """Replaces the current value of this SFNode field with a node described by the given value, which may
           be an existing Node to copy, a Plan object, a VRML nodestring, a filename ending in '.wbo', or a
           pathlib Path to such a file. A value of "" or None simply removes the old occupant, without replacing it.
           This automatically cleanses any references to the prior occupant of this SFNode by ancestor nodes,
           to avoid bugs that would arise from trying to refer to an outdated node later."""

        old_node = self.__dict__.pop('cached_value', None)  # decache this, will update to correct value on demand
        if old_node is not None:
            old_node.become_forgotten()  # make ancestors forget old occupant incl. decaching parent's children lists
        else:  # otherwise, we still need to decache parent's children as new occupant would make out of date
            self.node.decache_children()

        if isinstance(value, _pathlib_Path): value = value.read_text()  # convert pathlib Path to a string value

        # # TODO not sure if we need to explicitly remove the old value first in *all* cases? For now assuming no...
        if value is None or value == "":  # if we're setting it to be empty, just remove the current occupant
            if old_node is not None: wb.wb_supervisor_field_remove_sf(self._as_parameter_)
            # TODO decide whether to delete or set to None? May depend on if Webots repopulates default value
            del self.cached_value
            return
        # Otherwise we're importing a new Node into this SF field in the scene tree
        if isinstance(value, Node):
            wb.wb_supervisor_field_import_sf_node_from_string(self._as_parameter_, value.encode(field=self))
        else:
            value = str(value)
            if value.endswith('.wbo'):
                wb.wb_supervisor_field_import_sf_node(self._as_parameter_, value.encode())
            else:
                wb.wb_supervisor_field_import_sf_node_from_string(self._as_parameter_, value.encode())
        wb.wb_robot_step(0)  # let this change take effect
        if not self.hides_children and self.node.is_fully_cataloged:  # keep this branch of proxy tree fully cataloged!
            new_node = self.fetchSFNode()
            if new_node is not None and new_node is not old_node and not new_node.is_inside_proto:
                new_node.catalog_descendants()
        # otherwise self.cached_value will repopulate itself with the new node on demand

    @use_docstring_as_deprecation_warning
    def importSFNode(self, filename: str):
        """DEPRECATED: Field.importSFNode(filename) is deprecated. An equivalent is `field.value = filename`.
           It's usually simpler to use `node.fieldname = filename`"""
        self.setSFNode(filename)
    @use_docstring_as_deprecation_warning
    def importSFNodeFromString(self, nodeString: str):
        """DEPRECATED: Field.importSFNodeFromString(s) is deprecated. An equivalent is `field.value = s`.
           It's usually simpler to use `node.fieldname = s`"""
        self.setSFNode(nodeString)

    # --- MF container interface ---

    # Since SF and MF are merged within generic Field, this means that SF Field objects will have this interface too.
    # Fortunately, users' access to SF fields will mostly be mediated by the proxy scene tree, so they generally
    # shouldn't have much direct access to SF Field objects so won't be distracted by this.

    @cached_property
    def get_item(self) -> Callable:
        """Returns a function, bound to this Field, that can be used to return a single value from the associated
           Webots scene-tree field at a given index.
           E.g. mf_field.get_item(i) is equivalent to, and is used to implement, mf_field[i]."""
        if not self.node.is_real: return self.get_cached_item
        return getattr(self, f"get{self.type}")

    @cached_property
    def set_item(self) -> Callable:
        """Returns a function, bound to this Field, that can be used to set the single element in the associated
           Webots scene-tree field at a given index to a new given value.
           E.g. mf_field.set_item(i, v) is equivalent to, and is used to implement, mf_field[i]=v."""
        if not self.node.is_real: return self.set_cached_item
        return getattr(self, f"set{self.type}")

    @cached_property
    def insert(self) -> Callable:
        """Returns a function, bound to this Field, that can be used to insert a single value into the associated
           Webots scene-tree field just before the given index.
           E.g. mf_field.insert(i, v) works much like Python list.insert(i, v) except affecting the scene tree field.
           Note that mf_field.insert(-1, v) will insert v just before the current value with index -1, making v be
           the new second-to-last element in the field.  This is exactly like Python's list.insert(), but is *not*
           like Webots `wb_supervisor_field_insert_foo` functions in other languages, which, for negative indices,
           instead insert the new value just *after* the existing item with that index. The Python controller chose
           to retain consistency with Python rather than other Webots languages. If you want to append a new item
           just after the last item in a mf_field, do this just as you would with a python list, e.g. with
           field.append(value) or with field.insert(len(field), value)."""
        if not self.node.is_real: return self.insert_cached_item
        return getattr(self, f"insert{self.type}")

    def get_cached_item(self, item):
        """Version of field.get_item that gets from the field's .cached_value, used for merely planned fields."""
        return self.cached_value[item]

    def set_cached_item(self, index, new_value):
        """Version of field.set_item that sets to the field's .cached_value[index], used for merely planned fields."""
        if isinstance(new_value, Node): new_value = self.claim_a_version_of(new_value)
        self.cached_value[index] = new_value
        if isinstance(new_value, Node): self.node.decache_children()  # decache as this now out of date

    def insert_cached_item(self, index, value):
        """Version of field.insert that inserts into the field's .cached_value, used for merely planned fields."""
        pindex = index
        if index < 0 and not settings.negative_insertion_indices_are_pythonic:
            pindex = len(self) if index == -1 else index + 1
        if isinstance(value, Node): value = self.claim_a_version_of(value)
        self.cached_value.insert(pindex, value)
        if isinstance(value, Node): self.node.decache_children()  # decache as this is now out of date

    def __getitem__(self, index):
        """This will be called whenever the controller attempts to retrieve an indexed
         item from this field, e.g., with field[1].  This returns the result that
         would arise from calling the relevant getMF... function on the Webots field.
         Negative indices are interpreted as in python, working backwards from end of list.
         Slices like field[0:2] return the relevant elements as you would expect in Python.
         The full slice field[:] returns a list of all the entries, equivalent to list(field)."""

        # print(f"{Console.BLUE}Field.__getitem__ was just called for {self}[{index}]{Console.RESET}")
        n = len(self)
        if isinstance(index, int):
            # if index < 0: index = index + len(self) # wrap negative indices - Webots actually does this for us
            if index >= n or -index > n:
                if n < 0: raise IndexError(f'Field {self} is a single/SF field, so cannot be indexed.')
                raise IndexError(f'Index {index} out of bounds: field {self} has only {n} items.')
            return self.get_item(index)
        elif isinstance(index, slice):
            # slice.indices( length ) returns a list of parameters that can be used
            # to specify a range iterator that will step through the relevant indices
            return [self[i] for i in range(*index.indices(n))]
        else:
            raise IndexError(f'Index {index} is neither an integer nor a slice.')

    def __setitem__(self, index, value):
        """This is called whenever the controller attempts to alter an indexed item in this MF field,
           e.g., with field[index]=value.  This typically calls the relevant setMF___ function for the scene tree
           field with that index. Negative indices are interpreted as in Python, working back from end of list.
           If index is a Python slice, as in field[0:2] = valuelist, then the given value must be an iterable of
           zero or more values that could each be sent with the relevant Webots setMF___ function.  The Webots field
           values with the indices specified in the slice will be set to match the successive values listed in value.
           If more new values are listed than the length of the slice, the remainder will be inserted just before the
           stop point of the slice, whereas if fewer new values are given, then the remainder of the slice is removed
           from the Webots field. As a special case of this, field[i:i]=valuelist will insert the new values
           just before index i of the webots field, a simple means for multiple insertion.
           As additional special cases, field[i:i+1]=[] deletes entry i from the Webots field, field[i:j+1]=[] deletes
           the entries with indices i...j from it, and field[:]=[] deletes *all* entries from the Webots field.
           You can also accomplish this last task with node.field = [] or node.field.clear().
           (Note: here, as in Python generally, you may imagine the start and stop numbers in Python slices
           as being prefaced with "just before". So field[1:1] starts and stops just before index 1,
           capturing nothing and just providing an insertion point there. And field[i:i+1]
           starts just before index i and stops just before i+1, capturing just index i.)"""
        if isinstance(index, int):  # if index is simple integer
            # if index < 0: index = index + self.getCount() # wrap negative indices -- Webots already does this
            self.set_item(index, value)
            return
        if not isinstance(index, slice):
            raise TypeError(f'Index {index} is neither an integer nor a slice.')
        # otherwise, index must be a slice, e.g. field[0:4]
        # slice.indices( length ) returns a list of parameters that can be used
        # to specify a range iterator that will step through the relevant indices
        total_length = len(self)
        R = range(*index.indices(total_length))
        slice_length = len(R)
        new_length = len(value)

        # first we overwrite any overlap cases
        for i, val in zip(R, value):
            # logprint( f"Overwriting index {i} with value {val}.")
            self.set_item(i, val)
            # logprint( f"So now field[{i}]={self.get_item(i)}" )

        # then delete any remainder (starting from right to not change indices of later deletion targets)
        if slice_length > new_length:
            remainder = R[new_length:]
            if not index.step or index.step > 0:  # if slice went forwards
                remainder = reversed(remainder)  # must delete from right
            for i in remainder:  self.remove(i)

        # alternatively, insert any additional values just before stopping point of slice
        elif slice_length < new_length:
            remainder = value[slice_length:]
            if not index.step or index.step > 0:  # if slice goes rightward
                # We want a negative-index insertion point (i.e. relative to end of list)
                # so as elements are inserted, insertion point will stay to their right
                if index.stop is None or index.stop > total_length:  # if slice goes to end
                    insertion_point = -1  # insert at very end, in given order
                elif index.stop < 0:  # if given negative stopping point
                    # e.g. field[-1:-1]=new_values should insert just before last,
                    # at insertion point -2.
                    insertion_point = index.stop - 1
                else:  # if given positive stop for slice
                    # e.g., field[2:2] = newvalues should insert just before index 2
                    # so if the old total_length==3 and index 2 was the last [aka -1]
                    # this should insert elements one before last, i.e., at -2
                    insertion_point = index.stop - total_length - 1
            else:  # if slice goes backwards
                # For these we want non-negative insertion point (rel. to start of list)
                # so that later insertions will push earlier ones to the right
                # (alternatively we could use a negative insertion point and reverse
                # our list of things to insert, but I'll trust C to be faster than Python)
                if index.stop is None:  # if the backwards slice goes all the way
                    # note we needn't check if they expressed a more extreme stopping point
                    # because values more extreme than 0 are treated as negative indexes
                    insertion_point = 0  # insert at beginning (letting earlier insertions get pushed right)
                elif index.stop >= 0:  # if given positive stopping point
                    # e.g. field[1:0:-1]=new_values should overwrite index 1,
                    # (stopping before index 0) and then insert remainder between those,
                    # i.e., at insertion point 1
                    insertion_point = index.stop + 1
                else:  # if given negative stop for slice
                    # e.g., field[-1:-2:-1] = newvalues should overwrite the -1
                    # element, then insert the remainder just before that, at
                    # insertion point -2.  Generalizing this, we would expect
                    # field[:-1:-1] to insert at end of list.  Webots docs allow
                    # only a negative version of that command, so we'll keep
                    # this negative and reverse our list of things to insert
                    insertion_point = index.stop
                    remainder = reversed(remainder)  # must insert rightmost first
            # now that we've determined the right insertion point, let's insert!
            logprint(f"Attempting to insert {remainder} at position {insertion_point}.")
            for val in remainder:
                self.insert(insertion_point, val)
        # end handling of insertions

    # __iter__ and __reversed__ work automatically since __len__ and __getitem__ are defined

    def append(self, value):
        """Appends value at the end of the associated Webots scene tree field.  I.e., this behaves just
           like Python's list.append, except seeing through to the associated Webots scene tree field."""
        self.insert(-1, value, negative_indices_are_pythonic = False)

    def extend(self, valuelist):
        """Appends the listed values at the end of the associated Webots scene tree field. I.e., this behaves
           just like Python's list.extend, except seeing through to the associated Webots scene tree field."""
        for value in valuelist: self.insert(-1, value)

    def clear(self):
        """Clears all entries from the associated Webots scene tree field. I.e., this behaves just like
           Python's list.clear(), except seeing through to the associated Webots scene tree field."""
        self[:] = []

    def pop(self, index=-1) -> Any:
        """Removes and returns the element at the given index (default last) from the the associated Webots scene tree
           field. I.e., behaves just like Python's list.pop except seeing through to the associated scene tree field."""
        value = self.get_item(index)
        self.remove(index)
        return value

    def index(self, target, start:int = 0, end:int = None) -> int:
        """Returns the index for the first item in this MF field equal to target, or raises ValueError if there is none.
           If start and/or end are given the search is constrained as in python slices.
           I.e., this behaves just like Python's list.index except seeing through to the associated scene tree field."""
        if self.uses_cached_value:
            if end is not None: return self.cached_value.index(target, start, end)
            return self.cached_value.index(target, start)
        for i in len(self):
            if self[i] == target: return i
        raise ValueError(f"{target} is not in {self}")

    # --- MF basic get methods (used by Field.get_item) ---

    wb.wb_supervisor_field_get_mf_bool.restype = c_bool
    def getMFBool(self, index) -> bool:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return wb.wb_supervisor_field_get_mf_bool(self._as_parameter_, index)

    def getMFInt32(self, index) -> int:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return wb.wb_supervisor_field_get_mf_int32(self._as_parameter_, index)

    wb.wb_supervisor_field_get_mf_float.restype = c_double
    def getMFFloat(self, index) -> float:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return wb.wb_supervisor_field_get_mf_float(self._as_parameter_, index)

    wb.wb_supervisor_field_get_mf_vec2f.restype = c_void_p
    def getMFVec2f(self, index) -> Vec2f:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return Vec2f.from_address(wb.wb_supervisor_field_get_mf_vec2f(self._as_parameter_, index))

    wb.wb_supervisor_field_get_mf_vec3f.restype = c_void_p
    def getMFVec3f(self, index) -> Vec3f:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return Vec3f.from_address(wb.wb_supervisor_field_get_mf_vec3f(self._as_parameter_, index))

    wb.wb_supervisor_field_get_mf_rotation.restype = c_void_p
    def getMFRotation(self, index) -> Rotation:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return Rotation(Vec4f.from_address(wb.wb_supervisor_field_get_mf_rotation(self._as_parameter_, index)))

    wb.wb_supervisor_field_get_mf_color.restype = c_void_p
    def getMFColor(self, index) -> Color:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return Color(Vec3f.from_address(wb.wb_supervisor_field_get_mf_color(self._as_parameter_, index)))

    wb.wb_supervisor_field_get_mf_string.restype = c_char_p
    def getMFString(self, index) -> str:
        """Used by Field.get_item() to get the value of a particular element in this specific type of multi-field."""
        return wb.wb_supervisor_field_get_mf_string(self._as_parameter_, index).decode()

    # --- MF basic set methods (used by Field.set_item) ---

    def setMFBool(self, index, value):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_bool(self._as_parameter_, index, c_bool(value))

    def setMFInt32(self, index, value: int):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_int32(self._as_parameter_, index, value)

    def setMFFloat(self, index, value: float):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_float(self._as_parameter_, index, c_double(value))

    def setMFVec2f(self, index, value: Iterable2f):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_vec2f(self._as_parameter_, index, Vec2f(value))

    def setMFVec3f(self, index, value: Iterable3f):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_vec3f(self._as_parameter_, index, Vec3f(value))

    def setMFRotation(self, index, value: Iterable4f):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_rotation(self._as_parameter_, index, Vec4f(value))

    def setMFColor(self, index, value: Iterable3f):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_color(self._as_parameter_, index, Vec3f(value))

    def setMFString(self, index, value: str):
        """Used by Field.set_item() to set a new value for a particular element in this specific type of multi-field."""
        wb.wb_supervisor_field_set_mf_string(self._as_parameter_, index, value.encode())

    # --- MF basic insert methods (used by Field.insert) ---

    def insertMFBool(self, index, value,
                     negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_bool(self._as_parameter_, index, c_bool(value))

    def insertMFInt32(self, index, value: int,
                      negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_int32(self._as_parameter_, index, value)

    def insertMFFloat(self, index, value: float,
                      negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_float(self._as_parameter_, index, c_double(value))

    def insertMFVec2f(self, index, value: Iterable2f,
                      negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_vec2f(self._as_parameter_, index, Vec2f(value))

    def insertMFVec3f(self, index, value: Iterable3f,
                      negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_vec3f(self._as_parameter_, index, Vec3f(value))

    def insertMFRotation(self, index, value: Iterable4f,
                         negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_rotation(self._as_parameter_, index, Vec4f(value))

    def insertMFColor(self, index, value: Iterable3f,
                      negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_color(self._as_parameter_, index, Vec3f(value))

    def insertMFString(self, index, value: str,
                       negative_indices_are_pythonic = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new value just before the given index in this type of multi-field."""
        if index < 0: index -= negative_indices_are_pythonic
        wb.wb_supervisor_field_insert_mf_string(self._as_parameter_, index, value.encode())

    # --- MFNode methods (handled separately together due to complexity) ---

    def getMFNode(self, index) -> Node:
        """Used by Field.get_item() to get the Node at the given index of this multi-field.
           MFNode values are typically cached and the cache is updated as this supervisor alters the field."""
        cached_value = self.__dict__.get('cached_value', None)  # find cached list of nodes, or None
        if cached_value is not None: return self.cached_value[index]  # if there is one, use it
        return self.fetchMFNode(index)  # otherwise fetch single node through C-API

    wb.wb_supervisor_field_get_mf_node.restype = wbNodeRef
    def fetchMFNode(self, index) -> Node:
        """Uses the C-API to fetch the current Node at the given index of this multi-field. This typically should
           be called only by mid-level functions that maintain a cache of MFNode field contents."""
        try:
            child_ref = wb.wb_supervisor_field_get_mf_node(self._as_parameter_, index)
        except OSError:  # Webots' get_mf_node is currently bugged to sometimes produce this error inside nested protos
            WarnOnce(f"OSError when attempting to call wb_supervisor_field_get_mf_node({self.full_address}, {index})")
            return None
        else:
            return Node(child_ref, self.node, field=self)

    def insertMFNode(self, index: int, value: _ImportableAsNode,
                     negative_indices_are_pythonic:bool = settings.negative_insertion_indices_are_pythonic):
        """Used by Field.insert() to insert a new Node just before the given index of this multi-field.
           Also used by Field.__setitem__ to handle the insertion portion of changing MFNode field contents.
           The new node will match the given value, which may be an existing Node to copy, a Plan object,
           a VRML nodestring, a filename ending in '.wbo', or pathlib Path to such a file.
           Python and standard Webots treat negative insertion indices differently, with Python's list.insert(-n,v)
           inserting v just before the item whose index was -n, and Webots inserting v just after.
           `negative_indices_are_pythonic` determines which of these will occur for this insertion, or by default
           it will employ the blanket policy recorded settings.negative_insertion_indices_are_pythonic."""
        # TODO this should never trigger, as that case should be handled by insert_cached_item
        if not self.node.is_real: # If we're assigning a value to a merely Planned Field
            if not isinstance(value, Node):
                value = plan(value)
            self.cached_value.insert(index, self.claim_a_version_of(value) )
            self.node.decache_children()  # decache node's list(s) of children, as this just changed
            return

        # Otherwise this is a "real" scene tree node, so we need to import a new value via the C-API
        old_length = wb.wb_supervisor_field_get_count(self._as_parameter_)  # will change if import succeeds
        # Python and Webots treat negative indices differently; we'll follow the model chosen in settings
        windex = index if index >= 0 else index - negative_indices_are_pythonic
        pindex = index
        if index<0 and not negative_indices_are_pythonic:
            pindex = old_length if index == -1 else pindex + 1

        if isinstance(value, _pathlib_Path): value = value.read_text()  # convert pathlib Path to a string value
        if isinstance(value, Node):
            wb.wb_supervisor_field_import_mf_node_from_string(self._as_parameter_, windex, value.encode(field=self))
        else:
            value = str(value)
            if value.endswith('.wbo'):
                wb.wb_supervisor_field_import_mf_node(self._as_parameter_, windex, value.encode())
            else:
                wb.wb_supervisor_field_import_mf_node_from_string(self._as_parameter_, windex, value.encode())
        wb.wb_robot_step(0)  # let this change take effect

        # if this import succeeded, we'll need to update our proxy tree accordingly
        new_length = wb.wb_supervisor_field_get_count(self._as_parameter_)
        if new_length == old_length: 
            Warn(f"Attempt to import new node into {self} failed.")
            return  # import failed so no need to update our proxy
        self.node.decache_children()  # decache containing node's list(s) of children, as this just changed
        if 'cached_value' not in self.__dict__: return  # No cached value for us to update
        new_node = self.fetchMFNode(windex)  # fetch the newly added Node, using the C-API
        cached_value = self.cached_value
        if cached_value is None: cached_value = self.cached_value = []
        if isinstance(cached_value, Node): cached_value = self.cached_value = [cached_value]
        cached_value.insert(pindex, new_node)
        # TODO there may be some efficiency to be gained from caching what was just imported, rather than re-exporting!
        #  For now we'll re-export, as that is simpler and has slightly fewer potential points of divergence
        # TODO might want to condition this on a setting, as this may be preferable even when not fully cataloged
        if self.node.is_fully_cataloged and not new_node.is_inside_proto:
            new_node.catalog_descendants()  # keep this branch fully cataloged!

    def setMFNode(self, index: int, value: _ImportableAsNode):
        """Replaces the node at the given index of this MFNode field with a new node by first attempting to insert
           the new node, and then, if that succeeds, removing the old occupant.
           This is used by Field.set_item(), and hence also to implement field[index]=value.
           The new node will match the given value, which may be an existing Node to copy, a Plan object,
           a VRML nodestring, a filename ending in '.wbo', or pathlib Path to such a file.
           Like Python's list[index]=new_value this accepts negative indices and raises an IndexError for indices
           outside the current list. To insert items outside the current bounds, use field.insert() or .append()."""
        old_length = wb.wb_supervisor_field_get_count(self._as_parameter_)
        if index < 0: index = index + old_length  # convert all indices to non-negative to simplify things
        if not 0 <= index < old_length: raise IndexError("setMFNode index must be within the current list.")
        self.insertMFNode(index, value)   # inserts new node and calls robot.step(0) to let insertion take effect
        new_length = wb.wb_supervisor_field_get_count(self._as_parameter_)
        if new_length > old_length: self.remove(index + 1)  # if insertion succeeded, remove old element

    @use_docstring_as_deprecation_warning
    def importMFNode(self, index, new_value):
        """DEPRECATED: Field.importMFNode() and Field.importMFNodeFromString() are deprecated.
           An equivalent is field.insert(index, filename_or_nodestring)."""
        self.insert(index, new_value)
    importMFNodeFromString = importMFNode

# --- Field subclass declarations ---

# The following subclasses are currently trivial placeholders assigned as default types for Node.fields like .children

class SField(Field): pass
class MField(Field): pass

class MFNode(MField): pass
class MFBool(MField): pass
class MFInt32(MField): pass
class MFFloat(MField): pass
class MFVec2f(MField): pass
class MFVec3f(MField): pass
class MFRotation(MField): pass
class MFColor(MField): pass
class MFString(MField): pass

# TODO need to think through how many Field-like classes I want, with the main options being:
#  (1) Just a single Field class with all the methods
#  (2) A single Field superclass with all the methods, and a MFField subclass with container interface (roughly my original approach)
#  (3) A Field superclass with SFField and MFField subclasses with their respective methods / container interface
#  (9) Separate subclasses for each field datatype x multiplicity
#  Smaller-numbered options may help linters to avoid false negatives (saying a Field lacks methods it has)
#  Larger-numbered options may help linters to avoid false positives, though may be hard to avoid false negatives
#  So perhaps a prior question is how these classes will be signified to linters?
#   At a minimum, old-school Node.getField() can type-hint to generic Field, though that would give a lot of false
#    positives or negatives depending on which of the above options we choose.  Tis deprecated so we don't care.
#   In the proxy scene tree, we don't make SF fields directly available, so common SF attributes
#   like Node.translation may get type-hinted to the type of their .values.
#   Since users should generally have no (non-deprecated) access to SF Field objects, it wouldn't really matter if
#   those objects have additional MF-specific methods defined.  Weighs in favor of (1), though lumping SF-methods
#   into MF Fields will come with a slight cost of false positives.
#   In the proxy scene tree, MF fields are made directly available, and commonly used ones like .children could
#   be type-hinted fine-grainedly.  So it looks like the "(many)" option may be best for MF?!
#   However... I'm not sure I really want to proliferate (MF) field types, and I'm not sure how much this type-hinting
#   would actually help.  Probably the main help would be documenting the expected type of field.__getitem__
#   so, e.g., items accessed/iterated from a .children field would be linted as Nodes.
#   Actually, looks like I can type-hint Node.children as MField[Node] and pycharm infers it will contain Nodes - yay!
#   Since Nodes will sadly often require explicit `#type: Node` hints from the user anyway, it's not that costly
#   to require such hints in this case too.  There aren't many other commonly used MF fields to try to help.
#  For now, I think I'll keep it as (1) a single generic Field class
#   If we ever remove the clutter of semi-deprecated SF... and MF... methods, that'll greatly reduce false positives
#   At some point we could proliferate MF subclasses, like MFNode to provide a bit of linting help, but is low priority
#   An especially plausible MF subclass is MFNode, due to the complexity and sensitivity of Node manipulation,
#    and due to children field's being especially common and especially benefiting from linting help
#   Unfortunately as long as we want Field objects to have the full suite of deprecated methods, we won't get
#    the compartmentalization gains of spinning off this functionality to a subclass, unless we decide to have
#    the deprecated version of controller use import MFNode as Field...?
#   Most backwards compatibility doesn't actually require a single Field class, just that the constructors getField and
#    getFieldByIndex return objects with the expected methods, regardless of whether they have the other methods.
#   I'll leave this as a possible future development, but still low priority.

# === Planned Objects and VRML Parsing ===

class VRML_String(str):
    """A special subclass of string, used to represent a VRML (sub)string formatted as VRML, ready to import to Webots.
       This differs from ordinary strings in that its constructor converts the given value to VRML format, and
       if the given value is already a VRML_String it will be returned as is, rather than being wrapped in quotes,
       as ordinary strings would be unless the optional add_quotes parameter is set to False.
       The optional field, if given, specifies what field this node is being prepared for import into, which makes
       a difference for some self-assembling nodes, e.g. allowing Cones to be imported to a children field.
       Python booleans are converted to VRML TRUE and FALSE.
       If the given value is a tuple or any sort of vector, it is converted to a naked space-separated sequence of
       the VRML_String versions of its components.
       If the given value is a (non-vector) python list, such a sequence is returned, but wrapped in square-brackets.
       Nodes (both real and planned) are converted to a string using their .export() method.
       Everything else is converted to a string using python's str().
       """
    def __init__(self, value, indent=0, add_quotes: Any = str, field:Field = None):
        super().__init__()

    def __new__(cls, value, indent=0, add_quotes: Any = str, field:Field = None):
        # print(f"  Now creating a VRML string for {value} to be inserted into field {field}")
        # If the given value is already a VRML_String, it is returned as is (avoids multi-wrapping in quotes!)
        if isinstance(value, VRML_String): return value

        # If it is any other string, it is wrapped in double-quotes (unless add_quotes is False)
        if isinstance(value, str):
            return super().__new__(cls, f'"{value}"' if add_quotes else value)

        # If the given value is a Node, we use its own .export method to export it as VRML
        if isinstance(value, Node): return value.export(indent=indent+2, field=field)

        # Python booleans are converted to Webots' TRUE and FALSE.
        if value is True: return super().__new__(cls, 'TRUE')
        if value is False: return super().__new__(cls, 'FALSE')

        # A tuple or Vector is converted to a naked space-separated sequence of its recursively processed components.
        if isinstance(value, (tuple, GenericVector)):
            return super().__new__(cls, ' '.join(VRML_String(component, field=field) for component in value))

        # If the given value is a (non-vector) python list, such a sequence is returned, but wrapped square-brackets.
        if isinstance(value, list):
            if len(value) > 1 and isinstance(value[0], Node):
                sep = f"\n{(indent+2)*' '}"
                components = (VRML_String(comp, indent + 2, field=field) for comp in value)
                return super().__new__(cls, f"[{sep}{sep.join(components)}]\n{indent*' '}")
            return super().__new__(cls, f"[{', '.join(VRML_String(component, field=field) for component in value)}]")

        # Everything else is converted to a string using python's str().
        return super().__new__(cls, value)


# TODO merge this with Node
class OldPlannedNode:
    #TODO Docstring
    def __init__(self, _nodetype_name:str=None, **kwargs):
        if _nodetype_name is not None: # If given a node type, morph to the relevant subclass of PlannedNode
            self.__class__ = getattr( _PlanModule, _nodetype_name, _PlanModule.__getattr__(None, _nodetype_name) )
        # self.__dict__.update( locals() ) # would add any explicitly-expected kwargs to object's dict
        self.__dict__.update( kwargs ) # add any not-explicitly-expected kwargs to object's dict

    def __repr__(self):
        """A brief user-friendly representation of this node, e.g. as PlannedNode(Robot, DEF='ROBOT')"""
        if 'DEF' in self.__dict__: return f"PlannedNode({type(self).__name__}, DEF='{self.DEF}')"
        if 'name' in self.__dict__: return f"PlannedNode({type(self).__name__}, name='{self.name}')"
        return f"PlannedNode({type(self).__name__})"

    def __str__(self, indent=0) -> VRML_String:
        """The full VRML string representation of this node, ready for import into Webots."""
        DEF = f"DEF {self.DEF} " if 'DEF' in self.__dict__ else ""
        attributes = (f"{key} {VRML_String(value, indent+2)}" for key, value in self.__dict__.items() if key != 'DEF')
        # TODO, there could be other cases where this is short enough to make a one-liner?
        if len(self.__dict__)<=3 and 'children' not in self.__dict__:
            return VRML_String(f"{DEF}{type(self).__name__}{{{', '.join(attributes)}}}", indent, add_quotes=False)
        # otherwise
        sep = f"\n{(indent + 2) * ' '}"
        attribute_string = sep.join(attributes)
        return VRML_String(f"{DEF}{type(self).__name__}{{{sep}{attribute_string}}}\n{indent*' '}", indent+2, add_quotes=False)

    def __reduce__(self):
        """This would be used by copy() or pickle() to determine how a new copy of this PlannedNode could be made.
           Since some PlannedNode subclasses are dynamically generated, they won't automatically be pickle/copyable.
           Instead, we'll reconstruct the object by passing this node's _NodeType (i.e. its subclass' __name__)
           to PlannedNode itself, which will then morph the reconstructed node to the right subclass."""
        return PlannedNode, (type(self).__name__,), self.__dict__

    def __eq__(self, other):
        """Two PlannedNodes are deemed equal if they have same NodeType and explicitly define the same fields equally.
           (Note: this doesn't know default values for fields, so if one node explicitly defines a field to its
           default value, and another remains silent about that field, then they won't count as "equal", even though
           they would produce equivalent objects when imported into Webots."""
        return type(self).__name__ == type(other).__name__ and self.__dict__ == other.__dict__

def PlannedNode(nodetype: str = None,
                parent_field: 'Field' = None,
                DEF: str = '',
                name: str = '',
                is_real=False,
                USE = '',
                **fields) -> 'Node':
    node = object().__new__(Node)
    # For each node.attribute, we alter its __dict__ directly, to bypass the slow custom __setattr__
    # We also store entries in catalog for this node to enable fast lookup; filed under DEF, name, nodetype and id
    if nodetype is not None: node.__dict__['type'] = nodetype
    node.__dict__['DEF'] = DEF
    node.__dict__['name'] = name
    # TODO need to think about whether catalog should be relativized to roots, to enable similar searching in non-real nodes
    #  For now, only real nodes are cataloged. Mere planned branches should be small enough to recursively search
    node.__dict__['is_real'] = is_real
    node.__dict__['USE'] = USE
    node.__dict__['is_fully_cataloged'] = is_real
    if is_real:
        for nomiker in node.nomikers:
            catalog[nomiker].append(node)

    node.__dict__['fields'] = FieldsBearer(node)
    node.__dict__['_incoming_references'] = set()  # set of (a,n) pairs of ancestors with names for this node
    if parent_field is not None:
        node.__dict__['parent_field'] = parent_field
        node.__dict__['parent'] = parent_field.node

    if name: node.fields._create('name', name)
    for field_name, value in fields.items():
        node.fields._create(field_name, value)

    return node

# === VRML_Parser (parses VRML strings to Nodes and Fields) ===

class VRML_Parser:
    """VRML_Parser(s).parse() will parse the VRML string s (e.g. the contents of an exported .wbo file) to create
       a Node (or list thereof) whose .attributes will match the fields described in the string.
       These attributes may later be modified, and/or this Node could be used in constructing larger
       Nodes containing it.  Nodes may be imported into the Webots scene tree,
       e.g. with `world.ROBOT.children.append( planned_node )`.
    """
    # matches any number, potentially with leading minus sign, internal decimal point, and/or exponential notation
    re_numerical = re.compile(r"-?\d+(?:\.\d*)?(?:[eE]\d*)?")

    # Matches any number (as above), preceded by a single leading space.  Returns the number as group 1.
    # (This is usable as an alternative re_next function for ReadSessions to step through spaced numbers)
    re_spaced_number = re.compile(r" (-?\d+(?:\.\d*)?(?:[eE]\d*)?)")

    # The following prefers to consume \\, or \", or any non-", repeating until it hits a non-escaped "
    # Presumed to start just after open-". Group 1 will contain just the string content. Full match includes close-"
    re_string = re.compile(r'((?:\\\\|\\"|[^"])*?)"')

    def __init__(self, s: str, is_real=False):
        self.base = s
        self.is_real = is_real
        self.it = VRML_Parser.ReadSession(s)

    class ReadSession:
        """An iterator for a string that maintains a read-head and offers methods for iterating through substrings.
           The read-head points at what will typically be read next. Automatically skips whitespace and #comments."""
        def __init__(it, base: str, starting_index=0):
            it.base = base
            it.read_head = starting_index

        # This regex is the primary tool for seeking successive items, starting at the read_head.
        # It skips past #comments, commas, and whitespace and then group1 matches either a single symbol like {}[]"
        # or a contiguous string of characters (alphanumeric/_/-/.) that could be used in a word or number.
        re_next = re.compile(r'(?:#.*|[,\s]*)*([.\-\w]+|["{}[\]])')

        def __iter__(it): return it  # python will now view a ReadSession as an iterator, using it's __next__ method

        def __next__(it, re_next=re_next) -> str:
            """Returns the next matching item after the read_head, moves the read_head to just after this match,
               and sets match_start to its beginning.
               By default this uses a regular expression that skips past #comments, commas and whitespace, to match
               the next symbol like []{}" or contiguous word/number.  An alternative regular expression to determine
               what counts as a match may be given as re_next, which returns the match itself as its group 1."""
            it.match = re_next.match(it.base, it.read_head)
            if not it.match or not it.match[1]:
                it.read_head = it.match_start = len(it.base)
                raise StopIteration
            it.read_head = it.match.end()  # advance read-head to just after the end of this match
            it.match_start = it.match.start(1)  # note where this word/symbol started
            if HIGHLIGHT: it.highlight_slice(it.match_start, it.read_head)  # TODO comment out!
            return it.match[1]  # return this word/symbol

        def revert(it):
            """Reverts the read_head back to the beginning of the last value yielded by __next__."""
            it.goto(it.match_start)

        def peek(it, re_next=re_next) -> str:
            """Returns the next item after the read head (or None if there is none), much like __next__, but doesn't
               advance the read head until it.advance() is called."""
            it.peek_match = re_next.match(it.base, it.read_head)
            if not it.peek_match or not it.peek_match[1]: return None
            it.peek_match_start = it.peek_match.start(1)  # note where this word/symbol started
            if HIGHLIGHT: it.highlight_slice(it.peek_match_start, it.peek_match.end(), color=Console.CYAN_BACK)
            return it.peek_match[1]  # yield this word/symbol

        def advance(it):
            """Advances the read_head past the last peek."""
            it.match = it.peek_match
            it.read_head = it.match.end()  # advance read-head to just after the end of this match
            it.match_start = it.match.start(1)  # note where this word/symbol started

        def advance_and_peek(it, re_next=re_next) -> str:
            """For convenience, combines it.advance and it.peek to advance past the last peek and peek again."""
            it.advance()
            return it.peek(re_next=re_next)

        def peek_through(it, re_next=re_next) -> str:
            """This generator yields items after the read_head, using peek rather than __next__, waiting to see if this
               is called upon to continue (and if the read_head is still in position) before advancing the read_head."""
            value = it.peek(re_next=re_next)
            while value is not None:  # at end of string, this will exit and stop iteration
                yield value
                # if we get called back, and read_head hasn't been moved by anyone else, we'll advance it ourselves
                if it.read_head == it.peek_match.start(): it.advance()
                value = it.peek(re_next=re_next)

        def __index__(it):
            return it.read_head

        def __int__(it):
            return it.read_head

        def __getitem__(it, item):
            return it.base[item]

        def goto(it, position):
            it.read_head = int(position)

        def highlight_slice(self, start, end, color=Console.YELLOW_BACK):
            """Prints a version of the string, with the region from start to end highlighted.
               The string is condensed and has the leading section omitted to improve readability."""
            s = self.base
            s = f"{s[:start]}{color}{s[start:end]}{Console.RESET}{s[end:]}"  # add color highlighting
            window_start = start//150*150 - 25  # string still contains many spaces, so big window will likely shrink
            s = s[max(0, window_start):max(window_start+400, end+10)]  # print jumping window
            # aggressively compact s down to make it more readable on single line (this may mutilate contained strings!)
            s = s.replace('\n', '')  # remove newline characters onto single line
            while '  ' in s: s = s.replace('  ', ' ')  # condense multiple spaces
            print(s)

        def until(it, pred: Union[Callable, str], start: int = None) -> str:
            """This generator advances the read_head (starting at start, if given), yielding successive items,
               until encountering an item satisfying pred (i.e. callable pred(item) is true, or item==pred).
               Iteration stops without yielding this matching item, and the read_head is left pointing just after it.
               Useful for reading up to closing braces or brackets."""
            if start is not None: it.goto(start)
            if isinstance(pred, Callable):
                for item in it:
                    if pred(item): return
                    yield item
            else:
                for item in it:
                    if pred == item: return
                    yield item

        def satisfying(it, pred: Callable, start: int = None) -> str:
            """This generator advances the read_head (starting at start, if given), yielding successive items for
               which pred(item) is true. Upon encountering the first unsatisfactory item, iteration stops without
               yielding that item, though that item will have been stored in peek_match and the read_head will be
               pointing at it."""
            for item in it.peek_through():
                if not pred(item): return
                it.advance()
                yield item

    def parse(self, node: 'Node' = None, field: 'Field' = None) -> Union['Node', List['Node']]:
        """VRML_Parser(s).parse() will parse the VRML string s (e.g. the contents of an exported .wbo file).
           This will construct a Node (or list thereof) whose .attributes will match the fields
           described in the string.  These attributes may be modified and/or this Node could be used in
           constructing larger Planned-objects.  Planned nodes may be imported into the Webots scene tree,
           e.g. with `world.ROBOT.children.append( planned_node )`. """
        nodes = self.parseList(field=field)
        if field is not None:
            field.expected_value = nodes

        return nodes if len(nodes) > 1 else nodes[0]

    def parseList(self, field=None) -> list:
        """Using this parser's ReadSession, this attempts to read one or more items from this parser's base string,
           up to the end of the current list, as marked by end-of-string or a closing bracket ']'.
           Parsed items will be returned as a list, though contiguous numerical items will be combined into
           a single Vector entry in that list.  E.g. parsing '[1 2]' returns [ Vector(1,2) ]."""
        return [ self.parse_value(item, field, vectors_must_be_space_separated=True)
                 for item in self.it.until(']') ]

    def parse_value(self, item=None, field=None, vectors_must_be_space_separated=False):
        """This attempts to read a single value from the parser's base string, beginning with item, if given, and
           otherwise at the read_head.  If this value is numerical, contiguous numerical values will also be read,
           and combined into a vector.  If vectors_must_be_space_separated is True, those contiguous items must be
           space-separated, or they will be separated into separate vectors, which helpfully handles many cases
           where webots outputs lists of vectors with spaces separating the components of each vector and line-breaks
           separating the vectors. VRML outputted by this Python utility is handled similarly, though in short lists
           the vector-separator will be commas rather than line-breaks."""
        if item is None: item = next(self.it)  # iterate onto the next word/symbol
        if item == 'TRUE': return True
        if item == 'FALSE': return False
        if item == '[':
            return [self.parse_value(item, field, vectors_must_be_space_separated=True) for item in self.it.until(']')]
        # TODO: Not sure the following is used/needed
        if item == ']': return StopIteration  # This is being returned, not *raised*, to indicate we've reached list-end
        if item == '"':
            match = self.re_string.match(self.base, self.it.read_head)
            self.it.goto(match.end())  # just after close-"
            if HIGHLIGHT: self.it.highlight_slice(match.start(1)-1, match.end(), color=Console.MAGENTA_BACK)
            return match[1]  # includes everything from old read-head position to just before close-"
        if self.re_numerical.match(item):  # Adjacent numbers will be combined into a single vector value
            strings = [item]
            if vectors_must_be_space_separated:
                strings.extend(self.it.peek_through(re_next=self.re_spaced_number))
            else:
                strings.extend(self.it.satisfying(self.re_numerical.match))
            numbers = [float(s) if '.' in s or 'e' in s or 'E' in s else int(s) for s in strings]
            return numbers[0] if len(numbers) == 1 else Vector(numbers)
        if item == 'USE':
            return PlannedNode(parent_field=field, USE=next(self.it), is_real=self.is_real)
        # Otherwise, item was non-numerical, presumably another Node
        return self.parse_node(destination=field, first_item=item)

    def parse_node(self, destination: Union['Node', 'Field'] = None, first_item: str = None) -> 'Node':
        """Using this parser's ReadSession, this attempts to read a single node from parser's base string,
           starting at the read_head. If the optional first_item is given, that is presumed to be the first item
           in this node's definition (either 'DEF' or a nodetype like 'Solid') in which case the read_head's last
           match should be this item. Otherwise, the read_head should be situated at the first item to read.
           If destination is a Node, it is presumed to be an existing Node whose VRML definition is being parsed;
           otherwise a new Node will be constructed, and destination will be its parent_field."""
        nodetype = next(self.it) if first_item is None else first_item
        if HIGHLIGHT: highlight_start = self.it.match_start
        DEF = None
        if nodetype == 'DEF':  # handling cases like 'DEF MARVIN Robot { ... }'
            DEF = next(self.it)
            nodetype = next(self.it)

        if isinstance(destination, Node):
            node = destination
        else:
            node = PlannedNode(nodetype, parent_field=destination, DEF=DEF, is_real=self.is_real)

        opening_brace = next(self.it)
        assert(opening_brace == '{')  # TODO could remove; tis here for early warning of bugs
        # fields = { field_name:self.parse_value() for field_name in self.it.until( '}' ) }
        # if DEF is not None: fields['DEF']=DEF
        for field_name in self.it.until('}'):
            # TODO consider whether to remember this field's "hidden" status
            if field_name == 'hidden': field_name = next(self.it)
            # TODO sort out division of labor for assigning parentage to offspring; had been done by parse_value,
            #  but now can also be done by fields.set, so may not need to pass the containing field into parse_value???
            node.fields._create(field_name, self.parse_value())

        if node.is_real: node.__dict__['is_fully_cataloged'] = True
        node.fields._fraction_of_fields_known = 0.5  # all (non-default) node-containing fields have now been found

        if HIGHLIGHT: self.it.highlight_slice(highlight_start, self.it.read_head, color=Console.GREEN_BACK)
        return node

# === Creating merely planned nodes ===

class Plan:
    """The `world.plan` object will belong to this class, which defines __getattr__ and __call__ methods so you can
       create a planned Node with `plan.NodeType(DEF=DEFNAME, fieldname1=fieldvalue1, fieldname2=fieldvalue2, ... )`.
       When constructing planned Nodes in this way, MF field values should be given as [bracketed] lists, and
       vector-like values should be given as Vectors or (parenthesized) tuples, which will be converted to Vectors.
       Planned Nodes may be nested, e.g. with `plan.Shape(geometry=plan.Sphere())`. When nesting a child within
       a planned parent, if the child already has another parent, a new copy of that child will be used instead.
       You may also convert other objects to planned Nodes with `plan(source)`, where source
       may be an existing Node, a VRML-like string, a .wbo filename, or a pathlib.path to such a file.
       Planned Nodes may be imported into the scene tree, e.g with parent_node.children.append(my_planned_node).
       Unlike "real" Nodes whose fields "see through" to corresponding parts of the simulation, the fields of merely
       planned Nodes see only their planned values, and altering these fields alters what planned value will be used
       when this planned Node is next imported."""

    def __getattr__(self, NodeType:str) -> Callable[[Any], Node]:
        def planned_Node_constructor(*args, **kwargs):
            return PlannedNode(NodeType, *args, **kwargs)
        planned_Node_constructor.__name__ = NodeType
        setattr( plan, NodeType, planned_Node_constructor)    # cache this so that future lookups will get it directly
        return planned_Node_constructor

    def __call__( self, source ):
        """Calling plan(source) returns one (or a list of more) new PlannedNode(s) based on the given source,
           which may be a Node, a VRML-like string, a .wbo filename, or a pathlib.path to such a file.
           Each PlannedNode can have attributes for each field of the node (e.g. translation or rotation) potentially
           including a nested tree-like structure of contained nodes.  These attributes can be modified, e.g., by
           setting my_planned_node.translation = (0,0,1) or setting my_planned_node.rotaton.a = 1.57.
           This allows for customization of a PlannedNode before using the world module to import it into the Webots
           scene tree, e.g. with world.ROBOT.children.append( my_planned_node )."""
        if isinstance(source,Node): source = source.export()
        if not isinstance(source, str):  #non-string had better be pathlib.path
            base = source.read_text()  # store the entirety of file as self.base
            name = source.name
        elif source.endswith(".wbo"):  # given string is a filename to import
            name = source
            with open(source, 'r') as file: # open for reading
                base = file.read()     # store the entirety of file as self.base
                # print(f"Loaded {filename} node specs ({len(self.base)} characters)")
        else:
            base = source
            name = 'unnamed'
            # TODO this won't work right
            # name_slice = self.seek(["DEF", self.WORD]) # use defname of object as default name
            # if not name_slice: name_slice = self.seek( self.REST ) # or if that fails, use first line
            # self.name=source[name_slice]
        return VRML_Parser(base).parse()

    # TODO: this can probably be removed after testing; just including so psuedo will have .USE method in console
    USE = staticmethod( USE )

plan = Plan()

