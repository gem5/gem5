# Copyright (c) 2004 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import generators

import os
import re
import sys

#####################################################################
#
# M5 Python Configuration Utility
#
# The basic idea is to write simple Python programs that build Python
# objects corresponding to M5 SimObjects for the deisred simulation
# configuration.  For now, the Python emits a .ini file that can be
# parsed by M5.  In the future, some tighter integration between M5
# and the Python interpreter may allow bypassing the .ini file.
#
# Each SimObject class in M5 is represented by a Python class with the
# same name.  The Python inheritance tree mirrors the M5 C++ tree
# (e.g., SimpleCPU derives from BaseCPU in both cases, and all
# SimObjects inherit from a single SimObject base class).  To specify
# an instance of an M5 SimObject in a configuration, the user simply
# instantiates the corresponding Python object.  The parameters for
# that SimObject are given by assigning to attributes of the Python
# object, either using keyword assignment in the constructor or in
# separate assignment statements.  For example:
#
# cache = BaseCache('my_cache', root, size=64*K)
# cache.hit_latency = 3
# cache.assoc = 8
#
# (The first two constructor arguments specify the name of the created
# cache and its parent node in the hierarchy.)
#
# The magic lies in the mapping of the Python attributes for SimObject
# classes to the actual SimObject parameter specifications.  This
# allows parameter validity checking in the Python code.  Continuing
# the example above, the statements "cache.blurfl=3" or
# "cache.assoc='hello'" would both result in runtime errors in Python,
# since the BaseCache object has no 'blurfl' parameter and the 'assoc'
# parameter requires an integer, respectively.  This magic is done
# primarily by overriding the special __setattr__ method that controls
# assignment to object attributes.
#
# The Python module provides another class, ConfigNode, which is a
# superclass of SimObject.  ConfigNode implements the parent/child
# relationship for building the configuration hierarchy tree.
# Concrete instances of ConfigNode can be used to group objects in the
# hierarchy, but do not correspond to SimObjects themselves (like a
# .ini section with "children=" but no "type=".
#
# Once a set of Python objects have been instantiated in a hierarchy,
# calling 'instantiate(obj)' (where obj is the root of the hierarchy)
# will generate a .ini file.  See simple-4cpu.py for an example
# (corresponding to m5-test/simple-4cpu.ini).
#
#####################################################################

#####################################################################
#
# ConfigNode/SimObject classes
#
# The Python class hierarchy rooted by ConfigNode (which is the base
# class of SimObject, which in turn is the base class of all other M5
# SimObject classes) has special attribute behavior.  In general, an
# object in this hierarchy has three categories of attribute-like
# things:
#
# 1. Regular Python methods and variables.  These must start with an
# underscore to be treated normally.
#
# 2. SimObject parameters.  These values are stored as normal Python
# attributes, but all assignments to these attributes are checked
# against the pre-defined set of parameters stored in the class's
# _param_dict dictionary.  Assignments to attributes that do not
# correspond to predefined parameters, or that are not of the correct
# type, incur runtime errors.
#
# 3. Hierarchy children.  The child nodes of a ConfigNode are stored
# in the node's _children dictionary, but can be accessed using the
# Python attribute dot-notation (just as they are printed out by the
# simulator).  Children cannot be created using attribute assigment;
# they must be added by specifying the parent node in the child's
# constructor or using the '+=' operator.

# The SimObject parameters are the most complex, for a few reasons.
# First, both parameter descriptions and parameter values are
# inherited.  Thus parameter description lookup must go up the
# inheritance chain like normal attribute lookup, but this behavior
# must be explicitly coded since the lookup occurs in each class's
# _param_dict attribute.  Second, because parameter values can be set
# on SimObject classes (to implement default values), the parameter
# checking behavior must be enforced on class attribute assignments as
# well as instance attribute assignments.  Finally, because we allow
# class specialization via inheritance (e.g., see the L1Cache class in
# the simple-4cpu.py example), we must do parameter checking even on
# class instantiation.  To provide all these features, we use a
# metaclass to define most of the SimObject parameter behavior for
# this class hierarchy.
#
#####################################################################

# The metaclass for ConfigNode (and thus for everything that derives
# from ConfigNode, including SimObject).  This class controls how new
# classes that derive from ConfigNode are instantiated, and provides
# inherited class behavior (just like a class controls how instances
# of that class are instantiated, and provides inherited instance
# behavior).
class MetaConfigNode(type):

    # __new__ is called before __init__, and is where the statements
    # in the body of the class definition get loaded into the class's
    # __dict__.  We intercept this to filter out parameter assignments
    # and only allow "private" attributes to be passed to the base
    # __new__ (starting with underscore).
    def __new__(cls, name, bases, dict):
        priv_keys = [k for k in dict.iterkeys() if k.startswith('_')]
        priv_dict = {}
        for k in priv_keys: priv_dict[k] = dict[k]; del dict[k]
        # entries left in dict will get passed to __init__, where we'll
        # deal with them as params.
        return super(MetaConfigNode, cls).__new__(cls, name, bases, priv_dict)

    # initialization: start out with an empty param dict (makes life
    # simpler if we can assume _param_dict is always valid).  Also
    # build inheritance list to simplify searching for inherited
    # params.  Finally set parameters specified in class definition
    # (if any).
    def __init__(cls, name, bases, dict):
        super(MetaConfigNode, cls).__init__(cls, name, bases, {})
        # initialize _param_dict to empty
        cls._param_dict = {}
        # __mro__ is the ordered list of classes Python uses for
        # method resolution.  We want to pick out the ones that have a
        # _param_dict attribute for doing parameter lookups.
        cls._param_bases = \
                         [c for c in cls.__mro__ if hasattr(c, '_param_dict')]
        # initialize attributes with values from class definition
        for (pname, value) in dict.items():
            try:
                setattr(cls, pname, value)
            except Exception, exc:
                print "Error setting '%s' to '%s' on class '%s'\n" \
                      % (pname, value, cls.__name__), exc

    # set the class's parameter dictionary (called when loading
    # class descriptions)
    def set_param_dict(cls, param_dict):
        # should only be called once (current one should be empty one
        # from __init__)
        assert not cls._param_dict
        cls._param_dict = param_dict
        # initialize attributes with default values
        for (pname, param) in param_dict.items():
            try:
                setattr(cls, pname, param.default)
            except Exception, exc:
                print "Error setting '%s' default on class '%s'\n" \
                      % (pname, cls.__name__), exc

    # Set the class's parameter dictionary given a code string of
    # parameter initializers (as from an object description file).
    # Note that the caller must pass in the namespace in which to
    # execute the code (usually the caller's globals()), since if we
    # call globals() from inside this function all we get is this
    # module's internal scope.
    def init_params(cls, init_code, ctx):
        dict = {}
        try:
            exec fixPythonIndentation(init_code) in ctx, dict
        except Exception, exc:
            print "Error in %s.init_params:" % cls.__name__, exc
            raise
        cls.set_param_dict(dict)

    # Lookup a parameter description by name in the given class.  Use
    # the _param_bases list defined in __init__ to go up the
    # inheritance hierarchy if necessary.
    def lookup_param(cls, param_name):
        for c in cls._param_bases:
            param = c._param_dict.get(param_name)
            if param: return param
        return None

    # Set attribute (called on foo.attr_name = value when foo is an
    # instance of class cls).
    def __setattr__(cls, attr_name, value):
        # normal processing for private attributes
        if attr_name.startswith('_'):
            type.__setattr__(cls, attr_name, value)
            return
        # no '_': must be SimObject param
        param = cls.lookup_param(attr_name)
        if not param:
            raise AttributeError, \
                  "Class %s has no parameter %s" % (cls.__name__, attr_name)
        # It's ok: set attribute by delegating to 'object' class.
        # Note the use of param.make_value() to verify/canonicalize
        # the assigned value
        type.__setattr__(cls, attr_name, param.make_value(value))

    # generator that iterates across all parameters for this class and
    # all classes it inherits from
    def all_param_names(cls):
        for c in cls._param_bases:
            for p in c._param_dict.iterkeys():
                yield p

# The ConfigNode class is the root of the special hierarchy.  Most of
# the code in this class deals with the configuration hierarchy itself
# (parent/child node relationships).
class ConfigNode(object):
    # Specify metaclass.  Any class inheriting from ConfigNode will
    # get this metaclass.
    __metaclass__ = MetaConfigNode

    # Constructor.  Since bare ConfigNodes don't have parameters, just
    # worry about the name and the parent/child stuff.
    def __init__(self, _name, _parent=None):
        # Type-check _name
        if type(_name) != str:
            if isinstance(_name, ConfigNode):
                # special case message for common error of trying to
                # coerce a SimObject to the wrong type
                raise TypeError, \
                      "Attempt to coerce %s to %s" \
                      % (_name.__class__.__name__, self.__class__.__name__)
            else:
                raise TypeError, \
                      "%s name must be string (was %s, %s)" \
                      % (self.__class__.__name__, _name, type(_name))
        # if specified, parent must be a subclass of ConfigNode
        if _parent != None and not isinstance(_parent, ConfigNode):
            raise TypeError, \
                  "%s parent must be ConfigNode subclass (was %s, %s)" \
                  % (self.__class__.__name__, _name, type(_name))
        self._name = _name
        self._parent = _parent
        if (_parent):
            _parent._add_child(self)
        self._children = {}
        # keep a list of children in addition to the dictionary keys
        # so we can remember the order they were added and print them
        # out in that order.
        self._child_list = []

    # When printing (e.g. to .ini file), just give the name.
    def __str__(self):
        return self._name

    # Catch attribute accesses that could be requesting children, and
    # satisfy them.  Note that __getattr__ is called only if the
    # regular attribute lookup fails, so private and parameter lookups
    # will already be satisfied before we ever get here.
    def __getattr__(self, name):
        try:
            return self._children[name]
        except KeyError:
            raise AttributeError, \
                  "Node '%s' has no attribute or child '%s'" \
                  % (self._name, name)

    # Set attribute.  All attribute assignments go through here.  Must
    # be private attribute (starts with '_') or valid parameter entry.
    # Basically identical to MetaConfigClass.__setattr__(), except
    # this sets attributes on specific instances rather than on classes.
    def __setattr__(self, attr_name, value):
        if attr_name.startswith('_'):
            object.__setattr__(self, attr_name, value)
            return
        # not private; look up as param
        param = self.__class__.lookup_param(attr_name)
        if not param:
            raise AttributeError, \
                  "Class %s has no parameter %s" \
                  % (self.__class__.__name__, attr_name)
        # It's ok: set attribute by delegating to 'object' class.
        # Note the use of param.make_value() to verify/canonicalize
        # the assigned value.
        v = param.make_value(value)
        object.__setattr__(self, attr_name, v)

        # A little convenient magic: if the parameter is a ConfigNode
        # (or vector of ConfigNodes, or anything else with a
        # '_set_parent_if_none' function attribute) that does not have
        # a parent (and so is not part of the configuration
        # hierarchy), then make this node its parent.
        if hasattr(v, '_set_parent_if_none'):
            v._set_parent_if_none(self)

    def _path(self):
        # Return absolute path from root.
        if not self._parent and self._name != 'Universe':
            print >> sys.stderr, "Warning:", self._name, "has no parent"
        parent_path = self._parent and self._parent._path()
        if parent_path and parent_path != 'Universe':
            return parent_path + '.' + self._name
        else:
            return self._name

    # Add a child to this node.
    def _add_child(self, new_child):
        # set child's parent before calling this function
        assert new_child._parent == self
        if not isinstance(new_child, ConfigNode):
            raise TypeError, \
                  "ConfigNode child must also be of class ConfigNode"
        if new_child._name in self._children:
            raise AttributeError, \
                  "Node '%s' already has a child '%s'" \
                  % (self._name, new_child._name)
        self._children[new_child._name] = new_child
        self._child_list += [new_child]

    # operator overload for '+='.  You can say "node += child" to add
    # a child that was created with parent=None.  An early attempt
    # at playing with syntax; turns out not to be that useful.
    def __iadd__(self, new_child):
        if new_child._parent != None:
            raise AttributeError, \
                  "Node '%s' already has a parent" % new_child._name
        new_child._parent = self
        self._add_child(new_child)
        return self

    # Set this instance's parent to 'parent' if it doesn't already
    # have one.  See ConfigNode.__setattr__().
    def _set_parent_if_none(self, parent):
        if self._parent == None:
            parent += self

    # Print instance info to .ini file.
    def _instantiate(self):
        print '[' + self._path() + ']'	# .ini section header
        if self._child_list:
            # instantiate children in same order they were added for
            # backward compatibility (else we can end up with cpu1
            # before cpu0).
            print 'children =', ' '.join([c._name for c in self._child_list])
        self._instantiateParams()
        print
        # recursively dump out children
        for c in self._child_list:
            c._instantiate()

    # ConfigNodes have no parameters.  Overridden by SimObject.
    def _instantiateParams(self):
        pass

# SimObject is a minimal extension of ConfigNode, implementing a
# hierarchy node that corresponds to an M5 SimObject.  It prints out a
# "type=" line to indicate its SimObject class, prints out the
# assigned parameters corresponding to its class, and allows
# parameters to be set by keyword in the constructor.  Note that most
# of the heavy lifting for the SimObject param handling is done in the
# MetaConfigNode metaclass.

class SimObject(ConfigNode):
    # initialization: like ConfigNode, but handle keyword-based
    # parameter initializers.
    def __init__(self, _name, _parent=None, **params):
        ConfigNode.__init__(self, _name, _parent)
        for param, value in params.items():
            setattr(self, param, value)

    # print type and parameter values to .ini file
    def _instantiateParams(self):
        print "type =", self.__class__._name
        for pname in self.__class__.all_param_names():
            value = getattr(self, pname)
            if value != None:
                print pname, '=', value

    def _sim_code(cls):
        name = cls.__name__
        param_names = cls._param_dict.keys()
        param_names.sort()
        code = "BEGIN_DECLARE_SIM_OBJECT_PARAMS(%s)\n" % name
        decls = ["  " + cls._param_dict[pname].sim_decl(pname) \
                 for pname in param_names]
        code += "\n".join(decls) + "\n"
        code += "END_DECLARE_SIM_OBJECT_PARAMS(%s)\n\n" % name
        code += "BEGIN_INIT_SIM_OBJECT_PARAMS(%s)\n" % name
        inits = ["  " + cls._param_dict[pname].sim_init(pname) \
                 for pname in param_names]
        code += ",\n".join(inits) + "\n"
        code += "END_INIT_SIM_OBJECT_PARAMS(%s)\n\n" % name
        return code
    _sim_code = classmethod(_sim_code)

#####################################################################
#
# Parameter description classes
#
# The _param_dict dictionary in each class maps parameter names to
# either a Param or a VectorParam object.  These objects contain the
# parameter description string, the parameter type, and the default
# value (loaded from the PARAM section of the .odesc files).  The
# make_value() method on these objects is used to force whatever value
# is assigned to the parameter to the appropriate type.
#
# Note that the default values are loaded into the class's attribute
# space when the parameter dictionary is initialized (in
# MetaConfigNode.set_param_dict()); after that point they aren't
# used.
#
#####################################################################

def isNullPointer(value):
    return isinstance(value, NullSimObject)

# Regular parameter.
class Param(object):
    # Constructor.  E.g., Param(Int, "number of widgets", 5)
    def __init__(self, ptype, desc, default=None):
        self.ptype = ptype
        self.ptype_name = self.ptype.__name__
        self.desc = desc
        self.default = default

    # Convert assigned value to appropriate type.  Force parameter
    # value (rhs of '=') to ptype (or None, which means not set).
    def make_value(self, value):
        # nothing to do if None or already correct type.  Also allow NULL
        # pointer to be assigned where a SimObject is expected.
        if value == None or isinstance(value, self.ptype) or \
               isNullPointer(value) and issubclass(self.ptype, ConfigNode):
            return value
        # this type conversion will raise an exception if it's illegal
        return self.ptype(value)

    def sim_decl(self, name):
        return 'Param<%s> %s;' % (self.ptype_name, name)

    def sim_init(self, name):
        if self.default == None:
            return 'INIT_PARAM(%s, "%s")' % (name, self.desc)
        else:
            return 'INIT_PARAM_DFLT(%s, "%s", %s)' % \
                   (name, self.desc, str(self.default))

# The _VectorParamValue class is a wrapper for vector-valued
# parameters.  The leading underscore indicates that users shouldn't
# see this class; it's magically generated by VectorParam.  The
# parameter values are stored in the 'value' field as a Python list of
# whatever type the parameter is supposed to be.  The only purpose of
# storing these instead of a raw Python list is that we can override
# the __str__() method to not print out '[' and ']' in the .ini file.
class _VectorParamValue(object):
    def __init__(self, value):
        assert isinstance(value, list) or value == None
        self.value = value

    def __str__(self):
        return ' '.join(map(str, self.value))

    # Set member instance's parents to 'parent' if they don't already
    # have one.  Extends "magic" parenting of ConfigNodes to vectors
    # of ConfigNodes as well.  See ConfigNode.__setattr__().
    def _set_parent_if_none(self, parent):
        if self.value and hasattr(self.value[0], '_set_parent_if_none'):
            for v in self.value:
                v._set_parent_if_none(parent)

# Vector-valued parameter description.  Just like Param, except that
# the value is a vector (list) of the specified type instead of a
# single value.
class VectorParam(Param):

    # Inherit Param constructor.  However, the resulting parameter
    # will be a list of ptype rather than a single element of ptype.
    def __init__(self, ptype, desc, default=None):
        Param.__init__(self, ptype, desc, default)

    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def make_value(self, value):
        if value == None: return value
        if isinstance(value, list) or isinstance(value, tuple):
            # list: coerce each element into new list
            val_list = [Param.make_value(self, v) for v in iter(value)]
        else:
            # singleton: coerce & wrap in a list
            val_list = [Param.make_value(self, value)]
        # wrap list in _VectorParamValue (see above)
        return _VectorParamValue(val_list)

    def sim_decl(self, name):
        return 'VectorParam<%s> %s;' % (self.ptype_name, name)

    # sim_init inherited from Param

#####################################################################
#
# Parameter Types
#
# Though native Python types could be used to specify parameter types
# (the 'ptype' field of the Param and VectorParam classes), it's more
# flexible to define our own set of types.  This gives us more control
# over how Python expressions are converted to values (via the
# __init__() constructor) and how these values are printed out (via
# the __str__() conversion method).  Eventually we'll need these types
# to correspond to distinct C++ types as well.
#
#####################################################################

# Integer parameter type.
class Int(object):
    # Constructor.  Value must be Python int or long (long integer).
    def __init__(self, value):
        t = type(value)
        if t == int or t == long:
            self.value = value
        else:
            raise TypeError, "Int param got value %s %s" % (repr(value), t)

    # Use Python string conversion.  Note that this puts an 'L' on the
    # end of long integers; we can strip that off here if it gives us
    # trouble.
    def __str__(self):
        return str(self.value)

# Counter, Addr, and Tick are just aliases for Int for now.
class Counter(Int):
    pass

class Addr(Int):
    pass

class Tick(Int):
    pass

# Boolean parameter type.
class Bool(object):

    # Constructor.  Typically the value will be one of the Python bool
    # constants True or False (or the aliases true and false below).
    # Also need to take integer 0 or 1 values since bool was not a
    # distinct type in Python 2.2.  Parse a bunch of boolean-sounding
    # strings too just for kicks.
    def __init__(self, value):
        t = type(value)
        if t == bool:
            self.value = value
        elif t == int or t == long:
            if value == 1:
                self.value = True
            elif value == 0:
                self.value = False
        elif t == str:
            v = value.lower()
            if v == "true" or v == "t" or v == "yes" or v == "y":
                self.value = True
            elif v == "false" or v == "f" or v == "no" or v == "n":
                self.value = False
        # if we didn't set it yet, it must not be something we understand
        if not hasattr(self, 'value'):
            raise TypeError, "Bool param got value %s %s" % (repr(value), t)

    # Generate printable string version.
    def __str__(self):
        if self.value: return "true"
        else: return "false"

# String-valued parameter.
class String(object):
    # Constructor.  Value must be Python string.
    def __init__(self, value):
        t = type(value)
        if t == str:
            self.value = value
        else:
            raise TypeError, "String param got value %s %s" % (repr(value), t)

    # Generate printable string version.  Not too tricky.
    def __str__(self):
        return self.value

# Special class for NULL pointers.  Note the special check in
# make_param_value() above that lets these be assigned where a
# SimObject is required.
class NullSimObject(object):
    # Constructor.  No parameters, nothing to do.
    def __init__(self):
        pass

    def __str__(self):
        return "NULL"

# The only instance you'll ever need...
NULL = NullSimObject()

# Enumerated types are a little more complex.  The user specifies the
# type as Enum(foo) where foo is either a list or dictionary of
# alternatives (typically strings, but not necessarily so).  (In the
# long run, the integer value of the parameter will be the list index
# or the corresponding dictionary value.  For now, since we only check
# that the alternative is valid and then spit it into a .ini file,
# there's not much point in using the dictionary.)

# What Enum() must do is generate a new type encapsulating the
# provided list/dictionary so that specific values of the parameter
# can be instances of that type.  We define two hidden internal
# classes (_ListEnum and _DictEnum) to serve as base classes, then
# derive the new type from the appropriate base class on the fly.


# Base class for list-based Enum types.
class _ListEnum(object):
    # Constructor.  Value must be a member of the type's map list.
    def __init__(self, value):
        if value in self.map:
            self.value = value
            self.index = self.map.index(value)
        else:
            raise TypeError, "Enum param got bad value '%s' (not in %s)" \
                  % (value, self.map)

    # Generate printable string version of value.
    def __str__(self):
        return str(self.value)

class _DictEnum(object):
    # Constructor.  Value must be a key in the type's map dictionary.
    def __init__(self, value):
        if value in self.map:
            self.value = value
            self.index = self.map[value]
        else:
            raise TypeError, "Enum param got bad value '%s' (not in %s)" \
                  % (value, self.map.keys())

    # Generate printable string version of value.
    def __str__(self):
        return str(self.value)

# Enum metaclass... calling Enum(foo) generates a new type (class)
# that derives from _ListEnum or _DictEnum as appropriate.
class Enum(type):
    # counter to generate unique names for generated classes
    counter = 1

    def __new__(cls, map):
        if isinstance(map, dict):
            base = _DictEnum
            keys = map.keys()
        elif isinstance(map, list):
            base = _ListEnum
            keys = map
        else:
            raise TypeError, "Enum map must be list or dict (got %s)" % map
        classname = "Enum%04d" % Enum.counter
        Enum.counter += 1
        # New class derives from selected base, and gets a 'map'
        # attribute containing the specified list or dict.
        return type.__new__(cls, classname, (base,), { 'map': map })


#
# "Constants"... handy aliases for various values.
#

# For compatibility with C++ bool constants.
false = False
true = True

# Some memory range specifications use this as a default upper bound.
MAX_ADDR = 2**64 - 1

# For power-of-two sizing, e.g. 64*K gives an integer value 65536.
K = 1024
M = K*K
G = K*M

#####################################################################

# Munge an arbitrary Python code string to get it to execute (mostly
# dealing with indentation).  Stolen from isa_parser.py... see
# comments there for a more detailed description.
def fixPythonIndentation(s):
    # get rid of blank lines first
    s = re.sub(r'(?m)^\s*\n', '', s);
    if (s != '' and re.match(r'[ \t]', s[0])):
        s = 'if 1:\n' + s
    return s

# Hook to generate C++ parameter code.
def gen_sim_code(file):
    for objname in sim_object_list:
        print >> file, eval("%s._sim_code()" % objname)

# The final hook to generate .ini files.  Called from configuration
# script once config is built.
def instantiate(*objs):
    for obj in objs:
        obj._instantiate()


