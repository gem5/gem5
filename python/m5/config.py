# Copyright (c) 2004-2005 The Regents of The University of Michigan
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
import os, re, sys, types, inspect

import m5
panic = m5.panic
from convert import *
from multidict import multidict

noDot = False
try:
    import pydot
except:
    noDot = True

class Singleton(type):
    def __call__(cls, *args, **kwargs):
        if hasattr(cls, '_instance'):
            return cls._instance

        cls._instance = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instance

#####################################################################
#
# M5 Python Configuration Utility
#
# The basic idea is to write simple Python programs that build Python
# objects corresponding to M5 SimObjects for the desired simulation
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
# cache = BaseCache(size='64KB')
# cache.hit_latency = 3
# cache.assoc = 8
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
# _params dictionary.  Assignments to attributes that do not
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
# _params attribute.  Second, because parameter values can be set
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

def isSimObject(value):
    return isinstance(value, SimObject)

def isSimObjSequence(value):
    if not isinstance(value, (list, tuple)):
        return False

    for val in value:
        if not isNullPointer(val) and not isSimObject(val):
            return False

    return True

def isNullPointer(value):
    return isinstance(value, NullSimObject)

# The metaclass for ConfigNode (and thus for everything that derives
# from ConfigNode, including SimObject).  This class controls how new
# classes that derive from ConfigNode are instantiated, and provides
# inherited class behavior (just like a class controls how instances
# of that class are instantiated, and provides inherited instance
# behavior).
class MetaSimObject(type):
    # Attributes that can be set only at initialization time
    init_keywords = { 'abstract' : types.BooleanType,
                      'type' : types.StringType }
    # Attributes that can be set any time
    keywords = { 'check' : types.FunctionType,
                 'children' : types.ListType }

    # __new__ is called before __init__, and is where the statements
    # in the body of the class definition get loaded into the class's
    # __dict__.  We intercept this to filter out parameter assignments
    # and only allow "private" attributes to be passed to the base
    # __new__ (starting with underscore).
    def __new__(mcls, name, bases, dict):
        # Copy "private" attributes (including special methods such as __new__)
        # to the official dict.  Everything else goes in _init_dict to be
        # filtered in __init__.
        cls_dict = {}
        for key,val in dict.items():
            if key.startswith('_'):
                cls_dict[key] = val
                del dict[key]
        cls_dict['_init_dict'] = dict
        return super(MetaSimObject, mcls).__new__(mcls, name, bases, cls_dict)

    # initialization
    def __init__(cls, name, bases, dict):
        super(MetaSimObject, cls).__init__(name, bases, dict)

        # initialize required attributes
        cls._params = multidict()
        cls._values = multidict()
        cls._anon_subclass_counter = 0

        # We don't support multiple inheritance.  If you want to, you
        # must fix multidict to deal with it properly.
        if len(bases) > 1:
            raise TypeError, "SimObjects do not support multiple inheritance"

        base = bases[0]

        if isinstance(base, MetaSimObject):
            cls._params.parent = base._params
            cls._values.parent = base._values

            # If your parent has a value in it that's a config node, clone
            # it.  Do this now so if we update any of the values'
            # attributes we are updating the clone and not the original.
            for key,val in base._values.iteritems():

                # don't clone if (1) we're about to overwrite it with
                # a local setting or (2) we've already cloned a copy
                # from an earlier (more derived) base
                if cls._init_dict.has_key(key) or cls._values.has_key(key):
                    continue

                if isSimObject(val):
                    cls._values[key] = val()
                elif isSimObjSequence(val) and len(val):
                    cls._values[key] = [ v() for v in val ]

        # now process remaining _init_dict items
        for key,val in cls._init_dict.items():
            if isinstance(val, (types.FunctionType, types.TypeType)):
                type.__setattr__(cls, key, val)

            # param descriptions
            elif isinstance(val, ParamDesc):
                cls._new_param(key, val)

            # init-time-only keywords
            elif cls.init_keywords.has_key(key):
                cls._set_keyword(key, val, cls.init_keywords[key])

            # default: use normal path (ends up in __setattr__)
            else:
                setattr(cls, key, val)

    def _set_keyword(cls, keyword, val, kwtype):
        if not isinstance(val, kwtype):
            raise TypeError, 'keyword %s has bad type %s (expecting %s)' % \
                  (keyword, type(val), kwtype)
        if isinstance(val, types.FunctionType):
            val = classmethod(val)
        type.__setattr__(cls, keyword, val)

    def _new_param(cls, name, value):
        cls._params[name] = value
        if hasattr(value, 'default'):
            setattr(cls, name, value.default)

    # Set attribute (called on foo.attr = value when foo is an
    # instance of class cls).
    def __setattr__(cls, attr, value):
        # normal processing for private attributes
        if attr.startswith('_'):
            type.__setattr__(cls, attr, value)
            return

        if cls.keywords.has_key(attr):
            cls._set_keyword(attr, value, cls.keywords[attr])
            return

        # must be SimObject param
        param = cls._params.get(attr, None)
        if param:
            # It's ok: set attribute by delegating to 'object' class.
            try:
                cls._values[attr] = param.convert(value)
            except Exception, e:
                msg = "%s\nError setting param %s.%s to %s\n" % \
                      (e, cls.__name__, attr, value)
                e.args = (msg, )
                raise
        # I would love to get rid of this
        elif isSimObject(value) or isSimObjSequence(value):
           cls._values[attr] = value
        else:
            raise AttributeError, \
                  "Class %s has no parameter %s" % (cls.__name__, attr)

    def __getattr__(cls, attr):
        if cls._values.has_key(attr):
            return cls._values[attr]

        raise AttributeError, \
              "object '%s' has no attribute '%s'" % (cls.__name__, attr)

# The ConfigNode class is the root of the special hierarchy.  Most of
# the code in this class deals with the configuration hierarchy itself
# (parent/child node relationships).
class SimObject(object):
    # Specify metaclass.  Any class inheriting from SimObject will
    # get this metaclass.
    __metaclass__ = MetaSimObject

    def __init__(self, _value_parent = None, **kwargs):
        self._children = {}
        if _value_parent and type(_value_parent) != type(self):
            # this was called as a type conversion rather than a clone
            raise TypeError, "Cannot convert %s to %s" % \
                  (_value_parent.__class__.__name__, self.__class__.__name__)
        if not _value_parent:
            _value_parent = self.__class__
        # clone values
        self._values = multidict(_value_parent._values)
        for key,val in _value_parent._values.iteritems():
            if isSimObject(val):
                setattr(self, key, val())
            elif isSimObjSequence(val) and len(val):
                setattr(self, key, [ v() for v in val ])
        # apply attribute assignments from keyword args, if any
        for key,val in kwargs.iteritems():
            setattr(self, key, val)

    def __call__(self, **kwargs):
        return self.__class__(_value_parent = self, **kwargs)

    def __getattr__(self, attr):
        if self._values.has_key(attr):
            return self._values[attr]

        raise AttributeError, "object '%s' has no attribute '%s'" \
              % (self.__class__.__name__, attr)

    # Set attribute (called on foo.attr = value when foo is an
    # instance of class cls).
    def __setattr__(self, attr, value):
        # normal processing for private attributes
        if attr.startswith('_'):
            object.__setattr__(self, attr, value)
            return

        # must be SimObject param
        param = self._params.get(attr, None)
        if param:
            # It's ok: set attribute by delegating to 'object' class.
            try:
                value = param.convert(value)
            except Exception, e:
                msg = "%s\nError setting param %s.%s to %s\n" % \
                      (e, self.__class__.__name__, attr, value)
                e.args = (msg, )
                raise
        # I would love to get rid of this
        elif isSimObject(value) or isSimObjSequence(value):
            pass
        else:
            raise AttributeError, "Class %s has no parameter %s" \
                  % (self.__class__.__name__, attr)

        # clear out old child with this name, if any
        self.clear_child(attr)

        if isSimObject(value):
            value.set_path(self, attr)
        elif isSimObjSequence(value):
            value = SimObjVector(value)
            [v.set_path(self, "%s%d" % (attr, i)) for i,v in enumerate(value)]

        self._values[attr] = value

    # this hack allows tacking a '[0]' onto parameters that may or may
    # not be vectors, and always getting the first element (e.g. cpus)
    def __getitem__(self, key):
        if key == 0:
            return self
        raise TypeError, "Non-zero index '%s' to SimObject" % key

    # clear out children with given name, even if it's a vector
    def clear_child(self, name):
        if not self._children.has_key(name):
            return
        child = self._children[name]
        if isinstance(child, SimObjVector):
            for i in xrange(len(child)):
                del self._children["s%d" % (name, i)]
        del self._children[name]

    def add_child(self, name, value):
        self._children[name] = value

    def set_path(self, parent, name):
        if not hasattr(self, '_parent'):
            self._parent = parent
            self._name = name
            parent.add_child(name, self)

    def path(self):
        if not hasattr(self, '_parent'):
            return 'root'
        ppath = self._parent.path()
        if ppath == 'root':
            return self._name
        return ppath + "." + self._name

    def __str__(self):
        return self.path()

    def ini_str(self):
        return self.path()

    def find_any(self, ptype):
        if isinstance(self, ptype):
            return self, True

        found_obj = None
        for child in self._children.itervalues():
            if isinstance(child, ptype):
                if found_obj != None and child != found_obj:
                    raise AttributeError, \
                          'parent.any matched more than one: %s %s' % \
                          (found_obj.path, child.path)
                found_obj = child
        # search param space
        for pname,pdesc in self._params.iteritems():
            if issubclass(pdesc.ptype, ptype):
                match_obj = self._values[pname]
                if found_obj != None and found_obj != match_obj:
                    raise AttributeError, \
                          'parent.any matched more than one: %s' % obj.path
                found_obj = match_obj
        return found_obj, found_obj != None

    def unproxy(self, base):
        return self

    def print_ini(self):
        print '[' + self.path() + ']'	# .ini section header

        if hasattr(self, 'type') and not isinstance(self, ParamContext):
            print 'type=%s' % self.type

        child_names = self._children.keys()
        child_names.sort()
        np_child_names = [c for c in child_names \
                          if not isinstance(self._children[c], ParamContext)]
        if len(np_child_names):
            print 'children=%s' % ' '.join(np_child_names)

        param_names = self._params.keys()
        param_names.sort()
        for param in param_names:
            value = self._values.get(param, None)
            if value != None:
                if isproxy(value):
                    try:
                        value = value.unproxy(self)
                    except:
                        print >> sys.stderr, \
                              "Error in unproxying param '%s' of %s" % \
                              (param, self.path())
                        raise
                    setattr(self, param, value)
                print '%s=%s' % (param, self._values[param].ini_str())

        print	# blank line between objects

        for child in child_names:
            self._children[child].print_ini()

    # generate output file for 'dot' to display as a pretty graph.
    # this code is currently broken.
    def outputDot(self, dot):
        label = "{%s|" % self.path
        if isSimObject(self.realtype):
            label +=  '%s|' % self.type

        if self.children:
            # instantiate children in same order they were added for
            # backward compatibility (else we can end up with cpu1
            # before cpu0).
            for c in self.children:
                dot.add_edge(pydot.Edge(self.path,c.path, style="bold"))

        simobjs = []
        for param in self.params:
            try:
                if param.value is None:
                    raise AttributeError, 'Parameter with no value'

                value = param.value
                string = param.string(value)
            except Exception, e:
                msg = 'exception in %s:%s\n%s' % (self.name, param.name, e)
                e.args = (msg, )
                raise

            if isSimObject(param.ptype) and string != "Null":
                simobjs.append(string)
            else:
                label += '%s = %s\\n' % (param.name, string)

        for so in simobjs:
            label += "|<%s> %s" % (so, so)
            dot.add_edge(pydot.Edge("%s:%s" % (self.path, so), so,
                                    tailport="w"))
        label += '}'
        dot.add_node(pydot.Node(self.path,shape="Mrecord",label=label))

        # recursively dump out children
        for c in self.children:
            c.outputDot(dot)

class ParamContext(SimObject):
    pass

#####################################################################
#
# Proxy object support.
#
#####################################################################

class BaseProxy(object):
    def __init__(self, search_self, search_up):
        self._search_self = search_self
        self._search_up = search_up
        self._multiplier = None

    def __setattr__(self, attr, value):
        if not attr.startswith('_'):
            raise AttributeError, 'cannot set attribute on proxy object'
        super(BaseProxy, self).__setattr__(attr, value)

    # support multiplying proxies by constants
    def __mul__(self, other):
        if not isinstance(other, (int, long, float)):
            raise TypeError, "Proxy multiplier must be integer"
        if self._multiplier == None:
            self._multiplier = other
        else:
            # support chained multipliers
            self._multiplier *= other
        return self

    __rmul__ = __mul__

    def _mulcheck(self, result):
        if self._multiplier == None:
            return result
        return result * self._multiplier

    def unproxy(self, base):
        obj = base
        done = False

        if self._search_self:
            result, done = self.find(obj)

        if self._search_up:
            while not done:
                try: obj = obj._parent
                except: break

                result, done = self.find(obj)

        if not done:
            raise AttributeError, "Can't resolve proxy '%s' from '%s'" % \
                  (self.path(), base.path())

        if isinstance(result, BaseProxy):
            if result == self:
                raise RuntimeError, "Cycle in unproxy"
            result = result.unproxy(obj)

        return self._mulcheck(result)

    def getindex(obj, index):
        if index == None:
            return obj
        try:
            obj = obj[index]
        except TypeError:
            if index != 0:
                raise
            # if index is 0 and item is not subscriptable, just
            # use item itself (so cpu[0] works on uniprocessors)
        return obj
    getindex = staticmethod(getindex)

    def set_param_desc(self, pdesc):
        self._pdesc = pdesc

class AttrProxy(BaseProxy):
    def __init__(self, search_self, search_up, attr):
        super(AttrProxy, self).__init__(search_self, search_up)
        self._attr = attr
        self._modifiers = []

    def __getattr__(self, attr):
        # python uses __bases__ internally for inheritance
        if attr.startswith('_'):
            return super(AttrProxy, self).__getattr__(self, attr)
        if hasattr(self, '_pdesc'):
            raise AttributeError, "Attribute reference on bound proxy"
        self._modifiers.append(attr)
        return self

    # support indexing on proxies (e.g., Self.cpu[0])
    def __getitem__(self, key):
        if not isinstance(key, int):
            raise TypeError, "Proxy object requires integer index"
        self._modifiers.append(key)
        return self

    def find(self, obj):
        try:
            val = getattr(obj, self._attr)
        except:
            return None, False
        while isproxy(val):
            val = val.unproxy(obj)
        for m in self._modifiers:
            if isinstance(m, str):
                val = getattr(val, m)
            elif isinstance(m, int):
                val = val[m]
            else:
                assert("Item must be string or integer")
            while isproxy(val):
                val = val.unproxy(obj)
        return val, True

    def path(self):
        p = self._attr
        for m in self._modifiers:
            if isinstance(m, str):
                p += '.%s' % m
            elif isinstance(m, int):
                p += '[%d]' % m
            else:
                assert("Item must be string or integer")
        return p

class AnyProxy(BaseProxy):
    def find(self, obj):
        return obj.find_any(self._pdesc.ptype)

    def path(self):
        return 'any'

def isproxy(obj):
    if isinstance(obj, (BaseProxy, EthernetAddr)):
        return True
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            if isproxy(v):
                return True
    return False

class ProxyFactory(object):
    def __init__(self, search_self, search_up):
        self.search_self = search_self
        self.search_up = search_up

    def __getattr__(self, attr):
        if attr == 'any':
            return AnyProxy(self.search_self, self.search_up)
        else:
            return AttrProxy(self.search_self, self.search_up, attr)

# global objects for handling proxies
Parent = ProxyFactory(search_self = False, search_up = True)
Self = ProxyFactory(search_self = True, search_up = False)

#####################################################################
#
# Parameter description classes
#
# The _params dictionary in each class maps parameter names to
# either a Param or a VectorParam object.  These objects contain the
# parameter description string, the parameter type, and the default
# value (loaded from the PARAM section of the .odesc files).  The
# _convert() method on these objects is used to force whatever value
# is assigned to the parameter to the appropriate type.
#
# Note that the default values are loaded into the class's attribute
# space when the parameter dictionary is initialized (in
# MetaConfigNode._setparams()); after that point they aren't used.
#
#####################################################################

# Dummy base class to identify types that are legitimate for SimObject
# parameters.
class ParamValue(object):

    # default for printing to .ini file is regular string conversion.
    # will be overridden in some cases
    def ini_str(self):
        return str(self)

    # allows us to blithely call unproxy() on things without checking
    # if they're really proxies or not
    def unproxy(self, base):
        return self

# Regular parameter description.
class ParamDesc(object):
    def __init__(self, ptype_str, ptype, *args, **kwargs):
        self.ptype_str = ptype_str
        # remember ptype only if it is provided
        if ptype != None:
            self.ptype = ptype

        if args:
            if len(args) == 1:
                self.desc = args[0]
            elif len(args) == 2:
                self.default = args[0]
                self.desc = args[1]
            else:
                raise TypeError, 'too many arguments'

        if kwargs.has_key('desc'):
            assert(not hasattr(self, 'desc'))
            self.desc = kwargs['desc']
            del kwargs['desc']

        if kwargs.has_key('default'):
            assert(not hasattr(self, 'default'))
            self.default = kwargs['default']
            del kwargs['default']

        if kwargs:
            raise TypeError, 'extra unknown kwargs %s' % kwargs

        if not hasattr(self, 'desc'):
            raise TypeError, 'desc attribute missing'

    def __getattr__(self, attr):
        if attr == 'ptype':
            try:
                ptype = eval(self.ptype_str, m5.__dict__)
                if not isinstance(ptype, type):
                    panic("Param qualifier is not a type: %s" % self.ptype)
                self.ptype = ptype
                return ptype
            except NameError:
                pass
        raise AttributeError, "'%s' object has no attribute '%s'" % \
              (type(self).__name__, attr)

    def convert(self, value):
        if isinstance(value, BaseProxy):
            value.set_param_desc(self)
            return value
        if not hasattr(self, 'ptype') and isNullPointer(value):
            # deferred evaluation of SimObject; continue to defer if
            # we're just assigning a null pointer
            return value
        if isinstance(value, self.ptype):
            return value
        if isNullPointer(value) and issubclass(self.ptype, SimObject):
            return value
        return self.ptype(value)

# Vector-valued parameter description.  Just like ParamDesc, except
# that the value is a vector (list) of the specified type instead of a
# single value.

class VectorParamValue(list):
    def ini_str(self):
        return ' '.join([v.ini_str() for v in self])

    def unproxy(self, base):
        return [v.unproxy(base) for v in self]

class SimObjVector(VectorParamValue):
    def print_ini(self):
        for v in self:
            v.print_ini()

class VectorParamDesc(ParamDesc):
    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def convert(self, value):
        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            tmp_list = [ ParamDesc.convert(self, v) for v in value ]
            if isSimObjSequence(tmp_list):
                return SimObjVector(tmp_list)
            else:
                return VectorParamValue(tmp_list)
        else:
            # singleton: leave it be (could coerce to a single-element
            # list here, but for some historical reason we don't...
            return ParamDesc.convert(self, value)


class ParamFactory(object):
    def __init__(self, param_desc_class, ptype_str = None):
        self.param_desc_class = param_desc_class
        self.ptype_str = ptype_str

    def __getattr__(self, attr):
        if self.ptype_str:
            attr = self.ptype_str + '.' + attr
        return ParamFactory(self.param_desc_class, attr)

    # E.g., Param.Int(5, "number of widgets")
    def __call__(self, *args, **kwargs):
        caller_frame = inspect.stack()[1][0]
        ptype = None
        try:
            ptype = eval(self.ptype_str,
                         caller_frame.f_globals, caller_frame.f_locals)
            if not isinstance(ptype, type):
                raise TypeError, \
                      "Param qualifier is not a type: %s" % ptype
        except NameError:
            # if name isn't defined yet, assume it's a SimObject, and
            # try to resolve it later
            pass
        return self.param_desc_class(self.ptype_str, ptype, *args, **kwargs)

Param = ParamFactory(ParamDesc)
VectorParam = ParamFactory(VectorParamDesc)

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

# superclass for "numeric" parameter values, to emulate math
# operations in a type-safe way.  e.g., a Latency times an int returns
# a new Latency object.
class NumericParamValue(ParamValue):
    def __str__(self):
        return str(self.value)

    def __float__(self):
        return float(self.value)

    # hook for bounds checking
    def _check(self):
        return

    def __mul__(self, other):
        newobj = self.__class__(self)
        newobj.value *= other
        newobj._check()
        return newobj

    __rmul__ = __mul__

    def __div__(self, other):
        newobj = self.__class__(self)
        newobj.value /= other
        newobj._check()
        return newobj

    def __sub__(self, other):
        newobj = self.__class__(self)
        newobj.value -= other
        newobj._check()
        return newobj

class Range(ParamValue):
    type = int # default; can be overridden in subclasses
    def __init__(self, *args, **kwargs):

        def handle_kwargs(self, kwargs):
            if 'end' in kwargs:
                self.second = self.type(kwargs.pop('end'))
            elif 'size' in kwargs:
                self.second = self.first + self.type(kwargs.pop('size')) - 1
            else:
                raise TypeError, "Either end or size must be specified"

        if len(args) == 0:
            self.first = self.type(kwargs.pop('start'))
            handle_kwargs(self, kwargs)

        elif len(args) == 1:
            if kwargs:
                self.first = self.type(args[0])
                handle_kwargs(self, kwargs)
            elif isinstance(args[0], Range):
                self.first = self.type(args[0].first)
                self.second = self.type(args[0].second)
            else:
                self.first = self.type(0)
                self.second = self.type(args[0]) - 1

        elif len(args) == 2:
            self.first = self.type(args[0])
            self.second = self.type(args[1])
        else:
            raise TypeError, "Too many arguments specified"

        if kwargs:
            raise TypeError, "too many keywords: %s" % kwargs.keys()

    def __str__(self):
        return '%s:%s' % (self.first, self.second)

# Metaclass for bounds-checked integer parameters.  See CheckedInt.
class CheckedIntType(type):
    def __init__(cls, name, bases, dict):
        super(CheckedIntType, cls).__init__(name, bases, dict)

        # CheckedInt is an abstract base class, so we actually don't
        # want to do any processing on it... the rest of this code is
        # just for classes that derive from CheckedInt.
        if name == 'CheckedInt':
            return

        if not (hasattr(cls, 'min') and hasattr(cls, 'max')):
            if not (hasattr(cls, 'size') and hasattr(cls, 'unsigned')):
                panic("CheckedInt subclass %s must define either\n" \
                      "    'min' and 'max' or 'size' and 'unsigned'\n" \
                      % name);
            if cls.unsigned:
                cls.min = 0
                cls.max = 2 ** cls.size - 1
            else:
                cls.min = -(2 ** (cls.size - 1))
                cls.max = (2 ** (cls.size - 1)) - 1

# Abstract superclass for bounds-checked integer parameters.  This
# class is subclassed to generate parameter classes with specific
# bounds.  Initialization of the min and max bounds is done in the
# metaclass CheckedIntType.__init__.
class CheckedInt(NumericParamValue):
    __metaclass__ = CheckedIntType

    def _check(self):
        if not self.min <= self.value <= self.max:
            raise TypeError, 'Integer param out of bounds %d < %d < %d' % \
                  (self.min, self.value, self.max)

    def __init__(self, value):
        if isinstance(value, str):
            self.value = toInteger(value)
        elif isinstance(value, (int, long, float)):
            self.value = long(value)
        self._check()

class Int(CheckedInt):      size = 32; unsigned = False
class Unsigned(CheckedInt): size = 32; unsigned = True

class Int8(CheckedInt):     size =  8; unsigned = False
class UInt8(CheckedInt):    size =  8; unsigned = True
class Int16(CheckedInt):    size = 16; unsigned = False
class UInt16(CheckedInt):   size = 16; unsigned = True
class Int32(CheckedInt):    size = 32; unsigned = False
class UInt32(CheckedInt):   size = 32; unsigned = True
class Int64(CheckedInt):    size = 64; unsigned = False
class UInt64(CheckedInt):   size = 64; unsigned = True

class Counter(CheckedInt):  size = 64; unsigned = True
class Tick(CheckedInt):     size = 64; unsigned = True
class TcpPort(CheckedInt):  size = 16; unsigned = True
class UdpPort(CheckedInt):  size = 16; unsigned = True

class Percent(CheckedInt):  min = 0; max = 100

class Float(ParamValue, float):
    pass

class MemorySize(CheckedInt):
    size = 64
    unsigned = True
    def __init__(self, value):
        if isinstance(value, MemorySize):
            self.value = value.value
        else:
            self.value = toMemorySize(value)
        self._check()

class MemorySize32(CheckedInt):
    size = 32
    unsigned = True
    def __init__(self, value):
        if isinstance(value, MemorySize):
            self.value = value.value
        else:
            self.value = toMemorySize(value)
        self._check()

class Addr(CheckedInt):
    size = 64
    unsigned = True
    def __init__(self, value):
        if isinstance(value, Addr):
            self.value = value.value
        else:
            try:
                self.value = toMemorySize(value)
            except TypeError:
                self.value = long(value)
        self._check()

class AddrRange(Range):
    type = Addr

# String-valued parameter.  Just mixin the ParamValue class
# with the built-in str class.
class String(ParamValue,str):
    pass

# Boolean parameter type.  Python doesn't let you subclass bool, since
# it doesn't want to let you create multiple instances of True and
# False.  Thus this is a little more complicated than String.
class Bool(ParamValue):
    def __init__(self, value):
        try:
            self.value = toBool(value)
        except TypeError:
            self.value = bool(value)

    def __str__(self):
        return str(self.value)

    def ini_str(self):
        if self.value:
            return 'true'
        return 'false'

def IncEthernetAddr(addr, val = 1):
    bytes = map(lambda x: int(x, 16), addr.split(':'))
    bytes[5] += val
    for i in (5, 4, 3, 2, 1):
        val,rem = divmod(bytes[i], 256)
        bytes[i] = rem
        if val == 0:
            break
        bytes[i - 1] += val
    assert(bytes[0] <= 255)
    return ':'.join(map(lambda x: '%02x' % x, bytes))

class NextEthernetAddr(object):
    addr = "00:90:00:00:00:01"

    def __init__(self, inc = 1):
        self.value = NextEthernetAddr.addr
        NextEthernetAddr.addr = IncEthernetAddr(NextEthernetAddr.addr, inc)

class EthernetAddr(ParamValue):
    def __init__(self, value):
        if value == NextEthernetAddr:
            self.value = value
            return

        if not isinstance(value, str):
            raise TypeError, "expected an ethernet address and didn't get one"

        bytes = value.split(':')
        if len(bytes) != 6:
            raise TypeError, 'invalid ethernet address %s' % value

        for byte in bytes:
            if not 0 <= int(byte) <= 256:
                raise TypeError, 'invalid ethernet address %s' % value

        self.value = value

    def unproxy(self, base):
        if self.value == NextEthernetAddr:
            self.addr = self.value().value
        return self

    def __str__(self):
        if self.value == NextEthernetAddr:
            return self.addr
        else:
            return self.value

# Special class for NULL pointers.  Note the special check in
# make_param_value() above that lets these be assigned where a
# SimObject is required.
# only one copy of a particular node
class NullSimObject(object):
    __metaclass__ = Singleton

    def __call__(cls):
        return cls

    def _instantiate(self, parent = None, path = ''):
        pass

    def ini_str(self):
        return 'Null'

    def unproxy(self, base):
        return self

    def set_path(self, parent, name):
        pass
    def __str__(self):
        return 'Null'

# The only instance you'll ever need...
Null = NULL = NullSimObject()

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


# Metaclass for Enum types
class MetaEnum(type):
    def __init__(cls, name, bases, init_dict):
        if init_dict.has_key('map'):
            if not isinstance(cls.map, dict):
                raise TypeError, "Enum-derived class attribute 'map' " \
                      "must be of type dict"
            # build list of value strings from map
            cls.vals = cls.map.keys()
            cls.vals.sort()
        elif init_dict.has_key('vals'):
            if not isinstance(cls.vals, list):
                raise TypeError, "Enum-derived class attribute 'vals' " \
                      "must be of type list"
            # build string->value map from vals sequence
            cls.map = {}
            for idx,val in enumerate(cls.vals):
                cls.map[val] = idx
        else:
            raise TypeError, "Enum-derived class must define "\
                  "attribute 'map' or 'vals'"

        super(MetaEnum, cls).__init__(name, bases, init_dict)

    def cpp_declare(cls):
        s = 'enum %s {\n    ' % cls.__name__
        s += ',\n    '.join(['%s = %d' % (v,cls.map[v]) for v in cls.vals])
        s += '\n};\n'
        return s

# Base class for enum types.
class Enum(ParamValue):
    __metaclass__ = MetaEnum
    vals = []

    def __init__(self, value):
        if value not in self.map:
            raise TypeError, "Enum param got bad value '%s' (not in %s)" \
                  % (value, self.vals)
        self.value = value

    def __str__(self):
        return self.value

ticks_per_sec = None

# how big does a rounding error need to be before we warn about it?
frequency_tolerance = 0.001  # 0.1%

# convert a floting-point # of ticks to integer, and warn if rounding
# discards too much precision
def tick_check(float_ticks):
    if float_ticks == 0:
        return 0
    int_ticks = int(round(float_ticks))
    err = (float_ticks - int_ticks) / float_ticks
    if err > frequency_tolerance:
        print >> sys.stderr, "Warning: rounding error > tolerance"
        print >> sys.stderr, "    %f rounded to %d" % (float_ticks, int_ticks)
        #raise ValueError
    return int_ticks

def getLatency(value):
    if isinstance(value, Latency) or isinstance(value, Clock):
        return value.value
    elif isinstance(value, Frequency) or isinstance(value, RootClock):
        return 1 / value.value
    elif isinstance(value, str):
        try:
            return toLatency(value)
        except ValueError:
            try:
                return 1 / toFrequency(value)
            except ValueError:
                pass # fall through
    raise ValueError, "Invalid Frequency/Latency value '%s'" % value


class Latency(NumericParamValue):
    def __init__(self, value):
        self.value = getLatency(value)

    def __getattr__(self, attr):
        if attr in ('latency', 'period'):
            return self
        if attr == 'frequency':
            return Frequency(self)
        raise AttributeError, "Latency object has no attribute '%s'" % attr

    # convert latency to ticks
    def ini_str(self):
        return str(tick_check(self.value * ticks_per_sec))

class Frequency(NumericParamValue):
    def __init__(self, value):
        self.value = 1 / getLatency(value)

    def __getattr__(self, attr):
        if attr == 'frequency':
            return self
        if attr in ('latency', 'period'):
            return Latency(self)
        raise AttributeError, "Frequency object has no attribute '%s'" % attr

    # convert frequency to ticks per period
    def ini_str(self):
        return self.period.ini_str()

# Just like Frequency, except ini_str() is absolute # of ticks per sec (Hz).
# We can't inherit from Frequency because we don't want it to be directly
# assignable to a regular Frequency parameter.
class RootClock(ParamValue):
    def __init__(self, value):
        self.value = 1 / getLatency(value)

    def __getattr__(self, attr):
        if attr == 'frequency':
            return Frequency(self)
        if attr in ('latency', 'period'):
            return Latency(self)
        raise AttributeError, "Frequency object has no attribute '%s'" % attr

    def ini_str(self):
        return str(tick_check(self.value))

# A generic frequency and/or Latency value.  Value is stored as a latency,
# but to avoid ambiguity this object does not support numeric ops (* or /).
# An explicit conversion to a Latency or Frequency must be made first.
class Clock(NumericParamValue):
    def __init__(self, value):
        self.value = getLatency(value)

    def __getattr__(self, attr):
        if attr == 'frequency':
            return Frequency(self)
        if attr in ('latency', 'period'):
            return Latency(self)
        raise AttributeError, "Frequency object has no attribute '%s'" % attr

    def ini_str(self):
        return self.period.ini_str()

class NetworkBandwidth(float,ParamValue):
    def __new__(cls, value):
        val = toNetworkBandwidth(value) / 8.0
        return super(cls, NetworkBandwidth).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def ini_str(self):
        return '%f' % (ticks_per_sec / float(self))

class MemoryBandwidth(float,ParamValue):
    def __new__(self, value):
        val = toMemoryBandwidth(value)
        return super(cls, MemoryBandwidth).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def ini_str(self):
        return '%f' % (ticks_per_sec / float(self))

#
# "Constants"... handy aliases for various values.
#

# Some memory range specifications use this as a default upper bound.
MaxAddr = Addr.max
MaxTick = Tick.max
AllMemory = AddrRange(0, MaxAddr)

#####################################################################

# The final hook to generate .ini files.  Called from configuration
# script once config is built.
def instantiate(root):
    global ticks_per_sec
    ticks_per_sec = float(root.clock.frequency)
    root.print_ini()
    noDot = True # temporary until we fix dot
    if not noDot:
       dot = pydot.Dot()
       instance.outputDot(dot)
       dot.orientation = "portrait"
       dot.size = "8.5,11"
       dot.ranksep="equally"
       dot.rank="samerank"
       dot.write("config.dot")
       dot.write_ps("config.ps")

# __all__ defines the list of symbols that get exported when
# 'from config import *' is invoked.  Try to keep this reasonably
# short to avoid polluting other namespaces.
__all__ = ['SimObject', 'ParamContext', 'Param', 'VectorParam',
           'Parent', 'Self',
           'Enum', 'Bool', 'String', 'Float',
           'Int', 'Unsigned', 'Int8', 'UInt8', 'Int16', 'UInt16',
           'Int32', 'UInt32', 'Int64', 'UInt64',
           'Counter', 'Addr', 'Tick', 'Percent',
           'TcpPort', 'UdpPort', 'EthernetAddr',
           'MemorySize', 'MemorySize32',
           'Latency', 'Frequency', 'RootClock', 'Clock',
           'NetworkBandwidth', 'MemoryBandwidth',
           'Range', 'AddrRange', 'MaxAddr', 'MaxTick', 'AllMemory',
           'Null', 'NULL',
           'NextEthernetAddr', 'instantiate']

