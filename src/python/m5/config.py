# Copyright (c) 2004-2006 The Regents of The University of Michigan
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
#
# Authors: Steve Reinhardt
#          Nathan Binkert

import os, re, sys, types, inspect, copy

import m5
from m5 import panic, cc_main
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
# will generate a .ini file.
#
#####################################################################

# dict to look up SimObjects based on path
instanceDict = {}

#############################
#
# Utility methods
#
#############################

def isSimObject(value):
    return isinstance(value, SimObject)

def isSimObjectSequence(value):
    if not isinstance(value, (list, tuple)) or len(value) == 0:
        return False

    for val in value:
        if not isNullPointer(val) and not isSimObject(val):
            return False

    return True

def isSimObjectOrSequence(value):
    return isSimObject(value) or isSimObjectSequence(value)

def isNullPointer(value):
    return isinstance(value, NullSimObject)

# Apply method to object.
# applyMethod(obj, 'meth', <args>) is equivalent to obj.meth(<args>)
def applyMethod(obj, meth, *args, **kwargs):
    return getattr(obj, meth)(*args, **kwargs)

# If the first argument is an (non-sequence) object, apply the named
# method with the given arguments.  If the first argument is a
# sequence, apply the method to each element of the sequence (a la
# 'map').
def applyOrMap(objOrSeq, meth, *args, **kwargs):
    if not isinstance(objOrSeq, (list, tuple)):
        return applyMethod(objOrSeq, meth, *args, **kwargs)
    else:
        return [applyMethod(o, meth, *args, **kwargs) for o in objOrSeq]


# The metaclass for SimObject.  This class controls how new classes
# that derive from SimObject are instantiated, and provides inherited
# class behavior (just like a class controls how instances of that
# class are instantiated, and provides inherited instance behavior).
class MetaSimObject(type):
    # Attributes that can be set only at initialization time
    init_keywords = { 'abstract' : types.BooleanType,
                      'type' : types.StringType }
    # Attributes that can be set any time
    keywords = { 'check' : types.FunctionType }

    # __new__ is called before __init__, and is where the statements
    # in the body of the class definition get loaded into the class's
    # __dict__.  We intercept this to filter out parameter & port assignments
    # and only allow "private" attributes to be passed to the base
    # __new__ (starting with underscore).
    def __new__(mcls, name, bases, dict):
        # Copy "private" attributes, functions, and classes to the
        # official dict.  Everything else goes in _init_dict to be
        # filtered in __init__.
        cls_dict = {}
        value_dict = {}
        for key,val in dict.items():
            if key.startswith('_') or isinstance(val, (types.FunctionType,
                                                       types.TypeType)):
                cls_dict[key] = val
            else:
                # must be a param/port setting
                value_dict[key] = val
        cls_dict['_value_dict'] = value_dict
        return super(MetaSimObject, mcls).__new__(mcls, name, bases, cls_dict)

    # subclass initialization
    def __init__(cls, name, bases, dict):
        # calls type.__init__()... I think that's a no-op, but leave
        # it here just in case it's not.
        super(MetaSimObject, cls).__init__(name, bases, dict)

        # initialize required attributes

        # class-only attributes
        cls._params = multidict() # param descriptions
        cls._ports = multidict()  # port descriptions

        # class or instance attributes
        cls._values = multidict()   # param values
        cls._port_map = multidict() # port bindings
        cls._instantiated = False # really instantiated, cloned, or subclassed

        # We don't support multiple inheritance.  If you want to, you
        # must fix multidict to deal with it properly.
        if len(bases) > 1:
            raise TypeError, "SimObjects do not support multiple inheritance"

        base = bases[0]

        # Set up general inheritance via multidicts.  A subclass will
        # inherit all its settings from the base class.  The only time
        # the following is not true is when we define the SimObject
        # class itself (in which case the multidicts have no parent).
        if isinstance(base, MetaSimObject):
            cls._params.parent = base._params
            cls._ports.parent = base._ports
            cls._values.parent = base._values
            cls._port_map.parent = base._port_map
            # mark base as having been subclassed
            base._instantiated = True

        # Now process the _value_dict items.  They could be defining
        # new (or overriding existing) parameters or ports, setting
        # class keywords (e.g., 'abstract'), or setting parameter
        # values or port bindings.  The first 3 can only be set when
        # the class is defined, so we handle them here.  The others
        # can be set later too, so just emulate that by calling
        # setattr().
        for key,val in cls._value_dict.items():
            # param descriptions
            if isinstance(val, ParamDesc):
                cls._new_param(key, val)

            # port objects
            elif isinstance(val, Port):
                cls._ports[key] = val

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

        if cls._ports.has_key(attr):
            self._ports[attr].connect(self, attr, value)
            return

        if isSimObjectOrSequence(value) and cls._instantiated:
            raise RuntimeError, \
                  "cannot set SimObject parameter '%s' after\n" \
                  "    class %s has been instantiated or subclassed" \
                  % (attr, cls.__name__)

        # check for param
        param = cls._params.get(attr, None)
        if param:
            try:
                cls._values[attr] = param.convert(value)
            except Exception, e:
                msg = "%s\nError setting param %s.%s to %s\n" % \
                      (e, cls.__name__, attr, value)
                e.args = (msg, )
                raise
        elif isSimObjectOrSequence(value):
            # if RHS is a SimObject, it's an implicit child assignment
            cls._values[attr] = value
        else:
            raise AttributeError, \
                  "Class %s has no parameter \'%s\'" % (cls.__name__, attr)

    def __getattr__(cls, attr):
        if cls._values.has_key(attr):
            return cls._values[attr]

        raise AttributeError, \
              "object '%s' has no attribute '%s'" % (cls.__name__, attr)

# The SimObject class is the root of the special hierarchy.  Most of
# the code in this class deals with the configuration hierarchy itself
# (parent/child node relationships).
class SimObject(object):
    # Specify metaclass.  Any class inheriting from SimObject will
    # get this metaclass.
    __metaclass__ = MetaSimObject

    # Initialize new instance.  For objects with SimObject-valued
    # children, we need to recursively clone the classes represented
    # by those param values as well in a consistent "deep copy"-style
    # fashion.  That is, we want to make sure that each instance is
    # cloned only once, and that if there are multiple references to
    # the same original object, we end up with the corresponding
    # cloned references all pointing to the same cloned instance.
    def __init__(self, **kwargs):
        ancestor = kwargs.get('_ancestor')
        memo_dict = kwargs.get('_memo')
        if memo_dict is None:
            # prepare to memoize any recursively instantiated objects
            memo_dict = {}
        elif ancestor:
            # memoize me now to avoid problems with recursive calls
            memo_dict[ancestor] = self

        if not ancestor:
            ancestor = self.__class__
        ancestor._instantiated = True

        # initialize required attributes
        self._parent = None
        self._children = {}
        self._ccObject = None  # pointer to C++ object
        self._instantiated = False # really "cloned"

        # Inherit parameter values from class using multidict so
        # individual value settings can be overridden.
        self._values = multidict(ancestor._values)
        # clone SimObject-valued parameters
        for key,val in ancestor._values.iteritems():
            if isSimObject(val):
                setattr(self, key, val(_memo=memo_dict))
            elif isSimObjectSequence(val) and len(val):
                setattr(self, key, [ v(_memo=memo_dict) for v in val ])
        # clone port references.  no need to use a multidict here
        # since we will be creating new references for all ports.
        self._port_map = {}
        for key,val in ancestor._port_map.iteritems():
            self._port_map[key] = applyOrMap(val, 'clone', memo_dict)
        # apply attribute assignments from keyword args, if any
        for key,val in kwargs.iteritems():
            setattr(self, key, val)

    # "Clone" the current instance by creating another instance of
    # this instance's class, but that inherits its parameter values
    # and port mappings from the current instance.  If we're in a
    # "deep copy" recursive clone, check the _memo dict to see if
    # we've already cloned this instance.
    def __call__(self, **kwargs):
        memo_dict = kwargs.get('_memo')
        if memo_dict is None:
            # no memo_dict: must be top-level clone operation.
            # this is only allowed at the root of a hierarchy
            if self._parent:
                raise RuntimeError, "attempt to clone object %s " \
                      "not at the root of a tree (parent = %s)" \
                      % (self, self._parent)
            # create a new dict and use that.
            memo_dict = {}
            kwargs['_memo'] = memo_dict
        elif memo_dict.has_key(self):
            # clone already done & memoized
            return memo_dict[self]
        return self.__class__(_ancestor = self, **kwargs)

    def __getattr__(self, attr):
        if self._ports.has_key(attr):
            # return reference that can be assigned to another port
            # via __setattr__
            return self._ports[attr].makeRef(self, attr)

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

        if self._ports.has_key(attr):
            # set up port connection
            self._ports[attr].connect(self, attr, value)
            return

        if isSimObjectOrSequence(value) and self._instantiated:
            raise RuntimeError, \
                  "cannot set SimObject parameter '%s' after\n" \
                  "    instance been cloned %s" % (attr, `self`)

        # must be SimObject param
        param = self._params.get(attr, None)
        if param:
            try:
                value = param.convert(value)
            except Exception, e:
                msg = "%s\nError setting param %s.%s to %s\n" % \
                      (e, self.__class__.__name__, attr, value)
                e.args = (msg, )
                raise
        elif isSimObjectOrSequence(value):
            pass
        else:
            raise AttributeError, "Class %s has no parameter %s" \
                  % (self.__class__.__name__, attr)

        # clear out old child with this name, if any
        self.clear_child(attr)

        if isSimObject(value):
            value.set_path(self, attr)
        elif isSimObjectSequence(value):
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
        if not self._parent:
            self._parent = parent
            self._name = name
            parent.add_child(name, self)

    def path(self):
        if not self._parent:
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

        instanceDict[self.path()] = self

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

    # Call C++ to create C++ object corresponding to this object and
    # (recursively) all its children
    def createCCObject(self):
        self.getCCObject() # force creation
        for child in self._children.itervalues():
            child.createCCObject()

    # Get C++ object corresponding to this object, calling C++ if
    # necessary to construct it.  Does *not* recursively create
    # children.
    def getCCObject(self):
        if not self._ccObject:
            self._ccObject = -1 # flag to catch cycles in recursion
            self._ccObject = cc_main.createSimObject(self.path())
        elif self._ccObject == -1:
            raise RuntimeError, "%s: recursive call to getCCObject()" \
                  % self.path()
        return self._ccObject

    # Create C++ port connections corresponding to the connections in
    # _port_map (& recursively for all children)
    def connectPorts(self):
        for portRef in self._port_map.itervalues():
            applyOrMap(portRef, 'ccConnect')
        for child in self._children.itervalues():
            child.connectPorts()

    def startQuiesce(self, quiesce_event, recursive):
        count = 0
        # ParamContexts don't serialize
        if isinstance(self, SimObject) and not isinstance(self, ParamContext):
            if self._ccObject.quiesce(quiesce_event):
                count = 1
        if recursive:
            for child in self._children.itervalues():
                count += child.startQuiesce(quiesce_event, True)
        return count

    def resume(self):
        if isinstance(self, SimObject) and not isinstance(self, ParamContext):
            self._ccObject.resume()
        for child in self._children.itervalues():
            child.resume()

    def changeTiming(self, mode):
        if isinstance(self, SimObject) and not isinstance(self, ParamContext):
            self._ccObject.setMemoryMode(mode)
        for child in self._children.itervalues():
            child.changeTiming(mode)

    def takeOverFrom(self, old_cpu):
        cpu_ptr = cc_main.convertToBaseCPUPtr(old_cpu._ccObject)
        self._ccObject.takeOverFrom(cpu_ptr)

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
                obj = obj._parent
                if not obj:
                    break
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
# The _params dictionary in each class maps parameter names to either
# a Param or a VectorParam object.  These objects contain the
# parameter description string, the parameter type, and the default
# value (if any).  The convert() method on these objects is used to
# force whatever value is assigned to the parameter to the appropriate
# type.
#
# Note that the default values are loaded into the class's attribute
# space when the parameter dictionary is initialized (in
# MetaSimObject._new_param()); after that point they aren't used.
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
                ptype = eval(self.ptype_str, m5.objects.__dict__)
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
            if isSimObjectSequence(tmp_list):
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
        caller_frame = inspect.currentframe().f_back
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
            if hasattr(self, 'addr'):
                return self.addr
            else:
                return "NextEthernetAddr (unresolved)"
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
class Clock(ParamValue):
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
#
# Port objects
#
# Ports are used to interconnect objects in the memory system.
#
#####################################################################

# Port reference: encapsulates a reference to a particular port on a
# particular SimObject.
class PortRef(object):
    def __init__(self, simobj, name, isVec):
        assert(isSimObject(simobj))
        self.simobj = simobj
        self.name = name
        self.index = -1
        self.isVec = isVec # is this a vector port?
        self.peer = None   # not associated with another port yet
        self.ccConnected = False # C++ port connection done?

    # Set peer port reference.  Called via __setattr__ as a result of
    # a port assignment, e.g., "obj1.port1 = obj2.port2".
    def setPeer(self, other):
        if self.isVec:
            curMap = self.simobj._port_map.get(self.name, [])
            self.index = len(curMap)
            curMap.append(other)
        else:
            curMap = self.simobj._port_map.get(self.name)
            if curMap and not self.isVec:
                print "warning: overwriting port", self.simobj, self.name
            curMap = other
        self.simobj._port_map[self.name] = curMap
        self.peer = other

    def clone(self, memo):
        newRef = copy.copy(self)
        assert(isSimObject(newRef.simobj))
        newRef.simobj = newRef.simobj(_memo=memo)
        # Tricky: if I'm the *second* PortRef in the pair to be
        # cloned, then my peer is still in the middle of its clone
        # method, and thus hasn't returned to its owner's
        # SimObject.__init__ to get installed in _port_map.  As a
        # result I have no way of finding the *new* peer object.  So I
        # mark myself as "waiting" for my peer, and I let the *first*
        # PortRef clone call set up both peer pointers after I return.
        newPeer = newRef.simobj._port_map.get(self.name)
        if newPeer:
            if self.isVec:
                assert(self.index != -1)
                newPeer = newPeer[self.index]
            # other guy is all set up except for his peer pointer
            assert(newPeer.peer == -1) # peer must be waiting for handshake
            newPeer.peer = newRef
            newRef.peer = newPeer
        else:
            # other guy is in clone; just wait for him to do the work
            newRef.peer = -1 # mark as waiting for handshake
        return newRef

    # Call C++ to create corresponding port connection between C++ objects
    def ccConnect(self):
        if self.ccConnected: # already done this
            return
        peer = self.peer
        cc_main.connectPorts(self.simobj.getCCObject(), self.name, self.index,
                             peer.simobj.getCCObject(), peer.name, peer.index)
        self.ccConnected = True
        peer.ccConnected = True

# Port description object.  Like a ParamDesc object, this represents a
# logical port in the SimObject class, not a particular port on a
# SimObject instance.  The latter are represented by PortRef objects.
class Port(object):
    def __init__(self, desc):
        self.desc = desc
        self.isVec = False

    # Generate a PortRef for this port on the given SimObject with the
    # given name
    def makeRef(self, simobj, name):
        return PortRef(simobj, name, self.isVec)

    # Connect an instance of this port (on the given SimObject with
    # the given name) with the port described by the supplied PortRef
    def connect(self, simobj, name, ref):
        if not isinstance(ref, PortRef):
            raise TypeError, \
                  "assigning non-port reference port '%s'" % name
        myRef = self.makeRef(simobj, name)
        myRef.setPeer(ref)
        ref.setPeer(myRef)

# VectorPort description object.  Like Port, but represents a vector
# of connections (e.g., as on a Bus).
class VectorPort(Port):
    def __init__(self, desc):
        Port.__init__(self, desc)
        self.isVec = True

#####################################################################

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
           'NextEthernetAddr',
           'Port', 'VectorPort']

