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
import os, re, sys, types
noDot = False
try:
    import pydot
except:
    noDot = True

env = {}
env.update(os.environ)

def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

def issequence(value):
    return isinstance(value, tuple) or isinstance(value, list)

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

class Proxy(object):
    def __init__(self, path = ()):
        self._object = None
        self._path = path

    def __getattr__(self, attr):
        return Proxy(self._path + (attr, ))

    def __setattr__(self, attr, value):
        if not attr.startswith('_'):
            raise AttributeError, 'cannot set attribute %s' % attr
        super(Proxy, self).__setattr__(attr, value)

    def _convert(self):
        obj = self._object
        for attr in self._path:
            obj = obj.__getattribute__(attr)
        return obj

Super = Proxy()

def isSubClass(value, cls):
    try:
        return issubclass(value, cls)
    except:
        return False

def isParam(self):
    return isinstance(self, _Param)

def isConfigNode(value):
    try:
        return issubclass(value, ConfigNode)
    except:
        return False

def isSimObject(value):
    try:
        return issubclass(value, SimObject)
    except:
        return False

def isSimObjSequence(value):
    if not issequence(value):
        return False

    for val in value:
        if not isNullPointer(val) and not isConfigNode(val):
            return False

    return True

def isParamContext(value):
    try:
        return issubclass(value, ParamContext)
    except:
        return False


class_decorator = '_M5M5_SIMOBJECT_'
expr_decorator = '_M5M5_EXPRESSION_'
dot_decorator = '_M5M5_DOT_'

# The metaclass for ConfigNode (and thus for everything that derives
# from ConfigNode, including SimObject).  This class controls how new
# classes that derive from ConfigNode are instantiated, and provides
# inherited class behavior (just like a class controls how instances
# of that class are instantiated, and provides inherited instance
# behavior).
class MetaConfigNode(type):
    keywords = { 'abstract' : types.BooleanType,
                 'check' : types.FunctionType,
                 'type' : (types.NoneType, types.StringType) }

    # __new__ is called before __init__, and is where the statements
    # in the body of the class definition get loaded into the class's
    # __dict__.  We intercept this to filter out parameter assignments
    # and only allow "private" attributes to be passed to the base
    # __new__ (starting with underscore).
    def __new__(mcls, name, bases, dict):
        priv = { 'abstract' : False,
                 # initialize _params and _values dicts to empty
                 '_params' : {},
                 '_values' : {},
                 '_disable' : {} }

        for key,val in dict.items():
            del dict[key]

            # See description of decorators in the importer.py file
            # We just strip off the expr_decorator now since we don't
            # need from this point on.
            if key.startswith(expr_decorator):
                key = key[len(expr_decorator):]

            if mcls.keywords.has_key(key):
                if not isinstance(val, mcls.keywords[key]):
                    raise TypeError, \
                          'keyword %s has the wrong type %s should be %s' % \
                          (key, type(val), mcls.keywords[key])

                if isinstance(val, types.FunctionType):
                    val = classmethod(val)
                priv[key] = val

            elif key.startswith('_'):
                priv[key] = val

            elif not isNullPointer(val) and isConfigNode(val):
                dict[key] = val()

            elif isSimObjSequence(val):
                dict[key] = [ v() for v in val ]

            else:
                dict[key] = val

        # If your parent has a value in it that's a config node, clone it.
        for base in bases:
            if not isConfigNode(base):
                continue

            for key,value in base._values.iteritems():
                if dict.has_key(key):
                    continue

                if isConfigNode(value):
                    priv['_values'][key] = value()
                elif isSimObjSequence(value):
                    priv['_values'][key] = [ val() for val in value ]

        # entries left in dict will get passed to __init__, where we'll
        # deal with them as params.
        return super(MetaConfigNode, mcls).__new__(mcls, name, bases, priv)

    # initialization
    def __init__(cls, name, bases, dict):
        super(MetaConfigNode, cls).__init__(cls, name, bases, {})

        cls._bases = [c for c in cls.__mro__ if isConfigNode(c)]

        # initialize attributes with values from class definition
        for key,value in dict.iteritems():
            # turn an expression that was munged in the importer
            # because it had dots into a list so that we can find the
            # proper variable to modify.
            key = key.split(dot_decorator)
            c = cls
            for item in key[:-1]:
                c = getattr(c, item)
            setattr(c, key[-1], value)

    def _isvalue(cls, name):
        for c in cls._bases:
            if c._params.has_key(name):
                return True

        for c in cls._bases:
            if c._values.has_key(name):
                return True

        return False

    # generator that iterates across all parameters for this class and
    # all classes it inherits from
    def _getparams(cls):
        params = {}
        for c in cls._bases:
            for p,v in c._params.iteritems():
                if not params.has_key(p):
                    params[p] = v
        return params

    # Lookup a parameter description by name in the given class.
    def _getparam(cls, name, default = AttributeError):
        for c in cls._bases:
            if c._params.has_key(name):
                return c._params[name]
        if isSubClass(default, Exception):
            raise default, \
                  "object '%s' has no attribute '%s'" % (cls.__name__, name)
        else:
            return default

    def _setparam(cls, name, value):
        cls._params[name] = value

    def _hasvalue(cls, name):
        for c in cls._bases:
            if c._values.has_key(name):
                return True

        return False

    def _getvalues(cls):
        values = {}
        for i,c in enumerate(cls._bases):
            for p,v in c._values.iteritems():
                if not values.has_key(p):
                    values[p] = v
            for p,v in c._params.iteritems():
                if not values.has_key(p) and hasattr(v, 'default'):
                    v.valid(v.default)
                    v = v.default
                    cls._setvalue(p, v)
                    values[p] = v

        return values

    def _getvalue(cls, name, default = AttributeError):
        value = None
        for c in cls._bases:
            if c._values.has_key(name):
                value = c._values[name]
                break
        if value is not None:
            return value

        param = cls._getparam(name, None)
        if param is not None and hasattr(param, 'default'):
            param.valid(param.default)
            value = param.default
            cls._setvalue(name, value)
            return value

        if isSubClass(default, Exception):
            raise default, 'value for %s not found' % name
        else:
            return default

    def _setvalue(cls, name, value):
        cls._values[name] = value

    def _getdisable(cls, name):
        for c in cls._bases:
            if c._disable.has_key(name):
                return c._disable[name]
        return False

    def _setdisable(cls, name, value):
        cls._disable[name] = value

    def __getattr__(cls, attr):
        if cls._isvalue(attr):
            return Value(cls, attr)

        if attr == '_cppname' and hasattr(cls, 'type'):
            return cls.type + '*'

        raise AttributeError, \
              "object '%s' has no attribute '%s'" % (cls.__name__, attr)

    # Set attribute (called on foo.attr = value when foo is an
    # instance of class cls).
    def __setattr__(cls, attr, value):
        # normal processing for private attributes
        if attr.startswith('_'):
            type.__setattr__(cls, attr, value)
            return

        if cls.keywords.has_key(attr):
            raise TypeError, \
                  "keyword '%s' can only be set in a simobj definition" % attr

        if isParam(value):
            cls._setparam(attr, value)
            return

        # must be SimObject param
        param = cls._getparam(attr, None)
        if param:
            # It's ok: set attribute by delegating to 'object' class.
            # Note the use of param.make_value() to verify/canonicalize
            # the assigned value
            param.valid(value)
            cls._setvalue(attr, value)
        elif isConfigNode(value) or isSimObjSequence(value):
            cls._setvalue(attr, value)
        else:
            for p,v in cls._getparams().iteritems():
                print p,v
            raise AttributeError, \
                  "Class %s has no parameter %s" % (cls.__name__, attr)

    def add_child(cls, instance, name, child):
        if isNullPointer(child) or instance.top_child_names.has_key(name):
            return

        if issequence(child):
            kid = []
            for i,c in enumerate(child):
                n = '%s%d' % (name, i)
                k = c.instantiate(n, instance)

                instance.children.append(k)
                instance.child_names[n] = k
                instance.child_objects[c] = k
                kid.append(k)
        else:
            kid = child.instantiate(name, instance)
            instance.children.append(kid)
            instance.child_names[name] = kid
            instance.child_objects[child] = kid

        instance.top_child_names[name] = kid

    # Print instance info to .ini file.
    def instantiate(cls, name, parent = None):
        instance = Node(name, cls, cls.type, parent, isParamContext(cls))

        if hasattr(cls, 'check'):
            cls.check()

        for key,value in cls._getvalues().iteritems():
            if cls._getdisable(key):
                continue

            if isConfigNode(value):
                cls.add_child(instance, key, value)
            if issequence(value):
                list = [ v for v in value if isConfigNode(v) ]
                if len(list):
                    cls.add_child(instance, key, list)

        for pname,param in cls._getparams().iteritems():
            try:
                if cls._getdisable(pname):
                    continue

                try:
                    value = cls._getvalue(pname)
                except:
                    print 'Error getting %s' % pname
                    raise

                if isConfigNode(value):
                    value = instance.child_objects[value]
                elif issequence(value):
                    v = []
                    for val in value:
                        if isConfigNode(val):
                            v.append(instance.child_objects[val])
                        else:
                            v.append(val)
                    value = v

                p = NodeParam(pname, param, value)
                instance.params.append(p)
                instance.param_names[pname] = p
            except:
                print 'Exception while evaluating %s.%s' % \
                      (instance.path, pname)
                raise

        return instance

    def _convert(cls, value):
        realvalue = value
        if isinstance(value, Node):
            realvalue = value.realtype

        if isinstance(realvalue, Proxy):
            return value

        if realvalue == None or isNullPointer(realvalue):
            return value

        if isSubClass(realvalue, cls):
            return value

        raise TypeError, 'object %s type %s wrong type, should be %s' % \
              (repr(realvalue), realvalue, cls)

    def _string(cls, value):
        if isNullPointer(value):
            return 'Null'
        return Node._string(value)

# The ConfigNode class is the root of the special hierarchy.  Most of
# the code in this class deals with the configuration hierarchy itself
# (parent/child node relationships).
class ConfigNode(object):
    # Specify metaclass.  Any class inheriting from ConfigNode will
    # get this metaclass.
    __metaclass__ = MetaConfigNode
    type = None

    def __new__(cls, **kwargs):
        return MetaConfigNode(cls.__name__, (cls, ), kwargs)

    # Set attribute.  All attribute assignments go through here.  Must
    # be private attribute (starts with '_') or valid parameter entry.
    # Basically identical to MetaConfigClass.__setattr__(), except
    # this sets attributes on specific instances rather than on classes.
    #def __setattr__(self, attr, value):
    #    if attr.startswith('_'):
    #        object.__setattr__(self, attr, value)
    #        return
        # not private; look up as param
    #    param = self.__class__.lookup_param(attr)
    #    if not param:
    #        raise AttributeError, \
    #              "Class %s has no parameter %s" \
    #              % (self.__class__.__name__, attr)
        # It's ok: set attribute by delegating to 'object' class.
        # Note the use of param.make_value() to verify/canonicalize
        # the assigned value.
    #    v = param.convert(value)
    #    object.__setattr__(self, attr, v)

class ParamContext(ConfigNode):
    pass

# SimObject is a minimal extension of ConfigNode, implementing a
# hierarchy node that corresponds to an M5 SimObject.  It prints out a
# "type=" line to indicate its SimObject class, prints out the
# assigned parameters corresponding to its class, and allows
# parameters to be set by keyword in the constructor.  Note that most
# of the heavy lifting for the SimObject param handling is done in the
# MetaConfigNode metaclass.
class SimObject(ConfigNode):
    def _sim_code(cls):
        name = cls.__name__
        param_names = cls._params.keys()
        param_names.sort()
        code = "BEGIN_DECLARE_SIM_OBJECT_PARAMS(%s)\n" % name
        decls = ["  " + cls._params[pname].sim_decl(pname) \
                 for pname in param_names]
        code += "\n".join(decls) + "\n"
        code += "END_DECLARE_SIM_OBJECT_PARAMS(%s)\n\n" % name
        return code
    _sim_code = classmethod(_sim_code)

class NodeParam(object):
    def __init__(self, name, param, value):
        self.name = name
        self.param = param
        self.ptype = param.ptype
        self.convert = param.convert
        self.string = param.string
        self.value = value

class Node(object):
    all = {}
    def __init__(self, name, realtype, type, parent, paramcontext):
        self.name = name
        self.realtype = realtype
        self.type = type
        self.parent = parent
        self.children = []
        self.child_names = {}
        self.child_objects = {}
        self.top_child_names = {}
        self.params = []
        self.param_names = {}
        self.paramcontext = paramcontext

        path = [ self.name ]
        node = self.parent
        while node is not None:
            if node.name != 'root':
                path.insert(0, node.name)
            else:
                assert(node.parent is None)
            node = node.parent
        self.path = '.'.join(path)

    def find(self, realtype, path):
        if not path:
            if issubclass(self.realtype, realtype):
                return self, True

            obj = None
            for child in self.children:
                if issubclass(child.realtype, realtype):
                    if obj is not None:
                        raise AttributeError, \
                              'Super matched more than one: %s %s' % \
                              (obj.path, child.path)
                    obj = child
            return obj, obj is not None

        try:
            obj = self
            for node in path[:-1]:
                obj = obj.child_names[node]

            last = path[-1]
            if obj.child_names.has_key(last):
                value = obj.child_names[last]
                if issubclass(value.realtype, realtype):
                    return value, True
            elif obj.param_names.has_key(last):
                value = obj.param_names[last]
                realtype._convert(value.value)
                return value.value, True
        except KeyError:
            pass

        return None, False

    def unproxy(self, ptype, value):
        if not isinstance(value, Proxy):
            return value

        if value is None:
            raise AttributeError, 'Error while fixing up %s' % self.path

        obj = self
        done = False
        while not done:
            if obj is None:
                raise AttributeError, \
                      'Parent of %s type %s not found at path %s' \
                      % (self.name, ptype, value._path)
            found, done = obj.find(ptype, value._path)
            if isinstance(found, Proxy):
                done = False
            obj = obj.parent

        return found

    def fixup(self):
        self.all[self.path] = self

        for param in self.params:
            ptype = param.ptype
            pval = param.value

            try:
                if issequence(pval):
                    param.value = [ self.unproxy(ptype, pv) for pv in pval ]
                else:
                    param.value = self.unproxy(ptype, pval)
            except:
                print 'Error while fixing up %s:%s' % (self.path, param.name)
                raise

        for child in self.children:
            assert(child != self)
            child.fixup()

    # print type and parameter values to .ini file
    def display(self):
        print '[' + self.path + ']'	# .ini section header

        if isSimObject(self.realtype):
            print 'type = %s' % self.type

        if self.children:
            # instantiate children in same order they were added for
            # backward compatibility (else we can end up with cpu1
            # before cpu0).
            children = [ c.name for c in self.children if not c.paramcontext]
            print 'children =', ' '.join(children)

        for param in self.params:
            try:
                if param.value is None:
                    raise AttributeError, 'Parameter with no value'

                value = param.convert(param.value)
                string = param.string(value)
            except:
                print 'exception in %s:%s' % (self.path, param.name)
                raise

            print '%s = %s' % (param.name, string)

        print

        # recursively dump out children
        for c in self.children:
            c.display()

    # print type and parameter values to .ini file
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

                value = param.convert(param.value)
                string = param.string(value)
            except:
                print 'exception in %s:%s' % (self.name, param.name)
                raise
            if isConfigNode(param.ptype) and string != "Null":
                simobjs.append(string)
            else:
                label += '%s = %s\\n' % (param.name, string)

        for so in simobjs:
            label += "|<%s> %s" % (so, so)
            dot.add_edge(pydot.Edge("%s:%s" % (self.path, so), so, tailport="w"))
        label += '}'
        dot.add_node(pydot.Node(self.path,shape="Mrecord",label=label))

        # recursively dump out children
        for c in self.children:
            c.outputDot(dot)

    def _string(cls, value):
        if not isinstance(value, Node):
            raise AttributeError, 'expecting %s got %s' % (Node, value)
        return value.path
    _string = classmethod(_string)

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

def isNullPointer(value):
    return isinstance(value, NullSimObject)

class Value(object):
    def __init__(self, obj, attr):
        super(Value, self).__setattr__('attr', attr)
        super(Value, self).__setattr__('obj', obj)

    def _getattr(self):
        return self.obj._getvalue(self.attr)

    def __setattr__(self, attr, value):
        if attr == 'disable':
            self.obj._setdisable(self.attr, value)
        else:
            setattr(self._getattr(), attr, value)

    def __getattr__(self, attr):
        if attr == 'disable':
            return self.obj._getdisable(self.attr)
        else:
            return getattr(self._getattr(), attr)

    def __getitem__(self, index):
        return self._getattr().__getitem__(index)

    def __call__(self, *args, **kwargs):
        return self._getattr().__call__(*args, **kwargs)

    def __nonzero__(self):
        return bool(self._getattr())

    def __str__(self):
        return str(self._getattr())

# Regular parameter.
class _Param(object):
    def __init__(self, ptype_string, *args, **kwargs):
        self.ptype_string = ptype_string
        # can't eval ptype_string here to get ptype, since the type might
        # not have been defined yet.  Do it lazily in __getattr__.

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
                self.ptype = eval(self.ptype_string)
                return self.ptype
            except:
                raise TypeError, 'Param.%s: undefined type' % self.ptype_string
        else:
            raise AttributeError, "'%s' object has no attribute '%s'" % \
                  (type(self).__name__, attr)

    def valid(self, value):
        if not isinstance(value, Proxy):
            self.ptype._convert(value)

    def convert(self, value):
        return self.ptype._convert(value)

    def string(self, value):
        return self.ptype._string(value)

    def set(self, name, instance, value):
        instance.__dict__[name] = value

    def sim_decl(self, name):
        return '%s %s;' % (self.ptype._cppname, name)

class _ParamProxy(object):
    def __init__(self, type):
        self.ptype = type

    # E.g., Param.Int(5, "number of widgets")
    def __call__(self, *args, **kwargs):
        return _Param(self.ptype, *args, **kwargs)

    def __getattr__(self, attr):
        if attr == '__bases__':
            raise AttributeError, ''
        cls = type(self)
        return cls(attr)

    def __setattr__(self, attr, value):
        if attr != 'ptype':
            raise AttributeError, \
                  'Attribute %s not available in %s' % (attr, self.__class__)
        super(_ParamProxy, self).__setattr__(attr, value)


Param = _ParamProxy(None)

# Vector-valued parameter description.  Just like Param, except that
# the value is a vector (list) of the specified type instead of a
# single value.
class _VectorParam(_Param):
    def __init__(self, type, *args, **kwargs):
        _Param.__init__(self, type, *args, **kwargs)

    def valid(self, value):
        if value == None:
            return True

        if issequence(value):
            for val in value:
                if not isinstance(val, Proxy):
                    self.ptype._convert(val)
        elif not isinstance(value, Proxy):
            self.ptype._convert(value)

    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def convert(self, value):
        if value == None:
            return []

        if issequence(value):
            # list: coerce each element into new list
            return [ self.ptype._convert(v) for v in value ]
        else:
            # singleton: coerce & wrap in a list
            return self.ptype._convert(value)

    def string(self, value):
        if issequence(value):
            return ' '.join([ self.ptype._string(v) for v in value])
        else:
            return self.ptype._string(value)

    def sim_decl(self, name):
        return 'std::vector<%s> %s;' % (self.ptype._cppname, name)

class _VectorParamProxy(_ParamProxy):
    # E.g., VectorParam.Int(5, "number of widgets")
    def __call__(self, *args, **kwargs):
        return _VectorParam(self.ptype, *args, **kwargs)

VectorParam = _VectorParamProxy(None)

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
class _CheckedInt(object):
    def _convert(cls, value):
        t = type(value)
        if t == bool:
            return int(value)

        if t != int and t != long and t != float and t != str:
            raise TypeError, 'Integer parameter of invalid type %s' % t

        if t == str or t == float:
            value = long(value)

        if not cls._min <= value <= cls._max:
            raise TypeError, 'Integer parameter out of bounds %d < %d < %d' % \
                  (cls._min, value, cls._max)

        return value
    _convert = classmethod(_convert)

    def _string(cls, value):
        return str(value)
    _string = classmethod(_string)

class CheckedInt(type):
    def __new__(cls, cppname, min, max):
        # New class derives from _CheckedInt base with proper bounding
        # parameters
        dict = { '_cppname' : cppname, '_min' : min, '_max' : max }
        return type.__new__(cls, cppname, (_CheckedInt, ), dict)

class CheckedIntType(CheckedInt):
    def __new__(cls, cppname, size, unsigned):
        dict = {}
        if unsigned:
            min = 0
            max = 2 ** size - 1
        else:
            min = -(2 ** (size - 1))
            max = (2 ** (size - 1)) - 1

        return super(cls, CheckedIntType).__new__(cls, cppname, min, max)

Int      = CheckedIntType('int',      32, False)
Unsigned = CheckedIntType('unsigned', 32, True)

Int8     = CheckedIntType('int8_t',    8, False)
UInt8    = CheckedIntType('uint8_t',   8, True)
Int16    = CheckedIntType('int16_t',  16, False)
UInt16   = CheckedIntType('uint16_t', 16, True)
Int32    = CheckedIntType('int32_t',  32, False)
UInt32   = CheckedIntType('uint32_t', 32, True)
Int64    = CheckedIntType('int64_t',  64, False)
UInt64   = CheckedIntType('uint64_t', 64, True)

Counter  = CheckedIntType('Counter', 64, True)
Addr     = CheckedIntType('Addr',    64, True)
Tick     = CheckedIntType('Tick',    64, True)

Percent  = CheckedInt('int', 0, 100)

class Pair(object):
    def __init__(self, first, second):
        self.first = first
        self.second = second

class _Range(object):
    def _convert(cls, value):
        if not isinstance(value, Pair):
            raise TypeError, 'value %s is not a Pair' % value
        return Pair(cls._type._convert(value.first),
                    cls._type._convert(value.second))
    _convert = classmethod(_convert)

    def _string(cls, value):
        return '%s:%s' % (cls._type._string(value.first),
                          cls._type._string(value.second))
    _string = classmethod(_string)

def RangeSize(start, size):
    return Pair(start, start + size - 1)

class Range(type):
    def __new__(cls, type):
        dict = { '_cppname' : 'Range<%s>' % type._cppname, '_type' : type }
        clsname = 'Range_' + type.__name__
        return super(cls, Range).__new__(cls, clsname, (_Range, ), dict)

AddrRange = Range(Addr)

# Boolean parameter type.
class Bool(object):
    _cppname = 'bool'
    def _convert(value):
        t = type(value)
        if t == bool:
            return value

        if t == int or t == long:
            return bool(value)

        if t == str:
            v = value.lower()
            if v == "true" or v == "t" or v == "yes" or v == "y":
                return True
            elif v == "false" or v == "f" or v == "no" or v == "n":
                return False

        raise TypeError, 'Bool parameter (%s) of invalid type %s' % (v, t)
    _convert = staticmethod(_convert)

    def _string(value):
        if value:
            return "true"
        else:
            return "false"
    _string = staticmethod(_string)

# String-valued parameter.
class String(object):
    _cppname = 'string'

    # Constructor.  Value must be Python string.
    def _convert(cls,value):
        if value is None:
            return ''
        if isinstance(value, str):
            return value

        raise TypeError, \
              "String param got value %s %s" % (repr(value), type(value))
    _convert = classmethod(_convert)

    # Generate printable string version.  Not too tricky.
    def _string(cls, value):
        return value
    _string = classmethod(_string)


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
    __metaclass__ = Singleton
    addr = "00:90:00:00:00:01"

    def __init__(self, inc = 1):
        self.value = self.addr
        self.addr = IncEthernetAddr(self.addr, inc)

class EthernetAddr(object):
    _cppname = 'EthAddr'

    def _convert(cls, value):
        if value == NextEthernetAddr:
            return value

        if not isinstance(value, str):
            raise TypeError, "expected an ethernet address and didn't get one"

        bytes = value.split(':')
        if len(bytes) != 6:
            raise TypeError, 'invalid ethernet address %s' % value

        for byte in bytes:
            if not 0 <= int(byte) <= 256:
                raise TypeError, 'invalid ethernet address %s' % value

        return value
    _convert = classmethod(_convert)

    def _string(cls, value):
        if value == NextEthernetAddr:
            value = value().value
        return value
    _string = classmethod(_string)

# Special class for NULL pointers.  Note the special check in
# make_param_value() above that lets these be assigned where a
# SimObject is required.
# only one copy of a particular node
class NullSimObject(object):
    __metaclass__ = Singleton
    _cppname = 'NULL'

    def __call__(cls):
        return cls

    def _sim_code(cls):
        pass
    _sim_code = classmethod(_sim_code)

    def _instantiate(self, parent = None, path = ''):
        pass

    def _convert(cls, value):
        if value == Nxone:
            return

        if isinstance(value, cls):
            return value

        raise TypeError, 'object %s %s of the wrong type, should be %s' % \
              (repr(value), type(value), cls)
    _convert = classmethod(_convert)

    def _string():
        return 'NULL'
    _string = staticmethod(_string)

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


# Base class for Enum types.
class _Enum(object):
    def _convert(self, value):
        if value not in self.map:
            raise TypeError, "Enum param got bad value '%s' (not in %s)" \
                  % (value, self.map)
        return value
    _convert = classmethod(_convert)

    # Generate printable string version of value.
    def _string(self, value):
        return str(value)
    _string = classmethod(_string)

# Enum metaclass... calling Enum(foo) generates a new type (class)
# that derives from _ListEnum or _DictEnum as appropriate.
class Enum(type):
    # counter to generate unique names for generated classes
    counter = 1

    def __new__(cls, *args):
        if len(args) > 1:
            enum_map = args
        else:
            enum_map = args[0]

        if isinstance(enum_map, dict):
            map = enum_map
        elif issequence(enum_map):
            map = {}
            for idx,val in enumerate(enum_map):
                map[val] = idx
        else:
            raise TypeError, "Enum map must be list or dict (got %s)" % map

        classname = "Enum%04d" % Enum.counter
        Enum.counter += 1

        # New class derives from _Enum base, and gets a 'map'
        # attribute containing the specified list or dict.
        return type.__new__(cls, classname, (_Enum, ), { 'map': map })


#
# "Constants"... handy aliases for various values.
#

# Some memory range specifications use this as a default upper bound.
MAX_ADDR = Addr._max
MaxTick = Tick._max

# For power-of-two sizing, e.g. 64*K gives an integer value 65536.
K = 1024
M = K*K
G = K*M

#####################################################################

# The final hook to generate .ini files.  Called from configuration
# script once config is built.
def instantiate(root):
    if not issubclass(root, Root):
        raise AttributeError, 'Can only instantiate the Root of the tree'

    instance = root.instantiate('root')
    instance.fixup()
    instance.display()
    if not noDot:
       dot = pydot.Dot()
       instance.outputDot(dot)
       dot.orientation = "portrait"
       dot.size = "8.5,11"
       dot.ranksep="equally"
       dot.rank="samerank"
       dot.write("config.dot")
       dot.write_ps("config.ps")

from objects import *

