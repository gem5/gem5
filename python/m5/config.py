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
import os, re, sys, types, inspect

from convert import *

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
    def __init__(self, path):
        self._object = None
        if path == 'any':
            self._path = None
        else:
            # path is a list of (attr,index) tuples
            self._path = [(path,None)]
        self._index = None
        self._multiplier = None

    def __getattr__(self, attr):
        if attr == '__bases__':
            return super(Proxy, self).__getattr__(self, attr)
        self._path.append((attr,None))
        return self

    def __setattr__(self, attr, value):
        if not attr.startswith('_'):
            raise AttributeError, 'cannot set attribute %s' % attr
        super(Proxy, self).__setattr__(attr, value)

    # support indexing on proxies (e.g., parent.cpu[0])
    def __getitem__(self, key):
        if not isinstance(key, int):
            raise TypeError, "Proxy object requires integer index"
        if self._path == None:
            raise IndexError, "Index applied to 'any' proxy"
        # replace index portion of last path element with new index
        self._path[-1] = (self._path[-1][0], key)
        return self

    # support multiplying proxies by constants
    def __mul__(self, other):
        if not isinstance(other, int):
            raise TypeError, "Proxy multiplier must be integer"
        if self._multiplier == None:
            self._multiplier = other
        else:
            # support chained multipliers
            self._multiplier *= other
        return self

    def _mulcheck(self, result):
        if self._multiplier == None:
            return result
        if not isinstance(result, int):
            raise TypeError, "Proxy with multiplier resolves to " \
                  "non-integer value"
        return result * self._multiplier

    def unproxy(self, base, ptype):
        obj = base
        done = False
        while not done:
            if obj is None:
                raise AttributeError, \
                      'Parent of %s type %s not found at path %s' \
                      % (base.name, ptype, self._path)
            found, done = obj.find(ptype, self._path)
            if isinstance(found, Proxy):
                done = False
            obj = obj.parent

        return self._mulcheck(found)

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

class ProxyFactory(object):
    def __getattr__(self, attr):
        return Proxy(attr)

# global object for handling parent.foo proxies
parent = ProxyFactory()

def isSubClass(value, cls):
    try:
        return issubclass(value, cls)
    except:
        return False

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
    if not isinstance(value, (list, tuple)):
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


class_decorator = 'M5M5_SIMOBJECT_'
expr_decorator = 'M5M5_EXPRESSION_'
dot_decorator = '_M5M5_DOT_'

# 'Global' map of legitimate types for SimObject parameters.
param_types = {}

# Dummy base class to identify types that are legitimate for SimObject
# parameters.
class ParamType(object):
    pass

# Add types defined in given context (dict or module) that are derived
# from ParamType to param_types map.
def add_param_types(ctx):
    if isinstance(ctx, types.DictType):
        source_dict = ctx
    elif isinstance(ctx, types.ModuleType):
        source_dict = ctx.__dict__
    else:
        raise TypeError, \
              "m5.config.add_param_types requires dict or module as arg"
    for key,val in source_dict.iteritems():
        if isinstance(val, type) and issubclass(val, ParamType):
            param_types[key] = val

# The metaclass for ConfigNode (and thus for everything that derives
# from ConfigNode, including SimObject).  This class controls how new
# classes that derive from ConfigNode are instantiated, and provides
# inherited class behavior (just like a class controls how instances
# of that class are instantiated, and provides inherited instance
# behavior).
class MetaConfigNode(type):
    # Attributes that can be set only at initialization time
    init_keywords = {}
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
        return super(MetaConfigNode, mcls).__new__(mcls, name, bases, cls_dict)

    # initialization
    def __init__(cls, name, bases, dict):
        super(MetaConfigNode, cls).__init__(name, bases, dict)

        # initialize required attributes
        cls._params = {}
        cls._values = {}
        cls._param_types = {}
        cls._bases = [c for c in cls.__mro__ if isConfigNode(c)]
        cls._anon_subclass_counter = 0

        # If your parent has a value in it that's a config node, clone
        # it.  Do this now so if we update any of the values'
        # attributes we are updating the clone and not the original.
        for base in cls._bases:
            for key,val in base._values.iteritems():

                # don't clone if (1) we're about to overwrite it with
                # a local setting or (2) we've already cloned a copy
                # from an earlier (more derived) base
                if cls._init_dict.has_key(key) or cls._values.has_key(key):
                    continue

                if isConfigNode(val):
                    cls._values[key] = val()
                elif isSimObjSequence(val):
                    cls._values[key] = [ v() for v in val ]
                elif isNullPointer(val):
                    cls._values[key] = val

        # process param types from _init_dict, as these may be needed
        # by param descriptions also in _init_dict
        for key,val in cls._init_dict.items():
            if isinstance(val, type) and issubclass(val, ParamType):
                cls._param_types[key] = val
                if not issubclass(val, ConfigNode):
                    del cls._init_dict[key]

        # now process remaining _init_dict items
        for key,val in cls._init_dict.items():
            # param descriptions
            if isinstance(val, _Param):
                cls._params[key] = val
                # try to resolve local param types in local param_types scope
                val.maybe_resolve_type(cls._param_types)

            # init-time-only keywords
            elif cls.init_keywords.has_key(key):
                cls._set_keyword(key, val, cls.init_keywords[key])

            # See description of decorators in the importer.py file.
            # We just strip off the expr_decorator now since we don't
            # need from this point on.
            elif key.startswith(expr_decorator):
                key = key[len(expr_decorator):]
                # because it had dots into a list so that we can find the
                # proper variable to modify.
                key = key.split(dot_decorator)
                c = cls
                for item in key[:-1]:
                    c = getattr(c, item)
                setattr(c, key[-1], val)

            # default: use normal path (ends up in __setattr__)
            else:
                setattr(cls, key, val)


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
                    try:
                        v.valid(v.default)
                    except TypeError:
                        panic("Invalid default %s for param %s in node %s"
                              % (v.default,p,cls.__name__))
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

    def __getattr__(cls, attr):
        if cls._isvalue(attr):
            return Value(cls, attr)

        if attr == '_cpp_param_decl' and hasattr(cls, 'type'):
            return cls.type + '*'

        raise AttributeError, \
              "object '%s' has no attribute '%s'" % (cls.__name__, attr)

    def _set_keyword(cls, keyword, val, kwtype):
        if not isinstance(val, kwtype):
            raise TypeError, 'keyword %s has bad type %s (expecting %s)' % \
                  (keyword, type(val), kwtype)
        if isinstance(val, types.FunctionType):
            val = classmethod(val)
        type.__setattr__(cls, keyword, val)

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
        param = cls._getparam(attr, None)
        if param:
            # It's ok: set attribute by delegating to 'object' class.
            # Note the use of param.make_value() to verify/canonicalize
            # the assigned value
            try:
                param.valid(value)
            except:
                panic("Error setting param %s.%s to %s\n" % \
                      (cls.__name__, attr, value))
            cls._setvalue(attr, value)
        elif isConfigNode(value) or isSimObjSequence(value):
            cls._setvalue(attr, value)
        else:
            raise AttributeError, \
                  "Class %s has no parameter %s" % (cls.__name__, attr)

    def add_child(cls, instance, name, child):
        if isNullPointer(child) or instance.top_child_names.has_key(name):
            return

        if isinstance(child, (list, tuple)):
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
        instance = Node(name, cls, parent, isParamContext(cls))

        if hasattr(cls, 'check'):
            cls.check()

        for key,value in cls._getvalues().iteritems():
            if isConfigNode(value):
                cls.add_child(instance, key, value)
            if isinstance(value, (list, tuple)):
                vals = [ v for v in value if isConfigNode(v) ]
                if len(vals):
                    cls.add_child(instance, key, vals)

        for pname,param in cls._getparams().iteritems():
            try:
                value = cls._getvalue(pname)
            except:
                panic('Error getting %s' % pname)

            try:
                if isConfigNode(value):
                    value = instance.child_objects[value]
                elif isinstance(value, (list, tuple)):
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

    def __new__(cls, **kwargs):
        name = cls.__name__ + ("_%d" % cls._anon_subclass_counter)
        cls._anon_subclass_counter += 1
        return cls.__metaclass__(name, (cls, ), kwargs)

class ParamContext(ConfigNode):
    pass

class MetaSimObject(MetaConfigNode):
    # init_keywords and keywords are inherited from MetaConfigNode,
    # with overrides/additions
    init_keywords = MetaConfigNode.init_keywords
    init_keywords.update({ 'abstract' : types.BooleanType,
                           'type' : types.StringType })

    keywords = MetaConfigNode.keywords
    # no additional keywords

    cpp_classes = []

    # initialization
    def __init__(cls, name, bases, dict):
        super(MetaSimObject, cls).__init__(name, bases, dict)

        if hasattr(cls, 'type'):
            if name == 'SimObject':
                cls._cpp_base = None
            elif hasattr(cls._bases[1], 'type'):
                cls._cpp_base = cls._bases[1].type
            else:
                panic("SimObject %s derives from a non-C++ SimObject %s "\
                      "(no 'type')" % (cls, cls_bases[1].__name__))

            # This class corresponds to a C++ class: put it on the global
            # list of C++ objects to generate param structs, etc.
            MetaSimObject.cpp_classes.append(cls)

    def _cpp_decl(cls):
        name = cls.__name__
        code = ""
        code += "\n".join([e.cpp_declare() for e in cls._param_types.values()])
        code += "\n"
        param_names = cls._params.keys()
        param_names.sort()
        code += "struct Params"
        if cls._cpp_base:
            code += " : public %s::Params" % cls._cpp_base
        code += " {\n    "
        code += "\n    ".join([cls._params[pname].cpp_decl(pname) \
                               for pname in param_names])
        code += "\n};\n"
        return code

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
    def __init__(self, name, realtype, parent, paramcontext):
        self.name = name
        self.realtype = realtype
        if isSimObject(realtype):
            self.type = realtype.type
        else:
            self.type = None
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
                              'parent.any matched more than one: %s %s' % \
                              (obj.path, child.path)
                    obj = child
            return obj, obj is not None

        try:
            obj = self
            for (node,index) in path[:-1]:
                if obj.child_names.has_key(node):
                    obj = obj.child_names[node]
                else:
                    obj = obj.top_child_names[node]
                obj = Proxy.getindex(obj, index)

            (last,index) = path[-1]
            if obj.child_names.has_key(last):
                value = obj.child_names[last]
                return Proxy.getindex(value, index), True
            elif obj.top_child_names.has_key(last):
                value = obj.top_child_names[last]
                return Proxy.getindex(value, index), True
            elif obj.param_names.has_key(last):
                value = obj.param_names[last]
                realtype._convert(value.value)
                return Proxy.getindex(value.value, index), True
        except KeyError:
            pass

        return None, False

    def unproxy(self, param, ptype):
        if not isinstance(param, Proxy):
            return param
        return param.unproxy(self, ptype)

    def fixup(self):
        self.all[self.path] = self

        for param in self.params:
            ptype = param.ptype
            pval = param.value

            try:
                if isinstance(pval, (list, tuple)):
                    param.value = [ self.unproxy(pv, ptype) for pv in pval ]
                else:
                    param.value = self.unproxy(pval, ptype)
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
            # before cpu0).  Changing ordering can also influence timing
            # in the current memory system, as caches get added to a bus
            # in different orders which affects their priority in the
            # case of simulataneous requests.  We should uncomment the
            # following line once we take care of that issue.
            # self.children.sort(lambda x,y: cmp(x.name, y.name))
            children = [ c.name for c in self.children if not c.paramcontext]
            print 'children =', ' '.join(children)

        self.params.sort(lambda x,y: cmp(x.name, y.name))
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
        setattr(self._getattr(), attr, value)

    def __getattr__(self, attr):
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
    def __init__(self, ptype, *args, **kwargs):
        if isinstance(ptype, types.StringType):
            self.ptype_string = ptype
        elif isinstance(ptype, type):
            self.ptype = ptype
        else:
            raise TypeError, "Param type is not a type (%s)" % ptype

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

    def maybe_resolve_type(self, context):
        # check if already resolved... don't use hasattr(),
        # as that calls __getattr__()
        if self.__dict__.has_key('ptype'):
            return
        try:
            self.ptype = context[self.ptype_string]
        except KeyError:
            # no harm in trying... we'll try again later using global scope
            pass

    def __getattr__(self, attr):
        if attr == 'ptype':
            try:
                self.ptype = param_types[self.ptype_string]
                return self.ptype
            except:
                panic("undefined Param type %s" % self.ptype_string)
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

    def cpp_decl(self, name):
        return '%s %s;' % (self.ptype._cpp_param_decl, name)

class _ParamProxy(object):
    def __init__(self, type):
        self.ptype = type

    # E.g., Param.Int(5, "number of widgets")
    def __call__(self, *args, **kwargs):
        return _Param(self.ptype, *args, **kwargs)

    # Strange magic to theoretically allow dotted names as Param classes,
    # e.g., Param.Foo.Bar(...) to have a param of type Foo.Bar
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

        if isinstance(value, (list, tuple)):
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

        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            return [ self.ptype._convert(v) for v in value ]
        else:
            # singleton: coerce & wrap in a list
            return self.ptype._convert(value)

    def string(self, value):
        if isinstance(value, (list, tuple)):
            return ' '.join([ self.ptype._string(v) for v in value])
        else:
            return self.ptype._string(value)

    def cpp_decl(self, name):
        return 'std::vector<%s> %s;' % (self.ptype._cpp_param_decl, name)

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

        cls._cpp_param_decl = cls.cppname

    def _convert(cls, value):
        if isinstance(value, bool):
            return int(value)

        if not isinstance(value, (int, long, float, str)):
            raise TypeError, 'Integer param of invalid type %s' % type(value)

        if isinstance(value, (str, float)):
            value = long(float(value))

        if not cls.min <= value <= cls.max:
            raise TypeError, 'Integer param out of bounds %d < %d < %d' % \
                  (cls.min, value, cls.max)

        return value

    def _string(cls, value):
        return str(value)

# Abstract superclass for bounds-checked integer parameters.  This
# class is subclassed to generate parameter classes with specific
# bounds.  Initialization of the min and max bounds is done in the
# metaclass CheckedIntType.__init__.
class CheckedInt(ParamType):
    __metaclass__ = CheckedIntType

class Int(CheckedInt):      cppname = 'int';      size = 32; unsigned = False
class Unsigned(CheckedInt): cppname = 'unsigned'; size = 32; unsigned = True

class Int8(CheckedInt):     cppname =  'int8_t';  size =  8; unsigned = False
class UInt8(CheckedInt):    cppname = 'uint8_t';  size =  8; unsigned = True
class Int16(CheckedInt):    cppname =  'int16_t'; size = 16; unsigned = False
class UInt16(CheckedInt):   cppname = 'uint16_t'; size = 16; unsigned = True
class Int32(CheckedInt):    cppname =  'int32_t'; size = 32; unsigned = False
class UInt32(CheckedInt):   cppname = 'uint32_t'; size = 32; unsigned = True
class Int64(CheckedInt):    cppname =  'int64_t'; size = 64; unsigned = False
class UInt64(CheckedInt):   cppname = 'uint64_t'; size = 64; unsigned = True

class Counter(CheckedInt): cppname = 'Counter'; size = 64; unsigned = True
class Addr(CheckedInt):    cppname = 'Addr';    size = 64; unsigned = True
class Tick(CheckedInt):    cppname = 'Tick';    size = 64; unsigned = True

class Percent(CheckedInt): cppname = 'int'; min = 0; max = 100

class Pair(object):
    def __init__(self, first, second):
        self.first = first
        self.second = second

class MetaRange(type):
    def __init__(cls, name, bases, dict):
        super(MetaRange, cls).__init__(name, bases, dict)
        if name == 'Range':
            return
        cls._cpp_param_decl = 'Range<%s>' % cls.type._cpp_param_decl

    def _convert(cls, value):
        if not isinstance(value, Pair):
            raise TypeError, 'value %s is not a Pair' % value
        return Pair(cls.type._convert(value.first),
                    cls.type._convert(value.second))

    def _string(cls, value):
        return '%s:%s' % (cls.type._string(value.first),
                          cls.type._string(value.second))

class Range(ParamType):
    __metaclass__ = MetaRange

def RangeSize(start, size):
    return Pair(start, start + size - 1)

class AddrRange(Range): type = Addr

# Boolean parameter type.
class Bool(ParamType):
    _cpp_param_decl = 'bool'
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
class String(ParamType):
    _cpp_param_decl = 'string'

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

class EthernetAddr(ParamType):
    _cpp_param_decl = 'EthAddr'

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

    def __call__(cls):
        return cls

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

        cls._cpp_param_decl = name

        super(MetaEnum, cls).__init__(name, bases, init_dict)

    def cpp_declare(cls):
        s = 'enum %s {\n    ' % cls.__name__
        s += ',\n    '.join(['%s = %d' % (v,cls.map[v]) for v in cls.vals])
        s += '\n};\n'
        return s

# Base class for enum types.
class Enum(ParamType):
    __metaclass__ = MetaEnum
    vals = []

    def _convert(self, value):
        if value not in self.map:
            raise TypeError, "Enum param got bad value '%s' (not in %s)" \
                  % (value, self.vals)
        return value
    _convert = classmethod(_convert)

    # Generate printable string version of value.
    def _string(self, value):
        return str(value)
    _string = classmethod(_string)
#
# "Constants"... handy aliases for various values.
#

# Some memory range specifications use this as a default upper bound.
MAX_ADDR = Addr.max
MaxTick = Tick.max

# For power-of-two sizing, e.g. 64*K gives an integer value 65536.
K = 1024
M = K*K
G = K*M

#####################################################################

# The final hook to generate .ini files.  Called from configuration
# script once config is built.
def instantiate(root):
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

# SimObject is a minimal extension of ConfigNode, implementing a
# hierarchy node that corresponds to an M5 SimObject.  It prints out a
# "type=" line to indicate its SimObject class, prints out the
# assigned parameters corresponding to its class, and allows
# parameters to be set by keyword in the constructor.  Note that most
# of the heavy lifting for the SimObject param handling is done in the
# MetaConfigNode metaclass.
class SimObject(ConfigNode, ParamType):
    __metaclass__ = MetaSimObject
    type = 'SimObject'


# __all__ defines the list of symbols that get exported when
# 'from config import *' is invoked.  Try to keep this reasonably
# short to avoid polluting other namespaces.
__all__ = ['ConfigNode', 'SimObject', 'ParamContext', 'Param', 'VectorParam',
           'parent', 'Enum',
           'Int', 'Unsigned', 'Int8', 'UInt8', 'Int16', 'UInt16',
           'Int32', 'UInt32', 'Int64', 'UInt64',
           'Counter', 'Addr', 'Tick', 'Percent',
           'Pair', 'RangeSize', 'AddrRange', 'MAX_ADDR', 'NULL', 'K', 'M',
           'NextEthernetAddr',
           'instantiate']
