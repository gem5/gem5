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

import sys, types

from util import *
from multidict import multidict

# These utility functions have to come first because they're
# referenced in params.py... otherwise they won't be defined when we
# import params below, and the recursive import of this file from
# params.py will not find these names.
def isSimObject(value):
    return isinstance(value, SimObject)

def isSimObjectClass(value):
    return issubclass(value, SimObject)

def isSimObjectSequence(value):
    if not isinstance(value, (list, tuple)) or len(value) == 0:
        return False

    for val in value:
        if not isNullPointer(val) and not isSimObject(val):
            return False

    return True

def isSimObjectOrSequence(value):
    return isSimObject(value) or isSimObjectSequence(value)

# Have to import params up top since Param is referenced on initial
# load (when SimObject class references Param to create a class
# variable, the 'name' param)...
from params import *
# There are a few things we need that aren't in params.__all__ since
# normal users don't need them
from params import ParamDesc, isNullPointer, SimObjVector

noDot = False
try:
    import pydot
except:
    noDot = True

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

# The metaclass for SimObject.  This class controls how new classes
# that derive from SimObject are instantiated, and provides inherited
# class behavior (just like a class controls how instances of that
# class are instantiated, and provides inherited instance behavior).
class MetaSimObject(type):
    # Attributes that can be set only at initialization time
    init_keywords = { 'abstract' : types.BooleanType,
                      'type' : types.StringType }
    # Attributes that can be set any time
    keywords = { 'check' : types.FunctionType,
                 'cxx_type' : types.StringType,
                 'cxx_predecls' : types.ListType,
                 'swig_predecls' : types.ListType }

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
        cls._port_refs = multidict() # port ref objects
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
            cls._port_refs.parent = base._port_refs
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
                cls._new_port(key, val)

            # init-time-only keywords
            elif cls.init_keywords.has_key(key):
                cls._set_keyword(key, val, cls.init_keywords[key])

            # default: use normal path (ends up in __setattr__)
            else:
                setattr(cls, key, val)

        cls.cxx_type = cls.type + '*'
        # A forward class declaration is sufficient since we are just
        # declaring a pointer.
        cls.cxx_predecls = ['class %s;' % cls.type]
        cls.swig_predecls = cls.cxx_predecls

    def _set_keyword(cls, keyword, val, kwtype):
        if not isinstance(val, kwtype):
            raise TypeError, 'keyword %s has bad type %s (expecting %s)' % \
                  (keyword, type(val), kwtype)
        if isinstance(val, types.FunctionType):
            val = classmethod(val)
        type.__setattr__(cls, keyword, val)

    def _new_param(cls, name, pdesc):
        # each param desc should be uniquely assigned to one variable
        assert(not hasattr(pdesc, 'name'))
        pdesc.name = name
        cls._params[name] = pdesc
        if hasattr(pdesc, 'default'):
            cls._set_param(name, pdesc.default, pdesc)

    def _set_param(cls, name, value, param):
        assert(param.name == name)
        try:
            cls._values[name] = param.convert(value)
        except Exception, e:
            msg = "%s\nError setting param %s.%s to %s\n" % \
                  (e, cls.__name__, name, value)
            e.args = (msg, )
            raise

    def _new_port(cls, name, port):
        # each port should be uniquely assigned to one variable
        assert(not hasattr(port, 'name'))
        port.name = name
        cls._ports[name] = port
        if hasattr(port, 'default'):
            cls._cls_get_port_ref(name).connect(port.default)

    # same as _get_port_ref, effectively, but for classes
    def _cls_get_port_ref(cls, attr):
        # Return reference that can be assigned to another port
        # via __setattr__.  There is only ever one reference
        # object per port, but we create them lazily here.
        ref = cls._port_refs.get(attr)
        if not ref:
            ref = cls._ports[attr].makeRef(cls)
            cls._port_refs[attr] = ref
        return ref

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
            cls._cls_get_port_ref(attr).connect(value)
            return

        if isSimObjectOrSequence(value) and cls._instantiated:
            raise RuntimeError, \
                  "cannot set SimObject parameter '%s' after\n" \
                  "    class %s has been instantiated or subclassed" \
                  % (attr, cls.__name__)

        # check for param
        param = cls._params.get(attr)
        if param:
            cls._set_param(attr, value, param)
            return

        if isSimObjectOrSequence(value):
            # If RHS is a SimObject, it's an implicit child assignment.
            # Classes don't have children, so we just put this object
            # in _values; later, each instance will do a 'setattr(self,
            # attr, _values[attr])' in SimObject.__init__ which will
            # add this object as a child.
            cls._values[attr] = value
            return

        # no valid assignment... raise exception
        raise AttributeError, \
              "Class %s has no parameter \'%s\'" % (cls.__name__, attr)

    def __getattr__(cls, attr):
        if cls._values.has_key(attr):
            return cls._values[attr]

        raise AttributeError, \
              "object '%s' has no attribute '%s'" % (cls.__name__, attr)

    def __str__(cls):
        return cls.__name__

    def cxx_decl(cls):
        code = "#ifndef __PARAMS__%s\n#define __PARAMS__%s\n\n" % (cls, cls)

        if str(cls) != 'SimObject':
            base = cls.__bases__[0].type
        else:
            base = None

        # The 'dict' attribute restricts us to the params declared in
        # the object itself, not including inherited params (which
        # will also be inherited from the base class's param struct
        # here).
        params = cls._params.dict.values()
        try:
            ptypes = [p.ptype for p in params]
        except:
            print cls, p, p.ptype_str
            print params
            raise

        # get a list of lists of predeclaration lines
        predecls = [p.cxx_predecls() for p in params]
        # flatten
        predecls = reduce(lambda x,y:x+y, predecls, [])
        # remove redundant lines
        predecls2 = []
        for pd in predecls:
            if pd not in predecls2:
                predecls2.append(pd)
        predecls2.sort()
        code += "\n".join(predecls2)
        code += "\n\n";

        if base:
            code += '#include "params/%s.hh"\n\n' % base

        # Generate declarations for locally defined enumerations.
        enum_ptypes = [t for t in ptypes if issubclass(t, Enum)]
        if enum_ptypes:
            code += "\n".join([t.cxx_decl() for t in enum_ptypes])
            code += "\n\n"

        # now generate the actual param struct
        code += "struct %sParams" % cls
        if base:
            code += " : public %sParams" % base
        code += " {\n"
        decls = [p.cxx_decl() for p in params]
        decls.sort()
        code += "".join(["    %s\n" % d for d in decls])
        code += "};\n"

        # close #ifndef __PARAMS__* guard
        code += "\n#endif\n"
        return code

    def swig_decl(cls):

        code = '%%module %sParams\n' % cls

        if str(cls) != 'SimObject':
            base = cls.__bases__[0].type
        else:
            base = None

        # The 'dict' attribute restricts us to the params declared in
        # the object itself, not including inherited params (which
        # will also be inherited from the base class's param struct
        # here).
        params = cls._params.dict.values()
        ptypes = [p.ptype for p in params]

        # get a list of lists of predeclaration lines
        predecls = [p.swig_predecls() for p in params]
        # flatten
        predecls = reduce(lambda x,y:x+y, predecls, [])
        # remove redundant lines
        predecls2 = []
        for pd in predecls:
            if pd not in predecls2:
                predecls2.append(pd)
        predecls2.sort()
        code += "\n".join(predecls2)
        code += "\n\n";

        if base:
            code += '%%import "python/m5/swig/%sParams.i"\n\n' % base

        code += '%{\n'
        code += '#include "params/%s.hh"\n' % cls
        code += '%}\n\n'
        code += '%%include "params/%s.hh"\n\n' % cls

        return code

# The SimObject class is the root of the special hierarchy.  Most of
# the code in this class deals with the configuration hierarchy itself
# (parent/child node relationships).
class SimObject(object):
    # Specify metaclass.  Any class inheriting from SimObject will
    # get this metaclass.
    __metaclass__ = MetaSimObject
    type = 'SimObject'

    name = Param.String("Object name")

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
        self._port_refs = {}
        for key,val in ancestor._port_refs.iteritems():
            self._port_refs[key] = val.clone(self, memo_dict)
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

    def _get_port_ref(self, attr):
        # Return reference that can be assigned to another port
        # via __setattr__.  There is only ever one reference
        # object per port, but we create them lazily here.
        ref = self._port_refs.get(attr)
        if not ref:
            ref = self._ports[attr].makeRef(self)
            self._port_refs[attr] = ref
        return ref

    def __getattr__(self, attr):
        if self._ports.has_key(attr):
            return self._get_port_ref(attr)

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
            self._get_port_ref(attr).connect(value)
            return

        if isSimObjectOrSequence(value) and self._instantiated:
            raise RuntimeError, \
                  "cannot set SimObject parameter '%s' after\n" \
                  "    instance been cloned %s" % (attr, `self`)

        # must be SimObject param
        param = self._params.get(attr)
        if param:
            try:
                value = param.convert(value)
            except Exception, e:
                msg = "%s\nError setting param %s.%s to %s\n" % \
                      (e, self.__class__.__name__, attr, value)
                e.args = (msg, )
                raise
            self._set_child(attr, value)
            return

        if isSimObjectOrSequence(value):
            self._set_child(attr, value)
            return

        # no valid assignment... raise exception
        raise AttributeError, "Class %s has no parameter %s" \
              % (self.__class__.__name__, attr)


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

    def _maybe_set_parent(self, parent, name):
        if not self._parent:
            self._parent = parent
            self._name = name
            parent.add_child(name, self)

    def _set_child(self, attr, value):
        # if RHS is a SimObject, it's an implicit child assignment
        # clear out old child with this name, if any
        self.clear_child(attr)

        if isSimObject(value):
            value._maybe_set_parent(self, attr)
        elif isSimObjectSequence(value):
            value = SimObjVector(value)
            [v._maybe_set_parent(self, "%s%d" % (attr, i))
             for i,v in enumerate(value)]

        self._values[attr] = value

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

    def unproxy_all(self):
        for param in self._params.iterkeys():
            value = self._values.get(param)
            if value != None and proxy.isproxy(value):
                try:
                    value = value.unproxy(self)
                except:
                    print "Error in unproxying param '%s' of %s" % \
                          (param, self.path())
                    raise
                setattr(self, param, value)

        # Unproxy ports in sorted order so that 'append' operations on
        # vector ports are done in a deterministic fashion.
        port_names = self._ports.keys()
        port_names.sort()
        for port_name in port_names:
            port = self._port_refs.get(port_name)
            if port != None:
                port.unproxy(self)

        # Unproxy children in sorted order for determinism also.
        child_names = self._children.keys()
        child_names.sort()
        for child in child_names:
            self._children[child].unproxy_all()

    def print_ini(self):
        print '[' + self.path() + ']'	# .ini section header

        instanceDict[self.path()] = self

        if hasattr(self, 'type'):
            print 'type=%s' % self.type

        child_names = self._children.keys()
        child_names.sort()
        if len(child_names):
            print 'children=%s' % ' '.join(child_names)

        param_names = self._params.keys()
        param_names.sort()
        for param in param_names:
            value = self._values.get(param)
            if value != None:
                print '%s=%s' % (param, self._values[param].ini_str())

        port_names = self._ports.keys()
        port_names.sort()
        for port_name in port_names:
            port = self._port_refs.get(port_name, None)
            if port != None:
                print '%s=%s' % (port_name, port.ini_str())

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
            self._ccObject = internal.sim_object.createSimObject(self.path())
        elif self._ccObject == -1:
            raise RuntimeError, "%s: recursive call to getCCObject()" \
                  % self.path()
        return self._ccObject

    # Create C++ port connections corresponding to the connections in
    # _port_refs (& recursively for all children)
    def connectPorts(self):
        for portRef in self._port_refs.itervalues():
            portRef.ccConnect()
        for child in self._children.itervalues():
            child.connectPorts()

    def startDrain(self, drain_event, recursive):
        count = 0
        if isinstance(self, SimObject):
            count += self._ccObject.drain(drain_event)
        if recursive:
            for child in self._children.itervalues():
                count += child.startDrain(drain_event, True)
        return count

    def resume(self):
        if isinstance(self, SimObject):
            self._ccObject.resume()
        for child in self._children.itervalues():
            child.resume()

    def getMemoryMode(self):
        if not isinstance(self, m5.objects.System):
            return None

        system_ptr = internal.sim_object.convertToSystemPtr(self._ccObject)
        return system_ptr.getMemoryMode()

    def changeTiming(self, mode):
        if isinstance(self, m5.objects.System):
            # i don't know if there's a better way to do this - calling
            # setMemoryMode directly from self._ccObject results in calling
            # SimObject::setMemoryMode, not the System::setMemoryMode
            system_ptr = internal.sim_object.convertToSystemPtr(self._ccObject)
            system_ptr.setMemoryMode(mode)
        for child in self._children.itervalues():
            child.changeTiming(mode)

    def takeOverFrom(self, old_cpu):
        cpu_ptr = internal.sim_object.convertToBaseCPUPtr(old_cpu._ccObject)
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

# Function to provide to C++ so it can look up instances based on paths
def resolveSimObject(name):
    obj = instanceDict[name]
    return obj.getCCObject()

# __all__ defines the list of symbols that get exported when
# 'from config import *' is invoked.  Try to keep this reasonably
# short to avoid polluting other namespaces.
__all__ = ['SimObject']

# see comment on imports at end of __init__.py.
import proxy
import internal
import m5
