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

import sys, inspect, copy
import convert
from util import *

# Dummy base class to identify types that are legitimate for SimObject
# parameters.
class ParamValue(object):

    cxx_predecls = []
    swig_predecls = []

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
                ptype = eval(self.ptype_str, objects.__dict__)
                if not isinstance(ptype, type):
                    raise NameError
                self.ptype = ptype
                return ptype
            except NameError:
                raise TypeError, \
                      "Param qualifier '%s' is not a type" % self.ptype_str
        raise AttributeError, "'%s' object has no attribute '%s'" % \
              (type(self).__name__, attr)

    def convert(self, value):
        if isinstance(value, proxy.BaseProxy):
            value.set_param_desc(self)
            return value
        if not hasattr(self, 'ptype') and isNullPointer(value):
            # deferred evaluation of SimObject; continue to defer if
            # we're just assigning a null pointer
            return value
        if isinstance(value, self.ptype):
            return value
        if isNullPointer(value) and isSimObjectClass(self.ptype):
            return value
        return self.ptype(value)

    def cxx_predecls(self):
        return self.ptype.cxx_predecls

    def swig_predecls(self):
        return self.ptype.swig_predecls

    def cxx_decl(self):
        return '%s %s;' % (self.ptype.cxx_type, self.name)

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

    def cxx_predecls(self):
        return ['#include <vector>'] + self.ptype.cxx_predecls

    def swig_predecls(self):
        return ['%include "std_vector.i"'] + self.ptype.swig_predecls

    def cxx_decl(self):
        return 'std::vector< %s > %s;' % (self.ptype.cxx_type, self.name)

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
# the __str__() conversion method).
#
#####################################################################

# String-valued parameter.  Just mixin the ParamValue class with the
# built-in str class.
class String(ParamValue,str):
    cxx_type = 'std::string'
    cxx_predecls = ['#include <string>']
    swig_predecls = ['%include "std_string.i"\n' +
                     '%apply const std::string& {std::string *};']
    pass

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

# Metaclass for bounds-checked integer parameters.  See CheckedInt.
class CheckedIntType(type):
    def __init__(cls, name, bases, dict):
        super(CheckedIntType, cls).__init__(name, bases, dict)

        # CheckedInt is an abstract base class, so we actually don't
        # want to do any processing on it... the rest of this code is
        # just for classes that derive from CheckedInt.
        if name == 'CheckedInt':
            return

        if not cls.cxx_predecls:
            # most derived types require this, so we just do it here once
            cls.cxx_predecls = ['#include "sim/host.hh"']

        if not cls.swig_predecls:
            # most derived types require this, so we just do it here once
            cls.swig_predecls = ['%import "python/m5/swig/stdint.i"\n' +
                                 '%import "sim/host.hh"']

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
            self.value = convert.toInteger(value)
        elif isinstance(value, (int, long, float)):
            self.value = long(value)
        self._check()

class Int(CheckedInt):      cxx_type = 'int';      size = 32; unsigned = False
class Unsigned(CheckedInt): cxx_type = 'unsigned'; size = 32; unsigned = True

class Int8(CheckedInt):     cxx_type =   'int8_t'; size =  8; unsigned = False
class UInt8(CheckedInt):    cxx_type =  'uint8_t'; size =  8; unsigned = True
class Int16(CheckedInt):    cxx_type =  'int16_t'; size = 16; unsigned = False
class UInt16(CheckedInt):   cxx_type = 'uint16_t'; size = 16; unsigned = True
class Int32(CheckedInt):    cxx_type =  'int32_t'; size = 32; unsigned = False
class UInt32(CheckedInt):   cxx_type = 'uint32_t'; size = 32; unsigned = True
class Int64(CheckedInt):    cxx_type =  'int64_t'; size = 64; unsigned = False
class UInt64(CheckedInt):   cxx_type = 'uint64_t'; size = 64; unsigned = True

class Counter(CheckedInt):  cxx_type = 'Counter';  size = 64; unsigned = True
class Tick(CheckedInt):     cxx_type = 'Tick';     size = 64; unsigned = True
class TcpPort(CheckedInt):  cxx_type = 'uint16_t'; size = 16; unsigned = True
class UdpPort(CheckedInt):  cxx_type = 'uint16_t'; size = 16; unsigned = True

class Percent(CheckedInt):  cxx_type = 'int'; min = 0; max = 100

class Float(ParamValue, float):
    pass

class MemorySize(CheckedInt):
    cxx_type = 'uint64_t'
    size = 64
    unsigned = True
    def __init__(self, value):
        if isinstance(value, MemorySize):
            self.value = value.value
        else:
            self.value = convert.toMemorySize(value)
        self._check()

class MemorySize32(CheckedInt):
    size = 32
    unsigned = True
    def __init__(self, value):
        if isinstance(value, MemorySize):
            self.value = value.value
        else:
            self.value = convert.toMemorySize(value)
        self._check()

class Addr(CheckedInt):
    cxx_type = 'Addr'
    cxx_predecls = ['#include "targetarch/isa_traits.hh"']
    size = 64
    unsigned = True
    def __init__(self, value):
        if isinstance(value, Addr):
            self.value = value.value
        else:
            try:
                self.value = convert.toMemorySize(value)
            except TypeError:
                self.value = long(value)
        self._check()


class MetaRange(type):
    def __init__(cls, name, bases, dict):
        super(MetaRange, cls).__init__(name, bases, dict)
        if name == 'Range':
            return
        cls.cxx_type = 'Range< %s >' % cls.type.cxx_type
        cls.cxx_predecls = \
                       ['#include "base/range.hh"'] + cls.type.cxx_predecls

class Range(ParamValue):
    __metaclass__ = MetaRange
    type = Int # default; can be overridden in subclasses
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

class AddrRange(Range):
    type = Addr

class TickRange(Range):
    type = Tick

# Boolean parameter type.  Python doesn't let you subclass bool, since
# it doesn't want to let you create multiple instances of True and
# False.  Thus this is a little more complicated than String.
class Bool(ParamValue):
    cxx_type = 'bool'
    def __init__(self, value):
        try:
            self.value = convert.toBool(value)
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
    cxx_type = 'Net::EthAddr'
    cxx_predecls = ['#include "base/inet.hh"']
    swig_predecls = ['class Net::EthAddr;']
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

        cls.cxx_type = name + '::Enum'

        super(MetaEnum, cls).__init__(name, bases, init_dict)

    # Generate C++ class declaration for this enum type.
    # Note that we wrap the enum in a class/struct to act as a namespace,
    # so that the enum strings can be brief w/o worrying about collisions.
    def cxx_decl(cls):
        s = 'struct %s {\n  enum Enum {\n    ' % cls.__name__
        s += ',\n    '.join(['%s = %d' % (v,cls.map[v]) for v in cls.vals])
        s += '\n  };\n};\n'
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
            return convert.toLatency(value)
        except ValueError:
            try:
                return 1 / convert.toFrequency(value)
            except ValueError:
                pass # fall through
    raise ValueError, "Invalid Frequency/Latency value '%s'" % value


class Latency(NumericParamValue):
    cxx_type = 'Tick'
    cxx_predecls = ['#include "sim/host.hh"']
    swig_predecls = ['%import "python/m5/swig/stdint.i"\n' +
                     '%import "sim/host.hh"']
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
    cxx_type = 'Tick'
    cxx_predecls = ['#include "sim/host.hh"']
    swig_predecls = ['%import "python/m5/swig/stdint.i"\n' +
                     '%import "sim/host.hh"']
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
    cxx_type = 'Tick'
    cxx_predecls = ['#include "sim/host.hh"']
    swig_predecls = ['%import "python/m5/swig/stdint.i"\n' +
                     '%import "sim/host.hh"']
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
    cxx_type = 'Tick'
    cxx_predecls = ['#include "sim/host.hh"']
    swig_predecls = ['%import "python/m5/swig/stdint.i"\n' +
                     '%import "sim/host.hh"']
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
    cxx_type = 'float'
    def __new__(cls, value):
        val = convert.toNetworkBandwidth(value) / 8.0
        return super(cls, NetworkBandwidth).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def ini_str(self):
        return '%f' % (ticks_per_sec / float(self))

class MemoryBandwidth(float,ParamValue):
    cxx_type = 'float'
    def __new__(self, value):
        val = convert.toMemoryBandwidth(value)
        return super(cls, MemoryBandwidth).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def ini_str(self):
        return '%f' % (ticks_per_sec / float(self))

#
# "Constants"... handy aliases for various values.
#

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
NULL = NullSimObject()

def isNullPointer(value):
    return isinstance(value, NullSimObject)

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


__all__ = ['Param', 'VectorParam',
           'Enum', 'Bool', 'String', 'Float',
           'Int', 'Unsigned', 'Int8', 'UInt8', 'Int16', 'UInt16',
           'Int32', 'UInt32', 'Int64', 'UInt64',
           'Counter', 'Addr', 'Tick', 'Percent',
           'TcpPort', 'UdpPort', 'EthernetAddr',
           'MemorySize', 'MemorySize32',
           'Latency', 'Frequency', 'RootClock', 'Clock',
           'NetworkBandwidth', 'MemoryBandwidth',
           'Range', 'AddrRange', 'TickRange',
           'MaxAddr', 'MaxTick', 'AllMemory',
           'NextEthernetAddr', 'NULL',
           'Port', 'VectorPort']

# see comment on imports at end of __init__.py.
from SimObject import isSimObject, isSimObjectSequence, isSimObjectClass
import proxy
import objects
import cc_main
