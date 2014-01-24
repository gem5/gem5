# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2004-2006 The Regents of The University of Michigan
# Copyright (c) 2010-2011 Advanced Micro Devices, Inc.
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
#          Gabe Black
#          Andreas Hansson

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

import copy
import datetime
import re
import sys
import time
import math

import proxy
import ticks
from util import *

def isSimObject(*args, **kwargs):
    return SimObject.isSimObject(*args, **kwargs)

def isSimObjectSequence(*args, **kwargs):
    return SimObject.isSimObjectSequence(*args, **kwargs)

def isSimObjectClass(*args, **kwargs):
    return SimObject.isSimObjectClass(*args, **kwargs)

allParams = {}

class MetaParamValue(type):
    def __new__(mcls, name, bases, dct):
        cls = super(MetaParamValue, mcls).__new__(mcls, name, bases, dct)
        assert name not in allParams
        allParams[name] = cls
        return cls


# Dummy base class to identify types that are legitimate for SimObject
# parameters.
class ParamValue(object):
    __metaclass__ = MetaParamValue


    # Generate the code needed as a prerequisite for declaring a C++
    # object of this type.  Typically generates one or more #include
    # statements.  Used when declaring parameters of this type.
    @classmethod
    def cxx_predecls(cls, code):
        pass

    # Generate the code needed as a prerequisite for including a
    # reference to a C++ object of this type in a SWIG .i file.
    # Typically generates one or more %import or %include statements.
    @classmethod
    def swig_predecls(cls, code):
        pass

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
            ptype = SimObject.allClasses[self.ptype_str]
            assert isSimObjectClass(ptype)
            self.ptype = ptype
            return ptype

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

    def cxx_predecls(self, code):
        code('#include <cstddef>')
        self.ptype.cxx_predecls(code)

    def swig_predecls(self, code):
        self.ptype.swig_predecls(code)

    def cxx_decl(self, code):
        code('${{self.ptype.cxx_type}} ${{self.name}};')

# Vector-valued parameter description.  Just like ParamDesc, except
# that the value is a vector (list) of the specified type instead of a
# single value.

class VectorParamValue(list):
    __metaclass__ = MetaParamValue
    def __setattr__(self, attr, value):
        raise AttributeError, \
              "Not allowed to set %s on '%s'" % (attr, type(self).__name__)

    def ini_str(self):
        return ' '.join([v.ini_str() for v in self])

    def getValue(self):
        return [ v.getValue() for v in self ]

    def unproxy(self, base):
        if len(self) == 1 and isinstance(self[0], proxy.AllProxy):
            return self[0].unproxy(base)
        else:
             return [v.unproxy(base) for v in self]

class SimObjectVector(VectorParamValue):
    # support clone operation
    def __call__(self, **kwargs):
        return SimObjectVector([v(**kwargs) for v in self])

    def clear_parent(self, old_parent):
        for v in self:
            v.clear_parent(old_parent)

    def set_parent(self, parent, name):
        if len(self) == 1:
            self[0].set_parent(parent, name)
        else:
            width = int(math.ceil(math.log(len(self))/math.log(10)))
            for i,v in enumerate(self):
                v.set_parent(parent, "%s%0*d" % (name, width, i))

    def has_parent(self):
        return reduce(lambda x,y: x and y, [v.has_parent() for v in self])

    # return 'cpu0 cpu1' etc. for print_ini()
    def get_name(self):
        return ' '.join([v._name for v in self])

    # By iterating through the constituent members of the vector here
    # we can nicely handle iterating over all a SimObject's children
    # without having to provide lots of special functions on
    # SimObjectVector directly.
    def descendants(self):
        for v in self:
            for obj in v.descendants():
                yield obj

    def get_config_as_dict(self):
        a = []
        for v in self:
            a.append(v.get_config_as_dict())
        return a

    # If we are replacing an item in the vector, make sure to set the
    # parent reference of the new SimObject to be the same as the parent
    # of the SimObject being replaced. Useful to have if we created
    # a SimObjectVector of temporary objects that will be modified later in
    # configuration scripts.
    def __setitem__(self, key, value):
        val = self[key]
        if value.has_parent():
            warn("SimObject %s already has a parent" % value.get_name() +\
                 " that is being overwritten by a SimObjectVector")
        value.set_parent(val.get_parent(), val._name)
        super(SimObjectVector, self).__setitem__(key, value)

class VectorParamDesc(ParamDesc):
    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def convert(self, value):
        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            tmp_list = [ ParamDesc.convert(self, v) for v in value ]
        else:
            # singleton: coerce to a single-element list
            tmp_list = [ ParamDesc.convert(self, value) ]

        if isSimObjectSequence(tmp_list):
            return SimObjectVector(tmp_list)
        else:
            return VectorParamValue(tmp_list)

    def swig_module_name(self):
        return "%s_vector" % self.ptype_str

    def swig_predecls(self, code):
        code('%import "${{self.swig_module_name()}}.i"')

    def swig_decl(self, code):
        code('%module(package="m5.internal") ${{self.swig_module_name()}}')
        code('%{')
        self.ptype.cxx_predecls(code)
        code('%}')
        code()
        # Make sure the SWIGPY_SLICE_ARG is defined through this inclusion
        code('%include "std_container.i"')
        code()
        self.ptype.swig_predecls(code)
        code()
        code('%include "std_vector.i"')
        code()

        ptype = self.ptype_str
        cxx_type = self.ptype.cxx_type

        code('''\
%typemap(in) std::vector< $cxx_type >::value_type {
    if (SWIG_ConvertPtr($$input, (void **)&$$1, $$1_descriptor, 0) == -1) {
        if (SWIG_ConvertPtr($$input, (void **)&$$1,
                            $$descriptor($cxx_type), 0) == -1) {
            return NULL;
        }
    }
}

%typemap(in) std::vector< $cxx_type >::value_type * {
    if (SWIG_ConvertPtr($$input, (void **)&$$1, $$1_descriptor, 0) == -1) {
        if (SWIG_ConvertPtr($$input, (void **)&$$1,
                            $$descriptor($cxx_type *), 0) == -1) {
            return NULL;
        }
    }
}
''')

        code('%template(vector_$ptype) std::vector< $cxx_type >;')

    def cxx_predecls(self, code):
        code('#include <vector>')
        self.ptype.cxx_predecls(code)

    def cxx_decl(self, code):
        code('std::vector< ${{self.ptype.cxx_type}} > ${{self.name}};')

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
        ptype = None
        try:
            ptype = allParams[self.ptype_str]
        except KeyError:
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

    @classmethod
    def cxx_predecls(self, code):
        code('#include <string>')

    @classmethod
    def swig_predecls(cls, code):
        code('%include "std_string.i"')

    def getValue(self):
        return self

# superclass for "numeric" parameter values, to emulate math
# operations in a type-safe way.  e.g., a Latency times an int returns
# a new Latency object.
class NumericParamValue(ParamValue):
    def __str__(self):
        return str(self.value)

    def __float__(self):
        return float(self.value)

    def __long__(self):
        return long(self.value)

    def __int__(self):
        return int(self.value)

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
class CheckedIntType(MetaParamValue):
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
                      "    'min' and 'max' or 'size' and 'unsigned'\n",
                      name);
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
        elif isinstance(value, (int, long, float, NumericParamValue)):
            self.value = long(value)
        else:
            raise TypeError, "Can't convert object of type %s to CheckedInt" \
                  % type(value).__name__
        self._check()

    @classmethod
    def cxx_predecls(cls, code):
        # most derived types require this, so we just do it here once
        code('#include "base/types.hh"')

    @classmethod
    def swig_predecls(cls, code):
        # most derived types require this, so we just do it here once
        code('%import "stdint.i"')
        code('%import "base/types.hh"')

    def getValue(self):
        return long(self.value)

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

class Cycles(CheckedInt):
    cxx_type = 'Cycles'
    size = 64
    unsigned = True

    def getValue(self):
        from m5.internal.core import Cycles
        return Cycles(self.value)

class Float(ParamValue, float):
    cxx_type = 'double'

    def __init__(self, value):
        if isinstance(value, (int, long, float, NumericParamValue, Float)):
            self.value = float(value)
        else:
            raise TypeError, "Can't convert object of type %s to Float" \
                  % type(value).__name__

    def getValue(self):
        return float(self.value)

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
    cxx_type = 'uint32_t'
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
    def __add__(self, other):
        if isinstance(other, Addr):
            return self.value + other.value
        else:
            return self.value + other

class AddrRange(ParamValue):
    cxx_type = 'AddrRange'

    def __init__(self, *args, **kwargs):
        # Disable interleaving by default
        self.intlvHighBit = 0
        self.intlvBits = 0
        self.intlvMatch = 0

        def handle_kwargs(self, kwargs):
            # An address range needs to have an upper limit, specified
            # either explicitly with an end, or as an offset using the
            # size keyword.
            if 'end' in kwargs:
                self.end = Addr(kwargs.pop('end'))
            elif 'size' in kwargs:
                self.end = self.start + Addr(kwargs.pop('size')) - 1
            else:
                raise TypeError, "Either end or size must be specified"

            # Now on to the optional bit
            if 'intlvHighBit' in kwargs:
                self.intlvHighBit = int(kwargs.pop('intlvHighBit'))
            if 'intlvBits' in kwargs:
                self.intlvBits = int(kwargs.pop('intlvBits'))
            if 'intlvMatch' in kwargs:
                self.intlvMatch = int(kwargs.pop('intlvMatch'))

        if len(args) == 0:
            self.start = Addr(kwargs.pop('start'))
            handle_kwargs(self, kwargs)

        elif len(args) == 1:
            if kwargs:
                self.start = Addr(args[0])
                handle_kwargs(self, kwargs)
            elif isinstance(args[0], (list, tuple)):
                self.start = Addr(args[0][0])
                self.end = Addr(args[0][1])
            else:
                self.start = Addr(0)
                self.end = Addr(args[0]) - 1

        elif len(args) == 2:
            self.start = Addr(args[0])
            self.end = Addr(args[1])
        else:
            raise TypeError, "Too many arguments specified"

        if kwargs:
            raise TypeError, "Too many keywords: %s" % kwargs.keys()

    def __str__(self):
        return '%s:%s' % (self.start, self.end)

    def size(self):
        # Divide the size by the size of the interleaving slice
        return (long(self.end) - long(self.start) + 1) >> self.intlvBits

    @classmethod
    def cxx_predecls(cls, code):
        Addr.cxx_predecls(code)
        code('#include "base/addr_range.hh"')

    @classmethod
    def swig_predecls(cls, code):
        Addr.swig_predecls(code)

    def getValue(self):
        # Go from the Python class to the wrapped C++ class generated
        # by swig
        from m5.internal.range import AddrRange

        return AddrRange(long(self.start), long(self.end),
                         int(self.intlvHighBit), int(self.intlvBits),
                         int(self.intlvMatch))

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

    def getValue(self):
        return bool(self.value)

    def __str__(self):
        return str(self.value)

    # implement truth value testing for Bool parameters so that these params
    # evaluate correctly during the python configuration phase
    def __nonzero__(self):
        return bool(self.value)

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

_NextEthernetAddr = "00:90:00:00:00:01"
def NextEthernetAddr():
    global _NextEthernetAddr

    value = _NextEthernetAddr
    _NextEthernetAddr = IncEthernetAddr(_NextEthernetAddr, 1)
    return value

class EthernetAddr(ParamValue):
    cxx_type = 'Net::EthAddr'

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    @classmethod
    def swig_predecls(cls, code):
        code('%include "python/swig/inet.i"')

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
            if not 0 <= int(byte, base=16) <= 0xff:
                raise TypeError, 'invalid ethernet address %s' % value

        self.value = value

    def unproxy(self, base):
        if self.value == NextEthernetAddr:
            return EthernetAddr(self.value())
        return self

    def getValue(self):
        from m5.internal.params import EthAddr
        return EthAddr(self.value)

    def ini_str(self):
        return self.value

# When initializing an IpAddress, pass in an existing IpAddress, a string of
# the form "a.b.c.d", or an integer representing an IP.
class IpAddress(ParamValue):
    cxx_type = 'Net::IpAddress'

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    @classmethod
    def swig_predecls(cls, code):
        code('%include "python/swig/inet.i"')

    def __init__(self, value):
        if isinstance(value, IpAddress):
            self.ip = value.ip
        else:
            try:
                self.ip = convert.toIpAddress(value)
            except TypeError:
                self.ip = long(value)
        self.verifyIp()

    def __str__(self):
        tup = [(self.ip >> i)  & 0xff for i in (24, 16, 8, 0)]
        return '%d.%d.%d.%d' % tuple(tup)

    def __eq__(self, other):
        if isinstance(other, IpAddress):
            return self.ip == other.ip
        elif isinstance(other, str):
            try:
                return self.ip == convert.toIpAddress(other)
            except:
                return False
        else:
            return self.ip == other

    def __ne__(self, other):
        return not (self == other)

    def verifyIp(self):
        if self.ip < 0 or self.ip >= (1 << 32):
            raise TypeError, "invalid ip address %#08x" % self.ip

    def getValue(self):
        from m5.internal.params import IpAddress
        return IpAddress(self.ip)

# When initializing an IpNetmask, pass in an existing IpNetmask, a string of
# the form "a.b.c.d/n" or "a.b.c.d/e.f.g.h", or an ip and netmask as
# positional or keyword arguments.
class IpNetmask(IpAddress):
    cxx_type = 'Net::IpNetmask'

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    @classmethod
    def swig_predecls(cls, code):
        code('%include "python/swig/inet.i"')

    def __init__(self, *args, **kwargs):
        def handle_kwarg(self, kwargs, key, elseVal = None):
            if key in kwargs:
                setattr(self, key, kwargs.pop(key))
            elif elseVal:
                setattr(self, key, elseVal)
            else:
                raise TypeError, "No value set for %s" % key

        if len(args) == 0:
            handle_kwarg(self, kwargs, 'ip')
            handle_kwarg(self, kwargs, 'netmask')

        elif len(args) == 1:
            if kwargs:
                if not 'ip' in kwargs and not 'netmask' in kwargs:
                    raise TypeError, "Invalid arguments"
                handle_kwarg(self, kwargs, 'ip', args[0])
                handle_kwarg(self, kwargs, 'netmask', args[0])
            elif isinstance(args[0], IpNetmask):
                self.ip = args[0].ip
                self.netmask = args[0].netmask
            else:
                (self.ip, self.netmask) = convert.toIpNetmask(args[0])

        elif len(args) == 2:
            self.ip = args[0]
            self.netmask = args[1]
        else:
            raise TypeError, "Too many arguments specified"

        if kwargs:
            raise TypeError, "Too many keywords: %s" % kwargs.keys()

        self.verify()

    def __str__(self):
        return "%s/%d" % (super(IpNetmask, self).__str__(), self.netmask)

    def __eq__(self, other):
        if isinstance(other, IpNetmask):
            return self.ip == other.ip and self.netmask == other.netmask
        elif isinstance(other, str):
            try:
                return (self.ip, self.netmask) == convert.toIpNetmask(other)
            except:
                return False
        else:
            return False

    def verify(self):
        self.verifyIp()
        if self.netmask < 0 or self.netmask > 32:
            raise TypeError, "invalid netmask %d" % netmask

    def getValue(self):
        from m5.internal.params import IpNetmask
        return IpNetmask(self.ip, self.netmask)

# When initializing an IpWithPort, pass in an existing IpWithPort, a string of
# the form "a.b.c.d:p", or an ip and port as positional or keyword arguments.
class IpWithPort(IpAddress):
    cxx_type = 'Net::IpWithPort'

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    @classmethod
    def swig_predecls(cls, code):
        code('%include "python/swig/inet.i"')

    def __init__(self, *args, **kwargs):
        def handle_kwarg(self, kwargs, key, elseVal = None):
            if key in kwargs:
                setattr(self, key, kwargs.pop(key))
            elif elseVal:
                setattr(self, key, elseVal)
            else:
                raise TypeError, "No value set for %s" % key

        if len(args) == 0:
            handle_kwarg(self, kwargs, 'ip')
            handle_kwarg(self, kwargs, 'port')

        elif len(args) == 1:
            if kwargs:
                if not 'ip' in kwargs and not 'port' in kwargs:
                    raise TypeError, "Invalid arguments"
                handle_kwarg(self, kwargs, 'ip', args[0])
                handle_kwarg(self, kwargs, 'port', args[0])
            elif isinstance(args[0], IpWithPort):
                self.ip = args[0].ip
                self.port = args[0].port
            else:
                (self.ip, self.port) = convert.toIpWithPort(args[0])

        elif len(args) == 2:
            self.ip = args[0]
            self.port = args[1]
        else:
            raise TypeError, "Too many arguments specified"

        if kwargs:
            raise TypeError, "Too many keywords: %s" % kwargs.keys()

        self.verify()

    def __str__(self):
        return "%s:%d" % (super(IpWithPort, self).__str__(), self.port)

    def __eq__(self, other):
        if isinstance(other, IpWithPort):
            return self.ip == other.ip and self.port == other.port
        elif isinstance(other, str):
            try:
                return (self.ip, self.port) == convert.toIpWithPort(other)
            except:
                return False
        else:
            return False

    def verify(self):
        self.verifyIp()
        if self.port < 0 or self.port > 0xffff:
            raise TypeError, "invalid port %d" % self.port

    def getValue(self):
        from m5.internal.params import IpWithPort
        return IpWithPort(self.ip, self.port)

time_formats = [ "%a %b %d %H:%M:%S %Z %Y",
                 "%a %b %d %H:%M:%S %Z %Y",
                 "%Y/%m/%d %H:%M:%S",
                 "%Y/%m/%d %H:%M",
                 "%Y/%m/%d",
                 "%m/%d/%Y %H:%M:%S",
                 "%m/%d/%Y %H:%M",
                 "%m/%d/%Y",
                 "%m/%d/%y %H:%M:%S",
                 "%m/%d/%y %H:%M",
                 "%m/%d/%y"]


def parse_time(value):
    from time import gmtime, strptime, struct_time, time
    from datetime import datetime, date

    if isinstance(value, struct_time):
        return value

    if isinstance(value, (int, long)):
        return gmtime(value)

    if isinstance(value, (datetime, date)):
        return value.timetuple()

    if isinstance(value, str):
        if value in ('Now', 'Today'):
            return time.gmtime(time.time())

        for format in time_formats:
            try:
                return strptime(value, format)
            except ValueError:
                pass

    raise ValueError, "Could not parse '%s' as a time" % value

class Time(ParamValue):
    cxx_type = 'tm'

    @classmethod
    def cxx_predecls(cls, code):
        code('#include <time.h>')

    @classmethod
    def swig_predecls(cls, code):
        code('%include "python/swig/time.i"')

    def __init__(self, value):
        self.value = parse_time(value)

    def getValue(self):
        from m5.internal.params import tm

        c_time = tm()
        py_time = self.value

        # UNIX is years since 1900
        c_time.tm_year = py_time.tm_year - 1900;

        # Python starts at 1, UNIX starts at 0
        c_time.tm_mon =  py_time.tm_mon - 1;
        c_time.tm_mday = py_time.tm_mday;
        c_time.tm_hour = py_time.tm_hour;
        c_time.tm_min = py_time.tm_min;
        c_time.tm_sec = py_time.tm_sec;

        # Python has 0 as Monday, UNIX is 0 as sunday
        c_time.tm_wday = py_time.tm_wday + 1
        if c_time.tm_wday > 6:
            c_time.tm_wday -= 7;

        # Python starts at 1, Unix starts at 0
        c_time.tm_yday = py_time.tm_yday - 1;

        return c_time

    def __str__(self):
        return time.asctime(self.value)

    def ini_str(self):
        return str(self)

    def get_config_as_dict(self):
        return str(self)

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

allEnums = {}
# Metaclass for Enum types
class MetaEnum(MetaParamValue):
    def __new__(mcls, name, bases, dict):
        assert name not in allEnums

        cls = super(MetaEnum, mcls).__new__(mcls, name, bases, dict)
        allEnums[name] = cls
        return cls

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

        cls.cxx_type = 'Enums::%s' % name

        super(MetaEnum, cls).__init__(name, bases, init_dict)

    # Generate C++ class declaration for this enum type.
    # Note that we wrap the enum in a class/struct to act as a namespace,
    # so that the enum strings can be brief w/o worrying about collisions.
    def cxx_decl(cls, code):
        name = cls.__name__
        code('''\
#ifndef __ENUM__${name}__
#define __ENUM__${name}__

namespace Enums {
    enum $name {
''')
        code.indent(2)
        for val in cls.vals:
            code('$val = ${{cls.map[val]}},')
        code('Num_$name = ${{len(cls.vals)}}')
        code.dedent(2)
        code('''\
    };
extern const char *${name}Strings[Num_${name}];
}

#endif // __ENUM__${name}__
''')

    def cxx_def(cls, code):
        name = cls.__name__
        code('''\
#include "enums/$name.hh"
namespace Enums {
    const char *${name}Strings[Num_${name}] =
    {
''')
        code.indent(2)
        for val in cls.vals:
            code('"$val",')
        code.dedent(2)
        code('''
    };
} // namespace Enums
''')

    def swig_decl(cls, code):
        name = cls.__name__
        code('''\
%module(package="m5.internal") enum_$name

%{
#include "enums/$name.hh"
%}

%include "enums/$name.hh"
''')


# Base class for enum types.
class Enum(ParamValue):
    __metaclass__ = MetaEnum
    vals = []

    def __init__(self, value):
        if value not in self.map:
            raise TypeError, "Enum param got bad value '%s' (not in %s)" \
                  % (value, self.vals)
        self.value = value

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "enums/$0.hh"', cls.__name__)

    @classmethod
    def swig_predecls(cls, code):
        code('%import "python/m5/internal/enum_$0.i"', cls.__name__)

    def getValue(self):
        return int(self.map[self.value])

    def __str__(self):
        return self.value

# how big does a rounding error need to be before we warn about it?
frequency_tolerance = 0.001  # 0.1%

class TickParamValue(NumericParamValue):
    cxx_type = 'Tick'

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/types.hh"')

    @classmethod
    def swig_predecls(cls, code):
        code('%import "stdint.i"')
        code('%import "base/types.hh"')

    def getValue(self):
        return long(self.value)

class Latency(TickParamValue):
    def __init__(self, value):
        if isinstance(value, (Latency, Clock)):
            self.ticks = value.ticks
            self.value = value.value
        elif isinstance(value, Frequency):
            self.ticks = value.ticks
            self.value = 1.0 / value.value
        elif value.endswith('t'):
            self.ticks = True
            self.value = int(value[:-1])
        else:
            self.ticks = False
            self.value = convert.toLatency(value)

    def __getattr__(self, attr):
        if attr in ('latency', 'period'):
            return self
        if attr == 'frequency':
            return Frequency(self)
        raise AttributeError, "Latency object has no attribute '%s'" % attr

    def getValue(self):
        if self.ticks or self.value == 0:
            value = self.value
        else:
            value = ticks.fromSeconds(self.value)
        return long(value)

    # convert latency to ticks
    def ini_str(self):
        return '%d' % self.getValue()

class Frequency(TickParamValue):
    def __init__(self, value):
        if isinstance(value, (Latency, Clock)):
            if value.value == 0:
                self.value = 0
            else:
                self.value = 1.0 / value.value
            self.ticks = value.ticks
        elif isinstance(value, Frequency):
            self.value = value.value
            self.ticks = value.ticks
        else:
            self.ticks = False
            self.value = convert.toFrequency(value)

    def __getattr__(self, attr):
        if attr == 'frequency':
            return self
        if attr in ('latency', 'period'):
            return Latency(self)
        raise AttributeError, "Frequency object has no attribute '%s'" % attr

    # convert latency to ticks
    def getValue(self):
        if self.ticks or self.value == 0:
            value = self.value
        else:
            value = ticks.fromSeconds(1.0 / self.value)
        return long(value)

    def ini_str(self):
        return '%d' % self.getValue()

# A generic Frequency and/or Latency value. Value is stored as a
# latency, just like Latency and Frequency.
class Clock(TickParamValue):
    def __init__(self, value):
        if isinstance(value, (Latency, Clock)):
            self.ticks = value.ticks
            self.value = value.value
        elif isinstance(value, Frequency):
            self.ticks = value.ticks
            self.value = 1.0 / value.value
        elif value.endswith('t'):
            self.ticks = True
            self.value = int(value[:-1])
        else:
            self.ticks = False
            self.value = convert.anyToLatency(value)

    def __getattr__(self, attr):
        if attr == 'frequency':
            return Frequency(self)
        if attr in ('latency', 'period'):
            return Latency(self)
        raise AttributeError, "Frequency object has no attribute '%s'" % attr

    def getValue(self):
        return self.period.getValue()

    def ini_str(self):
        return self.period.ini_str()

class Voltage(float,ParamValue):
    cxx_type = 'double'
    def __new__(cls, value):
        # convert to voltage
        val = convert.toVoltage(value)
        return super(cls, Voltage).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def getValue(self):
        value = float(self)
        return value

    def ini_str(self):
        return '%f' % self.getValue()

class NetworkBandwidth(float,ParamValue):
    cxx_type = 'float'
    def __new__(cls, value):
        # convert to bits per second
        val = convert.toNetworkBandwidth(value)
        return super(cls, NetworkBandwidth).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def getValue(self):
        # convert to seconds per byte
        value = 8.0 / float(self)
        # convert to ticks per byte
        value = ticks.fromSeconds(value)
        return float(value)

    def ini_str(self):
        return '%f' % self.getValue()

class MemoryBandwidth(float,ParamValue):
    cxx_type = 'float'
    def __new__(cls, value):
        # convert to bytes per second
        val = convert.toMemoryBandwidth(value)
        return super(cls, MemoryBandwidth).__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def getValue(self):
        # convert to seconds per byte
        value = float(self)
        if value:
            value = 1.0 / float(self)
        # convert to ticks per byte
        value = ticks.fromSeconds(value)
        return float(value)

    def ini_str(self):
        return '%f' % self.getValue()

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

    def getValue(self):
        return None

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
    def __init__(self, simobj, name, role):
        assert(isSimObject(simobj) or isSimObjectClass(simobj))
        self.simobj = simobj
        self.name = name
        self.role = role
        self.peer = None   # not associated with another port yet
        self.ccConnected = False # C++ port connection done?
        self.index = -1  # always -1 for non-vector ports

    def __str__(self):
        return '%s.%s' % (self.simobj, self.name)

    def __len__(self):
        # Return the number of connected ports, i.e. 0 is we have no
        # peer and 1 if we do.
        return int(self.peer != None)

    # for config.ini, print peer's name (not ours)
    def ini_str(self):
        return str(self.peer)

    # for config.json
    def get_config_as_dict(self):
        return {'role' : self.role, 'peer' : str(self.peer)}

    def __getattr__(self, attr):
        if attr == 'peerObj':
            # shorthand for proxies
            return self.peer.simobj
        raise AttributeError, "'%s' object has no attribute '%s'" % \
              (self.__class__.__name__, attr)

    # Full connection is symmetric (both ways).  Called via
    # SimObject.__setattr__ as a result of a port assignment, e.g.,
    # "obj1.portA = obj2.portB", or via VectorPortElementRef.__setitem__,
    # e.g., "obj1.portA[3] = obj2.portB".
    def connect(self, other):
        if isinstance(other, VectorPortRef):
            # reference to plain VectorPort is implicit append
            other = other._get_next()
        if self.peer and not proxy.isproxy(self.peer):
            fatal("Port %s is already connected to %s, cannot connect %s\n",
                  self, self.peer, other);
        self.peer = other
        if proxy.isproxy(other):
            other.set_param_desc(PortParamDesc())
        elif isinstance(other, PortRef):
            if other.peer is not self:
                other.connect(self)
        else:
            raise TypeError, \
                  "assigning non-port reference '%s' to port '%s'" \
                  % (other, self)

    def clone(self, simobj, memo):
        if memo.has_key(self):
            return memo[self]
        newRef = copy.copy(self)
        memo[self] = newRef
        newRef.simobj = simobj
        assert(isSimObject(newRef.simobj))
        if self.peer and not proxy.isproxy(self.peer):
            peerObj = self.peer.simobj(_memo=memo)
            newRef.peer = self.peer.clone(peerObj, memo)
            assert(not isinstance(newRef.peer, VectorPortRef))
        return newRef

    def unproxy(self, simobj):
        assert(simobj is self.simobj)
        if proxy.isproxy(self.peer):
            try:
                realPeer = self.peer.unproxy(self.simobj)
            except:
                print "Error in unproxying port '%s' of %s" % \
                      (self.name, self.simobj.path())
                raise
            self.connect(realPeer)

    # Call C++ to create corresponding port connection between C++ objects
    def ccConnect(self):
        from m5.internal.pyobject import connectPorts

        if self.role == 'SLAVE':
            # do nothing and let the master take care of it
            return

        if self.ccConnected: # already done this
            return
        peer = self.peer
        if not self.peer: # nothing to connect to
            return

        # check that we connect a master to a slave
        if self.role == peer.role:
            raise TypeError, \
                "cannot connect '%s' and '%s' due to identical role '%s'" \
                % (peer, self, self.role)

        try:
            # self is always the master and peer the slave
            connectPorts(self.simobj.getCCObject(), self.name, self.index,
                         peer.simobj.getCCObject(), peer.name, peer.index)
        except:
            print "Error connecting port %s.%s to %s.%s" % \
                  (self.simobj.path(), self.name,
                   peer.simobj.path(), peer.name)
            raise
        self.ccConnected = True
        peer.ccConnected = True

# A reference to an individual element of a VectorPort... much like a
# PortRef, but has an index.
class VectorPortElementRef(PortRef):
    def __init__(self, simobj, name, role, index):
        PortRef.__init__(self, simobj, name, role)
        self.index = index

    def __str__(self):
        return '%s.%s[%d]' % (self.simobj, self.name, self.index)

# A reference to a complete vector-valued port (not just a single element).
# Can be indexed to retrieve individual VectorPortElementRef instances.
class VectorPortRef(object):
    def __init__(self, simobj, name, role):
        assert(isSimObject(simobj) or isSimObjectClass(simobj))
        self.simobj = simobj
        self.name = name
        self.role = role
        self.elements = []

    def __str__(self):
        return '%s.%s[:]' % (self.simobj, self.name)

    def __len__(self):
        # Return the number of connected peers, corresponding the the
        # length of the elements.
        return len(self.elements)

    # for config.ini, print peer's name (not ours)
    def ini_str(self):
        return ' '.join([el.ini_str() for el in self.elements])

    # for config.json
    def get_config_as_dict(self):
        return {'role' : self.role,
                'peer' : [el.ini_str() for el in self.elements]}

    def __getitem__(self, key):
        if not isinstance(key, int):
            raise TypeError, "VectorPort index must be integer"
        if key >= len(self.elements):
            # need to extend list
            ext = [VectorPortElementRef(self.simobj, self.name, self.role, i)
                   for i in range(len(self.elements), key+1)]
            self.elements.extend(ext)
        return self.elements[key]

    def _get_next(self):
        return self[len(self.elements)]

    def __setitem__(self, key, value):
        if not isinstance(key, int):
            raise TypeError, "VectorPort index must be integer"
        self[key].connect(value)

    def connect(self, other):
        if isinstance(other, (list, tuple)):
            # Assign list of port refs to vector port.
            # For now, append them... not sure if that's the right semantics
            # or if it should replace the current vector.
            for ref in other:
                self._get_next().connect(ref)
        else:
            # scalar assignment to plain VectorPort is implicit append
            self._get_next().connect(other)

    def clone(self, simobj, memo):
        if memo.has_key(self):
            return memo[self]
        newRef = copy.copy(self)
        memo[self] = newRef
        newRef.simobj = simobj
        assert(isSimObject(newRef.simobj))
        newRef.elements = [el.clone(simobj, memo) for el in self.elements]
        return newRef

    def unproxy(self, simobj):
        [el.unproxy(simobj) for el in self.elements]

    def ccConnect(self):
        [el.ccConnect() for el in self.elements]

# Port description object.  Like a ParamDesc object, this represents a
# logical port in the SimObject class, not a particular port on a
# SimObject instance.  The latter are represented by PortRef objects.
class Port(object):
    # Generate a PortRef for this port on the given SimObject with the
    # given name
    def makeRef(self, simobj):
        return PortRef(simobj, self.name, self.role)

    # Connect an instance of this port (on the given SimObject with
    # the given name) with the port described by the supplied PortRef
    def connect(self, simobj, ref):
        self.makeRef(simobj).connect(ref)

    # No need for any pre-declarations at the moment as we merely rely
    # on an unsigned int.
    def cxx_predecls(self, code):
        pass

    # Declare an unsigned int with the same name as the port, that
    # will eventually hold the number of connected ports (and thus the
    # number of elements for a VectorPort).
    def cxx_decl(self, code):
        code('unsigned int port_${{self.name}}_connection_count;')

class MasterPort(Port):
    # MasterPort("description")
    def __init__(self, *args):
        if len(args) == 1:
            self.desc = args[0]
            self.role = 'MASTER'
        else:
            raise TypeError, 'wrong number of arguments'

class SlavePort(Port):
    # SlavePort("description")
    def __init__(self, *args):
        if len(args) == 1:
            self.desc = args[0]
            self.role = 'SLAVE'
        else:
            raise TypeError, 'wrong number of arguments'

# VectorPort description object.  Like Port, but represents a vector
# of connections (e.g., as on a Bus).
class VectorPort(Port):
    def __init__(self, *args):
        self.isVec = True

    def makeRef(self, simobj):
        return VectorPortRef(simobj, self.name, self.role)

class VectorMasterPort(VectorPort):
    # VectorMasterPort("description")
    def __init__(self, *args):
        if len(args) == 1:
            self.desc = args[0]
            self.role = 'MASTER'
            VectorPort.__init__(self, *args)
        else:
            raise TypeError, 'wrong number of arguments'

class VectorSlavePort(VectorPort):
    # VectorSlavePort("description")
    def __init__(self, *args):
        if len(args) == 1:
            self.desc = args[0]
            self.role = 'SLAVE'
            VectorPort.__init__(self, *args)
        else:
            raise TypeError, 'wrong number of arguments'

# 'Fake' ParamDesc for Port references to assign to the _pdesc slot of
# proxy objects (via set_param_desc()) so that proxy error messages
# make sense.
class PortParamDesc(object):
    __metaclass__ = Singleton

    ptype_str = 'Port'
    ptype = Port

baseEnums = allEnums.copy()
baseParams = allParams.copy()

def clear():
    global allEnums, allParams

    allEnums = baseEnums.copy()
    allParams = baseParams.copy()

__all__ = ['Param', 'VectorParam',
           'Enum', 'Bool', 'String', 'Float',
           'Int', 'Unsigned', 'Int8', 'UInt8', 'Int16', 'UInt16',
           'Int32', 'UInt32', 'Int64', 'UInt64',
           'Counter', 'Addr', 'Tick', 'Percent',
           'TcpPort', 'UdpPort', 'EthernetAddr',
           'IpAddress', 'IpNetmask', 'IpWithPort',
           'MemorySize', 'MemorySize32',
           'Latency', 'Frequency', 'Clock', 'Voltage',
           'NetworkBandwidth', 'MemoryBandwidth',
           'AddrRange',
           'MaxAddr', 'MaxTick', 'AllMemory',
           'Time',
           'NextEthernetAddr', 'NULL',
           'MasterPort', 'SlavePort',
           'VectorMasterPort', 'VectorSlavePort']

import SimObject
