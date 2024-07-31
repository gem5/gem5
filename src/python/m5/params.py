# Copyright (c) 2012-2014, 2017-2019, 2021, 2024 Arm Limited
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
import math
import re
import sys
import time
from typing import List

from . import (
    proxy,
    ticks,
)
from .util import *


def isSimObject(*args, **kwargs):
    from . import SimObject

    return SimObject.isSimObject(*args, **kwargs)


def isSimObjectSequence(*args, **kwargs):
    from . import SimObject

    return SimObject.isSimObjectSequence(*args, **kwargs)


def isSimObjectClass(*args, **kwargs):
    from . import SimObject

    return SimObject.isSimObjectClass(*args, **kwargs)


allParams = {}


class MetaParamValue(type):
    def __new__(mcls, name, bases, dct):
        cls = super().__new__(mcls, name, bases, dct)
        if name in allParams:
            warn(
                "%s already exists in allParams. This may be caused by the "
                "Python 2.7 compatibility layer." % (name,)
            )
        allParams[name] = cls
        return cls


# Dummy base class to identify types that are legitimate for SimObject
# parameters.
class ParamValue(metaclass=MetaParamValue):
    cmd_line_settable = False

    # Generate the code needed as a prerequisite for declaring a C++
    # object of this type.  Typically generates one or more #include
    # statements.  Used when declaring parameters of this type.
    @classmethod
    def cxx_predecls(cls, code):
        pass

    @classmethod
    def pybind_predecls(cls, code):
        cls.cxx_predecls(code)

    # default for printing to .ini file is regular string conversion.
    # will be overridden in some cases
    def ini_str(self):
        return str(self)

    # default for printing to .json file is regular string conversion.
    # will be overridden in some cases, mostly to use native Python
    # types where there are similar JSON types
    def config_value(self):
        return str(self)

    # Prerequisites for .ini parsing with cxx_ini_parse
    @classmethod
    def cxx_ini_predecls(cls, code):
        pass

    # parse a .ini file entry for this param from string expression
    # src into lvalue dest (of the param's C++ type)
    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"// Unhandled param type: {cls.__name__}")
        code(f"{ret} false;")

    # allows us to blithely call unproxy() on things without checking
    # if they're really proxies or not
    def unproxy(self, base):
        return self

    # Produce a human readable version of the stored value
    def pretty_print(self, value):
        return str(value)


# Regular parameter description.
class ParamDesc:
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
                raise TypeError("too many arguments")

        if "desc" in kwargs:
            assert not hasattr(self, "desc")
            self.desc = kwargs["desc"]
            del kwargs["desc"]

        if "default" in kwargs:
            assert not hasattr(self, "default")
            self.default = kwargs["default"]
            del kwargs["default"]

        if kwargs:
            raise TypeError(f"extra unknown kwargs {kwargs}")

        if not hasattr(self, "desc"):
            raise TypeError("desc attribute missing")

    def __getattr__(self, attr):
        if attr == "ptype":
            from . import SimObject

            ptype = SimObject.allClasses[self.ptype_str]
            assert isSimObjectClass(ptype)
            self.ptype = ptype
            return ptype

        raise AttributeError(
            f"'{type(self).__name__}' object has no attribute '{attr}'"
        )

    def example_str(self):
        if hasattr(self.ptype, "ex_str"):
            return self.ptype.ex_str
        else:
            return self.ptype_str

    # Is the param available to be exposed on the command line
    def isCmdLineSettable(self):
        if hasattr(self.ptype, "cmd_line_settable"):
            return self.ptype.cmd_line_settable
        else:
            return False

    def convert(self, value):
        if isinstance(value, proxy.BaseProxy):
            value.set_param_desc(self)
            return value
        if "ptype" not in self.__dict__ and isNullPointer(value):
            # deferred evaluation of SimObject; continue to defer if
            # we're just assigning a null pointer
            return value
        if isinstance(value, self.ptype):
            return value
        if isNullPointer(value) and isSimObjectClass(self.ptype):
            return value
        return self.ptype(value)

    def pretty_print(self, value):
        if isinstance(value, proxy.BaseProxy):
            return str(value)
        if isNullPointer(value):
            return NULL
        return self.ptype(value).pretty_print(value)

    def cxx_predecls(self, code):
        code("#include <cstddef>")
        self.ptype.cxx_predecls(code)

    def pybind_predecls(self, code):
        self.ptype.pybind_predecls(code)

    def cxx_decl(self, code):
        code("${{self.ptype.cxx_type}} ${{self.name}};")


# Vector-valued parameter description.  Just like ParamDesc, except
# that the value is a vector (list) of the specified type instead of a
# single value.


class VectorParamValue(list, metaclass=MetaParamValue):
    def __setattr__(self, attr, value):
        raise AttributeError(
            f"Not allowed to set {attr} on '{type(self).__name__}'"
        )

    def config_value(self):
        return [v.config_value() for v in self]

    def ini_str(self):
        return " ".join([v.ini_str() for v in self])

    def getValue(self):
        return [v.getValue() for v in self]

    def unproxy(self, base):
        if len(self) == 1 and isinstance(self[0], proxy.BaseProxy):
            # The value is a proxy (e.g. Parent.any, Parent.all or
            # Parent.x) therefore try resolve it
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
            width = int(math.ceil(math.log(len(self)) / math.log(10)))
            for i, v in enumerate(self):
                v.set_parent(parent, "%s%0*d" % (name, width, i))

    def has_parent(self):
        return any([e.has_parent() for e in self if not isNullPointer(e)])

    # return 'cpu0 cpu1' etc. for print_ini()
    def get_name(self):
        return " ".join([v._name for v in self])

    # By iterating through the constituent members of the vector here
    # we can nicely handle iterating over all a SimObject's children
    # without having to provide lots of special functions on
    # SimObjectVector directly.
    def descendants(self):
        for v in self:
            yield from v.descendants()

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
            warn(
                f"SimObject {value.get_name()} already has a parent"
                + " that is being overwritten by a SimObjectVector"
            )
        value.set_parent(val.get_parent(), val._name)
        super().__setitem__(key, value)

    # Enumerate the params of each member of the SimObject vector. Creates
    # strings that will allow indexing into the vector by the python code and
    # allow it to be specified on the command line.
    def enumerateParams(self, flags_dict={}, cmd_line_str="", access_str=""):
        if hasattr(self, "_paramEnumed"):
            print(f"Cycle detected enumerating params at {cmd_line_str}?!")
        else:
            x = 0
            for vals in self:
                # Each entry in the SimObjectVector should be an
                # instance of a SimObject
                flags_dict = vals.enumerateParams(
                    flags_dict,
                    cmd_line_str + "%d." % x,
                    access_str + "[%d]." % x,
                )
                x = x + 1

        return flags_dict


class VectorParamDesc(ParamDesc):
    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def convert(self, value):
        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            tmp_list = [ParamDesc.convert(self, v) for v in value]
        elif isinstance(value, str):
            # If input is a csv string
            tmp_list = [
                ParamDesc.convert(self, v)
                for v in value.strip("[").strip("]").split(",")
            ]
        else:
            # singleton: coerce to a single-element list
            tmp_list = [ParamDesc.convert(self, value)]

        if isSimObjectSequence(tmp_list):
            return SimObjectVector(tmp_list)
        else:
            return VectorParamValue(tmp_list)

    # Produce a human readable example string that describes
    # how to set this vector parameter in the absence of a default
    # value.
    def example_str(self):
        s = super().example_str()
        help_str = "[" + s + "," + s + ", ...]"
        return help_str

    # Produce a human readable representation of the value of this vector param.
    def pretty_print(self, value):
        if isinstance(value, (list, tuple)):
            tmp_list = [ParamDesc.pretty_print(self, v) for v in value]
        elif isinstance(value, str):
            tmp_list = [
                ParamDesc.pretty_print(self, v) for v in value.split(",")
            ]
        else:
            tmp_list = [ParamDesc.pretty_print(self, value)]

        return tmp_list

    # This is a helper function for the new config system
    def __call__(self, value):
        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            tmp_list = [ParamDesc.convert(self, v) for v in value]
        elif isinstance(value, str):
            # If input is a csv string
            tmp_list = [
                ParamDesc.convert(self, v)
                for v in value.strip("[").strip("]").split(",")
            ]
        else:
            # singleton: coerce to a single-element list
            tmp_list = [ParamDesc.convert(self, value)]

        return VectorParamValue(tmp_list)

    def cxx_predecls(self, code):
        code("#include <vector>")
        self.ptype.cxx_predecls(code)

    def pybind_predecls(self, code):
        code("#include <vector>")
        self.ptype.pybind_predecls(code)

    def cxx_decl(self, code):
        code("std::vector< ${{self.ptype.cxx_type}} > ${{self.name}};")


class ParamFactory:
    def __init__(self, param_desc_class, ptype_str=None):
        self.param_desc_class = param_desc_class
        self.ptype_str = ptype_str

    def __getattr__(self, attr):
        if self.ptype_str:
            attr = self.ptype_str + "." + attr
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
class String(ParamValue, str):
    cxx_type = "std::string"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(self, code):
        code("#include <string>")

    def __call__(self, value):
        self = value
        return value

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{dest} = {src};")
        code(f"{ret} true;")

    def getValue(self):
        return self


# superclass for "numeric" parameter values, to emulate math
# operations in a type-safe way.  e.g., a Latency times an int returns
# a new Latency object.
class NumericParamValue(ParamValue):
    @staticmethod
    def unwrap(v):
        return v.value if isinstance(v, NumericParamValue) else v

    def __str__(self):
        return str(self.value)

    def __float__(self):
        return float(self.value)

    def __int__(self):
        return int(self.value)

    # hook for bounds checking
    def _check(self):
        return

    def __mul__(self, other):
        newobj = self.__class__(self)
        newobj.value *= NumericParamValue.unwrap(other)
        newobj._check()
        return newobj

    __rmul__ = __mul__

    def __truediv__(self, other):
        newobj = self.__class__(self)
        newobj.value /= NumericParamValue.unwrap(other)
        newobj._check()
        return newobj

    def __floordiv__(self, other):
        newobj = self.__class__(self)
        newobj.value //= NumericParamValue.unwrap(other)
        newobj._check()
        return newobj

    def __add__(self, other):
        newobj = self.__class__(self)
        newobj.value += NumericParamValue.unwrap(other)
        newobj._check()
        return newobj

    def __sub__(self, other):
        newobj = self.__class__(self)
        newobj.value -= NumericParamValue.unwrap(other)
        newobj._check()
        return newobj

    def __iadd__(self, other):
        self.value += NumericParamValue.unwrap(other)
        self._check()
        return self

    def __isub__(self, other):
        self.value -= NumericParamValue.unwrap(other)
        self._check()
        return self

    def __imul__(self, other):
        self.value *= NumericParamValue.unwrap(other)
        self._check()
        return self

    def __itruediv__(self, other):
        self.value /= NumericParamValue.unwrap(other)
        self._check()
        return self

    def __ifloordiv__(self, other):
        self.value //= NumericParamValue.unwrap(other)
        self._check()
        return self

    def __lt__(self, other):
        return self.value < NumericParamValue.unwrap(other)

    def config_value(self):
        return self.value

    @classmethod
    def cxx_ini_predecls(cls, code):
        # Assume that base/str.hh will be included anyway
        # code('#include "base/str.hh"')
        pass

    # The default for parsing PODs from an .ini entry is to extract from an
    # istringstream and let overloading choose the right type according to
    # the dest type.
    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{ret} to_number({src}, {dest});")


# Metaclass for bounds-checked integer parameters.  See CheckedInt.
class CheckedIntType(MetaParamValue):
    def __init__(self, name, bases, dict):
        super().__init__(name, bases, dict)

        # CheckedInt is an abstract base class, so we actually don't
        # want to do any processing on it... the rest of this code is
        # just for classes that derive from CheckedInt.
        if name == "CheckedInt":
            return

        if not hasattr(self, "min") or not hasattr(self, "max"):
            if (size := getattr(self, "size", None)) is not None:
                # TODO: Maybe only override min and max if they are not already set
                # (rather than overriding both if either are unset)
                if getattr(self, "unsigned", False):
                    self.min = 0
                    self.max = 2**size - 1
                else:
                    self.min = -(2 ** (size - 1))
                    self.max = (2 ** (size - 1)) - 1
            else:
                panic(
                    "CheckedInt subclass %s must define either\n"
                    "    'min' and 'max' or 'size' and 'unsigned'\n",
                    name,
                )


# Abstract superclass for bounds-checked integer parameters.  This
# class is subclassed to generate parameter classes with specific
# bounds.  Initialization of the min and max bounds is done in the
# metaclass CheckedIntType.__init__.
class CheckedInt(NumericParamValue, metaclass=CheckedIntType):
    cmd_line_settable = True

    def _check(self):
        if not self.min <= self.value <= self.max:
            raise TypeError(
                "Integer param out of bounds %d < %d < %d"
                % (self.min, self.value, self.max)
            )

    def __init__(self, value):
        if isinstance(value, str):
            self.value = convert.toInteger(value)
        elif isinstance(value, (int, float, NumericParamValue)):
            self.value = int(value)
        else:
            raise TypeError(
                f"Can't convert object of type {type(value).__name__} to CheckedInt"
            )
        self._check()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __index__(self):
        return int(self.value)

    @classmethod
    def cxx_predecls(cls, code):
        # most derived types require this, so we just do it here once
        code('#include "base/types.hh"')

    def getValue(self):
        return int(self.value)


class Int(CheckedInt):
    cxx_type = "int"
    size = 32
    unsigned = False


class Unsigned(CheckedInt):
    cxx_type = "unsigned"
    size = 32
    unsigned = True


class Int8(CheckedInt):
    cxx_type = "int8_t"
    size = 8
    unsigned = False


class UInt8(CheckedInt):
    cxx_type = "uint8_t"
    size = 8
    unsigned = True


class Int16(CheckedInt):
    cxx_type = "int16_t"
    size = 16
    unsigned = False


class UInt16(CheckedInt):
    cxx_type = "uint16_t"
    size = 16
    unsigned = True


class Int32(CheckedInt):
    cxx_type = "int32_t"
    size = 32
    unsigned = False


class UInt32(CheckedInt):
    cxx_type = "uint32_t"
    size = 32
    unsigned = True


class Int64(CheckedInt):
    cxx_type = "int64_t"
    size = 64
    unsigned = False


class UInt64(CheckedInt):
    cxx_type = "uint64_t"
    size = 64
    unsigned = True


class Counter(CheckedInt):
    cxx_type = "Counter"
    size = 64
    unsigned = True


class Tick(CheckedInt):
    cxx_type = "Tick"
    size = 64
    unsigned = True


class TcpPort(CheckedInt):
    cxx_type = "uint16_t"
    size = 16
    unsigned = True


class UdpPort(CheckedInt):
    cxx_type = "uint16_t"
    size = 16
    unsigned = True


class Percent(CheckedInt):
    cxx_type = "int"
    min = 0
    max = 100


class Cycles(CheckedInt):
    cxx_type = "Cycles"
    size = 64
    unsigned = True

    def getValue(self):
        from _m5.core import Cycles

        return Cycles(self.value)

    @classmethod
    def cxx_ini_predecls(cls, code):
        # Assume that base/str.hh will be included anyway
        # code('#include "base/str.hh"')
        pass

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code("uint64_t _temp;")
        code(f"bool _ret = to_number({src}, _temp);")
        code("if (_ret)")
        code(f"    {dest} = Cycles(_temp);")
        code(f"{ret} _ret;")


class Float(ParamValue, float):
    cxx_type = "double"
    cmd_line_settable = True

    def __init__(self, value):
        if isinstance(value, (int, float, NumericParamValue, Float, str)):
            self.value = float(value)
        else:
            raise TypeError(
                f"Can't convert object of type {type(value).__name__} to Float"
            )

    def __call__(self, value):
        self.__init__(value)
        return value

    def getValue(self):
        return float(self.value)

    def config_value(self):
        return self

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>")

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{ret} (std::istringstream({src}) >> {dest}).eof();")


class MemorySize(CheckedInt):
    cxx_type = "uint64_t"
    ex_str = "512MiB"
    size = 64
    unsigned = True

    def __init__(self, value):
        if isinstance(value, MemorySize):
            self.value = value.value
        else:
            self.value = convert.toMemorySize(value)
        self._check()


class MemorySize32(CheckedInt):
    cxx_type = "uint32_t"
    ex_str = "512MiB"
    size = 32
    unsigned = True

    def __init__(self, value):
        if isinstance(value, MemorySize):
            self.value = value.value
        else:
            self.value = convert.toMemorySize(value)
        self._check()


class Addr(CheckedInt):
    cxx_type = "Addr"
    size = 64
    unsigned = True

    def __init__(self, value):
        if isinstance(value, Addr):
            self.value = value.value
        else:
            try:
                # Often addresses are referred to with sizes. Ex: A device
                # base address is at "512MiB".  Use toMemorySize() to convert
                # these into addresses. If the address is not specified with a
                # "size", an exception will occur and numeric translation will
                # proceed below.
                self.value = convert.toMemorySize(value)
            except (TypeError, ValueError):
                # Convert number to string and use long() to do automatic
                # base conversion (requires base=0 for auto-conversion)
                self.value = int(str(value), base=0)

        self._check()

    def __add__(self, other):
        if isinstance(other, Addr):
            return self.value + other.value
        else:
            return self.value + other

    def pretty_print(self, value):
        try:
            val = convert.toMemorySize(value)
        except TypeError:
            val = int(value)
        return f"0x{int(val):x}"


class PcCountPair(ParamValue):
    # This parameter stores a Program Counter address and the a count value for
    # the Program Counter address
    cxx_type = "PcCountPair"
    cmd_line_settable = True

    def __init__(self, _pc, _count):
        self.pc = _pc
        self.count = _count

    def get_pc(self):
        return self.pc

    def get_count(self):
        return self.count

    def getValue(self):
        #  convert Python PcCountPair into C++ PcCountPair
        from _m5.pc import PcCountPair

        return PcCountPair(self.pc, self.count)

    def __str__(self):
        return "(%i,%i)" % (self.pc, self.count)

    def __eq__(self, other):
        return self.pc == other.get_pc() and self.count == other.get_count()

    def __hash__(self):
        return hash((int(self.pc), int(self.count)))

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "cpu/probes/pc_count_pair.hh"')

    @classmethod
    def pybind_predecls(cls, code):
        code('#include "cpu/probes/pc_count_pair.hh"')


class AddrRange(ParamValue):
    cxx_type = "AddrRange"

    def __init__(
        self,
        # we annotate start/end/size as int | str | Addr, but as long as it's something
        # that can construct an Addr, we're happy
        start: int | str | Addr | None | list | tuple = None,
        end: int | str | Addr | None = None,
        size: int | str | Addr | None = None,
        intlvMatch: int | None = None,
        intlvBits: int | None = None,
        masks: List[int] | None = None,
        intlvHighBit: int | None = None,
        xorHighBit: int | None = None,
    ):
        # Disable interleaving and hashing by default
        self.intlvBits = 0
        self.intlvMatch = 0
        self.masks = []

        # Handle getting the start and end addresses
        match (start, end, size):
            # case where start is a list/tuple
            case ([start, end] | (start, end), None, None):
                self.start = Addr(start)
                self.end = Addr(end)
            # cases where start is an int/str/None
            case (start, None, size) if size is not None:
                self.start = Addr(start or 0)
                self.end = self.start + Addr(size)
            case (start, end, _) if end is not None:
                self.start = Addr(start or 0)
                self.end = Addr(end)
            # case where only one address is specified (given as start but will be used as end)
            case (start, None, None) if start is not None:
                self.start = Addr(0)
                self.end = Addr(start)
            # failure case
            case params:
                raise TypeError(
                    f"Either end or size must be specified: {params}"
                )

        # now on to the optional bit
        if intlvMatch is not None:
            self.intlvMatch = int(intlvMatch)

        if masks is not None:
            self.masks = [int(x) for x in list(masks)]
            self.intlvBits = len(self.masks)
        elif intlvBits is not None:
            self.intlvBits = int(intlvBits)
            self.masks = [0] * self.intlvBits
            if intlvHighBit is None:
                raise TypeError("No interleave bits specified")
            intlv_high_bit = int(intlvHighBit)
            xor_high_bit = 0
            if xorHighBit is not None:
                xor_high_bit = int(xorHighBit)
            for i in range(0, self.intlvBits):
                bit1 = intlv_high_bit - i
                mask = 1 << bit1
                if xor_high_bit != 0:
                    bit2 = xor_high_bit - i
                    mask |= 1 << bit2
                self.masks[self.intlvBits - i - 1] = mask

    def __str__(self):
        if len(self.masks) == 0:
            return f"{self.start}:{self.end}"
        else:
            return "{}:{}:{}:{}".format(
                self.start,
                self.end,
                self.intlvMatch,
                ":".join(str(m) for m in self.masks),
            )

    def size(self):
        # Divide the size by the size of the interleaving slice
        return (int(self.end) - int(self.start)) >> self.intlvBits

    @classmethod
    def cxx_predecls(cls, code):
        Addr.cxx_predecls(code)
        code('#include "base/addr_range.hh"')

    @classmethod
    def pybind_predecls(cls, code):
        Addr.pybind_predecls(code)
        code('#include "base/addr_range.hh"')

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>")
        code("#include <vector>")
        code('#include "base/types.hh"')

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code("bool _ret = true;")
        code("uint64_t _start, _end, _intlvMatch = 0;")
        code("std::vector<Addr> _masks;")
        code("char _sep;")
        code("std::istringstream _stream(${src});")
        code("_stream >> _start;")
        code("_stream.get(_sep);")
        code("_ret = _sep == ':';")
        code("_stream >> _end;")
        code("if (!_stream.fail() && !_stream.eof()) {")
        code("    _stream.get(_sep);")
        code("    _ret = ret && _sep == ':';")
        code("    _stream >> _intlvMatch;")
        code("    while (!_stream.fail() && !_stream.eof()) {")
        code("        _stream.get(_sep);")
        code("        _ret = ret && _sep == ':';")
        code("        Addr mask;")
        code("        _stream >> mask;")
        code("        _masks.push_back(mask);")
        code("    }")
        code("}")
        code("_ret = _ret && !_stream.fail() && _stream.eof();")
        code("if (_ret)")
        code("   ${dest} = AddrRange(_start, _end, _masks, _intlvMatch);")
        code("${ret} _ret;")

    def getValue(self):
        # Go from the Python class to the wrapped C++ class
        from _m5.range import AddrRange

        return AddrRange(
            int(self.start), int(self.end), self.masks, int(self.intlvMatch)
        )

    def exclude(self, ranges):
        pybind_exclude = list([r.getValue() for r in ranges])
        pybind_include = self.getValue().exclude(pybind_exclude)

        return list([AddrRange(r.start(), r.end()) for r in pybind_include])

    def is_subset(self, addr_range):
        return self.getValue().isSubset(addr_range.getValue())


# Boolean parameter type.  Python doesn't let you subclass bool, since
# it doesn't want to let you create multiple instances of True and
# False.  Thus this is a little more complicated than String.
class Bool(ParamValue):
    cxx_type = "bool"
    cmd_line_settable = True

    def __init__(self, value):
        try:
            self.value = convert.toBool(value)
        except TypeError:
            self.value = bool(value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def getValue(self):
        return bool(self.value)

    def __str__(self):
        return str(self.value)

    # implement truth value testing for Bool parameters so that these params
    # evaluate correctly during the python configuration phase
    def __bool__(self):
        return bool(self.value)

    # Python 2.7 uses __nonzero__ instead of __bool__
    __nonzero__ = __bool__

    def ini_str(self):
        if self.value:
            return "true"
        return "false"

    def config_value(self):
        return self.value

    @classmethod
    def cxx_ini_predecls(cls, code):
        # Assume that base/str.hh will be included anyway
        # code('#include "base/str.hh"')
        pass

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"{ret} to_bool({src}, {dest});")


class HostSocket(ParamValue):
    cxx_type = "ListenSocketConfig"

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/socket.hh"')

    def __init__(self, value):
        if isinstance(value, HostSocket):
            self.value = value.value
        else:
            self.value = value

    def getValue(self):
        from _m5.socket import (
            listenSocketEmptyConfig,
            listenSocketInetConfig,
            listenSocketUnixAbstractConfig,
            listenSocketUnixFileConfig,
        )

        if isinstance(self.value, str):
            if self.value[0] == "@":
                return listenSocketUnixAbstractConfig(self.value[1:])
            else:
                d, f = os.path.split(self.value)
                return listenSocketUnixFileConfig(d, f)
        else:
            if self.value == 0:
                return listenSocketEmptyConfig()
            else:
                return listenSocketInetConfig(self.value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        if isinstance(self.value, str):
            return self.value
        else:
            return "#" + str(self.value)

    def ini_str(self):
        if isinstance(self.value, str):
            if self.value[0] == "@":
                return self.value
            else:
                return "P" + self.value
        else:
            return "#" + str(self.value)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code('#include "base/socket.hh"')

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"{ret} ListenSocketConfig::parseIni({src}, {dest});")


def IncEthernetAddr(addr, val=1):
    bytes = [int(x, 16) for x in addr.split(":")]
    bytes[5] += val
    for i in (5, 4, 3, 2, 1):
        val, rem = divmod(bytes[i], 256)
        bytes[i] = rem
        if val == 0:
            break
        bytes[i - 1] += val
    assert bytes[0] <= 255
    return ":".join(map(lambda x: f"{x:02x}", bytes))


_NextEthernetAddr = "00:90:00:00:00:01"


def NextEthernetAddr():
    global _NextEthernetAddr

    value = _NextEthernetAddr
    _NextEthernetAddr = IncEthernetAddr(_NextEthernetAddr, 1)
    return value


class EthernetAddr(ParamValue):
    cxx_type = "networking::EthAddr"
    ex_str = "00:90:00:00:00:01"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    def __init__(self, value):
        if value == NextEthernetAddr:
            self.value = value
            return

        if not isinstance(value, str):
            raise TypeError("expected an ethernet address and didn't get one")

        bytes = value.split(":")
        if len(bytes) != 6:
            raise TypeError(f"invalid ethernet address {value}")

        for byte in bytes:
            if not 0 <= int(byte, base=16) <= 0xFF:
                raise TypeError(f"invalid ethernet address {value}")

        self.value = value

    def __call__(self, value):
        self.__init__(value)
        return value

    def unproxy(self, base):
        if self.value == NextEthernetAddr:
            return EthernetAddr(self.value())
        return self

    def getValue(self):
        from _m5.net import EthAddr

        return EthAddr(self.value)

    def __str__(self):
        return self.value

    def ini_str(self):
        return self.value

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{dest} = networking::EthAddr({src});")
        code(f"{ret} true;")


# When initializing an IpAddress, pass in an existing IpAddress, a string of
# the form "a.b.c.d", or an integer representing an IP.
class IpAddress(ParamValue):
    cxx_type = "networking::IpAddress"
    ex_str = "127.0.0.1"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    def __init__(self, value):
        if isinstance(value, IpAddress):
            self.ip = value.ip
        else:
            try:
                self.ip = convert.toIpAddress(value)
            except TypeError:
                self.ip = int(value)
        self.verifyIp()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        tup = [(self.ip >> i) & 0xFF for i in (24, 16, 8, 0)]
        return "%d.%d.%d.%d" % tuple(tup)

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
            raise TypeError("invalid ip address %#08x" % self.ip)

    def getValue(self):
        from _m5.net import IpAddress

        return IpAddress(self.ip)


# When initializing an IpNetmask, pass in an existing IpNetmask, a string of
# the form "a.b.c.d/n" or "a.b.c.d/e.f.g.h", or an ip and netmask as
# positional or keyword arguments.
class IpNetmask(IpAddress):
    cxx_type = "networking::IpNetmask"
    ex_str = "127.0.0.0/24"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    def __init__(self, *args, **kwargs):
        def handle_kwarg(self, kwargs, key, elseVal=None):
            if key in kwargs:
                setattr(self, key, kwargs.pop(key))
            elif elseVal:
                setattr(self, key, elseVal)
            else:
                raise TypeError(f"No value set for {key}")

        if len(args) == 0:
            handle_kwarg(self, kwargs, "ip")
            handle_kwarg(self, kwargs, "netmask")

        elif len(args) == 1:
            if kwargs:
                if not "ip" in kwargs and not "netmask" in kwargs:
                    raise TypeError("Invalid arguments")
                handle_kwarg(self, kwargs, "ip", args[0])
                handle_kwarg(self, kwargs, "netmask", args[0])
            elif isinstance(args[0], IpNetmask):
                self.ip = args[0].ip
                self.netmask = args[0].netmask
            else:
                (self.ip, self.netmask) = convert.toIpNetmask(args[0])

        elif len(args) == 2:
            self.ip = args[0]
            self.netmask = args[1]
        else:
            raise TypeError("Too many arguments specified")

        if kwargs:
            raise TypeError(f"Too many keywords: {list(kwargs.keys())}")

        self.verify()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        return "%s/%d" % (super().__str__(), self.netmask)

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
            raise TypeError("invalid netmask %d" % netmask)

    def getValue(self):
        from _m5.net import IpNetmask

        return IpNetmask(self.ip, self.netmask)


# When initializing an IpWithPort, pass in an existing IpWithPort, a string of
# the form "a.b.c.d:p", or an ip and port as positional or keyword arguments.
class IpWithPort(IpAddress):
    cxx_type = "networking::IpWithPort"
    ex_str = "127.0.0.1:80"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"')

    def __init__(self, *args, **kwargs):
        def handle_kwarg(self, kwargs, key, elseVal=None):
            if key in kwargs:
                setattr(self, key, kwargs.pop(key))
            elif elseVal:
                setattr(self, key, elseVal)
            else:
                raise TypeError(f"No value set for {key}")

        if len(args) == 0:
            handle_kwarg(self, kwargs, "ip")
            handle_kwarg(self, kwargs, "port")

        elif len(args) == 1:
            if kwargs:
                if not "ip" in kwargs and not "port" in kwargs:
                    raise TypeError("Invalid arguments")
                handle_kwarg(self, kwargs, "ip", args[0])
                handle_kwarg(self, kwargs, "port", args[0])
            elif isinstance(args[0], IpWithPort):
                self.ip = args[0].ip
                self.port = args[0].port
            else:
                (self.ip, self.port) = convert.toIpWithPort(args[0])

        elif len(args) == 2:
            self.ip = args[0]
            self.port = args[1]
        else:
            raise TypeError("Too many arguments specified")

        if kwargs:
            raise TypeError(f"Too many keywords: {list(kwargs.keys())}")

        self.verify()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        return "%s:%d" % (super().__str__(), self.port)

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
        if self.port < 0 or self.port > 0xFFFF:
            raise TypeError("invalid port %d" % self.port)

    def getValue(self):
        from _m5.net import IpWithPort

        return IpWithPort(self.ip, self.port)


time_formats = [
    "%a %b %d %H:%M:%S %Z %Y",
    "%a %b %d %H:%M:%S %Y",
    "%Y/%m/%d %H:%M:%S",
    "%Y/%m/%d %H:%M",
    "%Y/%m/%d",
    "%m/%d/%Y %H:%M:%S",
    "%m/%d/%Y %H:%M",
    "%m/%d/%Y",
    "%m/%d/%y %H:%M:%S",
    "%m/%d/%y %H:%M",
    "%m/%d/%y",
]


def parse_time(value):
    from datetime import (
        date,
        datetime,
    )
    from time import (
        gmtime,
        strptime,
        struct_time,
        time,
    )

    if isinstance(value, struct_time):
        return value

    if isinstance(value, int):
        return gmtime(value)

    if isinstance(value, (datetime, date)):
        return value.timetuple()

    if isinstance(value, str):
        if value in ("Now", "Today"):
            return time.gmtime(time.time())

        for format in time_formats:
            try:
                return strptime(value, format)
            except ValueError:
                pass

    raise ValueError(f"Could not parse '{value}' as a time")


class Time(ParamValue):
    cxx_type = "tm"

    @classmethod
    def cxx_predecls(cls, code):
        code("#include <time.h>")

    def __init__(self, value):
        self.value = parse_time(value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def getValue(self):
        import calendar

        from _m5.core import tm

        return tm.gmtime(calendar.timegm(self.value))

    def __str__(self):
        return time.asctime(self.value)

    def ini_str(self):
        return str(self)

    def get_config_as_dict(self):
        assert false
        return str(self)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <time.h>")

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code("char *_parse_ret = strptime((${src}).c_str(),")
        code('    "%a %b %d %H:%M:%S %Y", &(${dest}));')
        code("${ret} _parse_ret && *_parse_ret == '\\0';")


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
        cls = super().__new__(mcls, name, bases, dict)
        allEnums[name] = cls
        return cls

    def __init__(cls, name, bases, init_dict):
        if "map" in init_dict:
            if not isinstance(cls.map, dict):
                raise TypeError(
                    "Enum-derived class attribute 'map' "
                    "must be of type dict"
                )
            # build list of value strings from map
            cls.vals = list(cls.map.keys())
            cls.vals.sort()
        elif "vals" in init_dict:
            if not isinstance(cls.vals, list):
                raise TypeError(
                    "Enum-derived class attribute 'vals' "
                    "must be of type list"
                )
            # build string->value map from vals sequence
            cls.map = {}
            for idx, val in enumerate(cls.vals):
                cls.map[val] = idx
        else:
            raise TypeError(
                "Enum-derived class must define attribute 'map' or 'vals'"
            )

        if getattr(cls, "is_class", False):
            cls.cxx_type = f"{name}"
        else:
            scope = init_dict.get("wrapper_name", "enums")
            cls.cxx_type = f"{scope}::{name}"
        super().__init__(name, bases, init_dict)


# Base class for enum types.
class Enum(ParamValue, metaclass=MetaEnum):
    vals = []
    cmd_line_settable = True

    # The name of the wrapping namespace or struct
    wrapper_name = "enums"

    # If true, the enum is wrapped in a struct rather than a namespace
    wrapper_is_struct = False

    is_class = False

    # If not None, use this as the enum name rather than this class name
    enum_name = None

    def __init__(self, value):
        if value not in self.map:
            raise TypeError(
                f"Enum param got bad value '{value}' (not in {self.vals})"
            )
        self.value = value

    def __call__(self, value):
        self.__init__(value)
        return value

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "enums/$0.hh"', cls.__name__)

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code("if (false) {")
        for elem_name in cls.map.keys():
            code(f'}} else if ({src} == "{elem_name}") {{')
            code.indent()
            name = cls.__name__ if cls.enum_name is None else cls.enum_name
            code(f"{dest} = {name if cls.is_class else 'enums'}::{elem_name};")
            code(f"{ret} true;")
            code.dedent()
        code("} else {")
        code(f"    {ret} false;")
        code("}")

    def getValue(self):
        import m5.internal.params

        e = getattr(m5.internal.params, f"enum_{self.__class__.__name__}")
        return e(self.map[self.value])

    def __str__(self):
        return self.value


# This param will generate a scoped c++ enum and its python bindings.
class ScopedEnum(Enum):
    vals = []
    cmd_line_settable = True

    # The name of the wrapping namespace or struct
    wrapper_name = None

    # If true, the enum is wrapped in a struct rather than a namespace
    wrapper_is_struct = False

    # If true, the generated enum is a scoped enum
    is_class = True

    # If not None, use this as the enum name rather than this class name
    enum_name = None


class ByteOrder(ScopedEnum):
    """Enum representing component's byte order (endianness)"""

    vals = ["big", "little"]


# how big does a rounding error need to be before we warn about it?
frequency_tolerance = 0.001  # 0.1%


class TickParamValue(NumericParamValue):
    cxx_type = "Tick"
    ex_str = "1MHz"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/types.hh"')

    def __call__(self, value):
        self.__init__(value)
        return value

    def getValue(self):
        return int(self.value)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>")

    # Ticks are expressed in seconds in JSON files and in plain
    # Ticks in .ini files.  Switch based on a config flag
    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code("${ret} to_number(${src}, ${dest});")


class Latency(TickParamValue):
    ex_str = "100ns"

    def __init__(self, value):
        if isinstance(value, (Latency, Clock)):
            self.ticks = value.ticks
            self.value = value.value
        elif isinstance(value, Frequency):
            self.ticks = value.ticks
            self.value = 1.0 / value.value
        elif value.endswith("t"):
            self.ticks = True
            self.value = int(value[:-1])
        else:
            self.ticks = False
            self.value = convert.toLatency(value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def __getattr__(self, attr):
        if attr in ("latency", "period"):
            return self
        if attr == "frequency":
            return Frequency(self)
        raise AttributeError(f"Latency object has no attribute '{attr}'")

    def getValue(self):
        if self.ticks or self.value == 0:
            value = self.value
        else:
            value = ticks.fromSeconds(self.value)
        return int(value)

    def config_value(self):
        return self.getValue()

    # convert latency to ticks
    def ini_str(self):
        return "%d" % self.getValue()


class Frequency(TickParamValue):
    ex_str = "1GHz"

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

    def __call__(self, value):
        self.__init__(value)
        return value

    def __getattr__(self, attr):
        if attr == "frequency":
            return self
        if attr in ("latency", "period"):
            return Latency(self)
        raise AttributeError(f"Frequency object has no attribute '{attr}'")

    # convert latency to ticks
    def getValue(self):
        if self.ticks or self.value == 0:
            value = self.value
        else:
            value = ticks.fromSeconds(1.0 / self.value)
        return int(value)

    def config_value(self):
        return self.getValue()

    def ini_str(self):
        return "%d" % self.getValue()


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
        elif value.endswith("t"):
            self.ticks = True
            self.value = int(value[:-1])
        else:
            self.ticks = False
            self.value = convert.anyToLatency(value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        return f"{Latency(self)}"

    def __getattr__(self, attr):
        if attr == "frequency":
            return Frequency(self)
        if attr in ("latency", "period"):
            return Latency(self)
        raise AttributeError(f"Frequency object has no attribute '{attr}'")

    def getValue(self):
        return self.period.getValue()

    def config_value(self):
        return self.period.config_value()

    def ini_str(self):
        return self.period.ini_str()


class Voltage(Float):
    ex_str = "1V"

    def __new__(cls, value):
        value = convert.toVoltage(value)
        return super().__new__(cls, value)

    def __init__(self, value):
        value = convert.toVoltage(value)
        super().__init__(value)


class Current(Float):
    ex_str = "1mA"

    def __new__(cls, value):
        value = convert.toCurrent(value)
        return super().__new__(cls, value)

    def __init__(self, value):
        value = convert.toCurrent(value)
        super().__init__(value)


class Energy(Float):
    ex_str = "1pJ"

    def __new__(cls, value):
        value = convert.toEnergy(value)
        return super().__new__(cls, value)

    def __init__(self, value):
        value = convert.toEnergy(value)
        super().__init__(value)


class Temperature(ParamValue):
    cxx_type = "Temperature"
    cmd_line_settable = True
    ex_str = "1C"

    def __init__(self, value):
        self.value = convert.toTemperature(value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        return str(self.value)

    def getValue(self):
        from _m5.core import Temperature

        return Temperature.from_kelvin(self.value)

    def config_value(self):
        return self.value

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/temperature.hh"')

    @classmethod
    def cxx_ini_predecls(cls, code):
        # Assume that base/str.hh will be included anyway
        # code('#include "base/str.hh"')
        pass

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code("double _temp;")
        code(f"bool _ret = to_number({src}, _temp);")
        code("if (_ret)")
        code(f"    {dest} = Temperature(_temp);")
        code(f"{ret} _ret;")


class NetworkBandwidth(float, ParamValue):
    cxx_type = "float"
    ex_str = "1Gbps"
    cmd_line_settable = True

    def __new__(cls, value):
        # convert to bits per second
        val = convert.toNetworkBandwidth(value)
        return super().__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def __call__(self, value):
        val = convert.toNetworkBandwidth(value)
        self.__init__(val)
        return value

    def getValue(self):
        # convert to seconds per byte
        value = 8.0 / float(self)
        # convert to ticks per byte
        value = ticks.fromSeconds(value)
        return float(value)

    def ini_str(self):
        return f"{self.getValue():f}"

    def config_value(self):
        return f"{self.getValue():f}"

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>")

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{ret} (std::istringstream({src}) >> {dest}).eof();")


class MemoryBandwidth(float, ParamValue):
    cxx_type = "float"
    ex_str = "1GiB/s"
    cmd_line_settable = True

    def __new__(cls, value):
        # convert to bytes per second
        val = convert.toMemoryBandwidth(value)
        return super().__new__(cls, val)

    def __call__(self, value):
        val = convert.toMemoryBandwidth(value)
        self.__init__(val)
        return value

    def getValue(self):
        # convert to seconds per byte
        value = float(self)
        if value:
            value = 1.0 / float(self)
        # convert to ticks per byte
        value = ticks.fromSeconds(value)
        return float(value)

    def ini_str(self):
        return f"{self.getValue():f}"

    def config_value(self):
        return f"{self.getValue():f}"

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>")

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{ret} (std::istringstream({src}) >> {dest}).eof();")


#
# "Constants"... handy aliases for various values.
#


# Special class for NULL pointers.  Note the special check in
# make_param_value() above that lets these be assigned where a
# SimObject is required.
# only one copy of a particular node
class NullSimObject(metaclass=Singleton):
    _name = "Null"

    def __call__(cls):
        return cls

    def _instantiate(self, parent=None, path=""):
        pass

    def ini_str(self):
        return "Null"

    def unproxy(self, base):
        return self

    def set_path(self, parent, name):
        pass

    def set_parent(self, parent, name):
        pass

    def clear_parent(self, old_parent):
        pass

    def descendants(self):
        return
        yield None

    def get_config_as_dict(self):
        return {}

    def __str__(self):
        return self._name

    def config_value(self):
        return None

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
class PortRef:
    def __init__(self, simobj, name, role, is_source):
        assert isSimObject(simobj) or isSimObjectClass(simobj)
        self.simobj = simobj
        self.name = name
        self.role = role
        self.is_source = is_source
        self.peer = None  # not associated with another port yet
        self.ccConnected = False  # C++ port connection done?
        self.index = -1  # always -1 for non-vector ports

    def __str__(self):
        return f"{self.simobj}.{self.name}"

    def __len__(self):
        # Return the number of connected ports, i.e. 0 is we have no
        # peer and 1 if we do.
        return int(self.peer != None)

    # for config.ini, print peer's name (not ours)
    def ini_str(self):
        return str(self.peer)

    # for config.json
    def get_config_as_dict(self):
        return {
            "role": self.role,
            "peer": str(self.peer),
            "is_source": str(self.is_source),
        }

    def __getattr__(self, attr):
        if attr == "peerObj" and self.peer is not None:
            # shorthand for proxies
            return self.peer.simobj
        raise AttributeError(
            f"'{self.__class__.__name__}' object has no attribute '{attr}'"
        )

    # Full connection is symmetric (both ways).  Called via
    # SimObject.__setattr__ as a result of a port assignment, e.g.,
    # "obj1.portA = obj2.portB", or via VectorPortElementRef.__setitem__,
    # e.g., "obj1.portA[3] = obj2.portB".
    def connect(self, other):
        if isinstance(other, VectorPortRef):
            # reference to plain VectorPort is implicit append
            other = other._get_next()
        if self.peer and not proxy.isproxy(self.peer):
            fatal(
                "Port %s is already connected to %s, cannot connect %s\n",
                self,
                self.peer,
                other,
            )
        self.peer = other

        if proxy.isproxy(other):
            other.set_param_desc(PortParamDesc())
            return
        elif not isinstance(other, PortRef):
            raise TypeError(
                f"assigning non-port reference '{other}' to port '{self}'"
            )

        if not Port.is_compat(self, other):
            fatal(
                "Ports %s and %s with roles '%s' and '%s' "
                "are not compatible",
                self,
                other,
                self.role,
                other.role,
            )

        if other.peer is not self:
            other.connect(self)

    # Allow a compatible port pair to be spliced between a port and its
    # connected peer. Useful operation for connecting instrumentation
    # structures into a system when it is necessary to connect the
    # instrumentation after the full system has been constructed.
    def splice(self, new_1, new_2):
        if not self.peer or proxy.isproxy(self.peer):
            fatal("Port %s not connected, cannot splice in new peers\n", self)

        if not isinstance(new_1, PortRef) or not isinstance(new_2, PortRef):
            raise TypeError(
                f"Splicing non-port references '{new_1}','{new_2}' to port '{self}'"
            )

        old_peer = self.peer

        if (
            old_peer is not None
            and Port.is_compat(old_peer, new_1)
            and Port.is_compat(self, new_2)
        ):
            old_peer.peer = new_1
            new_1.peer = old_peer
            self.peer = new_2
            new_2.peer = self
        elif (
            old_peer is not None
            and Port.is_compat(old_peer, new_2)
            and Port.is_compat(self, new_1)
        ):
            old_peer.peer = new_2
            new_2.peer = old_peer
            self.peer = new_1
            new_1.peer = self
        else:
            fatal(
                "Ports %s(%s) and %s(%s) can't be compatibly spliced with "
                "%s(%s) and %s(%s)",
                self,
                self.role,
                old_peer,
                old_peer.role if old_peer is not None else None,
                new_1,
                new_1.role,
                new_2,
                new_2.role,
            )

    def clone(self, simobj, memo):
        if self in memo:
            return memo[self]
        newRef = copy.copy(self)
        memo[self] = newRef
        newRef.simobj = simobj
        assert isSimObject(newRef.simobj)
        if self.peer and not proxy.isproxy(self.peer):
            peerObj = self.peer.simobj(_memo=memo)
            newRef.peer = self.peer.clone(peerObj, memo)
            assert not isinstance(newRef.peer, VectorPortRef)
        return newRef

    def unproxy(self, simobj):
        assert simobj is self.simobj
        if self.peer is not None and proxy.isproxy(self.peer):
            try:
                realPeer = self.peer.unproxy(self.simobj)
            except:
                print(
                    f"Error in unproxying port '{self.name}' of {self.simobj.path()}"
                )
                raise
            self.connect(realPeer)

    # Call C++ to create corresponding port connection between C++ objects
    def ccConnect(self):
        if self.ccConnected:  # already done this
            return

        peer = self.peer
        if not self.peer:  # nothing to connect to
            return

        port = self.simobj.getPort(self.name, self.index)
        peer_port = peer.simobj.getPort(peer.name, peer.index)
        port.bind(peer_port)

        self.ccConnected = True


# A reference to an individual element of a VectorPort... much like a
# PortRef, but has an index.
class VectorPortElementRef(PortRef):
    def __init__(self, simobj, name, role, is_source, index):
        PortRef.__init__(self, simobj, name, role, is_source)
        self.index = index

    def __str__(self):
        return "%s.%s[%d]" % (self.simobj, self.name, self.index)


# A reference to a complete vector-valued port (not just a single element).
# Can be indexed to retrieve individual VectorPortElementRef instances.
class VectorPortRef:
    def __init__(self, simobj, name, role, is_source):
        assert isSimObject(simobj) or isSimObjectClass(simobj)
        self.simobj = simobj
        self.name = name
        self.role = role
        self.is_source = is_source
        self.elements = []

    def __str__(self):
        return f"{self.simobj}.{self.name}[:]"

    def __len__(self):
        # Return the number of connected peers, corresponding the the
        # length of the elements.
        return len(self.elements)

    # for config.ini, print peer's name (not ours)
    def ini_str(self):
        return " ".join([el.ini_str() for el in self.elements])

    # for config.json
    def get_config_as_dict(self):
        return {
            "role": self.role,
            "peer": [el.ini_str() for el in self.elements],
            "is_source": str(self.is_source),
        }

    def __getitem__(self, key):
        if not isinstance(key, int):
            raise TypeError("VectorPort index must be integer")
        if key >= len(self.elements):
            # need to extend list
            ext = [
                VectorPortElementRef(
                    self.simobj, self.name, self.role, self.is_source, i
                )
                for i in range(len(self.elements), key + 1)
            ]
            self.elements.extend(ext)
        return self.elements[key]

    def _get_next(self):
        return self[len(self.elements)]

    def __setitem__(self, key, value):
        if not isinstance(key, int):
            raise TypeError("VectorPort index must be integer")
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
        if self in memo:
            return memo[self]
        newRef = copy.copy(self)
        memo[self] = newRef
        newRef.simobj = simobj
        assert isSimObject(newRef.simobj)
        newRef.elements = [el.clone(simobj, memo) for el in self.elements]
        return newRef

    def unproxy(self, simobj):
        [el.unproxy(simobj) for el in self.elements]

    def ccConnect(self):
        [el.ccConnect() for el in self.elements]


# Port description object.  Like a ParamDesc object, this represents a
# logical port in the SimObject class, not a particular port on a
# SimObject instance.  The latter are represented by PortRef objects.
class Port:
    # Port("role", "description")

    _compat_dict = {}

    @classmethod
    def compat(cls, role, peer):
        cls._compat_dict.setdefault(role, set()).add(peer)
        cls._compat_dict.setdefault(peer, set()).add(role)

    @classmethod
    def is_compat(cls, one, two):
        for port in one, two:
            if not port.role in Port._compat_dict:
                fatal("Unrecognized role '%s' for port %s\n", port.role, port)
        return one.role in Port._compat_dict[two.role]

    def __init__(self, role, desc, is_source=False):
        self.desc = desc
        self.role = role
        self.is_source = is_source

    # Generate a PortRef for this port on the given SimObject with the
    # given name
    def makeRef(self, simobj):
        return PortRef(simobj, self.name, self.role, self.is_source)

    # Connect an instance of this port (on the given SimObject with
    # the given name) with the port described by the supplied PortRef
    def connect(self, simobj, ref):
        self.makeRef(simobj).connect(ref)

    # No need for any pre-declarations at the moment as we merely rely
    # on an unsigned int.
    def cxx_predecls(self, code):
        pass

    def pybind_predecls(self, code):
        cls.cxx_predecls(self, code)

    # Declare an unsigned int with the same name as the port, that
    # will eventually hold the number of connected ports (and thus the
    # number of elements for a VectorPort).
    def cxx_decl(self, code):
        code("unsigned int port_${{self.name}}_connection_count;")


Port.compat("GEM5 REQUESTOR", "GEM5 RESPONDER")


class RequestPort(Port):
    # RequestPort("description")
    def __init__(self, desc):
        super().__init__("GEM5 REQUESTOR", desc, is_source=True)


class ResponsePort(Port):
    # ResponsePort("description")
    def __init__(self, desc):
        super().__init__("GEM5 RESPONDER", desc)


# VectorPort description object.  Like Port, but represents a vector
# of connections (e.g., as on a XBar).
class VectorPort(Port):
    def makeRef(self, simobj):
        return VectorPortRef(simobj, self.name, self.role, self.is_source)


class VectorRequestPort(VectorPort):
    # VectorRequestPort("description")
    def __init__(self, desc):
        super().__init__("GEM5 REQUESTOR", desc, is_source=True)


class VectorResponsePort(VectorPort):
    # VectorResponsePort("description")
    def __init__(self, desc):
        super().__init__("GEM5 RESPONDER", desc)


# Old names, maintained for compatibility.
MasterPort = RequestPort
SlavePort = ResponsePort
VectorMasterPort = VectorRequestPort
VectorSlavePort = VectorResponsePort


# 'Fake' ParamDesc for Port references to assign to the _pdesc slot of
# proxy objects (via set_param_desc()) so that proxy error messages
# make sense.
class PortParamDesc(metaclass=Singleton):
    ptype_str = "Port"
    ptype = Port


class DeprecatedParam:
    """A special type for deprecated parameter variable names.

    There are times when we need to change the name of parameter, but this
    breaks the external-facing python API used in configuration files. Using
    this "type" for a parameter will warn users that they are using the old
    name, but allow for backwards compatibility.

    Usage example:
    In the following example, the `time` parameter is changed to `delay`.

    ```
    class SomeDevice(SimObject):
        delay = Param.Latency('1ns', 'The time to wait before something')
        time = DeprecatedParam(delay, '`time` is now called `delay`')
    ```
    """

    def __init__(self, new_param, message=""):
        """new_param: the new parameter variable that users should be using
        instead of this parameter variable.
        message: an optional message to print when warning the user
        """
        self.message = message
        self.newParam = new_param
        # Note: We won't know the string variable names until later in the
        # SimObject initialization process. Note: we expect that the setters
        # will be called when the SimObject type (class) is initialized so
        # these variables should be filled in before the instance of the
        # SimObject with this parameter is constructed
        self._oldName = ""
        self._newName = ""

    @property
    def oldName(self):
        assert self._oldName != ""  # should already be set
        return self._oldName

    @oldName.setter
    def oldName(self, name):
        assert self._oldName == ""  # Cannot "re-set" this value
        self._oldName = name

    @property
    def newName(self):
        assert self._newName != ""  # should already be set
        return self._newName

    @newName.setter
    def newName(self, name):
        assert self._newName == ""  # Cannot "re-set" this value
        self._newName = name

    def printWarning(self, instance_name, simobj_name):
        """Issue a warning that this variable name should not be used.

        instance_name: str, the name of the instance used in python
        simobj_name: str, the name of the SimObject type
        """
        if not self.message:
            self.message = f"See {simobj_name} for more information"
        warn(f"{instance_name}.{self._oldName} is deprecated. {self.message}")


baseEnums = allEnums.copy()
baseParams = allParams.copy()


def clear():
    global allEnums, allParams

    allEnums = baseEnums.copy()
    allParams = baseParams.copy()


__all__ = [
    "Param",
    "VectorParam",
    "Enum",
    "ScopedEnum",
    "Bool",
    "String",
    "Float",
    "Int",
    "Unsigned",
    "Int8",
    "UInt8",
    "Int16",
    "UInt16",
    "Int32",
    "UInt32",
    "Int64",
    "UInt64",
    "Counter",
    "Addr",
    "Tick",
    "Percent",
    "TcpPort",
    "UdpPort",
    "EthernetAddr",
    "IpAddress",
    "IpNetmask",
    "IpWithPort",
    "MemorySize",
    "MemorySize32",
    "Latency",
    "Frequency",
    "Clock",
    "Voltage",
    "Current",
    "Energy",
    "Temperature",
    "NetworkBandwidth",
    "MemoryBandwidth",
    "AddrRange",
    "MaxAddr",
    "MaxTick",
    "AllMemory",
    "Time",
    "NextEthernetAddr",
    "NULL",
    "Port",
    "RequestPort",
    "ResponsePort",
    "MasterPort",
    "SlavePort",
    "VectorPort",
    "VectorRequestPort",
    "VectorResponsePort",
    "VectorMasterPort",
    "VectorSlavePort",
    "DeprecatedParam",
    "PcCountPair",
]
