# Copyright (c) 2012-2014, 2017-2019, 2021, 2024-2025 Arm Limited
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

from datetime import (
    date,
    datetime,
)
from time import (
    asctime,
    gmtime,
    strptime,
    struct_time,
    time,
)

from ..ticks import fromSeconds
from ..util import (
    convert,
    panic,
)
from .base_params import (
    MetaParamValue,
    ParamValue,
)

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
        code("#include <string>", add_once=True)

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
        # code('#include "base/str.hh"', add_once=True)
        pass

    # The default for parsing PODs from an .ini entry is to extract from an
    # istringstream and let overloading choose the right type according to
    # the dest type.
    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{ret} to_number({src}, {dest});")


# Metaclass for bounds-checked integer parameters.  See CheckedInt.
class CheckedIntType(MetaParamValue):
    def __init__(cls, name, bases, dict):
        super().__init__(name, bases, dict)

        # CheckedInt is an abstract base class, so we actually don't
        # want to do any processing on it... the rest of this code is
        # just for classes that derive from CheckedInt.
        if name == "CheckedInt":
            return

        if not (hasattr(cls, "min") and hasattr(cls, "max")):
            if not (hasattr(cls, "size") and hasattr(cls, "unsigned")):
                panic(
                    "CheckedInt subclass %s must define either\n"
                    "    'min' and 'max' or 'size' and 'unsigned'\n",
                    name,
                )
            if cls.unsigned:
                cls.min = 0
                cls.max = 2**cls.size - 1
            else:
                cls.min = -(2 ** (cls.size - 1))
                cls.max = (2 ** (cls.size - 1)) - 1


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
        code('#include "base/types.hh"', add_once=True)

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
        # code('#include "base/str.hh"', add_once=True)
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
        code("#include <sstream>", add_once=True)

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
        code('#include "cpu/probes/pc_count_pair.hh"', add_once=True)

    @classmethod
    def pybind_predecls(cls, code):
        code('#include "cpu/probes/pc_count_pair.hh"', add_once=True)


class AddrRange(ParamValue):
    cxx_type = "AddrRange"

    def __init__(self, *args, **kwargs):
        # Disable interleaving and hashing by default
        self.intlvBits = 0
        self.intlvMatch = 0
        self.masks = []

        def handle_kwargs(self, kwargs):
            # An address range needs to have an upper limit, specified
            # either explicitly with an end, or as an offset using the
            # size keyword.
            if "end" in kwargs:
                self.end = Addr(kwargs.pop("end"))
            elif "size" in kwargs:
                self.end = self.start + Addr(kwargs.pop("size"))
            else:
                raise TypeError("Either end or size must be specified")

            # Now on to the optional bit
            if "intlvMatch" in kwargs:
                self.intlvMatch = int(kwargs.pop("intlvMatch"))

            if "masks" in kwargs:
                self.masks = [int(x) for x in list(kwargs.pop("masks"))]
                self.intlvBits = len(self.masks)
            else:
                if "intlvBits" in kwargs:
                    self.intlvBits = int(kwargs.pop("intlvBits"))
                    self.masks = [0] * self.intlvBits
                    if "intlvHighBit" not in kwargs:
                        raise TypeError("No interleave bits specified")
                    intlv_high_bit = int(kwargs.pop("intlvHighBit"))
                    xor_high_bit = 0
                    if "xorHighBit" in kwargs:
                        xor_high_bit = int(kwargs.pop("xorHighBit"))
                    for i in range(0, self.intlvBits):
                        bit1 = intlv_high_bit - i
                        mask = 1 << bit1
                        if xor_high_bit != 0:
                            bit2 = xor_high_bit - i
                            mask |= 1 << bit2
                        self.masks[self.intlvBits - i - 1] = mask

        if len(args) == 0:
            self.start = Addr(kwargs.pop("start"))
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
                self.end = Addr(args[0])

        elif len(args) == 2:
            self.start = Addr(args[0])
            self.end = Addr(args[1])
        else:
            raise TypeError("Too many arguments specified")

        if kwargs:
            raise TypeError(f"Too many keywords: {list(kwargs.keys())}")

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
        code('#include "base/addr_range.hh"', add_once=True)

    @classmethod
    def pybind_predecls(cls, code):
        Addr.pybind_predecls(code)
        code('#include "base/addr_range.hh"', add_once=True)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>", add_once=True)
        code("#include <vector>", add_once=True)
        code('#include "base/types.hh"', add_once=True)

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
        # code('#include "base/str.hh"', add_once=True)
        pass

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"{ret} to_bool({src}, {dest});")


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
    if isinstance(value, struct_time):
        return value

    if isinstance(value, int):
        return gmtime(value)

    if isinstance(value, (datetime, date)):
        return value.timetuple()

    if isinstance(value, str):
        if value in ("Now", "Today"):
            return gmtime(time())

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
        code("#include <time.h>", add_once=True)

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
        return asctime(self.value)

    def ini_str(self):
        return str(self)

    def get_config_as_dict(self):
        assert false
        return str(self)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <time.h>", add_once=True)

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code("char *_parse_ret = strptime((${src}).c_str(),")
        code('    "%a %b %d %H:%M:%S %Y", &(${dest}));')
        code("${ret} _parse_ret && *_parse_ret == '\\0';")


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
        value = fromSeconds(value)
        return float(value)

    def ini_str(self):
        return f"{self.getValue():f}"

    def config_value(self):
        return f"{self.getValue():f}"

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>", add_once=True)

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code(f"{ret} (std::istringstream({src}) >> {dest}).eof();")


MaxAddr = Addr.max
MaxTick = Tick.max
AllMemory = AddrRange(0, MaxAddr)
