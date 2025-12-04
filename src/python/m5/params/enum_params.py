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

import importlib

from .base_params import (
    MetaParamValue,
    ParamValue,
)

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

        if cls.is_class:
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
        code('#include "enums/$0.hh"', cls.__name__, add_once=True)

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
        try:
            mod = importlib.import_module(
                f"_m5.enum_{self.__class__.__name__}"
            )
        except ImportError:
            raise AttributeError("Cannot get enum value, not linked to gem5")

        e = getattr(mod, f"enum_{self.__class__.__name__}")
        return e(self.map[self.value])

    def __str__(self):
        return self.value

    def __eq__(self, __o: object) -> bool:
        """Checks if two enum values are the same."""
        if not isinstance(__o, Enum):
            return False
        return type(self) == type(__o) and self.value == __o.value

    def __hash__(self):
        return hash(self.value)


# This param will generate a c++ enum and its python bindings.
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
