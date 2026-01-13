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

from ..util import convert
from .base_params import ParamValue
from .param_types import Float


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
        code('#include "base/temperature.hh"', add_once=True)

    @classmethod
    def cxx_ini_predecls(cls, code):
        # Assume that base/str.hh will be included anyway
        # code('#include "base/str.hh"', add_once=True)
        pass

    @classmethod
    def cxx_ini_parse(self, code, src, dest, ret):
        code("double _temp;")
        code(f"bool _ret = to_number({src}, _temp);")
        code("if (_ret)")
        code(f"    {dest} = Temperature(_temp);")
        code(f"{ret} _ret;")
