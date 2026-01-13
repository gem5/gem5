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

from .. import ticks
from ..util import (
    convert,
)
from .param_types import NumericParamValue

# how big does a rounding error need to be before we warn about it?
frequency_tolerance = 0.001  # 0.1%


class TickParamValue(NumericParamValue):
    cxx_type = "Tick"
    ex_str = "1MHz"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/types.hh"', add_once=True)

    def __call__(self, value):
        self.__init__(value)
        return value

    def getValue(self):
        return int(self.value)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>", add_once=True)

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
