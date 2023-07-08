# Copyright (c) 2007 The Regents of The University of Michigan
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

import decimal

import sys
from m5.util import warn

# fix the global frequency
def fixGlobalFrequency():
    import _m5.core

    _m5.core.fixClockFrequency()


def setGlobalFrequency(ticksPerSecond):
    from m5.util import convert
    import _m5.core

    if isinstance(ticksPerSecond, int):
        tps = ticksPerSecond
    elif isinstance(ticksPerSecond, float):
        tps = ticksPerSecond
    elif isinstance(ticksPerSecond, str):
        tps = round(convert.anyToFrequency(ticksPerSecond))
    else:
        raise TypeError(
            f"wrong type '{type(ticksPerSecond)}' for ticksPerSecond"
        )
    _m5.core.setClockFrequency(int(tps))


# how big does a rounding error need to be before we warn about it?
frequency_tolerance = 0.001  # 0.1%


def fromSeconds(value):
    import _m5.core

    if not isinstance(value, float):
        raise TypeError(f"can't convert '{type(value)}' to type tick")

    # once someone needs to convert to seconds, the global frequency
    # had better be fixed
    if not _m5.core.clockFrequencyFixed():
        raise AttributeError(
            "In order to do conversions, the global frequency must be fixed"
        )

    if value == 0:
        return 0

    # convert the value from time to ticks
    value *= _m5.core.getClockFrequency()

    int_value = int(
        decimal.Decimal(value).to_integral_value(decimal.ROUND_HALF_UP)
    )
    err = (value - int_value) / value
    if err > frequency_tolerance:
        warn(
            "rounding error > tolerance\n    %f rounded to %d",
            value,
            int_value,
        )
    return int_value


__all__ = [
    "setGlobalFrequency",
    "fixGlobalFrequency",
    "fromSeconds",
    "frequency_tolerance",
]
