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
#
# Authors: Nathan Binkert

from __future__ import print_function

import sys
from m5.util import warn

tps = 1.0e12         # default to 1 THz (1 Tick == 1 ps)
tps_fixed = False    # once set to true, can't be changed

# fix the global frequency and tell C++ about it
def fixGlobalFrequency():
    import _m5.core
    global tps, tps_fixed
    if not tps_fixed:
        tps_fixed = True
        _m5.core.setClockFrequency(int(tps))
        print("Global frequency set at %d ticks per second" % int(tps))

def setGlobalFrequency(ticksPerSecond):
    from m5.util import convert

    global tps, tps_fixed

    if tps_fixed:
        raise AttributeError, \
              "Global frequency already fixed at %f ticks/s." % tps

    if isinstance(ticksPerSecond, (int, long)):
        tps = ticksPerSecond
    elif isinstance(ticksPerSecond, float):
        tps = ticksPerSecond
    elif isinstance(ticksPerSecond, str):
        tps = round(convert.anyToFrequency(ticksPerSecond))
    else:
        raise TypeError, \
              "wrong type '%s' for ticksPerSecond" % type(ticksPerSecond)

# how big does a rounding error need to be before we warn about it?
frequency_tolerance = 0.001  # 0.1%

def fromSeconds(value):
    if not isinstance(value, float):
        raise TypeError, "can't convert '%s' to type tick" % type(value)

    # once someone needs to convert to seconds, the global frequency
    # had better be fixed
    if not tps_fixed:
        raise AttributeError, \
              "In order to do conversions, the global frequency must be fixed"

    if value == 0:
        return 0

    # convert the value from time to ticks
    value *= tps

    int_value = int(round(value))
    err = (value - int_value) / value
    if err > frequency_tolerance:
        warn("rounding error > tolerance\n    %f rounded to %d", value,
            int_value)
    return int_value

__all__ = [ 'setGlobalFrequency', 'fixGlobalFrequency', 'fromSeconds',
            'frequency_tolerance' ]
