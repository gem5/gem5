# Copyright (c) 2005 The Regents of The University of Michigan
# Copyright (c) 2010 Advanced Micro Devices, Inc.
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

import six
if six.PY3:
    long = int

# metric prefixes
atto  = 1.0e-18
femto = 1.0e-15
pico  = 1.0e-12
nano  = 1.0e-9
micro = 1.0e-6
milli = 1.0e-3

kilo = 1.0e3
mega = 1.0e6
giga = 1.0e9
tera = 1.0e12
peta = 1.0e15
exa  = 1.0e18

# power of 2 prefixes
kibi = 1024
mebi = kibi * 1024
gibi = mebi * 1024
tebi = gibi * 1024
pebi = tebi * 1024
exbi = pebi * 1024

metric_prefixes = {
    'Ei': exbi,
    'E': exa,
    'Pi': pebi,
    'P': peta,
    'Ti': tebi,
    'T': tera,
    'Gi': gibi,
    'G': giga,
    'M': mega,
    'ki': kibi,
    'k': kilo,
    'Mi': mebi,
    'm': milli,
    'u': micro,
    'n': nano,
    'p': pico,
    'f': femto,
    'a': atto,
}

binary_prefixes = {
    'Ei': exbi,
    'E' : exbi,
    'Pi': pebi,
    'P' : pebi,
    'Ti': tebi,
    'T' : tebi,
    'Gi': gibi,
    'G' : gibi,
    'Mi': mebi,
    'M' : mebi,
    'ki': kibi,
    'k' : kibi,
}

def assertStr(value):
    if not isinstance(value, str):
        raise TypeError("wrong type '%s' should be str" % type(value))


# memory size configuration stuff
def toNum(value, target_type, units, prefixes, converter):
    assertStr(value)

    def convert(val):
        try:
            return converter(val)
        except ValueError:
            raise ValueError(
                "cannot convert '%s' to %s" % (value, target_type))

    if units and not value.endswith(units):
        units = None
    if not units:
        return convert(value)

    value = value[:-len(units)]

    prefix = next((p for p in prefixes.keys() if value.endswith(p)), None)
    if not prefix:
        return convert(value)
    value = value[:-len(prefix)]

    return convert(value) * prefixes[prefix]

def toFloat(value, target_type='float', units=None, prefixes=[]):
    return toNum(value, target_type, units, prefixes, float)

def toMetricFloat(value, target_type='float', units=None):
    return toFloat(value, target_type, units, metric_prefixes)

def toBinaryFloat(value, target_type='float', units=None):
    return toFloat(value, target_type, units, binary_prefixes)

def toInteger(value, target_type='integer', units=None, prefixes=[]):
    intifier = lambda x: int(x, 0)
    return toNum(value, target_type, units, prefixes, intifier)

def toMetricInteger(value, target_type='integer', units=None):
    return toInteger(value, target_type, units, metric_prefixes)

def toBinaryInteger(value, target_type='integer', units=None):
    return toInteger(value, target_type, units, binary_prefixes)

def toBool(value):
    assertStr(value)

    value = value.lower()
    if value in ('true', 't', 'yes', 'y', '1'):
        return True
    if value in ('false', 'f', 'no', 'n', '0'):
        return False
    return result

def toFrequency(value):
    return toMetricFloat(value, 'frequency', 'Hz')

def toLatency(value):
    return toMetricFloat(value, 'latency', 's')

def anyToLatency(value):
    """result is a clock period"""
    try:
        return 1 / toFrequency(value)
    except (ValueError, ZeroDivisionError):
        pass

    try:
        return toLatency(value)
    except ValueError:
        pass

    raise ValueError("cannot convert '%s' to clock period" % value)

def anyToFrequency(value):
    """result is a clock period"""
    try:
        return toFrequency(value)
    except ValueError:
        pass

    try:
        return 1 / toLatency(value)
    except ValueError as ZeroDivisionError:
        pass

    raise ValueError("cannot convert '%s' to clock period" % value)

def toNetworkBandwidth(value):
    return toMetricFloat(value, 'network bandwidth', 'bps')

def toMemoryBandwidth(value):
    return toBinaryFloat(value, 'memory bandwidth', 'B/s')

def toMemorySize(value):
    return toBinaryInteger(value, 'memory size', 'B')

def toIpAddress(value):
    if not isinstance(value, str):
        raise TypeError("wrong type '%s' should be str" % type(value))

    bytes = value.split('.')
    if len(bytes) != 4:
        raise ValueError('invalid ip address %s' % value)

    for byte in bytes:
        if not 0 <= int(byte) <= 0xff:
            raise ValueError('invalid ip address %s' % value)

    return (int(bytes[0]) << 24) | (int(bytes[1]) << 16) | \
           (int(bytes[2]) << 8)  | (int(bytes[3]) << 0)

def toIpNetmask(value):
    if not isinstance(value, str):
        raise TypeError("wrong type '%s' should be str" % type(value))

    (ip, netmask) = value.split('/')
    ip = toIpAddress(ip)
    netmaskParts = netmask.split('.')
    if len(netmaskParts) == 1:
        if not 0 <= int(netmask) <= 32:
            raise ValueError('invalid netmask %s' % netmask)
        return (ip, int(netmask))
    elif len(netmaskParts) == 4:
        netmaskNum = toIpAddress(netmask)
        if netmaskNum == 0:
            return (ip, 0)
        testVal = 0
        for i in range(32):
            testVal |= (1 << (31 - i))
            if testVal == netmaskNum:
                return (ip, i + 1)
        raise ValueError('invalid netmask %s' % netmask)
    else:
        raise ValueError('invalid netmask %s' % netmask)

def toIpWithPort(value):
    if not isinstance(value, str):
        raise TypeError("wrong type '%s' should be str" % type(value))

    (ip, port) = value.split(':')
    ip = toIpAddress(ip)
    if not 0 <= int(port) <= 0xffff:
        raise ValueError('invalid port %s' % port)
    return (ip, int(port))

def toVoltage(value):
    return toMetricFloat(value, 'voltage', 'V')

def toCurrent(value):
    return toMetricFloat(value, 'current', 'A')

def toEnergy(value):
    return toMetricFloat(value, 'energy', 'J')
