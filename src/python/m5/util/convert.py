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
#
# Authors: Nathan Binkert
#          Gabe Black

# metric prefixes
exa  = 1.0e18
peta = 1.0e15
tera = 1.0e12
giga = 1.0e9
mega = 1.0e6
kilo = 1.0e3

milli = 1.0e-3
micro = 1.0e-6
nano  = 1.0e-9
pico  = 1.0e-12
femto = 1.0e-15
atto  = 1.0e-18

# power of 2 prefixes
kibi = 1024
mebi = kibi * 1024
gibi = mebi * 1024
tebi = gibi * 1024
pebi = tebi * 1024
exbi = pebi * 1024

# memory size configuration stuff
def toFloat(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('Ei'):
        return float(value[:-2]) * exbi
    elif value.endswith('Pi'):
        return float(value[:-2]) * pebi
    elif value.endswith('Ti'):
        return float(value[:-2]) * tebi
    elif value.endswith('Gi'):
        return float(value[:-2]) * gibi
    elif value.endswith('Mi'):
        return float(value[:-2]) * mebi
    elif value.endswith('ki'):
        return float(value[:-2]) * kibi
    elif value.endswith('E'):
        return float(value[:-1]) * exa
    elif value.endswith('P'):
        return float(value[:-1]) * peta
    elif value.endswith('T'):
        return float(value[:-1]) * tera
    elif value.endswith('G'):
        return float(value[:-1]) * giga
    elif value.endswith('M'):
        return float(value[:-1]) * mega
    elif value.endswith('k'):
        return float(value[:-1]) * kilo
    elif value.endswith('m'):
        return float(value[:-1]) * milli
    elif value.endswith('u'):
        return float(value[:-1]) * micro
    elif value.endswith('n'):
        return float(value[:-1]) * nano
    elif value.endswith('p'):
        return float(value[:-1]) * pico
    elif value.endswith('f'):
        return float(value[:-1]) * femto
    else:
        return float(value)

def toInteger(value):
    value = toFloat(value)
    result = long(value)
    if value != result:
        raise ValueError, "cannot convert '%s' to integer" % value

    return result

_bool_dict = {
    'true' : True,   't' : True,  'yes' : True, 'y' : True,  '1' : True,
    'false' : False, 'f' : False, 'no' : False, 'n' : False, '0' : False
    }

def toBool(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    value = value.lower()
    result = _bool_dict.get(value, None)
    if result == None:
        raise ValueError, "cannot convert '%s' to bool" % value
    return result

def toFrequency(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('THz'):
        return float(value[:-3]) * tera
    elif value.endswith('GHz'):
        return float(value[:-3]) * giga
    elif value.endswith('MHz'):
        return float(value[:-3]) * mega
    elif value.endswith('kHz'):
        return float(value[:-3]) * kilo
    elif value.endswith('Hz'):
        return float(value[:-2])

    raise ValueError, "cannot convert '%s' to frequency" % value

def toLatency(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('ps'):
        return float(value[:-2]) * pico
    elif value.endswith('ns'):
        return float(value[:-2]) * nano
    elif value.endswith('us'):
        return float(value[:-2]) * micro
    elif value.endswith('ms'):
        return float(value[:-2]) * milli
    elif value.endswith('s'):
        return float(value[:-1])

    raise ValueError, "cannot convert '%s' to latency" % value

def anyToLatency(value):
    """result is a clock period"""

    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    try:
        val = toFrequency(value)
        if val != 0:
            val = 1 / val
        return val
    except ValueError:
        pass

    try:
        val = toLatency(value)
        return val
    except ValueError:
        pass

    raise ValueError, "cannot convert '%s' to clock period" % value

def anyToFrequency(value):
    """result is a clock period"""

    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    try:
        val = toFrequency(value)
        return val
    except ValueError:
        pass

    try:
        val = toLatency(value)
        if val != 0:
            val = 1 / val
        return val
    except ValueError:
        pass

    raise ValueError, "cannot convert '%s' to clock period" % value

def toNetworkBandwidth(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('Tbps'):
        return float(value[:-4]) * tera
    elif value.endswith('Gbps'):
        return float(value[:-4]) * giga
    elif value.endswith('Mbps'):
        return float(value[:-4]) * mega
    elif value.endswith('kbps'):
        return float(value[:-4]) * kilo
    elif value.endswith('bps'):
        return float(value[:-3])
    else:
        return float(value)

    raise ValueError, "cannot convert '%s' to network bandwidth" % value

def toMemoryBandwidth(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('PB/s'):
        return float(value[:-4]) * pebi
    elif value.endswith('TB/s'):
        return float(value[:-4]) * tebi
    elif value.endswith('GB/s'):
        return float(value[:-4]) * gibi
    elif value.endswith('MB/s'):
        return float(value[:-4]) * mebi
    elif value.endswith('kB/s'):
        return float(value[:-4]) * kibi
    elif value.endswith('B/s'):
        return float(value[:-3])

    raise ValueError, "cannot convert '%s' to memory bandwidth" % value

def toMemorySize(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('PB'):
        return long(value[:-2]) * pebi
    elif value.endswith('TB'):
        return long(value[:-2]) * tebi
    elif value.endswith('GB'):
        return long(value[:-2]) * gibi
    elif value.endswith('MB'):
        return long(value[:-2]) * mebi
    elif value.endswith('kB'):
        return long(value[:-2]) * kibi
    elif value.endswith('B'):
        return long(value[:-1])

    raise ValueError, "cannot convert '%s' to memory size" % value

def toIpAddress(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    bytes = value.split('.')
    if len(bytes) != 4:
        raise ValueError, 'invalid ip address %s' % value

    for byte in bytes:
        if not 0 <= int(byte) <= 0xff:
            raise ValueError, 'invalid ip address %s' % value

    return (int(bytes[0]) << 24) | (int(bytes[1]) << 16) | \
           (int(bytes[2]) << 8)  | (int(bytes[3]) << 0)

def toIpNetmask(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    (ip, netmask) = value.split('/')
    ip = toIpAddress(ip)
    netmaskParts = netmask.split('.')
    if len(netmaskParts) == 1:
        if not 0 <= int(netmask) <= 32:
            raise ValueError, 'invalid netmask %s' % netmask
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
        raise ValueError, 'invalid netmask %s' % netmask
    else:
        raise ValueError, 'invalid netmask %s' % netmask

def toIpWithPort(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    (ip, port) = value.split(':')
    ip = toIpAddress(ip)
    if not 0 <= int(port) <= 0xffff:
        raise ValueError, 'invalid port %s' % port
    return (ip, int(port))

def toVoltage(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('mV'):
        return float(value[:-2]) * milli
    elif value.endswith('V'):
        return float(value[:-1])

    raise ValueError, "cannot convert '%s' to voltage" % value

def toCurrent(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('A'):
        return toFloat(value[:-1])

    raise ValueError, "cannot convert '%s' to current" % value
