# Copyright (c) 2021 Arm Limited
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

# Attempting to import `warn` results in an error about a circular import:
# ImportError: cannot import name 'warn' from partially initialized module 'm5.util' (most likely due to a circular import) (/home/bees/gem5-3rd-worktree/src/python/m5/util/__init__.py):

# from m5.util import warn
# from . import warn

# metric prefixes
from typing import Optional

atto = 1.0e-18
femto = 1.0e-15
pico = 1.0e-12
nano = 1.0e-9
micro = 1.0e-6
milli = 1.0e-3

kilo = 1.0e3
mega = 1.0e6
giga = 1.0e9
tera = 1.0e12
peta = 1.0e15
exa = 1.0e18

# power of 2 prefixes
kibi = 1024
mebi = kibi * 1024
gibi = mebi * 1024
tebi = gibi * 1024
pebi = tebi * 1024
exbi = pebi * 1024

metric_prefixes = {
    "Ei": exbi,
    "E": exa,
    "Pi": pebi,
    "P": peta,
    "Ti": tebi,
    "T": tera,
    "Gi": gibi,
    "G": giga,
    "M": mega,
    "Ki": kibi,
    "k": kilo,
    "Mi": mebi,
    "m": milli,
    "u": micro,
    "n": nano,
    "p": pico,
    "f": femto,
    "a": atto,
}

binary_prefixes = {
    "Ei": exbi,
    "E": exbi,
    "Pi": pebi,
    "P": pebi,
    "Ti": tebi,
    "T": tebi,
    "Gi": gibi,
    "G": gibi,
    "Mi": mebi,
    "M": mebi,
    "Ki": kibi,
    "k": kibi,
}

base_10_to_2 = {
    "k": "Ki",
    "M": "Mi",
    "G": "Gi",
    "T": "Ti",
    "P": "Pi",
    "E": "Ei",
}


def assertStr(value):
    if not isinstance(value, str):
        raise TypeError(f"wrong type '{type(value)}' should be str")


def _split_suffix(value, suffixes):
    """Split a string based on a suffix from a list of suffixes.

    :param value: String value to test for a matching suffix.
    :param suffixes: Container of suffixes to test.

    :returns: A tuple of (value, suffix). Suffix is the empty string
              if there is no match.

    """
    matches = [sfx for sfx in suffixes if value.endswith(sfx)]
    assert len(matches) <= 1

    return (value[: -len(matches[0])], matches[0]) if matches else (value, "")


def toNum(value, target_type, units, prefixes, converter):
    """Convert a string using units and prefixes to (typically) a float or
    integer.

    String values are assumed to either be a naked magnitude without a
    unit or prefix, or a magnitude with a unit and an optional prefix.

    :param value: String value to convert.
    :param target_type: Type name for error messages.
    :param units: Unit (string) or list of valid units.
    :param prefixes: Mapping of prefixes to multipliers.
    :param converter: Helper function to convert magnitude to native
                      type.

    :returns: Tuple of (converted value, unit)

    """
    assertStr(value)

    def convert(val):
        try:
            return converter(val)
        except ValueError:
            raise ValueError(f"cannot convert '{value}' to {target_type}")

    # Units can be None, the empty string, or a list/tuple. Convert
    # to a tuple for consistent handling.
    if not units:
        units = tuple()
    elif isinstance(units, str):
        units = (units,)
    else:
        units = tuple(units)

    magnitude_prefix, unit = _split_suffix(value, units)

    # We only allow a prefix if there is a unit
    if unit:
        magnitude, prefix = _split_suffix(magnitude_prefix, prefixes)
        scale = prefixes[prefix] if prefix else 1
    else:
        magnitude, prefix, scale = magnitude_prefix, "", 1

    return convert(magnitude) * scale, unit


def toFloat(value, target_type="float", units=None, prefixes=[]):
    return toNum(value, target_type, units, prefixes, float)[0]


def toMetricFloat(value, target_type="float", units=None):
    return toFloat(value, target_type, units, metric_prefixes)


def toBinaryFloat(value, target_type="float", units=None):
    return toFloat(value, target_type, units, binary_prefixes)


def toInteger(value, target_type="integer", units=None, prefixes=[]):
    return toNum(value, target_type, units, prefixes, lambda x: int(x, 0))[0]


def toMetricInteger(value, target_type="integer", units=None):
    return toInteger(value, target_type, units, metric_prefixes)


def toBinaryInteger(value, target_type="integer", units=None):
    return toInteger(value, target_type, units, binary_prefixes)


def toBool(value):
    assertStr(value)

    value = value.lower()
    if value in ("true", "t", "yes", "y", "1"):
        return True
    if value in ("false", "f", "no", "n", "0"):
        return False
    raise ValueError(f"cannot convert '{value}' to bool")


def toFrequency(value):
    return toMetricFloat(value, "frequency", "Hz")


def toLatency(value):
    return toMetricFloat(value, "latency", "s")


def anyToLatency(value):
    """Convert a magnitude and unit to a clock period."""

    magnitude, unit = toNum(
        value,
        target_type="latency",
        units=("Hz", "s"),
        prefixes=metric_prefixes,
        converter=float,
    )
    if unit == "s":
        return magnitude
    elif unit == "Hz":
        try:
            return 1.0 / magnitude
        except ZeroDivisionError:
            raise ValueError(f"cannot convert '{value}' to clock period")
    else:
        raise ValueError(f"'{value}' needs a valid unit to be unambiguous.")


def anyToFrequency(value):
    """Convert a magnitude and unit to a clock frequency."""

    magnitude, unit = toNum(
        value,
        target_type="frequency",
        units=("Hz", "s"),
        prefixes=metric_prefixes,
        converter=float,
    )
    if unit == "Hz":
        return magnitude
    elif unit == "s":
        try:
            return 1.0 / magnitude
        except ZeroDivisionError:
            raise ValueError(f"cannot convert '{value}' to frequency")
    else:
        raise ValueError(f"'{value}' needs a valid unit to be unambiguous.")


def toNetworkBandwidth(value):
    return toMetricFloat(value, "network bandwidth", "bps")


def toMemoryBandwidth(value):
    checkBaseConversion(value, "B/s")
    return toBinaryFloat(value, "memory bandwidth", "B/s")


def _base_10_to_2(value: str, unit: str) -> Optional[str]:
    """Convert a base 10 memory/cache size SI prefix strings to base 2. Used
    in `checkBaseConversion` to provide a warning message to the user. Will
    return None if no conversion is required.

    This function is intentionally separate from `checkBaseConversion` to aid
    in testing."""
    size_and_prefix, _ = _split_suffix(value, [unit])
    size, prefix = _split_suffix(size_and_prefix, binary_prefixes)
    if prefix in base_10_to_2.keys():
        return f"{size}{base_10_to_2[prefix]}"
    return None


def checkBaseConversion(value, unit):
    if type(value) is str:
        new_value = _base_10_to_2(value, unit)
        if new_value:
            from m5.util import warn

            warn(
                f"Base 10 memory/cache size {value} will be cast to base 2"
                + f" size {new_value}{unit}."
            )


def toMemorySize(value):
    checkBaseConversion(value, "B")
    return toBinaryInteger(value, "memory size", "B")


def toIpAddress(value):
    if not isinstance(value, str):
        raise TypeError(f"wrong type '{type(value)}' should be str")

    bytes = value.split(".")
    if len(bytes) != 4:
        raise ValueError(f"invalid ip address {value}")

    for byte in bytes:
        if not 0 <= int(byte) <= 0xFF:
            raise ValueError(f"invalid ip address {value}")

    return (
        (int(bytes[0]) << 24)
        | (int(bytes[1]) << 16)
        | (int(bytes[2]) << 8)
        | (int(bytes[3]) << 0)
    )


def toIpNetmask(value):
    if not isinstance(value, str):
        raise TypeError(f"wrong type '{type(value)}' should be str")

    (ip, netmask) = value.split("/")
    ip = toIpAddress(ip)
    netmaskParts = netmask.split(".")
    if len(netmaskParts) == 1:
        if not 0 <= int(netmask) <= 32:
            raise ValueError(f"invalid netmask {netmask}")
        return (ip, int(netmask))
    elif len(netmaskParts) == 4:
        netmaskNum = toIpAddress(netmask)
        if netmaskNum == 0:
            return (ip, 0)
        testVal = 0
        for i in range(32):
            testVal |= 1 << (31 - i)
            if testVal == netmaskNum:
                return (ip, i + 1)
        raise ValueError(f"invalid netmask {netmask}")
    else:
        raise ValueError(f"invalid netmask {netmask}")


def toIpWithPort(value):
    if not isinstance(value, str):
        raise TypeError(f"wrong type '{type(value)}' should be str")

    (ip, port) = value.split(":")
    ip = toIpAddress(ip)
    if not 0 <= int(port) <= 0xFFFF:
        raise ValueError(f"invalid port {port}")
    return (ip, int(port))


def toVoltage(value):
    return toMetricFloat(value, "voltage", "V")


def toCurrent(value):
    return toMetricFloat(value, "current", "A")


def toEnergy(value):
    return toMetricFloat(value, "energy", "J")


def toTemperature(value):
    """Convert a string value specified to a temperature in Kelvin"""

    magnitude, unit = toNum(
        value,
        target_type="temperature",
        units=("K", "C", "F"),
        prefixes=metric_prefixes,
        converter=float,
    )
    if unit == "K":
        kelvin = magnitude
    elif unit == "C":
        kelvin = magnitude + 273.15
    elif unit == "F":
        kelvin = (magnitude + 459.67) / 1.8
    else:
        raise ValueError(f"'{value}' needs a valid temperature unit.")

    if kelvin < 0:
        raise ValueError(f"{value} is an invalid temperature")

    return kelvin
