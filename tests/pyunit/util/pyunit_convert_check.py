#!/usr/bin/env python3
#
# Copyright (c) 2021 ARM Limited
# All rights reserved
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

import unittest

from m5.util import convert


def _ip(*args):
    return (args[0] << 24) | (args[1] << 16) | (args[2] << 8) | args[3]


class ConvertTestSuite(unittest.TestCase):
    """Test cases for unit conversion"""

    def test_toMetricFloat(self):
        def conv(x):
            return convert.toMetricFloat(x, "value", "X")

        self.assertEqual(conv("42"), 42e0)
        self.assertEqual(conv("42.5"), 42.5e0)
        self.assertEqual(conv("42kX"), 42e3)
        self.assertEqual(conv("42.5kX"), 42.5e3)
        self.assertEqual(conv("42MX"), 42e6)
        self.assertEqual(conv("42GX"), 42e9)
        self.assertEqual(conv("42TX"), 42e12)
        self.assertEqual(conv("42PX"), 42e15)
        self.assertEqual(conv("42EX"), 42e18)

        self.assertEqual(conv("42KiX"), 42 * 2**10)
        self.assertEqual(conv("42MiX"), 42 * 2**20)
        self.assertEqual(conv("42GiX"), 42 * 2**30)
        self.assertEqual(conv("42TiX"), 42 * 2**40)
        self.assertEqual(conv("42PiX"), 42 * 2**50)
        self.assertEqual(conv("42EiX"), 42 * 2**60)

        self.assertRaises(ValueError, conv, "42k")
        self.assertRaises(ValueError, conv, "42KX")
        self.assertRaises(ValueError, conv, "42kiX")

        self.assertEqual(convert.toMetricFloat("42"), 42)
        # Prefixes not allowed without a unit
        self.assertRaises(ValueError, convert.toMetricFloat, "42k")

    def test_toMetricInteger(self):
        def conv(x):
            return convert.toMetricInteger(x, "value", "X")

        self.assertEqual(conv("42"), 42 * 10**0)
        self.assertEqual(conv("42kX"), 42 * 10**3)
        self.assertEqual(conv("42MX"), 42 * 10**6)
        self.assertEqual(conv("42GX"), 42 * 10**9)
        self.assertEqual(conv("42TX"), 42 * 10**12)
        self.assertEqual(conv("42PX"), 42 * 10**15)
        self.assertEqual(conv("42EX"), 42 * 10**18)

        self.assertEqual(conv("42KiX"), 42 * 2**10)
        self.assertEqual(conv("42MiX"), 42 * 2**20)
        self.assertEqual(conv("42GiX"), 42 * 2**30)
        self.assertEqual(conv("42TiX"), 42 * 2**40)
        self.assertEqual(conv("42PiX"), 42 * 2**50)
        self.assertEqual(conv("42EiX"), 42 * 2**60)

        self.assertRaises(ValueError, conv, "42.1")
        self.assertRaises(ValueError, conv, "42.1kX")

        self.assertRaises(ValueError, conv, "42k")
        self.assertRaises(ValueError, conv, "42KX")
        self.assertRaises(ValueError, conv, "42kiX")

        self.assertEqual(convert.toMetricInteger("42"), 42)

        # Prefixes not allowed without a unit
        self.assertRaises(ValueError, convert.toMetricInteger, "42k")

    def test_toBool(self):
        conv = convert.toBool

        self.assertEqual(conv("TRUE"), True)
        self.assertEqual(conv("true"), True)
        self.assertEqual(conv("t"), True)
        self.assertEqual(conv("yes"), True)
        self.assertEqual(conv("y"), True)
        self.assertEqual(conv("1"), True)

        self.assertEqual(conv("FALSE"), False)
        self.assertEqual(conv("false"), False)
        self.assertEqual(conv("f"), False)
        self.assertEqual(conv("no"), False)
        self.assertEqual(conv("n"), False)
        self.assertEqual(conv("0"), False)

        self.assertRaises(ValueError, conv, "not a bool")
        self.assertRaises(ValueError, conv, "2")

    def test_toFrequency(self):
        conv = convert.toFrequency

        self.assertEqual(conv("42"), 42.0)
        self.assertEqual(conv("42Hz"), 42)
        self.assertEqual(conv("42kHz"), 42e3)

        # Prefixes need a unit
        self.assertRaises(ValueError, conv, "42k")
        # Seconds isn't a valid unit unless using anyToFrequency.
        self.assertRaises(ValueError, conv, "42s")

    def test_toLatency(self):
        conv = convert.toLatency

        self.assertEqual(conv("42"), 42.0)
        self.assertEqual(conv("42s"), 42.0)

        # We allow prefixes for seconds.
        self.assertEqual(conv("42ks"), 42e3)

        # Prefixe need a unit
        self.assertRaises(ValueError, conv, "42k")
        # Hz shouldn't be converted unless using anyToLatency
        self.assertRaises(ValueError, conv, "42Hz")

    def test_anyToLatency(self):
        conv = convert.anyToLatency

        self.assertEqual(conv("42s"), 42.0)

        # We currently allow prefixes for seconds.
        self.assertEqual(conv("42ks"), 42e3)

        self.assertEqual(conv("10Hz"), 0.1)
        self.assertEqual(conv("1kHz"), 1e-3)

        self.assertRaises(ValueError, conv, "42k")
        self.assertRaises(ValueError, conv, "42")

    def test_anyToFrequency(self):
        conv = convert.anyToFrequency

        self.assertEqual(conv("42kHz"), 42e3)

        self.assertEqual(conv("0.1s"), 10.0)
        self.assertEqual(conv("1ms"), 1000.0)

        self.assertRaises(ValueError, conv, "42k")
        self.assertRaises(ValueError, conv, "42")

    def test_toNetworkBandwidth(self):
        conv = convert.toNetworkBandwidth

        self.assertEqual(conv("42"), 42.0)
        self.assertEqual(conv("42bps"), 42.0)
        self.assertEqual(conv("42kbps"), 42e3)

        self.assertRaises(ValueError, conv, "42Kbps")

    def test_toMemoryBandwidth(self):
        conv = convert.toMemoryBandwidth

        self.assertEqual(conv("42"), 42.0)
        self.assertEqual(conv("42B/s"), 42.0)

        self.assertEqual(conv("42MB/s"), 42 * 2**20)
        self.assertEqual(conv("42MiB/s"), 42 * 2**20)

        self.assertRaises(ValueError, conv, "42KB/s")
        self.assertRaises(ValueError, conv, "42Mi")

    def test_toMemorySize(self):
        conv = convert.toMemorySize

        self.assertEqual(conv("42"), 42.0)
        self.assertEqual(conv("42B"), 42.0)

        self.assertEqual(conv("42kB"), 42 * 2**10)
        self.assertEqual(conv("42MB"), 42 * 2**20)

        self.assertEqual(conv("42KiB"), 42 * 2**10)
        self.assertEqual(conv("42MiB"), 42 * 2**20)

    def test_toIpAddress(self):
        conv = convert.toIpAddress

        self.assertEqual(conv("255.255.255.255"), _ip(255, 255, 255, 255))
        self.assertEqual(conv("1.2.3.4"), _ip(1, 2, 3, 4))

        self.assertRaises(TypeError, conv, 0)
        self.assertRaises(ValueError, conv, "0.0.0")
        self.assertRaises(ValueError, conv, "0.0.0.300")
        self.assertRaises(ValueError, conv, "0.0.0.0.0")

    def test_toIpNetmask(self):
        conv = convert.toIpNetmask

        self.assertEqual(conv("1.2.3.4/24"), (_ip(1, 2, 3, 4), 24))
        self.assertEqual(conv("1.2.3.4/255.255.255.0"), (_ip(1, 2, 3, 4), 24))

        self.assertEqual(conv("1.2.3.4/0"), (_ip(1, 2, 3, 4), 0))
        self.assertEqual(conv("1.2.3.4/0.0.0.0"), (_ip(1, 2, 3, 4), 0))

        self.assertRaises(ValueError, conv, "0.0.0.0")
        self.assertRaises(ValueError, conv, "0.0.0.0/")
        self.assertRaises(ValueError, conv, "0.0.0.0/64")

    def test_toIpWithPort(self):
        conv = convert.toIpWithPort

        self.assertEqual(conv("1.2.3.4:42"), (_ip(1, 2, 3, 4), 42))

        self.assertRaises(ValueError, conv, "0.0.0.0")
        self.assertRaises(ValueError, conv, "0.0.0.0:")
        self.assertRaises(ValueError, conv, "0.0.0.0:65536")

    def test_toVoltage(self):
        conv = convert.toVoltage

        self.assertEqual(conv("42"), 42)
        self.assertEqual(conv("42V"), 42)
        self.assertEqual(conv("42kV"), 42e3)

    def test_toCurrent(self):
        conv = convert.toCurrent

        self.assertEqual(conv("42"), 42)
        self.assertEqual(conv("42A"), 42)
        self.assertEqual(conv("42kA"), 42e3)

    def test_toEnergy(self):
        conv = convert.toEnergy

        self.assertEqual(conv("42"), 42)
        self.assertEqual(conv("42J"), 42)
        self.assertEqual(conv("42kJ"), 42e3)

    def test_temperature(self):
        conv = convert.toTemperature

        self.assertEqual(conv("1.0K"), 1.0)
        self.assertEqual(conv("1.0mK"), 1.0e-3)

        self.assertEqual(conv("0C"), 273.15)
        self.assertEqual(conv("-1C"), 272.15)
        self.assertRaises(ValueError, conv, "1.0")
        self.assertRaises(ValueError, conv, "-1K")

        self.assertEqual(conv("32F"), 273.15)

    def test_base_10_to_2(self):
        conv = convert._base_10_to_2

        self.assertEqual(conv("1k"), "1Ki")
        self.assertIsNone(conv("1Ki"))

        # Leaving the rest of this test for Erin to finish.
