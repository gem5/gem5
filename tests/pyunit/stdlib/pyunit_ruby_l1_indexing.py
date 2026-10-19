# Copyright (c) 2026 The Regents of the University of California
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

import unittest

from m5.objects import (
    SrcClockDomain,
    VoltageDomain,
)
from m5.params import Unsigned

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.processors.linear_generator import LinearGenerator
from gem5.isas import ISA
from gem5.runtime import get_supported_protocols


class RubyL1IndexingTestSuite(unittest.TestCase):
    """Check that L1 set indices start after the cache line offset."""

    def _create_controller(self, protocol, cache_line_size):
        from m5.objects import SimpleNetwork

        kwargs = {
            "network": SimpleNetwork(),
            "core": LinearGenerator().get_cores()[0],
            "cache_line_size": cache_line_size,
            "target_isa": ISA.NULL,
            "clk_domain": SrcClockDomain(
                clock="3GHz", voltage_domain=VoltageDomain()
            ),
        }
        if protocol == CoherenceProtocol.MESI_TWO_LEVEL:
            from gem5.components.cachehierarchies.ruby.caches.mesi_two_level.l1_cache import (
                L1Cache,
            )

            controller = L1Cache(
                l1i_size="32KiB",
                l1i_assoc=8,
                l1d_size="32KiB",
                l1d_assoc=8,
                num_l2Caches=1,
                **kwargs,
            )
            return [controller.L1Icache, controller.L1Dcache]

        from gem5.components.cachehierarchies.ruby.caches.mi_example.l1_cache import (
            L1Cache,
        )

        controller = L1Cache(size="32KiB", assoc=8, **kwargs)
        return [controller.cacheMemory]

    def _check_index_bits(self, protocol):
        for line_size, expected_bit in ((32, 5), (64, 6), (128, 7)):
            for value in (line_size, Unsigned(line_size)):
                with self.subTest(
                    line_size=line_size, value_type=type(value).__name__
                ):
                    for cache in self._create_controller(protocol, value):
                        self.assertEqual(
                            int(cache.start_index_bit), expected_bit
                        )

    def _check_invalid_line_sizes(self, protocol):
        for line_size in (0, -64, 48, 96):
            with self.subTest(line_size=line_size):
                with self.assertRaises(ValueError):
                    self._create_controller(protocol, line_size)

    @unittest.skipUnless(
        CoherenceProtocol.MESI_TWO_LEVEL in get_supported_protocols(),
        "MESI_Two_Level is not compiled into this binary",
    )
    def test_mesi_two_level_index_bits(self):
        self._check_index_bits(CoherenceProtocol.MESI_TWO_LEVEL)

    @unittest.skipUnless(
        CoherenceProtocol.MI_EXAMPLE in get_supported_protocols(),
        "MI_example is not compiled into this binary",
    )
    def test_mi_example_index_bits(self):
        self._check_index_bits(CoherenceProtocol.MI_EXAMPLE)

    @unittest.skipUnless(
        CoherenceProtocol.MESI_TWO_LEVEL in get_supported_protocols(),
        "MESI_Two_Level is not compiled into this binary",
    )
    def test_mesi_two_level_invalid_line_sizes(self):
        self._check_invalid_line_sizes(CoherenceProtocol.MESI_TWO_LEVEL)

    @unittest.skipUnless(
        CoherenceProtocol.MI_EXAMPLE in get_supported_protocols(),
        "MI_example is not compiled into this binary",
    )
    def test_mi_example_invalid_line_sizes(self):
        self._check_invalid_line_sizes(CoherenceProtocol.MI_EXAMPLE)
