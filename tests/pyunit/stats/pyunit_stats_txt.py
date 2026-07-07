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

import math
import unittest
from pathlib import Path

if __package__:
    from .stats_txt import (
        StatsParseError,
        parse_stats_file,
        parse_stats_text,
    )
else:
    from stats_txt import (
        StatsParseError,
        parse_stats_file,
        parse_stats_text,
    )


_FIXTURES = Path(__file__).parent / "fixtures"


class StatsTxtParserTestCase(unittest.TestCase):
    def test_single_dump_with_scalar_stats(self):
        dumps = parse_stats_file(_FIXTURES / "single_dump.txt")

        self.assertEqual(1, len(dumps))
        self.assertEqual(
            ["simSeconds", "simTicks", "system.cpu.numCycles"],
            list(dumps[0].stats.keys()),
        )
        self.assertEqual(0.000001, dumps[0]["simSeconds"])
        self.assertEqual(1000.0, dumps[0]["simTicks"])
        self.assertEqual(500.0, dumps[0]["system.cpu.numCycles"])

    def test_multiple_dumps_preserve_order_and_messages(self):
        dumps = parse_stats_file(_FIXTURES / "multiple_dumps.txt")

        self.assertEqual(2, len(dumps))
        self.assertEqual("before reset", dumps[0].message)
        self.assertEqual("after reset", dumps[1].message)
        self.assertEqual(100.0, dumps[0]["simInsts"])
        self.assertEqual(2.0, dumps[1]["simInsts"])

    def test_subnames_and_numeric_edge_values(self):
        dumps = parse_stats_file(_FIXTURES / "edge_values.txt")
        stats = dumps[0].stats

        self.assertIn("system.cpu.dcache.overallAccesses::cpu.data", stats)
        self.assertEqual(
            42.0,
            stats["system.cpu.dcache.overallAccesses::cpu.data"],
        )
        self.assertEqual(
            0.0,
            stats["system.cpu.branch-predictor.hits[0]"],
        )
        self.assertEqual(-7.5, stats["system.cpu.negative"])
        self.assertEqual(1250.0, stats["system.cpu.scientific"])
        self.assertEqual(4.0, stats["system.cpu.percentages"])
        self.assertTrue(math.isnan(stats["system.cpu.not_a_number"]))
        self.assertTrue(math.isinf(stats["system.cpu.infinity"]))
        self.assertNotIn("system.cpu.oneline_vector", stats)

    def test_comments_after_values_are_ignored(self):
        dumps = parse_stats_text("""
            ---------- Begin Simulation Statistics ----------
            system.cpu.numCycles 12 # description with words and 99
            ---------- End Simulation Statistics   ----------
            """)

        self.assertEqual(12.0, dumps[0]["system.cpu.numCycles"])

    def test_empty_dump_is_returned(self):
        dumps = parse_stats_file(_FIXTURES / "empty_dump.txt")

        self.assertEqual(1, len(dumps))
        self.assertEqual([], list(dumps[0].stats.items()))

    def test_file_without_dump_sections_returns_no_dumps(self):
        self.assertEqual([], parse_stats_file(_FIXTURES / "no_dump.txt"))

    def test_malformed_value_raises_clear_error(self):
        with self.assertRaisesRegex(
            StatsParseError,
            "invalid stat value 'not_a_number' for 'system.cpu.numCycles'",
        ):
            parse_stats_file(_FIXTURES / "malformed_line.txt")

    def test_duplicate_stat_name_raises_clear_error(self):
        with self.assertRaisesRegex(
            StatsParseError,
            "duplicate stat name 'simInsts'",
        ):
            parse_stats_file(_FIXTURES / "duplicate_stat.txt")

    def test_unterminated_dump_raises_clear_error(self):
        with self.assertRaisesRegex(
            StatsParseError,
            "stats dump was not terminated",
        ):
            parse_stats_text("""
                ---------- Begin Simulation Statistics ----------
                simInsts 1
                """)

    def test_end_before_begin_raises_clear_error(self):
        with self.assertRaisesRegex(
            StatsParseError,
            "found stats dump end before begin",
        ):
            parse_stats_text(
                "---------- End Simulation Statistics   ----------"
            )
