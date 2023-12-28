# Copyright (c) 2023 The Regents of the University of California
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

import os
import unittest

from m5.params import PcCountPair

from gem5.resources.looppoint import (
    Looppoint,
    LooppointCsvLoader,
    LooppointJsonLoader,
    LooppointRegion,
    LooppointRegionPC,
    LooppointRegionWarmup,
    LooppointSimulation,
)


class LooppointRegionPCTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.LooppointRegionPC class."""

    def test_construction_with_relative(self) -> None:
        region_pc = LooppointRegionPC(pc=444, globl=65, relative=454)

        self.assertEqual(444, region_pc.get_pc())
        self.assertEqual(65, region_pc.get_global())
        self.assertEqual(454, region_pc.get_relative())

    def test_construction_without_relative(self) -> None:
        region_pc = LooppointRegionPC(pc=43454, globl=653434)

        self.assertEqual(43454, region_pc.get_pc())
        self.assertEqual(653434, region_pc.get_global())
        self.assertIsNone(region_pc.get_relative())

    def test_get_pc_count_pair(self) -> None:
        region_pc = LooppointRegionPC(pc=1, globl=2)
        expected = PcCountPair(1, 2)
        self.assertEqual(expected, region_pc.get_pc_count_pair())

    def update_relative_count(self) -> None:
        pass  # Not really sure what to do here...

    def test_to_json_with_relative(self) -> None:
        region_pc = LooppointRegionPC(pc=100, globl=200, relative=300)
        json_contents = region_pc.to_json()

        self.assertEqual(3, len(json_contents))
        self.assertTrue("pc" in json_contents)
        self.assertEqual(100, json_contents["pc"])
        self.assertTrue("global" in json_contents)
        self.assertEqual(200, json_contents["global"])
        self.assertTrue("relative" in json_contents)
        self.assertEqual(300, json_contents["relative"])

    def test_to_json_without_relative(self) -> None:
        region_pc = LooppointRegionPC(pc=1111, globl=2222)
        json_contents = region_pc.to_json()

        self.assertEqual(2, len(json_contents))
        self.assertTrue("pc" in json_contents)
        self.assertEqual(1111, json_contents["pc"])
        self.assertTrue("global" in json_contents)
        self.assertEqual(2222, json_contents["global"])
        self.assertFalse("relative" in json_contents)


class LooppointRegionWarmupTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.LooppointWarmup class."""

    def test_construction(self) -> None:
        region_warmup = LooppointRegionWarmup(
            start=PcCountPair(123, 456), end=PcCountPair(789, 1011)
        )

        self.assertEqual(PcCountPair(123, 456), region_warmup.get_start())
        self.assertEqual(PcCountPair(789, 1011), region_warmup.get_end())

    def test_get_pc_count_pairs(self) -> None:
        region_warmup = LooppointRegionWarmup(
            start=PcCountPair(1, 1), end=PcCountPair(2, 2)
        )

        output = region_warmup.get_pc_count_pairs()
        self.assertEqual(2, len(output))
        self.assertEqual(PcCountPair(1, 1), output[0])
        self.assertEqual(PcCountPair(2, 2), output[1])

    def test_to_json(self) -> None:
        region_warmup = LooppointRegionWarmup(
            start=PcCountPair(100, 200), end=PcCountPair(101, 202)
        )

        expected = {
            "start": {"pc": 100, "count": 200},
            "end": {"pc": 101, "count": 202},
        }

        self.assertDictEqual(expected, region_warmup.to_json())


class LooppointSimulationTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.LooppointSimulation class."""

    def test_construction_with(self) -> None:
        sim = LooppointSimulation(
            start=LooppointRegionPC(pc=444, globl=65, relative=454),
            end=LooppointRegionPC(pc=555, globl=699),
        )

        sim_start = sim.get_start()

        self.assertEqual(444, sim_start.get_pc())
        self.assertEqual(65, sim_start.get_global())
        self.assertEqual(454, sim_start.get_relative())

        sim_end = sim.get_end()

        self.assertEqual(555, sim_end.get_pc())
        self.assertEqual(699, sim_end.get_global())
        self.assertIsNone(sim_end.get_relative())

    def test_get_pc_count_pairs(self) -> None:
        sim = LooppointSimulation(
            start=LooppointRegionPC(pc=56, globl=45, relative=34),
            end=LooppointRegionPC(pc=23, globl=12),
        )

        sim_pc_count_pairs = sim.get_pc_count_pairs()
        self.assertEqual(2, len(sim_pc_count_pairs))
        self.assertEqual(PcCountPair(56, 45), sim_pc_count_pairs[0])
        self.assertEqual(PcCountPair(23, 12), sim_pc_count_pairs[1])

    def test_get_json(self) -> None:
        sim = LooppointSimulation(
            start=LooppointRegionPC(pc=1, globl=2, relative=3),
            end=LooppointRegionPC(pc=4, globl=5),
        )
        expected = {
            "start": {
                "pc": 1,
                "global": 2,
                "relative": 3,
            },
            "end": {
                "pc": 4,
                "global": 5,
            },
        }
        self.assertDictEqual(expected, sim.to_json())


class LooppointRegionTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.LooppointRegion class."""

    def test_construction_with_warmup(self):
        region = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=1, globl=2, relative=3),
                end=LooppointRegionPC(pc=6, globl=7),
            ),
            multiplier=5.6,
            warmup=LooppointRegionWarmup(
                start=PcCountPair(100, 200), end=PcCountPair(101, 202)
            ),
        )

        self.assertTrue(
            isinstance(region.get_simulation(), LooppointSimulation)
        )
        self.assertEqual(5.6, region.get_multiplier())
        self.assertIsNotNone(region.get_warmup())
        self.assertTrue(isinstance(region.get_warmup(), LooppointRegionWarmup))

    def test_construction_without_warmup(self):
        region = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=56, globl=2345, relative=344),
                end=LooppointRegionPC(pc=645, globl=457),
            ),
            multiplier=5444.4,
        )

        self.assertTrue(
            isinstance(region.get_simulation(), LooppointSimulation)
        )
        self.assertEqual(5444.4, region.get_multiplier())
        self.assertIsNone(region.get_warmup())

    def test_get_pc_count_pairs_with_warmup(self):
        region = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=1, globl=2, relative=3),
                end=LooppointRegionPC(pc=6, globl=7),
            ),
            multiplier=5.6,
            warmup=LooppointRegionWarmup(
                start=PcCountPair(100, 200), end=PcCountPair(101, 202)
            ),
        )
        pc_count_pairs = region.get_pc_count_pairs()

        self.assertEqual(4, len(pc_count_pairs))
        self.assertEqual(PcCountPair(1, 2), pc_count_pairs[0])
        self.assertEqual(PcCountPair(6, 7), pc_count_pairs[1])
        self.assertEqual(PcCountPair(100, 200), pc_count_pairs[2])
        self.assertEqual(PcCountPair(101, 202), pc_count_pairs[3])

    def test_get_pc_count_pairs_without_warmup(self):
        region = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=56, globl=2345, relative=344),
                end=LooppointRegionPC(pc=645, globl=457),
            ),
            multiplier=5444.4,
        )

        pc_count_pairs = region.get_pc_count_pairs()

        self.assertEqual(2, len(pc_count_pairs))
        self.assertEqual(PcCountPair(56, 2345), pc_count_pairs[0])
        self.assertEqual(PcCountPair(645, 457), pc_count_pairs[1])


class LooppointTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.Looppoint class."""

    def test_construction(self):
        region1 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=56, globl=2345, relative=344),
                end=LooppointRegionPC(pc=645, globl=457),
            ),
            multiplier=5444.4,
        )
        region2 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=67, globl=254, relative=3345),
                end=LooppointRegionPC(pc=64554, globl=7454),
            ),
            multiplier=5.6,
            warmup=LooppointRegionWarmup(
                start=PcCountPair(100, 200), end=PcCountPair(101, 202)
            ),
        )

        looppoint = Looppoint(
            regions={
                1: region1,
                3: region2,
            }
        )

        self.assertEqual(2, len(looppoint.get_regions()))
        self.assertTrue(1 in looppoint.get_regions())
        self.assertEqual(region1, looppoint.get_regions()[1])
        self.assertTrue(3 in looppoint.get_regions())
        self.assertEqual(region2, looppoint.get_regions()[3])

    def test_get_targets(self):
        region1 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=56, globl=2345, relative=344),
                end=LooppointRegionPC(pc=645, globl=457),
            ),
            multiplier=5444.4,
        )
        region2 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=67, globl=254, relative=3345),
                end=LooppointRegionPC(pc=64554, globl=7454),
            ),
            multiplier=5.6,
            warmup=LooppointRegionWarmup(
                start=PcCountPair(100, 200), end=PcCountPair(101, 202)
            ),
        )

        looppoint = Looppoint(
            regions={
                1: region1,
                3: region2,
            }
        )

        targets = looppoint.get_targets()
        self.assertEqual(6, len(targets))
        self.assertEqual(PcCountPair(56, 2345), targets[0])
        self.assertEqual(PcCountPair(645, 457), targets[1])
        self.assertEqual(PcCountPair(67, 254), targets[2])
        self.assertEqual(PcCountPair(64554, 7454), targets[3])
        self.assertEqual(PcCountPair(100, 200), targets[4])
        self.assertEqual(PcCountPair(101, 202), targets[5])

    def test_get_region_start_id_map(self):
        region1 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=56, globl=2345, relative=344),
                end=LooppointRegionPC(pc=645, globl=457),
            ),
            multiplier=5444.4,
        )
        region2 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=67, globl=254, relative=3345),
                end=LooppointRegionPC(pc=64554, globl=7454),
            ),
            multiplier=5.6,
            warmup=LooppointRegionWarmup(
                start=PcCountPair(100, 200), end=PcCountPair(101, 202)
            ),
        )

        looppoint = Looppoint(
            regions={
                1: region1,
                3: region2,
            }
        )

        region_start_id_map = looppoint.get_region_start_id_map()

        self.assertEqual(2, len(region_start_id_map))

        # The start of region1.
        self.assertTrue(PcCountPair(56, 2345) in region_start_id_map)
        self.assertEqual(1, region_start_id_map[PcCountPair(56, 2345)])

        # The start of region2.  Since this has a warmup, it's the warmup.
        self.assertTrue(PcCountPair(100, 200) in region_start_id_map)
        self.assertEqual(3, region_start_id_map[PcCountPair(100, 200)])

    def test_to_json(self) -> None:
        region1 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=56, globl=2345, relative=344),
                end=LooppointRegionPC(pc=645, globl=457),
            ),
            multiplier=5444.4,
        )
        region2 = LooppointRegion(
            simulation=LooppointSimulation(
                start=LooppointRegionPC(pc=67, globl=254, relative=3345),
                end=LooppointRegionPC(pc=64554, globl=7454),
            ),
            multiplier=5.6,
            warmup=LooppointRegionWarmup(
                start=PcCountPair(100, 200), end=PcCountPair(101, 202)
            ),
        )

        looppoint = Looppoint(
            regions={
                1: region1,
                3: region2,
            }
        )

        expected = {
            1: {
                "simulation": {
                    "start": {
                        "pc": 56,
                        "global": 2345,
                        "relative": 344,
                    },
                    "end": {
                        "pc": 645,
                        "global": 457,
                    },
                },
                "multiplier": 5444.4,
            },
            3: {
                "simulation": {
                    "start": {
                        "pc": 67,
                        "global": 254,
                        "relative": 3345,
                    },
                    "end": {
                        "pc": 64554,
                        "global": 7454,
                    },
                },
                "multiplier": 5.6,
                "warmup": {
                    "start": {
                        "pc": 100,
                        "count": 200,
                    },
                    "end": {
                        "pc": 101,
                        "count": 202,
                    },
                },
            },
        }

        # Need to increase the max for if there is an error.
        self.maxDiff = 2056
        self.assertDictEqual(expected, looppoint.to_json())


class LooppointCSVLoaderTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.LooppointCsvLoader class."""

    def test_load_pinpoints_matrix(self):
        looppoint = LooppointCsvLoader(
            pinpoints_file=os.path.join(
                os.path.realpath(os.path.dirname(__file__)),
                "refs",
                "matrix.1_92.global.pinpoints_reduced.csv",
            )
        )

        regions = looppoint.get_regions()
        self.assertEqual(3, len(regions))

        region1 = regions[1]
        self.assertEqual(4.0, region1.get_multiplier())

        region1start = region1.get_simulation().get_start()
        self.assertEqual(0x4069D0, region1start.get_pc())
        self.assertEqual(211076617, region1start.get_global())
        self.assertIsNone(region1start.get_relative())

        region1end = region1.get_simulation().get_end()
        self.assertEqual(0x4069D0, region1end.get_pc())
        self.assertEqual(219060252, region1end.get_global())
        self.assertIsNotNone(region1end.get_relative())
        self.assertEqual(1060676, region1end.get_relative())

        self.assertIsNone(region1.get_warmup())

        region2 = regions[2]
        self.assertEqual(5.001, region2.get_multiplier())

        region2start = region2.get_simulation().get_start()
        self.assertEqual(0x4069D0, region2start.get_pc())
        self.assertEqual(407294228, region2start.get_global())
        self.assertIsNone(region2start.get_relative())

        region2end = region2.get_simulation().get_end()
        self.assertEqual(0x4069D0, region2end.get_pc())
        self.assertEqual(415282447, region2end.get_global())
        self.assertIsNotNone(region2end.get_relative())
        self.assertEqual(1035231, region2end.get_relative())

        region2warmup = region2.get_warmup()
        self.assertIsNotNone(region2warmup)
        self.assertEqual(
            PcCountPair(0x406880, 48111518), region2warmup.get_start()
        )
        self.assertEqual(
            PcCountPair(0x4069D0, 407294228), region2warmup.get_end()
        )

        region3 = regions[3]
        self.assertEqual(4.0, region3.get_multiplier())

        region3start = region3.get_simulation().get_start()
        self.assertEqual(0x4069D0, region3start.get_pc())
        self.assertEqual(187978221, region3start.get_global())
        self.assertIsNone(region3start.get_relative())

        region3end = region3.get_simulation().get_end()
        self.assertEqual(0x406880, region3end.get_pc())
        self.assertEqual(23520614, region3end.get_global())
        self.assertIsNotNone(region3end.get_relative())
        self.assertEqual(144352, region3end.get_relative())

        self.assertIsNone(region3.get_warmup())

    def test_load_pinpoints_matrix_region_1(self):
        looppoint = LooppointCsvLoader(
            pinpoints_file=os.path.join(
                os.path.realpath(os.path.dirname(__file__)),
                "refs",
                "matrix.1_92.global.pinpoints_reduced.csv",
            ),
            region_id=1,
        )

        regions = looppoint.get_regions()
        self.assertEqual(1, len(regions))

        self.assertTrue(1 in regions)
        region1 = regions[1]
        self.assertEqual(4.0, region1.get_multiplier())

        region1start = region1.get_simulation().get_start()
        self.assertEqual(0x4069D0, region1start.get_pc())
        self.assertEqual(211076617, region1start.get_global())
        self.assertIsNone(region1start.get_relative())

        region1end = region1.get_simulation().get_end()
        self.assertEqual(0x4069D0, region1end.get_pc())
        self.assertEqual(219060252, region1end.get_global())
        self.assertIsNotNone(region1end.get_relative())
        self.assertEqual(1060676, region1end.get_relative())

        self.assertIsNone(region1.get_warmup())


class LooppointJsonLoaderTestSuite(unittest.TestCase):
    """Tests the resources.looppoint.LooppointJsonLoader class."""

    def test_load_pinpoints_matrix_region_1(self):
        looppoint = LooppointJsonLoader(
            looppoint_file=os.path.join(
                os.path.realpath(os.path.dirname(__file__)),
                "refs",
                "output.json",
            ),
            region_id="1",
        )

        self.assertEqual(1, len(looppoint.get_regions()))
        self.assertTrue("1" in looppoint.get_regions())
        region = looppoint.get_regions()["1"]

        self.assertEqual(4.0, region.get_multiplier())

        region_start = region.get_simulation().get_start()
        self.assertEqual(4221392, region_start.get_pc())
        self.assertEqual(211076617, region_start.get_global())
        self.assertIsNotNone(region_start.get_relative())
        self.assertEqual(15326617, region_start.get_relative())

        region_end = region.get_simulation().get_end()
        self.assertEqual(4221392, region_end.get_pc())
        self.assertEqual(219060252, region_end.get_global())
        self.assertIsNotNone(region_end.get_relative())
        self.assertEqual(23310252, region_end.get_relative())

        region_warmup = region.get_warmup()
        self.assertIsNotNone(region_warmup)

        self.assertEqual(
            PcCountPair(4221056, 23520614), region_warmup.get_start()
        )
        self.assertEqual(
            PcCountPair(4221392, 211076617), region_warmup.get_end()
        )

    def test_load_pinpoints_matrix_region_2(self):
        looppoint = LooppointJsonLoader(
            looppoint_file=os.path.join(
                os.path.realpath(os.path.dirname(__file__)),
                "refs",
                "output.json",
            ),
            region_id="2",
        )

        self.assertEqual(1, len(looppoint.get_regions()))
        self.assertTrue("2" in looppoint.get_regions())
        region = looppoint.get_regions()["2"]

        self.assertEqual(5.001, region.get_multiplier())

        region_start = region.get_simulation().get_start()
        self.assertEqual(4221392, region_start.get_pc())
        self.assertEqual(407294228, region_start.get_global())
        self.assertIsNone(region_start.get_relative())

        region_end = region.get_simulation().get_end()
        self.assertEqual(4221392, region_end.get_pc())
        self.assertEqual(415282447, region_end.get_global())
        self.assertIsNone(region_end.get_relative())

        region_warmup = region.get_warmup()
        self.assertIsNone(region_warmup)
