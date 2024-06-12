# Copyright (c) 2024 The Regents of The University of California
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
from datetime import datetime

from m5.ext.pystats import (
    Distribution,
    Scalar,
    SimObjectGroup,
    SimObjectVectorGroup,
    SimStat,
    SparseHist,
    Vector,
    Vector2d,
)


def _get_mock_simstat() -> SimStat:
    """Used to create a mock SimStat for testing.
    This SimStat is contains all simstat Statistic types and attempts to use
    most of the different types of values that can be stored in a Statistic.
    """
    simobject_vector_group = SimObjectVectorGroup(
        value=[
            SimObjectGroup(
                **{
                    "vector2d": Vector2d(
                        value={
                            0: Vector(
                                value={
                                    "a": Scalar(value=1, description="one"),
                                    "b": Scalar(value=2.0, description="two"),
                                    "c": Scalar(value=-3, description="three"),
                                }
                            ),
                            1: Vector(
                                value={
                                    1: Scalar(value=4),
                                    0.2: Scalar(value=5.0),
                                    0.3: Scalar(value=6),
                                },
                                description="vector 1",
                            ),
                        },
                        description="vector 2d",
                    ),
                }
            ),
            SimObjectGroup(
                **{
                    "distribution": Distribution(
                        value={
                            0: Scalar(1),
                            1: Scalar(2),
                            2: Scalar(3),
                            3: Scalar(4),
                            4: Scalar(5),
                        },
                        min=0,
                        max=4,
                        num_bins=5,
                        bin_size=1,
                    ),
                    "sparse_hist": SparseHist(
                        value={
                            0.5: Scalar(4),
                            0.51: Scalar(1),
                            0.511: Scalar(4),
                            5: Scalar(2),
                        },
                        description="sparse hist",
                    ),
                },
            ),
        ],
    )

    return SimStat(
        creation_time=datetime.fromisoformat("2021-01-01T00:00:00"),
        time_conversion=None,
        simulated_begin_time=123,
        simulated_end_time=558644,
        simobject_vector=simobject_vector_group,
    )


class NavigatingPyStatsTestCase(unittest.TestCase):
    """A test case for navigating the PyStats data structure, primarily
    on how to access children of a SimStat object, and the "find" methods to
    search for a specific statistic.
    """

    def setUp(self) -> None:
        """Overrides the setUp method to create a mock SimStat for testing.
        Runs before each test method.
        """
        self.failFast = True
        self.simstat = _get_mock_simstat()
        super().setUp()

    def test_simstat_index(self):
        self.assertTrue("simobject_vector" in self.simstat)
        self.assertIsInstance(
            self.simstat["simobject_vector"], SimObjectVectorGroup
        )

    def test_simstat_attribute(self):
        self.assertTrue(hasattr(self.simstat, "simobject_vector"))
        self.assertIsInstance(
            self.simstat.simobject_vector, SimObjectVectorGroup
        )

    def test_simobject_vector_attribute(self):
        # To maintan compatibility with the old way of accessing the vector,
        # the simobject vectors values can be accessed by attributes of that
        # simoobject vector name and the index appended to it.
        # E.g., `simstat.simobject_vector0 is the same
        # is simstat.simobject_vector[0]`. In cases where there is already
        # an attribute with the same name as the vector+index, the attribute
        # will be returned.
        self.assertEqual(
            self.simstat.simobject_vector0, self.simstat.simobject_vector[0]
        )

    def test_simobject_vector_index(self):
        self.assertTrue(self.simstat.simobject_vector[0], SimObjectGroup)

    def test_simobject_group_index(self):
        self.assertTrue("vector2d" in self.simstat.simobject_vector[0])
        self.assertIsInstance(
            self.simstat.simobject_vector[0]["vector2d"], Vector2d
        )

    def test_simobject_group_attribute(self):
        self.assertTrue(hasattr(self.simstat.simobject_vector[0], "vector2d"))
        self.assertIsInstance(
            self.simstat.simobject_vector[0].vector2d, Vector2d
        )

    def test_vector2d_index(self):
        self.assertEqual(2, len(self.simstat.simobject_vector[0]["vector2d"]))
        self.assertTrue(0 in self.simstat.simobject_vector[0].vector2d)
        self.assertIsInstance(
            self.simstat.simobject_vector[0].vector2d[0], Vector
        )

    def test_vector_index_int(self):
        self.assertEqual(3, len(self.simstat.simobject_vector[0].vector2d[1]))
        self.assertTrue(1 in self.simstat.simobject_vector[0].vector2d[1])
        self.assertIsInstance(
            self.simstat.simobject_vector[0].vector2d[1][1], Scalar
        )

    def test_vector_index_str(self):
        self.assertEqual(3, len(self.simstat.simobject_vector[0].vector2d[0]))
        self.assertTrue("a" in self.simstat.simobject_vector[0].vector2d[0])
        self.assertIsInstance(
            self.simstat.simobject_vector[0].vector2d[0]["a"], Scalar
        )

    def test_vector_index_float(self):
        self.assertEqual(3, len(self.simstat.simobject_vector[0].vector2d[1]))
        self.assertTrue(0.2 in self.simstat.simobject_vector[0].vector2d[1])
        self.assertIsInstance(
            self.simstat.simobject_vector[0].vector2d[1][0.2], Scalar
        )

    def test_distriibution_index(self):
        self.assertTrue(0 in self.simstat.simobject_vector[1]["distribution"])
        self.assertIsInstance(
            self.simstat.simobject_vector[1]["distribution"][0], Scalar
        )

    def test_sparse_hist_index(self):
        self.assertTrue(0.5 in self.simstat.simobject_vector[1]["sparse_hist"])
        self.assertIsInstance(
            self.simstat.simobject_vector[1]["sparse_hist"][0.5], Scalar
        )

    def test_pystat_find(self):
        self.assertEqual(
            self.simstat.find("sparse_hist"),
            [self.simstat.simobject_vector[1]["sparse_hist"]],
        )
