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

"""Check that statistics exports preserve the shape of root selections."""

import csv
import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import (
    Mock,
    patch,
)

from m5.ext.pystats import (
    Scalar,
    SimObjectGroup,
    SimStat,
    StorageType,
)
from m5.objects import SubSystem
from m5.params import SimObjectVector
from m5.stats import gem5stats


class OrderedStatsTest(unittest.TestCase):
    def setUp(self):
        self.first = SubSystem()
        self.second = SubSystem()
        self.first._name = self.second._name = "generator"
        self.values = {id(self.first): 11.0, id(self.second): 22.0}
        self.when = datetime(2026, 9, 14)
        root = Mock()
        root.resolveStat.side_effect = lambda name: Mock(
            value={"finalTick": 100, "simTicks": 40}[name]
        )
        self.patch("Root").getInstance.return_value = root
        self.patch("datetime").now.return_value = self.when
        self.queue = self.patch("_m5_stats.processDumpQueue")
        self.prepare = self.patch("_prepare_stats")
        self.patch("_process_simobject_stats").side_effect = self.process

    def patch(self, name):
        patcher = (
            patch.object(gem5stats, name)
            if "." not in name
            else patch("m5.stats.gem5stats." + name)
        )
        self.addCleanup(patcher.stop)
        return patcher.start()

    def process(self, obj):
        return SimObjectGroup(
            name=obj.get_name(),
            count=Scalar(
                value=self.values[id(obj)],
                unit="Count",
                description="",
                datatype=StorageType.f64,
            ),
        )

    def expected(self, value, name="generator"):
        return {
            "type": "SimObject",
            "time_conversion": None,
            "creation_time": "2026-09-14T00:00:00",
            "simulated_begin_time": 60,
            "simulated_end_time": 100,
            "name": name,
            "count": {
                "value": value,
                "type": "Scalar",
                "description": "",
                "unit": "Count",
                "datatype": "f64",
            },
        }

    def export(self, roots):
        return gem5stats.JsonOutputVistor(None).visit_simstat(
            gem5stats.get_simstat(roots)
        )

    def test_single_object_is_unchanged(self):
        stats = gem5stats.get_simstat(self.first)
        self.assertIsInstance(stats, SimStat)
        self.assertEqual(
            gem5stats.JsonOutputVistor(None).visit_simstat(stats),
            self.expected(11.0),
        )

    def test_list_preserves_colliding_names_and_order(self):
        self.assertEqual(
            self.export([self.first, self.second]),
            [self.expected(11.0), self.expected(22.0)],
        )
        self.assertEqual(
            self.export([self.second, self.first]),
            [self.expected(22.0), self.expected(11.0)],
        )

    def test_singleton_collections_match_the_object(self):
        for roots in (
            [self.first],
            SimObjectVector([self.first]),
            [[self.first]],
        ):
            with self.subTest(roots=roots):
                stats = gem5stats.get_simstat(roots)
                self.assertIsInstance(stats, SimStat)
                self.assertEqual(
                    gem5stats.JsonOutputVistor(None).visit_simstat(stats),
                    self.expected(11.0),
                )

    def test_singleton_prepares_once(self):
        gem5stats.get_simstat([self.first])
        self.queue.assert_called_once_with()
        self.prepare.assert_called_once_with(self.first)

    def test_empty_list(self):
        self.assertEqual(self.export([]), [])

    def test_vector_preserves_shape(self):
        for roots, values in (
            ([], []),
            ([self.first], [11.0]),
            ([self.first, self.second], [11.0, 22.0]),
        ):
            with self.subTest(values=values):
                self.assertEqual(
                    self.export(SimObjectVector(roots)),
                    (
                        self.expected(values[0])
                        if len(values) == 1
                        else [self.expected(v) for v in values]
                    ),
                )

    def test_vector_inside_list(self):
        self.assertEqual(
            self.export([SimObjectVector([self.first, self.second])]),
            [self.expected(11.0), self.expected(22.0)],
        )

    def test_repeated_objects_are_not_deduplicated(self):
        self.assertEqual(
            self.export([self.first, self.first]),
            [self.expected(11.0), self.expected(11.0)],
        )

    def test_names_do_not_become_constructor_arguments(self):
        for name in ("name", "creation_time", "type", "self", "root"):
            with self.subTest(name=name):
                self.first._name = self.second._name = name
                self.assertEqual(
                    self.export([self.first, self.second]),
                    [self.expected(11.0, name), self.expected(22.0, name)],
                )

    def test_prepare_queue_once_for_collection(self):
        gem5stats.get_simstat([self.first, SimObjectVector([self.second])])
        self.queue.assert_called_once_with()
        self.assertEqual(
            [call.args[0] for call in self.prepare.call_args_list],
            [self.first, self.second],
        )

    def test_skip_preparation(self):
        gem5stats.get_simstat([self.first, self.second], prepare_stats=False)
        self.queue.assert_not_called()
        self.prepare.assert_not_called()

    def test_json_file_contains_array(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stats.json"
            gem5stats.JsonOutputVistor(path).dump([self.first, self.second])
            self.assertEqual(
                json.loads(path.read_text()),
                [self.expected(11.0), self.expected(22.0)],
            )

    def test_csv_keeps_input_order_and_names_across_dumps(self):
        self.first._name = "first"
        self.second._name = "second"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stats.csv"
            visitor = gem5stats.CsvOutputVisitor(path)
            visitor.dump([self.first, self.second])
            visitor.dump([self.second, self.first])
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(
                [row["0.name"] for row in rows], ["first", "second"]
            )
            self.assertEqual(
                [row["1.name"] for row in rows], ["second", "first"]
            )
            self.assertEqual(
                [float(row["0.count.value"]) for row in rows], [11.0, 22.0]
            )
            self.assertEqual(
                [float(row["1.count.value"]) for row in rows], [22.0, 11.0]
            )

    def test_json_singleton_file_is_an_object(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stats.json"
            for roots in ([self.first], SimObjectVector([self.first])):
                with self.subTest(roots=roots):
                    gem5stats.JsonOutputVistor(path).dump(roots)
                    self.assertEqual(
                        json.loads(path.read_text()), self.expected(11.0)
                    )

    def test_csv_singleton_and_object_columns_are_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stats.csv"
            visitor = gem5stats.CsvOutputVisitor(path)
            for roots in (
                self.first,
                [self.first],
                SimObjectVector([self.first]),
            ):
                visitor.dump(roots)
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertEqual(
                set(rows[0]),
                {
                    "time_conversion",
                    "creation_time",
                    "simulated_begin_time",
                    "simulated_end_time",
                    "count.value",
                },
            )
            self.assertEqual(float(rows[0]["count.value"]), 11.0)
            self.assertTrue(all(row == rows[0] for row in rows))

    def test_csv_rejects_changed_collection_schema_before_writing(self):
        for initial, changed in (
            ([self.first], [self.first, self.second]),
            ([self.first, self.second], [self.first]),
            ([self.first, self.second], self.first),
            (self.first, [self.first, self.second]),
        ):
            with self.subTest(initial=initial, changed=changed):
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "stats.csv"
                    visitor = gem5stats.CsvOutputVisitor(path)
                    visitor.dump(initial)
                    before = path.read_bytes()
                    with self.assertRaisesRegex(ValueError, "columns"):
                        visitor.dump(changed)
                    self.assertEqual(path.read_bytes(), before)

    def test_csv_rejects_ambiguous_flattening(self):
        visitor = gem5stats.CsvOutputVisitor(None)
        for value in (
            {"a.b": 11, "a": {"b": 22}},
            {"a": {"b": 22}, "a.b": 11},
            {"a": [11], "a0": 22},
            {"a0": 22, "a": [11]},
        ):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "column"):
                    visitor.flatten_dict(value)
