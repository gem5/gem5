# Copyright (c) 2026 Akanksha Chaudhari
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

import contextlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

_ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "salam_ppa_report_under_test", _ROOT / "util/SALAM-tools/ppa_report.py"
)
report = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(report)


class PpaReportTest(unittest.TestCase):
    # Bound derived from double arithmetic and the report's 17-digit output;
    # fixed independently of measured benchmark results.
    REL_TOLERANCE = 1e-12
    ABS_TOLERANCE = 1e-24

    def setUp(self):
        self.model = {
            "type": "SALAMPowerModel",
            "path": "system.acc.ppa",
            "enabled": True,
            "keys": ["fu:integer_adder"],
            "ids": ["synthetic"],
            "sources": [json.dumps({"synthetic": True})],
        }
        self.config = {"system": {"acc": {"ppa": self.model}}}
        self.stats = self.component("system.acc.ppa", 20, 15e-12, 20e-12)

    def component(
        self, path, area, dynamic, leakage, duration=5e-6, complete=True
    ):
        fields = {
            "elapsedSeconds": duration,
            "knownAreaUm2": area,
            "knownDynamicEnergyJ": dynamic,
            "knownLeakageEnergyJ": leakage,
            "knownTotalEnergyJ": dynamic + leakage,
            "areaComplete": 1,
            "dynamicComplete": int(complete),
            "leakageComplete": 1,
            "approximate": 0,
            "timingCompatibilityAvailable": 1,
            "timingCompatible": 1,
            "readWords": 3,
            "writeWords": 2,
            "memoryWordCountsAvailable": 1,
            "missingEventCount": 0 if complete else 3,
        }
        return {path + "." + name: value for name, value in fields.items()}

    def assertNumerical(self, actual, expected):
        self.assertTrue(
            math.isclose(
                actual,
                expected,
                rel_tol=self.REL_TOLERANCE,
                abs_tol=self.ABS_TOLERANCE,
            ),
            (actual, expected),
        )

    def test_independent_energy_and_average_power(self):
        # Two 10um2 units; three 5pJ operations; two 2uW units for 5us.
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["known_area_um2"], 20)
        self.assertNumerical(result["known_dynamic_j"], 15e-12)
        self.assertNumerical(result["known_leakage_j"], 20e-12)
        self.assertNumerical(result["known_total_j"], 35e-12)
        self.assertNumerical(result["known_average_w"], 7e-6)

    def test_unknown_word_geometry_keeps_only_known_subtotals(self):
        self.stats["system.acc.ppa.memoryWordCountsAvailable"] = 0
        result = report.aggregate(self.stats, self.config)
        component = result["components"][0]
        self.assertEqual(component["known_read_words"], 3)
        self.assertEqual(component["known_write_words"], 2)
        self.assertFalse(component["memory_word_counts_available"])

    def test_unknown_timing_does_not_invent_a_physical_validation(self):
        self.stats["system.acc.ppa.timingCompatibilityAvailable"] = 0
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["fu_timing_status"], "unavailable")
        self.assertEqual(
            result["components"][0]["fu_timing_status"], "unavailable"
        )

    def test_known_timing_violation_survives_other_unknown_constraints(self):
        self.stats["system.acc.ppa.timingCompatibilityAvailable"] = 0
        self.stats["system.acc.ppa.timingCompatible"] = 0
        self.stats["system.acc.ppa.areaComplete"] = 0
        self.stats["system.acc.ppa.dynamicComplete"] = 0
        self.stats["system.acc.ppa.leakageComplete"] = 0
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(result["status"], "incomplete")
        self.assertEqual(result["fu_timing_status"], "incompatible")
        self.assertEqual(result["known_area_um2"], 20)

    def test_device_identity_comes_from_the_configuration(self):
        self.config["system"]["acc"].update(
            type="HWInterface", path="system.acc"
        )
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(result["components"][0]["owner_type"], "HWInterface")
        self.assertEqual(result["components"][0]["owner_path"], "system.acc")

    def test_shared_memory_has_one_physical_contribution(self):
        second = dict(self.model, path="system.acc2.ppa")
        memory = dict(self.model, path="system.spm.ppa")
        self.config["system"]["acc"]["memory"] = "system.spm"
        self.config["system"]["acc2"] = {"ppa": second, "memory": "system.spm"}
        self.config["system"]["spm"] = {"ppa": memory}
        self.stats.update(self.component(second["path"], 30, 7e-12, 11e-12))
        self.stats.update(self.component(memory["path"], 100, 13e-12, 17e-12))
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(len(result["components"]), 3)
        self.assertEqual(result["known_area_um2"], 150)
        self.assertNumerical(result["known_dynamic_j"], 35e-12)
        self.assertNumerical(result["known_leakage_j"], 48e-12)
        self.assertNumerical(result["known_total_j"], 83e-12)
        self.assertNumerical(result["known_average_w"], 16.6e-6)

    def test_duplicate_physical_definition_rejected(self):
        self.config["duplicate"] = dict(self.model)
        with self.assertRaisesRegex(ValueError, "duplicate physical"):
            report.aggregate(self.stats, self.config)

    def test_mismatched_observation_intervals_rejected(self):
        self.config["other"] = dict(self.model, path="system.other.ppa")
        self.stats.update(
            self.component("system.other.ppa", 1, 2, 3, duration=6e-6)
        )
        with self.assertRaisesRegex(ValueError, "different observation"):
            report.aggregate(self.stats, self.config)

    def test_zero_duration_retains_energy_but_no_average(self):
        self.stats["system.acc.ppa.elapsedSeconds"] = 0
        result = report.aggregate(self.stats, self.config)
        self.assertIsNone(result["known_average_w"])
        self.assertIsNone(result["components"][0]["known_average_w"])
        self.assertNumerical(result["known_total_j"], 35e-12)
        self.assertEqual(result["known_area_um2"], 20)
        json.dumps(result, allow_nan=False)

    def test_partial_coverage_is_not_complete_total(self):
        self.stats["system.acc.ppa.dynamicComplete"] = 0
        self.stats["system.acc.ppa.missingEventCount"] = 3
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(result["status"], "incomplete")
        self.assertFalse(result["coverage"]["dynamic"])
        self.assertTrue(result["coverage"]["area"])
        self.assertEqual(result["components"][0]["missing_event_count"], 3)

    def test_disabled_component_makes_aggregate_incomplete(self):
        self.config["other"] = dict(
            self.model, path="system.other.ppa", enabled=False
        )
        result = report.aggregate(self.stats, self.config)
        self.assertEqual(result["status"], "incomplete")
        self.assertEqual(result["disabled"], ["system.other.ppa"])
        self.assertEqual(len(result["components"]), 1)

    def test_disabled_and_absent_are_distinct(self):
        self.model["enabled"] = False
        self.assertEqual(
            report.aggregate({}, self.config)["status"], "disabled"
        )
        self.assertEqual(report.aggregate({}, {})["status"], "unavailable")

    def test_proxy_coverage_remains_visible(self):
        self.stats["system.acc.ppa.approximate"] = 1
        result = report.aggregate(self.stats, self.config)
        self.assertTrue(result["approximate"])
        self.assertEqual(
            result["components"][0]["characterization_ids"], ["synthetic"]
        )
        self.assertEqual(
            result["components"][0]["characterization_sources"],
            [{"synthetic": True}],
        )

    def test_missing_nonfinite_and_negative_values_rejected(self):
        key = "system.acc.ppa.knownDynamicEnergyJ"
        original = self.stats[key]
        for invalid in (float("inf"), float("nan"), -1):
            with self.subTest(invalid=invalid):
                self.stats[key] = invalid
                with self.assertRaises(ValueError):
                    report.aggregate(self.stats, self.config)
        del self.stats[key]
        with self.assertRaisesRegex(ValueError, "missing or invalid"):
            report.aggregate(self.stats, self.config)
        self.stats[key] = original

    def test_coverage_flags_are_binary(self):
        for suffix in (
            "areaComplete",
            "dynamicComplete",
            "leakageComplete",
            "approximate",
            "timingCompatibilityAvailable",
            "timingCompatible",
            "memoryWordCountsAvailable",
        ):
            key = "system.acc.ppa." + suffix
            original = self.stats[key]
            for invalid in (-1, 2, 0.5):
                with self.subTest(flag=suffix, invalid=invalid):
                    self.stats[key] = invalid
                    with self.assertRaisesRegex(ValueError, "flag"):
                        report.aggregate(self.stats, self.config)
            self.stats[key] = original

    def test_event_counts_are_nonnegative_integers(self):
        key = "system.acc.ppa.missingEventCount"
        for invalid in (-1, 0.5):
            with self.subTest(invalid=invalid):
                self.stats[key] = invalid
                with self.assertRaisesRegex(ValueError, "count"):
                    report.aggregate(self.stats, self.config)
        for valid in (0, 3, 3.0):
            self.stats[key] = valid
            result = report.aggregate(self.stats, self.config)
            self.assertEqual(
                result["components"][0]["missing_event_count"], int(valid)
            )

    def test_total_energy_consistency_is_checked(self):
        self.stats["system.acc.ppa.knownTotalEnergyJ"] = 70e-12
        with self.assertRaisesRegex(ValueError, "inconsistent component"):
            report.aggregate(self.stats, self.config)

    def test_prefix_respects_component_boundaries(self):
        self.config["other"] = dict(self.model, path="system.accidental.ppa")
        self.stats.update(self.component("system.accidental.ppa", 99, 2, 3))
        result = report.aggregate(self.stats, self.config, "system.acc")
        self.assertEqual(result["known_area_um2"], 20)
        self.assertEqual(len(result["components"]), 1)

    def write_window(self, path):
        path.write_text(
            "---------- Begin Simulation Statistics ----------\n"
            + "\n".join(f"{k} {v}" for k, v in self.stats.items())
            + "\n---------- End Simulation Statistics ----------\n"
        )

    def test_repeated_dumps_do_not_accumulate(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stats.txt"
            self.write_window(path)
            path.write_text(path.read_text() * 2)
            windows = report.windows(path)
            self.assertEqual(len(windows), 2)
            first = report.aggregate(windows[0], self.config)
            second = report.aggregate(windows[1], self.config)
            self.assertEqual(first, second)
            self.assertNumerical(second["known_total_j"], 35e-12)

    def test_zero_window_and_truncated_output_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stats.txt"
            for text in (
                "",
                "Begin Simulation Statistics\n",
                "End Simulation Statistics\n",
            ):
                with self.subTest(text=text):
                    path.write_text(text)
                    with self.assertRaises(ValueError):
                        report.windows(path)

    def test_strict_cli_rejects_partial_and_disabled(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stats, config, output = (
                root / name
                for name in ("stats.txt", "config.json", "report.json")
            )
            for mode in ("partial", "disabled", "complete"):
                with self.subTest(mode=mode):
                    self.model["enabled"] = mode != "disabled"
                    self.stats["system.acc.ppa.dynamicComplete"] = int(
                        mode != "partial"
                    )
                    self.write_window(stats)
                    config.write_text(json.dumps(self.config))
                    args = [
                        "ppa_report.py",
                        "--stats",
                        str(stats),
                        "--config",
                        str(config),
                        "--output",
                        str(output),
                        "--strict",
                    ]
                    with (
                        patch.object(sys, "argv", args),
                        contextlib.redirect_stderr(io.StringIO()),
                    ):
                        if mode == "complete":
                            report.main()
                        else:
                            with self.assertRaises(SystemExit) as error:
                                report.main()
                            self.assertNotEqual(error.exception.code, 0)
                    expected = {
                        "partial": "incomplete",
                        "disabled": "disabled",
                        "complete": "complete",
                    }[mode]
                    self.assertEqual(
                        json.loads(output.read_text())["status"], expected
                    )


if __name__ == "__main__":
    unittest.main()
