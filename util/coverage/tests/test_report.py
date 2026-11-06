#!/usr/bin/env python3
# Copyright (c) 2026 The Regents of The University of California
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

"""Coverage reports preserve inventories and reject incomplete campaigns."""

import copy
import gzip
import hashlib
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "report", Path(__file__).resolve().parents[1] / "report.py"
)
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


class Reports(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "inputs"
        self.output = self.root / "report"
        self.uid = "SuiteUID:gem5/example:test-opt"
        self.plan = {
            "revision": "abc",
            "tests": [{"id": "quick-0", "suites": [self.uid]}],
            "native": [{"group": "sst"}],
            "exclusions": [
                {
                    "suites": ["SuiteUID:excluded"],
                    "reason": "Known instrumentation failure",
                }
            ],
        }
        self.write("coverage-plan/plan.json", self.plan)
        self.write(
            "task/status.json",
            {
                "revision": "abc",
                "task_id": "quick-0",
                "stages": {"test": "success"},
            },
        )
        self.write(
            "native/status.json",
            {
                "revision": "abc",
                "group": "sst",
                "extraction": "complete",
                "stages": {"run": "success"},
            },
        )
        self.write(
            "native/native.json",
            {
                "gcovr/format_version": "0.11",
                "files": [
                    {
                        "file": "src/native.cc",
                        "lines": [
                            {"line_number": 1, "count": 1},
                            {"line_number": 2, "count": 0},
                        ],
                    }
                ],
            },
        )
        self.baseline = {
            "schema_version": 1,
            "format": "gem5-coverage-baseline",
            "revision": "abc",
            "language": "native",
            "build_id": "build",
            "files": [{"path": "src/example.cc", "lines": {"1": 0, "2": 0}}],
        }
        self.baseline_id = hashlib.sha256(
            json.dumps(
                self.baseline,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
                allow_nan=False,
            ).encode()
        ).hexdigest()
        self.write(
            f"task/coverage/baselines/{self.baseline_id}/baseline.json",
            self.baseline,
        )
        self.profile("first", 1, 2)

    def write(self, name, data):
        report.write_json(self.source / name, data)

    def profile(self, identity, line, count):
        native = {
            "schema_version": 2,
            "language": "native",
            "revision": "abc",
            "test_uid": self.uid,
            "invocation_id": identity,
            "outcome": "passed",
            "collection": "complete",
            "build": {"build_id": "build"},
            "baseline_id": self.baseline_id,
            "files": [{"path": "src/example.cc", "lines": {str(line): count}}],
        }
        self.write(f"task/coverage/{identity}/coverage.json", native)
        python = {
            **native,
            "schema_version": 1,
            "language": "python",
            "invocation_id": identity + "-python",
            "parent_invocation_id": identity,
            "files": [
                {"path": "configs/example.py", "lines": {"1": 3, "2": 0}}
            ],
        }
        self.write(f"task/coverage/{identity}/python-coverage.json", python)

    def build(self):
        return report.build(self.source, self.output, "abc")

    def test_complete_inventory_and_bidirectional_queries(self):
        summary = self.build()
        self.assertTrue(summary["complete"], summary)
        self.assertEqual(
            summary["coverage"]["native"], {"lines": 4, "covered": 2}
        )
        with gzip.open(self.output / "coverage-native.info.gz", "rt") as f:
            self.assertIn("DA:2,0", f.read())
        database = self.output / "index.sqlite3"
        hits = report.query(database, location="src/example.cc:1")
        self.assertEqual(hits[0]["uid"], self.uid)
        self.assertEqual(hits[0]["invocations"], ["first"])
        self.assertEqual(
            report.query(database, location="src/example.cc:2"), []
        )
        self.assertEqual(
            report.query(database, uid="group:sst")[0]["path"], "src/native.cc"
        )
        self.assertEqual(
            {r["language"] for r in report.query(database, uid=self.uid)},
            {"native", "python"},
        )

    def test_repeated_invocations_union_lines_and_add_counts(self):
        self.profile("second", 2, 5)
        self.profile("third", 1, 7)
        self.assertTrue(self.build()["complete"])
        rows = report.query(self.output / "index.sqlite3", uid=self.uid)
        native = [r for r in rows if r["language"] == "native"]
        self.assertEqual([r["count"] for r in native], [9, 5])
        self.assertEqual(set(native[0]["invocations"]), {"first", "third"})

    def test_missing_workloads_save_outputs_and_report_incomplete(self):
        (self.source / "native/status.json").unlink()
        (self.source / "task/status.json").unlink()
        summary = self.build()
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["missing"]["groups"], ["sst"])
        self.assertEqual(summary["missing"]["tasks"], ["quick-0"])
        self.assertTrue((self.output / "coverage-native.info.gz").is_file())
        self.assertEqual(summary["exclusions"], self.plan["exclusions"])

    def test_failed_stage_retains_native_data_without_publishing(self):
        status = report.read_json(self.source / "native/status.json")
        status["stages"] = {"run": "failure"}
        self.write("native/status.json", status)
        summary = self.build()
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["coverage"]["native"]["lines"], 4)
        with report.sqlite3.connect(self.output / "index.sqlite3") as database:
            outcome = database.execute(
                "SELECT outcome FROM invocations WHERE id='group:sst'"
            ).fetchone()[0]
        self.assertEqual(outcome, "failed")

    def test_failed_invocation_remains_queryable_but_incomplete(self):
        path = self.source / "task/coverage/first/coverage.json"
        profile = report.read_json(path)
        profile["outcome"] = "failed"
        report.write_json(path, profile)
        summary = self.build()
        self.assertFalse(summary["complete"])
        self.assertTrue(
            report.query(self.output / "index.sqlite3", uid=self.uid)
        )

    def test_bad_profile_does_not_poison_later_baseline_inventory(self):
        self.profile("z-valid", 2, 1)
        path = self.source / "task/coverage/first/coverage.json"
        item = report.read_json(path)
        item["files"][0]["lines"] = {"999": 1}
        report.write_json(path, item)
        summary = self.build()
        self.assertFalse(summary["complete"])
        rows = report.query(self.output / "index.sqlite3", uid=self.uid)
        self.assertTrue(
            any(r["line"] == 2 and r["language"] == "native" for r in rows)
        )
        self.assertEqual(summary["coverage"]["native"]["lines"], 4)

    def test_reject_revision_bad_counts_traversal_and_hash(self):
        path = self.source / "task/coverage/first/coverage.json"
        original = report.read_json(path)
        changes = [
            {"revision": "other"},
            {"files": [{"path": "../escape", "lines": {"1": 1}}]},
            {"files": [{"path": "src/example.cc", "lines": {"1": -1}}]},
            {"files": [{"path": "src/example.cc", "lines": {"1": True}}]},
            {"baseline_id": "0" * 64},
            {"build": {"build_id": "other"}},
        ]
        for change in changes:
            with self.subTest(change=change):
                report.write_json(path, {**original, **change})
                summary = self.build()
                self.assertFalse(summary["complete"])
                self.assertTrue(summary["errors"])
        report.write_json(path, original)
        baseline_path = (
            self.source
            / f"task/coverage/baselines/{self.baseline_id}/baseline.json"
        )
        changed = copy.deepcopy(self.baseline)
        changed["files"][0]["lines"]["3"] = 0
        report.write_json(baseline_path, changed)
        self.assertFalse(self.build()["complete"])

    def test_compressed_baseline(self):
        path = (
            self.source
            / f"task/coverage/baselines/{self.baseline_id}/baseline.json"
        )
        with gzip.open(str(path) + ".gz", "wt") as f:
            f.write(path.read_text())
        path.unlink()
        self.assertTrue(self.build()["complete"])

    def test_missing_python_pair_is_incomplete(self):
        (self.source / "task/coverage/first/python-coverage.json").unlink()
        self.assertIn(
            "Native/Python invocation pairing is incomplete",
            self.build()["errors"],
        )

    def test_unplanned_invocation_is_incomplete(self):
        self.plan["tests"][0]["suites"] = []
        self.write("coverage-plan/plan.json", self.plan)
        self.assertFalse(self.build()["complete"])

    def test_native_export_records_extraction_failure(self):
        status = self.source / "native/status.json"
        with patch.object(
            report.subprocess,
            "run",
            side_effect=subprocess.CalledProcessError(1, "gcovr"),
        ):
            with self.assertRaises(subprocess.CalledProcessError):
                report.native("sst", status, self.output)
        self.assertEqual(
            report.read_json(self.output / "status.json")["extraction"],
            "error",
        )

    def test_duplicate_template_lines_sum_and_excluded_lines_are_ignored(self):
        path = self.source / "native/native.json"
        data = report.read_json(path)
        data["files"][0]["lines"].extend(
            [
                {"line_number": 1, "count": 5, "function_name": "other"},
                {"line_number": 3, "count": 100, "gcovr/excluded": True},
            ]
        )
        report.write_json(path, data)
        self.assertTrue(self.build()["complete"])
        rows = report.query(self.output / "index.sqlite3", uid="group:sst")
        self.assertEqual([(r["line"], r["count"]) for r in rows], [(1, 6)])

    def test_malformed_plan_still_saves_incomplete_summary(self):
        for plan in ({"revision": "abc"}, [], {**self.plan, "tests": []}):
            with self.subTest(plan=plan):
                self.write("coverage-plan/plan.json", plan)
                self.assertFalse(self.build()["complete"])
                self.assertTrue((self.output / "summary.json").is_file())

    def test_stale_native_json_cannot_mask_failed_extraction(self):
        path = self.source / "native/status.json"
        status = report.read_json(path)
        status["extraction"] = "error"
        report.write_json(path, status)
        summary = self.build()
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["missing"]["groups"], ["sst"])

    def compress(self, path, keep_original=False):
        with gzip.open(str(path) + ".gz", "wt") as stream:
            stream.write(path.read_text())
        if not keep_original:
            path.unlink()

    def test_compressed_native_and_invocation_inputs(self):
        for path in (
            list(self.source.rglob("coverage.json"))
            + list(self.source.rglob("python-coverage.json"))
            + [self.source / "native/native.json"]
        ):
            self.compress(path)
        self.assertTrue(self.build()["complete"])
        self.assertTrue(
            report.query(self.output / "index.sqlite3", uid=self.uid)
        )

    def test_ambiguous_plain_and_compressed_reports_are_incomplete(self):
        self.compress(self.source / "native/native.json", keep_original=True)
        self.assertFalse(self.build()["complete"])
        (self.source / "native/native.json.gz").unlink()
        self.compress(
            self.source / "task/coverage/first/coverage.json",
            keep_original=True,
        )
        self.assertFalse(self.build()["complete"])

    def test_native_export_compresses_its_json(self):
        source_json = report.read_json(self.source / "native/native.json")

        def extract(command, **kwargs):
            report.write_json(Path(command[-1]), source_json)

        with patch.object(report.subprocess, "run", side_effect=extract):
            report.native(
                "sst", self.source / "native/status.json", self.output
            )
        self.assertFalse((self.output / "native.json").exists())
        self.assertEqual(
            report.read_json(self.output / "native.json.gz"), source_json
        )
        self.assertEqual(
            report.read_json(self.output / "status.json")["extraction"],
            "complete",
        )

    def test_malformed_profile_is_reported_after_output_creation(self):
        (self.source / "task/coverage/first/coverage.json").write_text("{")
        summary = self.build()
        self.assertFalse(summary["complete"])
        self.assertTrue((self.output / "summary.json").is_file())

    def test_query_rejects_source_traversal(self):
        self.build()
        with self.assertRaises(ValueError):
            report.query(
                self.output / "index.sqlite3", location="../outside:1"
            )


if __name__ == "__main__":
    unittest.main()
