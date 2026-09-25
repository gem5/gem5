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
#

"""Exercise accounting boundaries and reproducible report recovery."""

import gzip
import importlib.util
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import yaml

MODULE = Path(__file__).resolve().parents[1] / "report.py"
sys.path.insert(0, str(MODULE.parent))
spec = importlib.util.spec_from_file_location("coverage_report", MODULE)
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)

REVISION = "a" * 40
UID = "SuiteUID:tests/gem5/example/test.py:example-opt"


class CoverageReportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "input"
        self.source.mkdir()
        self.output = self.root / "output"
        self.plan()

    def test_declared_python_scope_is_visible_without_hiding_results(self):
        self.profile()
        self.profile(
            identity="python",
            language="python",
            parent_invocation_id="one",
            exclusions=["Child interpreters are outside this measurement."],
        )
        summary = report.summarize(self.source, self.output, REVISION)
        self.assertTrue(summary["complete"])
        self.assertEqual(
            summary["profile_exclusions"],
            [
                {
                    "reason": "Child interpreters are outside this measurement.",
                    "tests": [UID],
                }
            ],
        )
        self.assertIn(
            "Child interpreters are outside this measurement.",
            (self.output / "summary.md").read_text(),
        )

    def plan(self, uid=UID, length="quick", excluded=None):
        report.write_json(
            self.source / length / "expected.json",
            {
                "schema_version": 1,
                "revision": REVISION,
                "length": length,
                "expected": [uid],
                "excluded": excluded or {},
            },
        )

    def profile(self, identity="one", **values):
        data = {
            "schema_version": 1,
            "revision": REVISION,
            "test_uid": UID,
            "invocation_id": identity,
            "build": {"target": "build/ALL/gem5.opt"},
            "outcome": "passed",
            "collection": "complete",
            "files": [{"path": "src/example.cc", "lines": {"1": 1, "2": 0}}],
        }
        data.update(values)
        report.write_json(self.source / identity / "coverage.json", data)
        xml_uid = data["test_uid"].replace("SuiteUID:", "TestUID:") + ":run"
        status = {
            "passed": "Passed",
            "failed": "Failed",
            "interrupted": "Unknown",
        }[data["outcome"]]
        (self.source / identity / "results.xml").write_text(
            f'<testsuites><testcase classname="{xml_uid}" '
            f'status="{status}"/></testsuites>'
        )

    def run_report(self, campaign=False):
        return report.summarize(self.source, self.output, REVISION, campaign)

    def test_raw_archive_preserves_shared_notes(self):
        source = self.root / "test-results"
        for name in ("one", "two"):
            (source / "coverage" / name).mkdir(parents=True)
        notes = source / "coverage" / "one" / "shared.gcno"
        notes.write_bytes(b"shared notes")
        os.link(notes, source / "coverage" / "two" / "shared.gcno")
        report.write_json(source / "coverage" / "one" / "coverage.json", {})
        report.package(source, self.output, REVISION)
        with tarfile.open(
            self.output.with_name(self.output.name + "-raw")
            / "raw-profiles.tar.gz"
        ) as archive:
            notes = [
                entry for entry in archive if entry.name.endswith(".gcno")
            ]
            self.assertEqual(sum(entry.islnk() for entry in notes), 1)
        self.assertTrue((self.output / "records/one/coverage.json").exists())
        self.assertFalse(list((self.output / "records").rglob("*.gcno")))

    def test_merges_invocations_and_retains_group_flags(self):
        self.profile()
        self.profile(
            "two",
            files=[{"path": "src/example.cc", "lines": {"1": 0, "2": 3}}],
        )
        result = self.run_report()
        self.assertTrue(result["complete"])
        self.assertEqual(result["counts"]["expected"], 1)
        self.assertEqual(result["counts"]["invocations"], 2)
        uploads = json.loads((self.output / "uploads.json").read_text())
        self.assertEqual(
            uploads[0]["flags"],
            "overall-testdir-gem5-example,overall-length-quick,gem5-example-quick",
        )
        lcov = gzip.decompress(
            (self.output / uploads[0]["file"]).read_bytes()
        ).decode()
        self.assertIn("DA:1,1\nDA:2,3", lcov)
        self.assertIn("LH:2", lcov)

    def test_compressed_report_is_deterministic_and_upload_is_plain(self):
        self.profile()
        self.run_report()
        path = next(self.output.glob("*.info.gz"))
        original = path.read_bytes()
        self.run_report()
        self.assertEqual(path.read_bytes(), original)
        workflow = yaml.safe_load(
            (MODULE.parents[2] / ".github/workflows/codecov.yaml").read_text()
        )
        step = next(
            step
            for step in workflow["jobs"]["upload"]["steps"]
            if step.get("id") == "prepare"
        )
        retained = self.root / "coverage-report"
        retained.mkdir()
        prepared = self.root / "prepared"
        prepared.mkdir()
        output = self.root / "github-output"
        for filename, content in (
            ("example.info.gz", original),
            ("legacy.xml", b"<coverage/>"),
        ):
            with self.subTest(filename=filename):
                (retained / filename).write_bytes(content)
                output.write_text("")
                env = dict(
                    os.environ,
                    REPORT_FILE=filename,
                    RUNNER_TEMP=str(prepared),
                    GITHUB_OUTPUT=str(output),
                )
                subprocess.run(
                    ["bash", "-e", "-c", step["run"]],
                    cwd=self.root,
                    env=env,
                    check=True,
                )
                selected = Path(
                    output.read_text().strip().removeprefix("file=")
                )
                expected = (
                    gzip.decompress(content)
                    if filename.endswith(".gz")
                    else content
                )
                self.assertEqual(selected.read_bytes(), expected)
        env["REPORT_FILE"] = "../github-output"
        rejected = subprocess.run(
            ["bash", "-e", "-c", step["run"]],
            cwd=self.root,
            env=env,
            capture_output=True,
        )
        self.assertNotEqual(rejected.returncode, 0)

    def test_validation_groups_baselines_before_loading(self):
        for identity, graph in (("a", "one"), ("b", "two"), ("c", "one")):
            self.profile(identity, build={"build_id": graph})
        loaded = []
        original = report.load_sparse_record

        def observe(path, **kwargs):
            record = original(path, **kwargs)
            loaded.append(record["build"]["build_id"])
            return record

        with mock.patch.object(
            report, "load_sparse_record", side_effect=observe
        ):
            self.assertTrue(self.run_report()["complete"])
        self.assertEqual(loaded[:3], ["one", "one", "two"])
        bad = self.source / "broken.json"
        bad.write_text("{")
        self.assertEqual(report.profile_order(bad), ("", "", "", str(bad)))

    def test_multiple_builds_are_written_one_graph_at_a_time(self):
        for identity, graph, hits in (
            ("a", "one", 1),
            ("b", "two", 0),
            ("c", "one", 2),
        ):
            self.profile(
                identity,
                build={"build_id": graph},
                files=[
                    {
                        "path": "src/example.cc",
                        "lines": {"1": hits},
                        "branches": [{"id": graph, "line": 1, "count": hits}],
                    }
                ],
            )
        with mock.patch.object(
            report, "write_graph_lcov", wraps=report.write_graph_lcov
        ) as write:
            self.assertTrue(self.run_report()["complete"])
        self.assertEqual(write.call_count, 2)
        for call in write.call_args_list:
            graphs = {
                json.loads(path.read_text())["build"]["build_id"]
                for path in call.args[1]
            }
            self.assertEqual(len(graphs), 1)
        data = gzip.decompress(
            next(self.output.glob("*.info.gz")).read_bytes()
        ).decode()
        self.assertEqual(data.count("SF:src/example.cc"), 2)
        self.assertIn("DA:1,3", data)
        self.assertIn("DA:1,0", data)
        self.assertIn("BRDA:1,0,gem5-one,3", data)
        self.assertIn("BRDA:1,0,gem5-two,0", data)
        self.assertEqual(
            len(json.loads((self.output / "uploads.json").read_text())), 1
        )

    def test_two_thousand_shared_baselines(self):
        tests = [
            f"SuiteUID:tests/gem5/example/test.py:suite-{i}"
            for i in range(2000)
        ]
        report.write_json(
            self.source / "quick" / "expected.json",
            {
                "schema_version": 1,
                "revision": REVISION,
                "length": "quick",
                "expected": tests,
                "excluded": {},
            },
        )
        lines = {str(line): 0 for line in range(1, 1001)}
        lines["1"] = 1
        for index, uid in enumerate(tests):
            self.profile(
                str(index),
                test_uid=uid,
                files=[{"path": "src/shared.cc", "lines": lines}],
            )
        result = self.run_report()
        self.assertTrue(result["complete"])
        self.assertEqual(result["counts"]["completed"], 2000)
        lcov = gzip.decompress(
            next(self.output.glob("*.info.gz")).read_bytes()
        ).decode()
        self.assertIn("DA:1,2000", lcov)
        self.assertIn("LF:1000", lcov)

    def test_passed_invocation_without_suite_results_is_unfinished(self):
        self.profile()
        (self.source / "one" / "results.xml").unlink()
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertEqual(result["counts"]["unfinished"], 1)
        self.assertEqual(result["counts"]["missing_profiles"], 0)
        self.assertTrue(list(self.output.glob("*.info.gz")))

    def test_missing_profile_is_visible(self):
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertEqual(result["counts"]["missing_profiles"], 1)
        self.assertEqual(result["counts"]["unfinished"], 1)

    def test_failed_execution_can_have_complete_collection(self):
        self.profile(outcome="failed")
        result = self.run_report()
        self.assertTrue(result["complete"])
        self.assertEqual(result["counts"]["failed"], 1)

    def test_partial_profile_does_not_pollute_lcov(self):
        self.profile(collection="error", outcome="failed")
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertEqual(result["counts"]["failed"], 1)
        self.assertEqual(
            json.loads((self.output / "uploads.json").read_text()), []
        )

    def test_revision_mismatch_is_rejected(self):
        self.profile(revision="b" * 40)
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertTrue(
            any("revision mismatch" in error for error in result["errors"])
        )

    def test_duplicate_profile_is_rejected(self):
        self.profile()
        self.profile("two", invocation_id="one")
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertTrue(
            any("Duplicate" in error for error in result["errors"])
        )

    def test_unsafe_path_is_rejected(self):
        for value in (
            "../outside.cc",
            "/etc/passwd",
            "src/file\nDA:1,99",
            "src\\file.cc",
        ):
            with self.subTest(path=value):
                self.profile(files=[{"path": value, "lines": {"1": 1}}])
                self.assertFalse(self.run_report()["complete"])

    def test_no_executable_lines_is_not_complete(self):
        self.profile(files=[])
        self.assertFalse(self.run_report()["complete"])

    def test_skipped_suite_from_xml_is_not_missing(self):
        (self.source / "results.xml").write_text(
            "<testsuites><testsuite><testcase "
            'classname="TestUID:tests/gem5/example/test.py:example-opt:run" '
            'status="Skipped"/></testsuite></testsuites>'
        )
        result = self.run_report()
        self.assertEqual(result["counts"]["skipped"], 1)
        self.assertEqual(result["counts"]["missing_profiles"], 0)

    def test_passing_xml_without_native_invocation_stays_missing(self):
        (self.source / "results.xml").write_text(
            "<testsuites><testcase "
            'classname="TestUID:tests/gem5/example/test.py:example-opt:run" '
            'status="Passed"/></testsuites>'
        )
        result = self.run_report()
        self.assertEqual(result["counts"]["completed"], 1)
        self.assertEqual(result["counts"]["missing_profiles"], 1)
        self.assertFalse(result["complete"])

    def test_xml_verifier_failure_overrides_passed_invocation(self):
        self.profile()
        (self.source / "results.xml").write_text(
            "<testsuites><testcase "
            'classname="TestUID:tests/gem5/example/test.py:example-opt:verify" '
            'status="Failed"/></testsuites>'
        )
        self.assertEqual(self.run_report()["counts"]["failed"], 1)

    def test_explicit_x86_exclusion(self):
        uid = "SuiteUID:tests/gem5/x86_boot_tests/test.py:boot"
        self.plan(uid, "very-long", {uid: "Known instrumentation crash"})
        self.profile()
        result = self.run_report()
        self.assertEqual(result["counts"]["excluded"], 1)
        self.assertEqual(result["counts"]["missing_profiles"], 0)

    def test_missing_campaign_manifests_and_native_groups(self):
        self.profile()
        result = self.run_report(campaign=True)
        self.assertFalse(result["complete"])
        self.assertTrue(
            any("Missing discovery" in item for item in result["errors"])
        )
        self.assertTrue(
            any("Missing aggregate" in item for item in result["errors"])
        )

    def test_empty_selection_fails_before_execution(self):
        listing = self.root / "listing"
        listing.write_text("No suites found\n")
        with self.assertRaises(ValueError):
            report.manifest(
                listing, REVISION, "quick", self.root / "expected.json"
            )

    def test_malformed_xml_is_reported(self):
        self.profile()
        (self.source / "results.xml").write_text("<invalid")
        self.assertFalse(self.run_report()["complete"])
        self.assertTrue((self.output / "summary.json").exists())

    def test_malformed_record_preserves_accounting_report(self):
        report.write_json(self.source / "bad" / "coverage.json", [])
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertTrue((self.output / "summary.json").exists())

    def test_manifest_revision_mismatch(self):
        path = self.source / "quick" / "expected.json"
        data = json.loads(path.read_text())
        data["revision"] = "b" * 40
        report.write_json(path, data)
        self.assertFalse(self.run_report()["complete"])

    def native_fixture(self):
        config = (
            Path(__file__).resolve().parents[3]
            / ".github/coverage-native.json"
        )
        definition = report.native_groups(config)["unittests-fast"]
        report.write_json(
            self.source / "expected-native.json",
            {
                "schema_version": 1,
                "revision": REVISION,
                "groups": [definition],
            },
        )
        return definition

    def test_recovered_aggregate_reconstructs_flags(self):
        self.profile()
        self.native_fixture()
        directory = self.source / "native"
        report.write_json(
            directory / "aggregate.json",
            {
                "revision": REVISION,
                "group": "unittests-fast",
                "flags": "untrusted-label",
                "schema_version": 2,
                "extraction": "complete",
                "outcomes": {"build_and_test": "success"},
                "counters": {"files": 1, "bytes": 100},
            },
        )
        (directory / "coverage.xml").write_text(
            '<coverage><packages><class><lines><line number="1" hits="1"/>'
            "</lines></class></packages></coverage>"
        )
        self.assertTrue(self.run_report()["complete"])
        uploads = json.loads((self.output / "uploads.json").read_text())
        native = next(
            item for item in uploads if item["file"].endswith(".xml")
        )
        self.assertEqual(
            native["flags"],
            "unittests-fast,overall-length-quick,overall-unittests",
        )

    def test_failed_native_build_keeps_partial_report_incomplete(self):
        self.test_recovered_aggregate_reconstructs_flags()
        path = self.source / "native/aggregate.json"
        data = json.loads(path.read_text())
        data["outcomes"]["build_and_test"] = "failure"
        report.write_json(path, data)
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertTrue(result["aggregates"][0]["report_present"])
        self.assertTrue(
            (self.output / "aggregate-unittests-fast.xml").exists()
        )

    def test_native_notes_without_runtime_counters_are_incomplete(self):
        self.test_recovered_aggregate_reconstructs_flags()
        path = self.source / "native/aggregate.json"
        data = json.loads(path.read_text())
        data["counters"] = {"files": 0, "bytes": 0}
        report.write_json(path, data)
        self.assertFalse(self.run_report()["complete"])

    def test_native_plan_requires_all_groups_and_stages(self):
        self.test_recovered_aggregate_reconstructs_flags()
        path = self.source / "expected-native.json"
        data = json.loads(path.read_text())
        data["groups"][0]["stages"].append("new_stage")
        report.write_json(path, data)
        self.assertFalse(self.run_report()["complete"])
        data["groups"].append(dict(data["groups"][0], group="new-group"))
        report.write_json(path, data)
        self.assertIn(
            "Missing aggregate reports: new-group", self.run_report()["errors"]
        )

    def test_sparse_baseline_and_branch_counts_export(self):
        from schema import canonical_hash

        baseline = {
            "schema_version": 1,
            "format": "gem5-coverage-baseline",
            "revision": REVISION,
            "language": "native",
            "build_id": "build-one",
            "files": [
                {
                    "path": "src/example.cc",
                    "lines": {"1": 0, "2": 0},
                    "branches": [{"id": "branch-one", "line": 1, "count": 0}],
                }
            ],
        }
        identity = canonical_hash(baseline)
        report.write_json(
            self.source / "baselines" / identity / "baseline.json", baseline
        )
        self.profile(
            schema_version=2,
            baseline_id=identity,
            build={"build_id": "build-one"},
            files=[
                {
                    "path": "src/example.cc",
                    "lines": {"1": 3},
                    "branches": [{"id": "branch-one", "count": 2}],
                }
            ],
        )
        self.assertTrue(self.run_report()["complete"])
        text = gzip.decompress(
            next(self.output.glob("*.info.gz")).read_bytes()
        ).decode()
        self.assertIn("DA:2,0", text)
        self.assertIn("BRDA:1,0,gem5-branch-one,2", text)

    def test_required_python_profile_accounted_per_parent(self):
        self.profile()
        plan = self.source / "quick/expected.json"
        data = json.loads(plan.read_text())
        data["required_languages"] = ["native", "python"]
        report.write_json(plan, data)
        result = self.run_report()
        self.assertFalse(result["complete"])
        self.assertEqual(result["counts"]["missing_python_profiles"], 1)
        native = json.loads((self.source / "one/coverage.json").read_text())
        python = dict(
            native,
            language="python",
            invocation_id="one-python",
            parent_invocation_id="one",
        )
        report.write_json(self.source / "one/python-coverage.json", python)
        result = self.run_report()
        self.assertTrue(result["complete"])
        self.assertEqual(result["counts"]["invocations"], 1)
        self.assertEqual(result["counts"]["python_invocations"], 1)
        self.assertTrue(list(self.output.glob("python-*.info.gz")))
        python["parent_invocation_id"] = "unknown"
        report.write_json(self.source / "one/python-coverage.json", python)
        self.assertFalse(self.run_report()["complete"])

    def test_package_retains_python_and_shared_baselines(self):
        source = self.root / "package-input"
        coverage = source / "coverage"
        report.write_json(coverage / "one/python-coverage.json", {})
        report.write_json(coverage / "baselines/hash/baseline.json", {})
        report.package(source, self.output, REVISION)
        self.assertTrue(
            (self.output / "records/one/python-coverage.json").is_file()
        )
        self.assertTrue(
            (self.output / "records/baselines/hash/baseline.json.gz").is_file()
        )

    def test_sparse_groups_seed_shared_baseline_without_expansion(self):
        from schema import canonical_hash

        baseline = {
            "schema_version": 1,
            "format": "gem5-coverage-baseline",
            "revision": REVISION,
            "language": "native",
            "build_id": "shared",
            "files": [
                {
                    "path": "src/shared.cc",
                    "lines": {str(i): 0 for i in range(1, 1201)},
                }
            ],
        }
        identity = canonical_hash(baseline)
        report.write_json(
            self.source / "baselines" / identity / "baseline.json", baseline
        )
        for attempt in range(100):
            self.profile(
                str(attempt),
                schema_version=2,
                baseline_id=identity,
                build={"build_id": "shared"},
                files=[{"path": "src/shared.cc", "lines": {"1": 1}}],
            )
        with mock.patch(
            "schema.expand_baseline",
            side_effect=AssertionError("No per-invocation expansion"),
        ):
            result = self.run_report()
        self.assertTrue(result["complete"])
        text = gzip.decompress(
            next(self.output.glob("testlib-*.info.gz")).read_bytes()
        ).decode()
        self.assertIn("DA:1,100", text)
        self.assertIn("DA:1200,0", text)
        self.assertIn("LF:1200", text)

    def test_branch_identity_is_stable_across_upload_groups(self):
        other = "SuiteUID:tests/gem5/other/test.py:other"
        self.plan(other, "long")
        for identity, uid, branch in (
            ("one", UID, "compiled-one"),
            ("two", other, "compiled-two"),
        ):
            self.profile(
                identity,
                test_uid=uid,
                files=[
                    {
                        "path": "src/shared.cc",
                        "lines": {"1": 1},
                        "branches": [{"id": branch, "line": 1, "count": 1}],
                    }
                ],
            )
        self.assertTrue(self.run_report()["complete"])
        branches = [
            row
            for path in self.output.glob("testlib-*.info.gz")
            for row in gzip.decompress(path.read_bytes()).decode().splitlines()
            if row.startswith("BRDA:")
        ]
        self.assertEqual(
            set(branches),
            {"BRDA:1,0,gem5-compiled-one,1", "BRDA:1,0,gem5-compiled-two,1"},
        )

    def test_small_package_compresses_baseline_and_retains_generated_source(
        self,
    ):
        from schema import (
            canonical_hash,
            load_record,
        )

        source = self.root / "package-input"
        coverage = source / "coverage"
        baseline = {
            "schema_version": 1,
            "format": "gem5-coverage-baseline",
            "revision": REVISION,
            "language": "native",
            "build_id": "shared",
            "files": [{"path": "build/ALL/generated.cc", "lines": {"1": 0}}],
        }
        identity = canonical_hash(baseline)
        record = {
            "schema_version": 2,
            "language": "native",
            "revision": REVISION,
            "test_uid": UID,
            "invocation_id": "one",
            "baseline_id": identity,
            "outcome": "passed",
            "collection": "complete",
            "build": {"build_id": "shared"},
            "files": [{"path": "build/ALL/generated.cc", "lines": {"1": 1}}],
        }
        report.write_json(coverage / "one/coverage.json", record)
        report.write_json(
            coverage / "baselines" / identity / "baseline.json", baseline
        )
        generated = (
            coverage
            / "baselines"
            / identity
            / "sources/build/ALL/generated.cc"
        )
        generated.parent.mkdir(parents=True)
        generated.write_text("return 1;\n")
        (coverage / "one/raw.gcno").write_bytes(b"native notes")
        raw = self.root / "coverage-raw-one"
        report.package(source, self.output, REVISION, raw)
        self.assertFalse((self.output / "raw-profiles.tar.gz").exists())
        self.assertFalse(list(self.output.rglob("*.gcno")))
        self.assertFalse(list(self.output.rglob("baseline.json")))
        expanded = load_record(
            self.output / "records/one/coverage.json", root=self.output
        )
        self.assertEqual(expanded["files"][0]["lines"], {"1": 1})
        compressed = next(self.output.rglob("baseline.json.gz"))
        self.assertEqual(compressed.read_bytes()[4:8], b"\0" * 4)
        with gzip.open(compressed, "rt") as stream:
            self.assertEqual(json.load(stream), baseline)
        with tarfile.open(raw / "raw-profiles.tar.gz") as archive:
            self.assertTrue(
                any(
                    name.endswith("baseline.json")
                    for name in archive.getnames()
                )
            )
        with tarfile.open(self.output / "source-snapshots.tar.gz") as archive:
            self.assertEqual(len(archive.getnames()), 1)
            self.assertEqual(
                archive.extractfile(archive.getmembers()[0]).read(),
                b"return 1;\n",
            )
        self.assertEqual(
            json.loads((self.output / "artifact.json").read_text()),
            json.loads((raw / "artifact.json").read_text()),
        )

    def test_native_package_separates_sources_from_counters(self):
        build = self.root / "build/ALL"
        build.mkdir(parents=True)
        (build / "file.gcno").write_bytes(b"notes")
        (build / "file.gcda").write_bytes(b"counts")
        (build / "generated.cc").write_text("return 1;\n")
        report.write_json(
            self.output / "aggregate.json",
            {"revision": REVISION, "group": "unittests-fast"},
        )
        raw = self.root / "native-raw"
        report.package_native(self.output, raw, REVISION, self.root)
        with tarfile.open(raw / "raw-gcov.tar.gz") as archive:
            self.assertEqual(
                set(archive.getnames()),
                {"build/ALL/file.gcno", "build/ALL/file.gcda"},
            )
        with tarfile.open(self.output / "source-snapshots.tar.gz") as archive:
            self.assertEqual(archive.getnames(), ["build/ALL/generated.cc"])


if __name__ == "__main__":
    unittest.main()
