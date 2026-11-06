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

import copy
import json
import subprocess
import sys
import tempfile
import unittest
import weakref
from pathlib import Path
from unittest import mock

from util.coverage import (
    analysis,
    index,
    index_artifact,
    schema,
)


class IndexArtifactTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "profiles"
        self.output = self.root / "artifact"
        self.source.mkdir()
        self.write_native("a-one", "graph-a", "one", 2, compact=False)
        self.write_native("a-two", "graph-a", "two", 3, compact=True)
        self.write_native("b-one", "graph-b", "one", 4, compact=True)
        self.write_python()

    def write_native(self, name, graph, suite, count, compact):
        branches = [
            {
                "id": graph + suffix,
                "line": line,
                "count": 0,
                "ordinal": line - 1,
                "function": "function_name",
                "unit": "build/ALL/example.gcno",
            }
            for suffix, line in (("-hit", 1), ("-zero", 2))
        ]
        if compact:
            for branch in branches:
                branch.pop("unit")
                branch.pop("function")
        baseline = {
            "schema_version": 1,
            "format": "gem5-coverage-baseline",
            "language": "native",
            "build_id": graph,
            "revision": "a" * 40,
            "files": [
                {
                    "path": "src/shared.cc",
                    "lines": {"1": 0, "2": 0},
                    "branches": branches,
                }
            ],
        }
        identity = schema.canonical_hash(baseline)
        folder = self.source / "baselines" / identity
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "baseline.json").write_text(json.dumps(baseline))
        record = {
            "schema_version": 2,
            "language": "native",
            "revision": "a" * 40,
            "build": {"build_id": graph, "compatibility_id": graph},
            "baseline_id": identity,
            "test_uid": "SuiteUID:tests/gem5/test.py:" + suite,
            "invocation_id": name,
            "outcome": "passed",
            "collection": "complete",
            "files": [
                {
                    "path": "src/shared.cc",
                    "lines": {"1": count},
                    "branches": [{"id": graph + "-hit", "count": count}],
                }
            ],
        }
        folder = self.source / name
        folder.mkdir(exist_ok=True)
        (folder / "coverage.json").write_text(json.dumps(record))

    def write_python(self):
        record = {
            "schema_version": 1,
            "language": "python",
            "revision": "a" * 40,
            "build": {"compatibility_id": "python"},
            "test_uid": "SuiteUID:tests/gem5/test.py:two",
            "invocation_id": "python",
            "outcome": "passed",
            "collection": "complete",
            "files": [
                {
                    "path": "config.py",
                    "lines": {"1": 1, "2": 0},
                    "branches": [
                        {
                            "id": "python-arc",
                            "line": 1,
                            "to_line": 2,
                            "count": 1,
                        }
                    ],
                }
            ],
        }
        (self.source / "python-coverage.json").write_text(json.dumps(record))

    def test_two_native_graphs_and_python_match_global_public_queries(self):
        expected = index.build_index(schema.read_index_records(self.source))
        actual = index_artifact.build_artifact(
            self.source, self.output, chunk_size=1
        )
        self.assertEqual(actual["summary"], expected["summary"])
        self.assertEqual(actual["invocations"], expected["invocations"])
        self.assertEqual(actual["tests"], expected["tests"])
        self.assertEqual(actual["branches"], {})
        for language, path in (
            ("native", "src/shared.cc"),
            ("python", "config.py"),
        ):
            for line in (1, 2, 99):
                self.assertEqual(
                    index.tests_for_line(actual, path, line, language),
                    index.tests_for_line(expected, path, line, language),
                )
            for identity in (*expected["branches"][language][path], "unknown"):
                self.assertEqual(
                    index.tests_for_branch(
                        actual, path, identity, language, self.output
                    ),
                    index.tests_for_branch(expected, path, identity, language),
                )
        for uid in expected["tests"]:
            self.assertEqual(
                index.lines_for_test(actual, uid),
                index.lines_for_test(expected, uid),
            )
        graphs = {
            entry["build_id"]
            for entry in actual["branch_shards"]["native"]["src/shared.cc"]
        }
        self.assertEqual(graphs, {"graph-a", "graph-b"})

    def test_missing_profiles_remain_visible_without_measured_zeros(self):
        missing = {
            "schema_version": 2,
            "language": "native",
            "revision": "a" * 40,
            "build": {},
            "test_uid": "SuiteUID:tests/gem5/test.py:missing",
            "invocation_id": "0-missing",
            "outcome": "interrupted",
            "collection": "missing",
            "files": [],
        }
        folder = self.source / "missing"
        folder.mkdir()
        (folder / "coverage.json").write_text(json.dumps(missing))
        actual = index_artifact.build_artifact(self.source, self.output)
        self.assertEqual(actual["summary"]["collection"]["missing"], 1)
        self.assertEqual(
            actual["invocations"][0]["invocation_id"], "0-missing"
        )
        self.assertFalse(
            index.lines_for_test(actual, missing["test_uid"])["coverage_known"]
        )
        branch = index.tests_for_branch(
            actual, "src/shared.cc", "graph-a-hit", artifact_root=self.output
        )
        self.assertEqual(branch["count"], 5)
        self.assertEqual(len(branch["tests"]), 2)
        self.assertNotIn(
            missing["test_uid"], {row["test_uid"] for row in branch["tests"]}
        )

    def test_mixed_revisions_across_graphs_are_rejected(self):
        path = self.source / "python-coverage.json"
        record = json.loads(path.read_text())
        record["revision"] = "b" * 40
        path.write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError, "different source revisions"):
            index_artifact.build_artifact(self.source, self.output)
        self.assertFalse(schema._BASELINES)
        self.assertFalse(schema._CATALOGS)

    def test_previous_branch_graph_and_baseline_cache_are_released(self):
        class Tracked(dict):
            pass

        references = []
        original = index.build_index

        def tracked(*args, **kwargs):
            self.assertTrue(
                all(reference() is None for reference in references)
            )
            self.assertFalse(schema._BASELINES)
            self.assertFalse(schema._CATALOGS)
            graph = original(*args, **kwargs)
            for language in graph["branches"].values():
                for path, branches in language.items():
                    language[path] = Tracked(branches)
                    references.append(weakref.ref(language[path]))
            return graph

        with mock.patch.object(index, "build_index", side_effect=tracked):
            index_artifact.build_artifact(self.source, self.output)
        self.assertTrue(all(reference() is None for reference in references))
        self.assertFalse(schema._BASELINES)
        self.assertFalse(schema._CATALOGS)

    def test_cli_roundtrip_keeps_zero_unknown_and_compact_core(self):
        script = Path(index.__file__)
        subprocess.run(
            [
                sys.executable,
                str(script),
                "build",
                str(self.source),
                "--output",
                str(self.output),
            ],
            check=True,
            capture_output=True,
        )
        stored = self.output / "index.json"
        core = analysis.load_index(stored)
        self.assertEqual(core["schema_version"], 2)
        suggestions = analysis.suggest_tests(core, ["src/shared.cc"])
        self.assertEqual(len(suggestions["suggested_tests"]), 2)
        self.assertFalse(core["branches"])
        self.assertTrue(core["branch_shards"])
        for identity, count in (
            ("graph-a-hit", 5),
            ("graph-b-zero", 0),
            ("absent", None),
        ):
            process = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "tests-for-branch",
                    str(stored),
                    "src/shared.cc",
                    identity,
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            result = json.loads(process.stdout)
            self.assertEqual(result["count"], count)
            self.assertEqual(result["measured"], count is not None)
        page = (self.output / "index.html").read_text()
        view = json.loads(
            page.split('<script id="coverage-data" type="application/json">')[
                1
            ].split("</script>")[0]
        )
        self.assertFalse(view["branches"])
        self.assertNotIn("branch_shards", view)
        self.assertTrue(view["branch_pages"])

    def test_missing_corrupt_and_escaping_shards_are_errors_not_zero(self):
        core = index_artifact.build_artifact(self.source, self.output)
        descriptor = core["branch_shards"]["native"]["src/shared.cc"][0]
        path = self.output / descriptor["file"]
        content = path.read_text()
        path.write_text(content + "alert('not data');")
        with self.assertRaises(ValueError):
            index.tests_for_branch(
                core,
                "src/shared.cc",
                descriptor["first"],
                artifact_root=self.output,
            )
        path.unlink()
        with self.assertRaises(FileNotFoundError):
            index.tests_for_branch(
                core,
                "src/shared.cc",
                descriptor["first"],
                artifact_root=self.output,
            )
        outside = self.root / "outside.js"
        outside.write_text(content)
        path.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, "escapes"):
            index.tests_for_branch(
                core,
                "src/shared.cc",
                descriptor["first"],
                artifact_root=self.output,
            )
        unsafe = copy.deepcopy(descriptor)
        unsafe["file"] = "../outside.js"
        with self.assertRaisesRegex(ValueError, "path"):
            index_artifact.read_branch_shard(self.output, unsafe)


if __name__ == "__main__":
    unittest.main()
