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

"""Check public profile compatibility, sparse storage and language isolation."""

import json
import tempfile
import unittest
from pathlib import Path

from util.coverage import (
    index,
    schema,
)


def profile(**changes):
    record = {
        "schema_version": 1,
        "test_uid": "SuiteUID:tests/gem5/example/test.py:example",
        "invocation_id": "native-one",
        "revision": "a" * 40,
        "build": {"compatibility_id": "gcc13-all-opt", "build_id": "graph"},
        "language": "native",
        "outcome": "passed",
        "collection": "complete",
        "files": [{"path": "src/example.cc", "lines": {"1": 1, "2": 0}}],
    }
    record.update(changes)
    return record


def baseline():
    return {
        "schema_version": 1,
        "format": "gem5-coverage-baseline",
        "revision": "a" * 40,
        "language": "native",
        "build_id": "graph",
        "files": [
            {
                "path": "src/example.cc",
                "lines": {"1": 0, "2": 0},
                "branches": [
                    {
                        "id": "branch-a",
                        "line": 1,
                        "count": 0,
                        "unit": "build/ALL/example.gcno",
                        "function": "f",
                        "ordinal": 0,
                        "fallthrough": False,
                        "throw": False,
                    }
                ],
            }
        ],
    }


class SchemaTest(unittest.TestCase):
    def test_v1_validation_rejects_invalid_build_schema_and_duplicate_files(
        self,
    ):
        for changes in (
            {"schema_version": True},
            {"build": None},
            {"language": "other"},
            {"files": profile()["files"] * 2},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                schema.normalize_record(profile(**changes))
        self.assertEqual(
            schema.normalize_record(profile(files=[]))["files"], []
        )

    def test_sparse_baseline_expands_zeros_and_branch_metadata(self):
        base = baseline()
        sparse = profile(
            schema_version=2,
            baseline_id=schema.canonical_hash(base),
            files=[
                {
                    "path": "src/example.cc",
                    "lines": {"1": 4},
                    "branches": [{"id": "branch-a", "count": 2}],
                }
            ],
        )
        expanded = schema.normalize_record(sparse, baseline=base)
        self.assertEqual(expanded["files"][0]["lines"], {"1": 4, "2": 0})
        self.assertEqual(expanded["files"][0]["branches"][0]["count"], 2)
        self.assertEqual(expanded["storage_schema_version"], 2)
        self.assertEqual(base["files"][0]["lines"]["1"], 0)
        with self.assertRaisesRegex(ValueError, "hash"):
            schema.normalize_record(
                {**sparse, "baseline_id": "f" * 64}, baseline=base
            )
        with self.assertRaisesRegex(ValueError, "baseline"):
            schema.normalize_record(
                {
                    **sparse,
                    "files": [{"path": "src/example.cc", "lines": {"9": 1}}],
                },
                baseline=base,
            )

    def test_missing_profile_before_baseline_creation_remains_visible(self):
        record = schema.normalize_record(
            profile(
                schema_version=2,
                files=[],
                collection="missing",
                outcome="interrupted",
            )
        )
        self.assertEqual(record["collection"], "missing")
        with self.assertRaises(ValueError):
            schema.normalize_record(profile(schema_version=2, files=[]))

    def test_filesystem_loader_verifies_baseline_and_scans_python(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = baseline()
            identity = schema.canonical_hash(base)
            bpath = root / "baselines" / identity / "baseline.json"
            bpath.parent.mkdir(parents=True)
            bpath.write_text(json.dumps(base))
            folder = root / "run"
            folder.mkdir()
            sparse = profile(
                schema_version=2,
                baseline_id=identity,
                files=[{"path": "src/example.cc", "lines": {"1": 3}}],
            )
            (folder / "coverage.json").write_text(json.dumps(sparse))
            python = profile(
                language="python",
                invocation_id="native-one-python",
                files=[
                    {
                        "path": "src/example.py",
                        "lines": {"1": 1},
                        "branches": [
                            {"id": "arc", "line": 1, "to_line": -1, "count": 1}
                        ],
                    }
                ],
            )
            (folder / "python-coverage.json").write_text(json.dumps(python))
            result = index.build_index(schema.read_index_records(root))
            self.assertEqual(
                result["summary"]["languages"]["native"]["measured_lines"], 2
            )
            self.assertEqual(
                result["summary"]["languages"]["python"]["measured_lines"], 1
            )
            self.assertEqual(
                index.tests_for_line(result, "src/example.py", 1)["measured"],
                False,
            )
            self.assertTrue(
                index.tests_for_line(result, "src/example.py", 1, "python")[
                    "measured"
                ]
            )
            self.assertEqual(
                index.tests_for_branch(
                    result, "src/example.py", "arc", "python"
                )["count"],
                1,
            )
            self.assertEqual(
                index.tests_for_branch(result, "src/example.cc", "branch-a")[
                    "tests"
                ],
                [],
            )
            bpath.write_text(json.dumps({**base, "revision": "b" * 40}))
            with self.assertRaisesRegex(ValueError, "hash"):
                schema.load_record(folder / "coverage.json", root=root)

    def test_both_readers_reject_wrong_or_missing_build_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = baseline()
            identity = schema.canonical_hash(base)
            bpath = root / "baselines" / identity / "baseline.json"
            bpath.parent.mkdir(parents=True)
            bpath.write_text(json.dumps(base))
            path = root / "coverage.json"
            for build in ({"build_id": "other"}, {}, None):
                sparse = profile(
                    schema_version=2,
                    baseline_id=identity,
                    build=build,
                    files=[],
                )
                path.write_text(json.dumps(sparse))
                for reader in (schema.load_record, schema.load_sparse_record):
                    with (
                        self.subTest(build=build, reader=reader.__name__),
                        self.assertRaises(ValueError),
                    ):
                        reader(path, root=root)

    def test_baseline_reference_cannot_escape_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = profile(schema_version=2, baseline_id="../secret")
            path = root / "coverage.json"
            path.write_text(json.dumps(record))
            with self.assertRaises(ValueError):
                schema.load_record(path, root=root)

    def test_sparse_index_and_expanded_index_have_same_measurements(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = baseline()
            identity = schema.canonical_hash(base)
            path = root / "baselines" / identity / "baseline.json"
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(base))
            for i in range(20):
                folder = root / str(i)
                folder.mkdir()
                record = profile(
                    schema_version=2,
                    baseline_id=identity,
                    invocation_id=f"native-{i}",
                    files=[{"path": "src/example.cc", "lines": {"1": i + 1}}],
                )
                (folder / "coverage.json").write_text(json.dumps(record))
            dense = index.build_index(schema.read_records(root))
            sparse = index.build_index(schema.read_index_records(root))
            self.assertEqual(dense["files"], sparse["files"])
            self.assertEqual(dense["branches"], sparse["branches"])
            self.assertEqual(dense["summary"], sparse["summary"])


if __name__ == "__main__":
    unittest.main()
