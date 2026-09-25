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

"""Exercise advisory test selection and source-coordinate boundaries."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from util.coverage import (
    analysis,
    index,
)


def record(name, lines, revision="a" * 40, **changes):
    data = {
        "schema_version": 1,
        "test_uid": f"SuiteUID:tests/gem5/example/test.py:{name}",
        "invocation_id": name,
        "revision": revision,
        "build": {"compatibility_id": "gcc13-all-opt"},
        "language": "native",
        "outcome": "passed",
        "collection": "complete",
        "files": [{"path": "src/example.cc", "lines": lines}],
    }
    data.update(changes)
    return data


class SuggestionsTest(unittest.TestCase):
    def setUp(self):
        self.index = index.build_index(
            [record("one", {"1": 1, "2": 0}), record("two", {"1": 0, "2": 1})]
        )

    def test_file_selection_retains_all_observed_suites_and_uncertainty(self):
        result = analysis.suggest_tests(self.index, ["src/example.cc"])
        self.assertEqual(len(result["suggested_tests"]), 2)
        self.assertTrue(result["advisory_only"])
        self.assertEqual(result["freshness"], "target-not-verified")
        self.assertFalse(result["collection"]["expected_test_inventory_known"])

    def test_deleted_lines_use_only_old_coordinates(self):
        patch = """diff --git a/src/example.cc b/src/example.cc
--- a/src/example.cc
+++ b/src/example.cc
@@ -2 +1,0 @@
-deleted
"""
        result = analysis.suggest_tests(
            self.index,
            patch=patch,
            base_revision="a" * 40,
            target_revision="b" * 40,
        )
        self.assertEqual(len(result["suggested_tests"]), 1)
        self.assertTrue(
            result["suggested_tests"][0]["test_uid"].endswith(":two")
        )
        self.assertEqual(
            result["suggested_tests"][0]["evidence"][0]["line"], 2
        )
        with self.assertRaisesRegex(ValueError, "base"):
            analysis.suggest_tests(
                self.index, patch=patch, base_revision="b" * 40
            )

    def test_added_lines_broaden_to_file_and_remain_unproven(self):
        patch = """diff --git a/src/example.cc b/src/example.cc
--- a/src/example.cc
+++ b/src/example.cc
@@ -1,0 +2,1 @@
+new code
"""
        result = analysis.suggest_tests(
            self.index, patch=patch, base_revision="a" * 40
        )
        self.assertEqual(len(result["suggested_tests"]), 2)
        self.assertEqual(
            result["unresolved_changes"][0][
                "added_lines_without_prior_coordinates"
            ],
            [2],
        )
        self.assertEqual(
            result["suggested_tests"][0]["evidence"][0]["basis"], "file-only"
        )

    def test_new_file_is_unknown_not_an_empty_safe_test_set(self):
        patch = """diff --git a/src/new.cc b/src/new.cc
new file mode 100644
--- /dev/null
+++ b/src/new.cc
@@ -0,0 +1 @@
+new code
"""
        result = analysis.suggest_tests(
            self.index, patch=patch, base_revision="a" * 40
        )
        self.assertEqual(result["suggested_tests"], [])
        self.assertTrue(
            result["unresolved_changes"][0]["no_observed_covering_suite"]
        )
        self.assertIn("never automatically skip CI", result["warning"])

    def test_truncated_diff_and_unsafe_path_are_rejected(self):
        with self.assertRaises(ValueError):
            analysis.parse_diff(
                "diff --git a/src/a.cc b/src/a.cc\n@@ -1,2 +1,2 @@\n-only one\n"
            )
        with self.assertRaises(ValueError):
            analysis.suggest_tests(self.index, ["../outside"])

    def test_evidence_is_bounded_without_losing_match_counts(self):
        many = index.build_index(
            [record("many", {str(i): 1 for i in range(1, 101)})]
        )
        result = analysis.suggest_tests(
            many, ["src/example.cc", "src/example.cc"]
        )
        self.assertEqual(result["suggested_tests"][0]["matching_lines"], 100)
        self.assertEqual(len(result["suggested_tests"][0]["evidence"]), 20)

    def test_repository_cli_resolves_commits_and_real_diff(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def git(*arguments):
                return subprocess.check_output(
                    ["git", "-C", str(root), *arguments], text=True
                ).strip()

            git("init", "-q")
            source = root / "src/example.cc"
            source.parent.mkdir()
            source.write_text("old line\n")
            git("add", ".")
            git(
                "-c",
                "user.name=Fixture",
                "-c",
                "user.email=fixture@example.invalid",
                "commit",
                "-qm",
                "base",
            )
            base = git("rev-parse", "HEAD")
            source.write_text("added line\nold line\n")
            git("add", ".")
            git(
                "-c",
                "user.name=Fixture",
                "-c",
                "user.email=fixture@example.invalid",
                "commit",
                "-qm",
                "target",
            )
            target = git("rev-parse", "HEAD")
            data = index.build_index([record("test", {"1": 1}, revision=base)])
            saved = root / "index.json"
            saved.write_text(json.dumps(data))
            script = Path(analysis.__file__)
            run = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "suggest",
                    str(saved),
                    "--repository",
                    str(root),
                    "--target-revision",
                    target,
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            answer = json.loads(run.stdout)
            self.assertEqual(answer["tested_revision"], base)
            self.assertEqual(answer["target_revision"], target)
            self.assertEqual(
                answer["diff_provenance"], "verified repository commits"
            )
            self.assertEqual(
                answer["suggested_tests"][0]["evidence"][0]["line"], 1
            )


if __name__ == "__main__":
    unittest.main()
