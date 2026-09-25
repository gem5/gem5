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
from pathlib import Path

from util.coverage import index as coverage_index


def record(invocation, test="a", lines=None, **changes):
    result = {
        "schema_version": 1,
        "test_uid": f"SuiteUID:gem5/example/test.py:{test}",
        "invocation_id": invocation,
        "revision": "a" * 40,
        "build": {"binary": "build/ALL/gem5.opt"},
        "outcome": "passed",
        "collection": "complete",
        "files": [{"path": "src/example.cc", "lines": lines or {}}],
    }
    result.update(changes)
    return result


class CoverageIndexTest(unittest.TestCase):
    def test_union_preserves_both_directions_and_zero_lines(self):
        first = record("first", lines={"1": 2, "2": 0})
        second = record("second", "b", {"1": 3, "3": 1})
        result = coverage_index.build_index([second, first])
        line = coverage_index.tests_for_line(result, "src/example.cc", 1)
        self.assertEqual(line["count"], 5)
        self.assertEqual(
            [test["test_uid"] for test in line["tests"]],
            [first["test_uid"], second["test_uid"]],
        )
        zero = coverage_index.tests_for_line(result, "src/example.cc", 2)
        self.assertTrue(zero["measured"])
        self.assertEqual(zero["tests"], [])
        absent = coverage_index.tests_for_line(result, "src/example.cc", 9)
        self.assertFalse(absent["measured"])
        self.assertIsNone(absent["count"])
        forward = coverage_index.lines_for_test(result, first["test_uid"])
        self.assertEqual(
            [(line["path"], line["line"]) for line in forward["lines"]],
            [("src/example.cc", 1)],
        )
        self.assertEqual(result["summary"]["measured_lines"], 3)
        self.assertEqual(result["summary"]["covered_lines"], 2)

    def test_attempts_are_retained_and_queries_deterministic(self):
        first = record("attempt-1", lines={"10": 2})
        retry = record("attempt-2", lines={"2": 1}, outcome="failed")
        result = coverage_index.build_index([retry, first])
        self.assertEqual(
            result, coverage_index.build_index([first, retry, first])
        )
        forward = coverage_index.lines_for_test(result, first["test_uid"])
        self.assertEqual(forward["invocation_ids"], ["attempt-1", "attempt-2"])
        self.assertEqual([line["line"] for line in forward["lines"]], [2, 10])
        self.assertEqual(len(result["invocations"]), 2)
        self.assertEqual(result["invocations"][1]["outcome"], "failed")

    def test_conflicting_ids_and_mixed_revisions_rejected(self):
        first = record("same", lines={"1": 1})
        with self.assertRaisesRegex(ValueError, "invocation_id"):
            coverage_index.build_index([first, record("same", "different")])
        with self.assertRaisesRegex(ValueError, "revision"):
            coverage_index.build_index(
                [first, record("other", revision="b" * 40)]
            )

    def test_missing_and_error_are_unknown_not_zero(self):
        missing = record(
            "missing",
            "missing",
            collection="missing",
            files=[],
            outcome="interrupted",
        )
        error = record("error", "error", {"1": 99}, collection="error")
        result = coverage_index.build_index([missing, error])
        self.assertEqual(result["files"], {})
        self.assertEqual(result["summary"]["collection"]["missing"], 1)
        self.assertEqual(result["summary"]["collection"]["error"], 1)
        for item in [missing, error]:
            forward = coverage_index.lines_for_test(result, item["test_uid"])
            self.assertFalse(forward["coverage_known"])
            self.assertEqual(forward["lines"], [])
        # Metadata describes the partial raw artifact without indexing it.
        partial = result["invocations"][0]
        self.assertEqual(partial["profile_summary"]["covered_lines"], 1)
        self.assertEqual(len(partial["record_sha256"]), 64)
        self.assertNotIn("files", partial)

    def test_complete_empty_profile_is_known(self):
        item = record("empty", files=[])
        result = coverage_index.build_index([item])
        forward = coverage_index.lines_for_test(result, item["test_uid"])
        self.assertTrue(forward["coverage_known"])
        self.assertEqual(forward["lines"], [])

    def test_reject_invalid_counts_paths_and_duplicate_files(self):
        for lines in ({"1": -1}, {"0": 1}, {"1": True}, {"01": 1}):
            with self.subTest(lines=lines), self.assertRaises(ValueError):
                coverage_index.build_index([record("bad", lines=lines)])
        for path in ("../secret", "/absolute", "src/../secret", "a\\b"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                coverage_index.build_index(
                    [record("bad", files=[{"path": path, "lines": {}}])]
                )
        item = record("bad")
        item["files"].append(copy.deepcopy(item["files"][0]))
        with self.assertRaisesRegex(ValueError, "duplicate file"):
            coverage_index.build_index([item])

    def test_html_embeds_data_without_executable_markup(self):
        attack = '</script><img src=x onerror="alert(1)">'
        item = record("safe", build={"description": attack})
        result = coverage_index.build_index([item])
        page = coverage_index.render_html(result)
        self.assertNotIn(attack, page)
        data = page.split(
            '<script id="coverage-data" type="application/json">'
        )[1]
        embedded = data.split("</script>", 1)[0]
        self.assertEqual(json.loads(embedded), result)
        self.assertNotIn("innerHTML", page)
        with self.assertRaises(ValueError):
            coverage_index.build_index([item], "javascript:alert(1)")
        with self.assertRaises(ValueError):
            coverage_index.build_index(
                [record("invalid-json", build={"value": float("nan")})]
            )

    def test_shared_baselines_and_memberships_stay_compact(self):
        # A generator prevents the builder from depending on a replayable
        # collection. Shared hits and zeros must not repeat in each profile.
        lines = {str(i): int(i <= 900) for i in range(1, 1001)}
        records = (
            record(f"invocation-{i:032d}", f"test-{i}", lines)
            for i in range(2000)
        )
        result = coverage_index.build_index(records)
        self.assertEqual(len(result["invocations"]), 2000)
        self.assertEqual(len(result["memberships"]), 2)
        self.assertEqual(result["summary"]["covered_lines"], 900)
        self.assertTrue(
            all("files" not in item for item in result["invocations"])
        )
        self.assertLess(
            len(coverage_index.render_html(result).encode()), 2_000_000
        )
        self.assertEqual(
            len(
                coverage_index.tests_for_line(result, "src/example.cc", 1)[
                    "tests"
                ]
            ),
            2000,
        )

    def test_cli_build_and_queries(self):
        script = Path(coverage_index.__file__).resolve()
        item = record("example", lines={"12": 3})
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "profiles" / "nested" / "invocation"
            source.mkdir(parents=True)
            (source / "coverage.json").write_text(json.dumps(item))
            # Other workflow artifacts are not invocation profiles.
            (source / "expected.json").write_text("not a coverage record")
            output = root / "report"
            subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "build",
                    str(root / "profiles"),
                    "--output",
                    str(output),
                ],
                check=True,
                capture_output=True,
            )
            index = output / "index.json"
            for command, query in [
                ("tests-for-line", "src/example.cc:12"),
                ("lines-for-test", item["test_uid"]),
            ]:
                response = subprocess.run(
                    [sys.executable, str(script), command, str(index), query],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                self.assertIn("example", response.stdout)
                json.loads(response.stdout)
            self.assertTrue((output / "index.html").is_file())
            empty = root / "empty"
            empty.mkdir()
            response = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "build",
                    str(empty),
                    "--output",
                    str(root / "empty-report"),
                ],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(response.returncode, 0)
            self.assertIn("No coverage.json", response.stderr)


if __name__ == "__main__":
    unittest.main()
