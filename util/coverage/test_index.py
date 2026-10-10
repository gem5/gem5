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
    def test_full_and_compact_branch_descriptors_can_be_combined(self):
        branch = {
            "id": "same-compiled-edge",
            "line": 1,
            "count": 1,
            "unit": "build/ALL/example.gcno",
            "function": "example",
            "ordinal": 0,
            "fallthrough": True,
            "throw": False,
        }
        full = record(
            "full",
            files=[
                {
                    "path": "src/example.cc",
                    "lines": {"1": 1},
                    "branches": [branch],
                }
            ],
        )
        compact = copy.deepcopy(full)
        compact["invocation_id"] = "compact"
        compact["files"][0]["branches"][0].pop("unit")
        compact["files"][0]["branches"][0].pop("function")
        data = coverage_index.build_index([full, compact])
        self.assertEqual(data, coverage_index.build_index([compact, full]))
        result = coverage_index.tests_for_branch(
            data, "src/example.cc", branch["id"]
        )
        self.assertEqual(result["count"], 2)
        self.assertEqual(
            result["tests"][0]["invocation_ids"], ["compact", "full"]
        )
        conflicting = copy.deepcopy(compact)
        conflicting["files"][0]["branches"][0]["line"] = 2
        with self.assertRaisesRegex(ValueError, "Conflicting metadata"):
            coverage_index.build_index([full, conflicting])

    def test_browser_sidecars_preserve_all_branches_and_bound_each_chunk(self):
        branches = [
            {
                "id": f"branch-{number}",
                "line": 1 + number // 100,
                "count": number % 2,
                "ordinal": number,
            }
            for number in range(250)
        ]
        item = record(
            "branches",
            files=[
                {
                    "path": "src/example.cc",
                    "lines": {"1": 1, "2": 1, "3": 1},
                    "branches": branches,
                }
            ],
        )
        data = coverage_index.build_index([item])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            coverage_index.write_browser(data, output, chunk_size=80)
            html = (output / "index.html").read_text()
            view = json.loads(
                html.split(
                    '<script id="coverage-data" type="application/json">'
                )[1].split("</script>")[0]
            )
            self.assertEqual(view["branches"], {})
            pages = view["branch_pages"]["native"]["src/example.cc"]
            self.assertEqual(set(pages), {"1", "2", "3"})
            recovered = {}
            for path in (output / "branches").glob("*.js"):
                payload = json.loads(path.read_text().split("]=", 1)[1][:-2])
                self.assertLessEqual(len(payload), 80)
                recovered.update(payload)
            self.assertEqual(
                set(recovered), {branch["id"] for branch in branches}
            )
            for branch in branches:
                self.assertNotIn("id", recovered[branch["id"]]["metadata"])
                self.assertEqual(
                    recovered[branch["id"]]["count"], branch["count"]
                )
                queried = coverage_index.tests_for_branch(
                    data, "src/example.cc", branch["id"]
                )
                self.assertTrue(queried["measured"])
                self.assertEqual(queried["metadata"]["id"], branch["id"])
            self.assertEqual(
                data["summary"]["languages"]["native"]["measured_branches"],
                250,
            )

    def test_retained_generated_source_links_and_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_dir = root / "sources"
            source_dir.mkdir()
            page = source_dir / "pages" / "generated page.html"
            page.parent.mkdir()
            page.write_text('<pre id="L1">generated source</pre>')
            mapping = source_dir / "map.json"
            mapping.write_text(
                json.dumps(
                    {
                        "build/ALL/generated.cc": {
                            "page": "pages/generated page.html"
                        }
                    }
                )
            )
            profiles = root / "profiles"
            profiles.mkdir()
            item = record(
                "generated",
                files=[{"path": "build/ALL/generated.cc", "lines": {"1": 2}}],
            )
            (profiles / "coverage.json").write_text(json.dumps(item))
            output = root / "index"
            script = Path(coverage_index.__file__)
            subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "build",
                    str(profiles),
                    "--output",
                    str(output),
                    "--source-map",
                    str(mapping),
                ],
                check=True,
                capture_output=True,
            )
            result = json.loads((output / "index.json").read_text())
            self.assertEqual(
                result["source_links"]["build/ALL/generated.cc"],
                "./../sources/pages/generated%20page.html",
            )
            html = (output / "index.html").read_text()
            self.assertIn("generated%20page.html", html)
            self.assertIn("sourceLink(location, item.path, item.line)", html)

    def test_source_map_rejects_escape_missing_pages_and_non_html(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sources = root / "sources"
            sources.mkdir()
            outside = root / "outside.html"
            outside.write_text("outside")
            (sources / "link.html").symlink_to(outside)
            (sources / "code.py").write_text("print('not HTML')")
            mapping = sources / "map.json"
            for page in (
                "../outside.html",
                "link.html",
                "missing.html",
                "code.py",
                "https://host/page.html",
                "/absolute.html",
            ):
                with self.subTest(page=page), self.assertRaises(ValueError):
                    mapping.write_text(
                        json.dumps({"src/a.cc": {"page": page}})
                    )
                    coverage_index.attach_source_links(
                        {}, mapping, root / "index"
                    )
            mapping.write_text(json.dumps({"../bad.cc": {"page": "code.py"}}))
            with self.assertRaises(ValueError):
                coverage_index.attach_source_links({}, mapping, root / "index")

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
