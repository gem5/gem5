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

"""Exercise the offline landing page using retained coverage artifacts."""

import importlib.util
import io
import json
import subprocess
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest import mock

MODULE = Path(__file__).resolve().parents[1] / "landing.py"
spec = importlib.util.spec_from_file_location("coverage_landing", MODULE)
landing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(landing)


class LandingTest(unittest.TestCase):
    def test_union_and_retained_generated_source_escaping(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output = root / "source", root / "report"
            source.mkdir()
            output.mkdir()
            summary = {
                "revision": "a" * 40,
                "complete": False,
                "scope": "Native suite invocations",
                "counts": {},
                "suites": [],
                "errors": ["<script>bad</script>"],
                "aggregates": [],
            }
            (output / "summary.json").write_text(json.dumps(summary))
            (output / "one.info").write_text(
                "SF:build/ALL/generated.cc\nDA:1,1\nDA:2,0\n"
            )
            (output / "aggregate-units.xml").write_text(
                '<coverage><class filename="build/ALL/generated.cc"><lines><line number="2" hits="5"/></lines></class></coverage>'
            )
            text = b"<script>source</script>\nreturn 1;\n"
            with tarfile.open(
                source / "source-snapshots.tar.gz", "w:gz"
            ) as archive:
                info = tarfile.TarInfo("build/ALL/generated.cc")
                info.size = len(text)
                archive.addfile(info, io.BytesIO(text))
            result = landing.build(source, output, "a" * 40)
            self.assertIn("build/ALL/generated.cc", result)
            page = (output / "index.html").read_text()
            self.assertIn("2 / 2 executable lines hit", page)
            self.assertIn("Incomplete collection", page)
            self.assertNotIn("<script>bad", page)
            rendered = (
                output / "sources" / result["build/ALL/generated.cc"]["page"]
            ).read_text()
            self.assertIn("&lt;script&gt;source", rendered)
            self.assertIn('id="L2"', rendered)

    def test_source_links_and_oversized_members_are_not_followed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with tarfile.open(root / "raw-gcov.tar.gz", "w:gz") as archive:
                member = tarfile.TarInfo("build/link.cc")
                member.type = tarfile.SYMTYPE
                member.linkname = "/etc/passwd"
                archive.addfile(member)
                member = tarfile.TarInfo("../outside.cc")
                member.size = 1
                archive.addfile(member, io.BytesIO(b"x"))
            found, missing = landing.source_texts(
                root, {"build/link.cc"}, "a" * 40
            )
            self.assertEqual(found, {})
            self.assertIn("build/link.cc", missing)

    def test_conflicting_generated_sources_are_not_linked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for index, contents in enumerate((b"first", b"second")):
                folder = root / str(index)
                folder.mkdir()
                with tarfile.open(
                    folder / "raw-gcov.tar.gz", "w:gz"
                ) as archive:
                    member = tarfile.TarInfo("build/generated.cc")
                    member.size = len(contents)
                    archive.addfile(member, io.BytesIO(contents))
            found, unavailable = landing.source_texts(
                root, {"build/generated.cc"}, "a" * 40
            )
            self.assertEqual(found, {})
            self.assertIn("conflicting", unavailable["build/generated.cc"])

    def test_source_uses_committed_blob_not_dirty_checkout(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def git(*args):
                return subprocess.check_output(
                    ["git", *args], cwd=root, text=True
                ).strip()

            git("init", "-q")
            (root / "source.cc").write_text("original\n")
            git("add", "source.cc")
            git(
                "-c",
                "user.name=Coverage Test",
                "-c",
                "user.email=test@example.com",
                "-c",
                "core.hooksPath=/dev/null",
                "commit",
                "-qm",
                "fixture",
            )
            revision = git("rev-parse", "HEAD")
            (root / "source.cc").write_text("dirty\n")
            found, _ = landing.source_texts(
                root, {"source.cc"}, revision, root
            )
            self.assertEqual(found["source.cc"], "original\n")

    def test_python_sources_and_legacy_suite_definition_are_browsable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary = {
                "revision": "a" * 40,
                "complete": True,
                "scope": "separate languages",
                "profile_exclusions": [
                    {
                        "reason": "No <child> interpreter coverage.",
                        "tests": ["SuiteUID:gem5/example/test.py:case"],
                    }
                ],
                "counts": {},
                "errors": [],
                "suites": [
                    {
                        "test_uid": "SuiteUID:gem5/example/test.py:case",
                        "outcome": "completed",
                        "exclusion": None,
                        "missing_profiles": False,
                    }
                ],
            }
            (root / "summary.json").write_text(json.dumps(summary))
            (root / "testlib-group.info").write_text(
                "SF:src/native.cc\nDA:1,0\n"
            )
            (root / "python-group.info").write_text(
                "SF:src/python/example.py\nDA:1,1\n"
            )
            with mock.patch.object(
                landing,
                "source_texts",
                return_value=(
                    {
                        "src/python/example.py": "print(1)\n",
                        "tests/gem5/example/test.py": "# definition\n",
                    },
                    {},
                ),
            ) as sources:
                mapping = landing.build(root, root, "a" * 40)
            self.assertIn("src/python/example.py", mapping)
            self.assertIn(
                "tests/gem5/example/test.py", sources.call_args.args[1]
            )
            text = (root / "index.html").read_text()
            self.assertIn("0 / 1 executable lines hit", text)
            self.assertIn("1 / 1 executable Python lines hit", text)
            self.assertIn("No &lt;child&gt; interpreter coverage.", text)
            source = (
                root / "sources" / mapping["src/python/example.py"]["page"]
            )
            self.assertIn('id="L1" class="hit"', source.read_text())

    def test_lightweight_baseline_sources_need_no_raw_download(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with tarfile.open(
                root / "source-snapshots.tar.gz", "w:gz"
            ) as archive:
                member = tarfile.TarInfo(
                    "coverage/baselines/hash/sources/build/ALL/generated.cc"
                )
                member.size = 3
                archive.addfile(member, io.BytesIO(b"x;\n"))
            found, unavailable = landing.source_texts(
                root, {"build/ALL/generated.cc"}, "a" * 40
            )
            self.assertEqual(found, {"build/ALL/generated.cc": "x;\n"})
            self.assertFalse(unavailable)


if __name__ == "__main__":
    unittest.main()
