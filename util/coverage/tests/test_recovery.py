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

"""Test safe native extraction retries and their accounting evidence."""

import importlib.util
import io
import json
import os
import sys
import tarfile
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

MODULE = Path(__file__).resolve().parents[1] / "recovery.py"
sys.path.insert(0, str(MODULE.parent))
spec = importlib.util.spec_from_file_location("coverage_recovery", MODULE)
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)
from schema import canonical_hash


class RecoveryTest(unittest.TestCase):
    def test_archive_restores_internal_hardlinks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "raw.tar.gz"
            with tarfile.open(archive, "w:gz") as stream:
                member = tarfile.TarInfo("coverage/note.gcno")
                member.size = 3
                stream.addfile(member, io.BytesIO(b"abc"))
                member = tarfile.TarInfo("coverage/copy.gcno")
                member.type = tarfile.LNKTYPE
                member.linkname = "coverage/note.gcno"
                stream.addfile(member)
            output = recovery.extract_archive(archive, root / "output")
            self.assertEqual(
                (output / "coverage/copy.gcno").read_bytes(), b"abc"
            )
            self.assertEqual(
                os.stat(output / "coverage/copy.gcno").st_ino,
                os.stat(output / "coverage/note.gcno").st_ino,
            )

    def test_unsafe_archive_members_and_expansion_limit_fail(self):
        for name, kind, limit in (
            ("../escape", tarfile.REGTYPE, 10),
            ("link", tarfile.SYMTYPE, 10),
            ("large", tarfile.REGTYPE, 1),
        ):
            with (
                self.subTest(name=name),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                archive = root / "raw.tar.gz"
                with tarfile.open(archive, "w:gz") as stream:
                    member = tarfile.TarInfo(name)
                    member.type = kind
                    member.linkname = "/etc/passwd"
                    member.size = 3 if kind == tarfile.REGTYPE else 0
                    stream.addfile(member, io.BytesIO(b"abc"))
                with self.assertRaises(ValueError):
                    recovery.extract_archive(archive, root / "output", limit)

    def test_profile_retry_keeps_outcome_and_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record_dir = root / "coverage/invocation"
            objects = record_dir / "raw/producer/gem5/build/ALL"
            objects.mkdir(parents=True)
            (objects / "file.gcno").write_bytes(b"notes")
            (objects / "file.gcda").write_bytes(b"counters")
            baseline = {
                "schema_version": 1,
                "format": "gem5-coverage-baseline",
                "revision": "a" * 40,
                "language": "native",
                "build_id": "build",
                "files": [
                    {"path": "src/file.cc", "lines": {"1": 0}, "branches": []}
                ],
            }
            identity = canonical_hash(baseline)
            baseline_dir = root / "coverage/baselines" / identity
            baseline_dir.mkdir(parents=True)
            (baseline_dir / "baseline.json").write_text(json.dumps(baseline))
            record = {
                "schema_version": 2,
                "revision": "a" * 40,
                "language": "native",
                "test_uid": "SuiteUID:tests/example.py:example",
                "invocation_id": "attempt",
                "outcome": "failed",
                "collection": "error",
                "files": [],
                "baseline_id": identity,
                "build": {
                    "build_id": "build",
                    "build_root": "/producer/gem5",
                    "gcov_version": "gcov exact",
                    "gcc_versions": ["14.2"],
                },
            }
            path = record_dir / "coverage.json"
            path.write_text(json.dumps(record))
            output = record_dir / "output.json"
            output.write_text(json.dumps(record))
            decoder = mock.Mock()
            decoder._read_profiles.return_value = (
                {"src/file.cc": {"lines": {"1": 3}, "branches": []}},
                ["14.2"],
            )
            recovery.recover_profile(
                decoder,
                path,
                output,
                "a" * 40,
                "gcov",
                "gcov exact",
                root / "coverage",
            )
            result = json.loads(output.read_text())
            self.assertEqual(result["collection"], "complete")
            self.assertEqual(result["outcome"], "failed")
            self.assertEqual(result["invocation_id"], "attempt")
            self.assertEqual(result["files"][0]["lines"], {"1": 3})
            output.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, "Original gcov"):
                recovery.recover_profile(
                    decoder,
                    path,
                    output,
                    "a" * 40,
                    "gcov",
                    "wrong gcov",
                    root / "coverage",
                )
            self.assertEqual(
                json.loads(output.read_text())["collection"], "error"
            )

    def test_aggregate_retry_preserves_measurement_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "coverage.xml"
            recovery.aggregate_xml(
                {"src/test.cc": {"lines": {"1": 1}, "branches": []}},
                output,
                "1700000000",
            )
            self.assertEqual(
                ET.parse(output).getroot().get("timestamp"), "1700000000"
            )


if __name__ == "__main__":
    unittest.main()
