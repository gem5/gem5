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

"""Checks for compatible, relocatable coverage build artifacts."""

import importlib.util
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "coverage_build", Path(__file__).parents[1] / "build.py"
)
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class BuildArtifactsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "producer"
        self.root.mkdir()
        self.target = "build/ALL/gem5.opt"
        self.image = "ghcr.io/gem5/test@sha256:" + "a" * 64
        self.revision = "b" * 40
        directory = self.root / "build/ALL"
        directory.mkdir(parents=True)
        (directory / "gem5.opt").write_bytes(b"executable")
        (directory / "sample.gcno").write_bytes(b"notes")
        (directory / "sample.gcda").write_bytes(b"stale counters")
        (directory / "sample.o").write_bytes(b"object")
        (directory / "sample.py.gcno").write_bytes(b"python generated notes")
        (directory / "generated.cc").write_text("int main() {}\n")
        (directory / ".config").write_text("CONFIG_USE_TEST_OBJECTS=y\n")
        self.output = Path(self.temp.name) / "artifacts"

    def package(self):
        with patch.object(
            subprocess,
            "check_output",
            side_effect=[self.revision, "gcc 13", "gcov 13"],
        ):
            return build.package(
                self.root, self.target, self.image, self.output
            )

    def test_round_trip_and_no_stale_counters(self):
        manifest = self.package()
        self.assertEqual(
            set(manifest["files"]),
            {
                self.target,
                "build/ALL/sample.gcno",
                "build/ALL/generated.cc",
                "build/ALL/.config",
            },
        )
        consumer = Path(self.temp.name) / "consumer"
        consumer.mkdir()
        build.unpack(
            consumer,
            self.output / "ALL-gem5.opt.json",
            self.revision,
            self.image,
        )
        self.assertEqual((consumer / self.target).read_bytes(), b"executable")
        self.assertTrue((consumer / self.target).stat().st_mode & 0o100)
        self.assertTrue((consumer / "build/ALL/ALL-gem5.opt.json").exists())
        self.assertFalse((consumer / "build/ALL/sample.gcda").exists())

    def test_rejects_incompatible_build_before_installing(self):
        self.package()
        consumer = Path(self.temp.name) / "consumer"
        consumer.mkdir()
        for revision, image in [
            ("c" * 40, self.image),
            (self.revision, "other"),
        ]:
            with self.assertRaisesRegex(ValueError, "does not match"):
                build.unpack(
                    consumer,
                    self.output / "ALL-gem5.opt.json",
                    revision,
                    image,
                )
        self.assertEqual(list(consumer.iterdir()), [])

    def test_rejects_checksum_mismatch(self):
        self.package()
        path = self.output / "ALL-gem5.opt.json"
        manifest = json.loads(path.read_text())
        manifest["files"][self.target] = "0" * 64
        path.write_text(json.dumps(manifest))
        consumer = Path(self.temp.name) / "consumer"
        consumer.mkdir()
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            build.unpack(consumer, path, self.revision, self.image)
        self.assertEqual(list(consumer.iterdir()), [])

    def test_target_and_protocol_validation(self):
        (self.root / "build_opts").mkdir()
        for name in ("VEGA", "VEGA_X86", "ALL"):
            (self.root / "build_opts" / name).touch()
        self.assertEqual(
            build.configuration(self.root, "build/VEGA_X86/gem5.opt"),
            ("VEGA_X86", None),
        )
        self.assertEqual(
            build.configuration(
                self.root, "build/ALL_MESI_Two_Level/gem5.opt"
            ),
            ("ALL", "MESI_Two_Level"),
        )
        for target in (
            "../gem5.opt",
            "build/../../gem5.opt",
            "build/ALL/x;bad",
            "/build/ALL/gem5.opt",
        ):
            with self.assertRaises(ValueError):
                build.target_name(target)

    def test_discovery_unions_lengths_without_duplicates(self):
        outputs = [
            f"{self.root}/{self.target}\n",
            f"{self.root}/{self.target}\n{self.root}/build/NULL/gem5.opt\n",
            "",
        ]
        with patch.object(subprocess, "check_output", side_effect=outputs):
            self.assertEqual(
                build.discover(self.root), [self.target, "build/NULL/gem5.opt"]
            )

    def test_requires_immutable_image(self):
        with self.assertRaisesRegex(ValueError, "immutable"):
            build.package(self.root, self.target, "image:latest", self.output)

    def test_empty_plan_skips_artifact_download(self):
        output = io.StringIO()
        with (
            patch.object(sys, "argv", ["build.py", "plan", "--pattern"]),
            patch.object(build, "discover", return_value=[]),
            patch.object(sys, "stdout", output),
        ):
            build.main()
        self.assertEqual(output.getvalue().strip(), "")


if __name__ == "__main__":
    unittest.main()
