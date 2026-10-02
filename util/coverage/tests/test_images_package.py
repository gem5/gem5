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


"""Validate image selection and artifact packaging without a runner."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


def module(name):
    spec = importlib.util.spec_from_file_location(
        "coverage_" + name, Path(__file__).parents[1] / (name + ".py")
    )
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


images, package = module("images"), module("package")


class ImageAndPackagingTest(unittest.TestCase):
    def test_platform_selection_and_ambiguous_index_rejection(self):
        image = "ghcr.io/gem5/test:latest"
        entry = {
            "digest": "sha256:" + "a" * 64,
            "platform": {"os": "linux", "architecture": "amd64"},
        }
        with patch.object(
            images,
            "inspect",
            return_value={
                "manifests": [entry, {"platform": {"architecture": "arm64"}}]
            },
        ):
            self.assertEqual(
                images.pin(image), "ghcr.io/gem5/test@" + entry["digest"]
            )
        for entries in ([], [entry, entry]):
            with patch.object(
                images, "inspect", return_value={"manifests": entries}
            ):
                with self.assertRaises(ValueError):
                    images.pin(image)

    def test_single_image_requires_matching_platform(self):
        with patch.object(
            images,
            "inspect",
            side_effect=[
                {"digest": "sha256:" + "b" * 64},
                {"os": "linux", "architecture": "amd64"},
            ],
        ):
            self.assertTrue(images.pin("gem5:test").endswith("b" * 64))
        with patch.object(
            images,
            "inspect",
            side_effect=[
                {"digest": "wrong-platform"},
                {"os": "linux", "architecture": "arm64"},
            ],
        ):
            with self.assertRaises(ValueError):
                images.pin("gem5:test")

    def test_report_inputs_exclude_raw_notes_and_keep_baseline_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "profiles"
            baseline = source / "baselines/identity/baseline.json"
            baseline.parent.mkdir(parents=True)
            baseline.write_text("{}")
            invocation = source / "invocation"
            invocation.mkdir()
            (invocation / "coverage.json").write_text("{}")
            (invocation / "python-coverage.json").write_text("{}")
            (invocation / "notes.gcno").write_bytes(b"notes")
            status = root / "status.json"
            status.write_text('{"revision":"test"}')
            output = root / "report-input"
            archive = root / "raw.tar.gz"
            package.package(source, status, output, archive)
            stored = output / "coverage/baselines/identity/baseline.json.gz"
            self.assertTrue(stored.is_file())
            with package.gzip.open(stored, "rt") as data:
                self.assertEqual(json.load(data), {})
            self.assertFalse(list(output.rglob("*.gcno")))
            self.assertEqual(
                json.loads((output / "status.json").read_text()),
                {"revision": "test"},
            )
            with package.tarfile.open(archive) as data:
                self.assertIn(
                    "coverage/invocation/notes.gcno", data.getnames()
                )

    def test_absent_execution_retains_a_failed_status(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            output = root / "output"
            package.package(
                root / "absent",
                root / "absent-status",
                output,
                root / "raw.tar.gz",
            )
            self.assertEqual(
                json.loads((output / "status.json").read_text())["stages"][
                    "test"
                ],
                "failure",
            )


if __name__ == "__main__":
    unittest.main()
