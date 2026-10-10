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

"""Check that output comparisons preserve shared reference files."""

import importlib.util
import os
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import (
    Mock,
    patch,
)


class DiffOutFileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[2] / "ext/testlib/helper.py"
        spec = importlib.util.spec_from_file_location("testlib_helper", path)
        cls.helper = importlib.util.module_from_spec(spec)
        # Loading the helper installs TestLib's process timing wrapper. Keep
        # the PyUnit runner's waitpid unchanged; these checks do not use timing.
        with patch.object(os, "waitpid", os.waitpid):
            spec.loader.exec_module(cls.helper)

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.reference = self.directory / "reference.txt"
        self.output = self.directory / "output.txt"
        self.reference.write_text("ignore: reference banner\n-50000\n")
        self.output.write_text("ignore: output banner\n-50000\n")
        self.filters = (r"^ignore:",)

    def compare(self, output=None):
        return self.helper.diff_out_file(
            str(self.reference),
            str(output or self.output),
            Mock(),
            ignore_regexes=self.filters,
        )

    def assertInputsUnchanged(self, reference, output):
        self.assertEqual(self.reference.read_bytes(), reference)
        self.assertEqual(self.output.read_bytes(), output)
        self.assertEqual(
            set(self.directory.iterdir()), {self.reference, self.output}
        )

    def test_filtered_comparison_preserves_inputs(self):
        reference = self.reference.read_bytes()
        output = self.output.read_bytes()
        self.assertIsNone(self.compare())
        self.assertInputsUnchanged(reference, output)

    def test_mismatch_preserves_inputs(self):
        self.output.write_text("ignore: output banner\nwrong result\n")
        reference = self.reference.read_bytes()
        output = self.output.read_bytes()
        diff = self.compare()
        self.assertIn("wrong result", diff)
        self.assertIn("-50000", diff)
        self.assertInputsUnchanged(reference, output)

    def test_fallback_match(self):
        reference = self.reference.read_bytes()
        output = self.output.read_bytes()
        with patch.object(
            self.helper, "log_call", side_effect=FileNotFoundError
        ):
            self.assertIsNone(self.compare())
        self.assertInputsUnchanged(reference, output)

    def test_fallback_mismatch(self):
        self.output.write_text("ignore: output banner\nwrong result\n")
        reference = self.reference.read_bytes()
        output = self.output.read_bytes()
        with patch.object(
            self.helper, "log_call", side_effect=FileNotFoundError
        ):
            diff = self.compare()
        self.assertIn(str(self.reference), diff)
        self.assertIn(str(self.output), diff)
        self.assertIn("wrong result", diff)
        self.assertInputsUnchanged(reference, output)

    def test_parallel_comparisons_share_read_only_reference(self):
        reference = self.reference.read_bytes()
        self.reference.chmod(0o444)
        outputs = [self.directory / f"output-{i}.txt" for i in range(24)]
        for output in outputs:
            output.write_text("ignore: output banner\n-50000\n")
        with ThreadPoolExecutor(max_workers=3) as executor:
            results = list(executor.map(self.compare, outputs))
        self.assertEqual(results, [None] * len(outputs))
        self.assertEqual(self.reference.read_bytes(), reference)
        for output in outputs:
            self.assertEqual(
                output.read_text(), "ignore: output banner\n-50000\n"
            )
        self.assertEqual(
            set(self.directory.iterdir()),
            {self.reference, self.output, *outputs},
        )
