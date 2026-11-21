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

"""Build-free checks for the pilot's pytest command line."""

import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SMOKE = Path("tests/gem5/m5_util/m5_exit_pytest.py")
QUICK_ID = f"{SMOKE}::test_riscv_m5_exit"
LONG_ID = "tests/gem5/synthetic/long_pytest.py::test_long"


class SelectionTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name).resolve()
        shutil.copy(ROOT / "pytest-gem5.ini", self.root)
        package = self.root / "tests/gem5_testing"
        package.mkdir(parents=True)
        for name in ("__init__.py", "plugin.py"):
            shutil.copy(ROOT / "tests/gem5_testing" / name, package)
        smoke = self.root / SMOKE
        smoke.parent.mkdir(parents=True)
        shutil.copy(ROOT / SMOKE, smoke)
        long_case = self.root / "tests/gem5/synthetic/long_pytest.py"
        long_case.parent.mkdir()
        long_case.write_text(
            "import pytest\n"
            "@pytest.mark.gem5(length='long', tags=('m5-util',), "
            "build='build/ALL/gem5.opt', environment='host', resources={})\n"
            "def test_long(): pass\n"
        )

    def preview(self, *args, cwd=None, config=None, expected=0):
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "-c",
                str(config or self.root / "pytest-gem5.ini"),
                "--disable-plugin-autoload",
                "--collect-only",
                "-q",
                *args,
            ],
            cwd=cwd or self.root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(
            result.returncode, expected, result.stdout + result.stderr
        )
        return result.stdout + result.stderr

    def test_display_does_not_consume_a_positional_file(self):
        smoke = self.root / SMOKE
        content = smoke.read_bytes()
        output = self.preview("--gem5-plan", str(SMOKE), "--length=all")
        self.assertIn(QUICK_ID, output)
        self.assertNotIn(LONG_ID, output)
        self.assertEqual(smoke.read_bytes(), content)

    def test_plan_is_independent_of_local_build_availability(self):
        destination = self.root / "plan.json"
        args = ("--gem5-plan-file=plan.json", "--tag=riscv", "--tag=m5-util")
        self.assertIn("gem5.opt, missing", self.preview(*args))
        before = destination.read_bytes()
        plan = json.loads(before)
        self.assertEqual(plan["schema_version"], 1)
        self.assertEqual(plan["builds"], ["build/ALL/gem5.opt"])
        self.assertEqual(
            [case["id"] for case in plan["executions"]], [QUICK_ID]
        )
        binary = self.root / "build/ALL/gem5.opt"
        binary.parent.mkdir(parents=True)
        binary.touch()
        self.assertNotIn("gem5.opt, missing", self.preview(*args))
        self.assertEqual(destination.read_bytes(), before)

    def test_exact_node_id_can_select_a_non_default_length(self):
        output = self.preview("tests/gem5")
        self.assertIn(QUICK_ID, output)
        self.assertNotIn(LONG_ID, output)
        output = self.preview(
            "long_pytest.py::test_long",
            cwd=self.root / "tests/gem5/synthetic",
        )
        self.assertIn(LONG_ID, output)
        self.assertNotIn(QUICK_ID, output)

    def test_plan_ids_stay_checkout_relative_through_symlink(self):
        alias = self.root / "alias"
        alias.symlink_to(self.root, target_is_directory=True)
        self.preview(
            str(SMOKE),
            "--gem5-plan-file=plan.json",
            config=alias / "pytest-gem5.ini",
        )
        plan = json.loads((self.root / "plan.json").read_text())
        self.assertEqual(
            [case["id"] for case in plan["executions"]], [QUICK_ID]
        )

    def test_zero_matches_and_unknown_tags_leave_no_plan(self):
        destination = "--gem5-plan-file=plan.json"
        self.preview(destination, "--tag=riscv", "--length=long", expected=5)
        output = self.preview(destination, "--tag=invalid", expected=4)
        self.assertIn("Unknown gem5 tag", output)
        self.assertFalse((self.root / "plan.json").exists())

    def test_explicit_testlib_file_is_not_imported(self):
        testlib = self.root / "tests/gem5/m5_util/test_exit.py"
        testlib.write_text("raise AssertionError('unexpected import')\n")
        output = self.preview(str(testlib), expected=4)
        self.assertIn("outside the gem5 pytest suite", output)
        self.assertNotIn("unexpected import", output)


if __name__ == "__main__":
    unittest.main()
