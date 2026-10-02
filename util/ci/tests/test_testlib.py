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

"""Verify shared workload selection and commands without running gem5."""

import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / "testlib.py"
spec = importlib.util.spec_from_file_location("ci_testlib_commands", MODULE)
workloads = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workloads)


class TestLibCommands(unittest.TestCase):
    def test_ordinary_quick_uses_existing_thread_and_build_selection(self):
        command = workloads.run_command(
            "gem5/example", "quick", 8, skip_build=True
        )
        self.assertEqual(
            command,
            [
                sys.executable,
                "main.py",
                "run",
                "gem5/example",
                "--skip-build",
                "-t",
                "8",
            ],
        )

    def test_ordinary_daily_and_weekly_resource_and_thread_options(self):
        with patch.dict(os.environ, {"GEM5_RESOURCE_DIR": "/resources"}):
            long = workloads.run_command(
                "gem5/example", "long", 8, skip_build=True
            )
            weekly = workloads.run_command("gem5/example", "very-long", 8)
        self.assertEqual(
            long[4:],
            [
                "--skip-build",
                "--length=long",
                "-j8",
                "-vv",
                "-t",
                "8",
                "--bin-path",
                "/resources",
            ],
        )
        self.assertEqual(
            weekly[4:],
            ["--length=very-long", "-j8", "-vv", "--bin-path", "/resources"],
        )
        self.assertFalse(any("gcov" in arg for arg in long + weekly))
        self.assertNotIn("--skip-build", weekly)
        self.assertNotIn("-t", weekly)

    def test_coverage_serializes_test_processes_and_traces_both_languages(
        self,
    ):
        with patch.dict(os.environ, {"GEM5_RESOURCE_DIR": "/resources"}):
            for length in workloads.LENGTHS:
                with self.subTest(length=length):
                    command = workloads.run_command(
                        "gem5/example", length, 8, True, True
                    )
                    self.assertIn("--skip-build", command)
                    self.assertIn(f"--length={length}", command)
                    self.assertEqual(command[command.index("-t") + 1], "1")
                    self.assertIn("--gcov=per-test", command)
                    self.assertIn("--python-coverage", command)
                    self.assertIn("--host=x86_64", command)

    def test_legacy_downloads_stay_isolated(self):
        with patch.dict(
            os.environ,
            {"RUNNER_TEMP": "/tmp/ci-test", "GEM5_RESOURCE_DIR": "/resources"},
        ):
            for directory in workloads.LEGACY_DOWNLOADS:
                for length, coverage in [
                    ("long", False),
                    ("very-long", False),
                    ("quick", True),
                ]:
                    with self.subTest(directory=directory, length=length):
                        command = workloads.run_command(
                            directory, length, 4, coverage
                        )
                        self.assertEqual(
                            command[command.index("--bin-path") + 1],
                            "/tmp/ci-test/gem5-testlib-downloads",
                        )

    def test_discovery_retains_daily_target_host_restriction(self):
        for length in workloads.LENGTHS:
            normal_suites = workloads.list_command(length)
            normal_targets = workloads.list_command(length, targets=True)
            self.assertNotIn("--host=x86_64", normal_suites)
            self.assertEqual(
                "--host=x86_64" in normal_targets, length == "long"
            )
            for targets in [False, True]:
                coverage = workloads.list_command(
                    length, True, targets=targets
                )
                self.assertIn("--host=x86_64", coverage)
                self.assertIn("--gcov=per-test", coverage)
                self.assertIn("--python-coverage", coverage)

    def test_discovery_groups_exact_uids_and_deduplicates_targets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            suites = (
                f"SuiteUID:{root}/tests/gem5/b/test.py:one\n"
                f"SuiteUID:{root}/tests/gem5/a/test.py:two\n"
                f"SuiteUID:{root}/tests/gem5/b/test.py:three\n"
            )
            with patch.object(
                workloads.subprocess,
                "check_output",
                side_effect=[
                    suites,
                    "build/ALL/gem5.opt\n"
                    "build/ARM/gem5.opt\n"
                    "build/ALL/gem5.opt\n",
                ],
            ):
                data = workloads.discover("quick", root=root)
            self.assertEqual(data["directories"], ["gem5/a", "gem5/b"])
            self.assertEqual(len(data["suites"]["gem5/b"]), 2)
            self.assertEqual(
                data["targets"], ["build/ALL/gem5.opt", "build/ARM/gem5.opt"]
            )

    def test_empty_and_invalid_discovery_fail_closed(self):
        for suites in [
            "",
            "diagnostic output\n",
            "SuiteUID:/outside-checkout/test.py:one\n",
        ]:
            with (
                self.subTest(suites=suites),
                patch.object(
                    workloads.subprocess,
                    "check_output",
                    side_effect=[suites, "build/ALL/gem5.opt\n"],
                ),
            ):
                with self.assertRaises(ValueError):
                    workloads.discover("quick")

    def test_zero_binary_suite_retained_for_ordinary_daily(self):
        suites = f"SuiteUID:{workloads.ROOT}/tests/gem5/example/test.py:one\n"
        with patch.object(
            workloads.subprocess, "check_output", side_effect=[suites, ""]
        ):
            data = workloads.discover("long", directory="gem5/example")
        self.assertEqual(data["directories"], ["gem5/example"])
        self.assertEqual(data["targets"], [])


if __name__ == "__main__":
    unittest.main()
