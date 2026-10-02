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

"""Check native workloads and failure reporting without compiling gem5."""

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / "native.py"
spec = importlib.util.spec_from_file_location("native_ci", MODULE)
native = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = native
spec.loader.exec_module(native)


class NativeTests(unittest.TestCase):
    def test_known_catalog_and_instrumented_builds(self):
        catalog = native.groups()
        self.assertEqual(
            {item["group"] for item in catalog},
            {
                "unittests-fast",
                "unittests-opt",
                "unittests-debug",
                "sst",
                "systemc",
                "dramsys",
            },
        )
        for item in catalog:
            group = item["group"]
            for coverage in [False, True]:
                with self.subTest(group=group, coverage=coverage):
                    plan = native.stages(group, coverage, 7)
                    self.assertEqual(len(plan), len({s.name for s in plan}))
                    builds = [
                        s.command
                        for s in plan
                        if s.command
                        and s.command[0] == "scons"
                        and s.command[1].startswith("build/")
                    ]
                    self.assertTrue(builds)
                    for command in builds:
                        self.assertEqual("--gcov" in command, coverage)
                        self.assertEqual(command[-2:], ["-j", "7"])

    def test_systemc_separates_shared_library_only_with_coverage(self):
        for coverage, library in [
            (False, "ARM"),
            (True, "ARM-systemc-library"),
        ]:
            with self.subTest(coverage=coverage):
                plan = {
                    s.name: s for s in native.stages("systemc", coverage, 4)
                }
                command = plan["build-library"].command
                self.assertEqual(command[1], f"build/{library}/libgem5_opt.so")
                self.assertIn("--duplicate-sources", command)
                self.assertIn(
                    "--duplicate-sources", plan["build-binary"].command
                )
                self.assertEqual(
                    plan["build-integration"].command,
                    ["make", f"ARCH={library}"],
                )
                self.assertEqual(
                    plan["run-continue"].environment["LD_LIBRARY_PATH"],
                    f"build/{library}/:/opt/systemc/lib-linux64/",
                )
                self.assertEqual("configure-library" in plan, coverage)
                self.assertEqual("clear-counters" in plan, coverage)

    def test_ordinary_integration_commands(self):
        sst = {s.name: s for s in native.stages("sst")}
        self.assertEqual(
            sst["prepare-makefile"].command,
            ["mv", "Makefile.linux", "Makefile"],
        )
        self.assertEqual(sst["prepare-makefile"].directory, "ext/sst")
        self.assertEqual(
            sst["run-tests"].command,
            ["sst", "--add-lib-path=./", "sst/example.py"],
        )
        dramsys = native.stages("dramsys")
        self.assertEqual(
            dramsys[0].command,
            [
                "git",
                "clone",
                "https://github.com/tukl-msd/DRAMSys",
                "--branch",
                "v5.6.0",
                "--depth",
                "1",
                "DRAMSys",
            ],
        )
        self.assertEqual(
            [s.command[1] for s in dramsys[-3:]],
            [
                "configs/example/gem5_library/dramsys/arm-hello-dramsys.py",
                "configs/example/gem5_library/dramsys/dramsys-traffic.py",
                "configs/example/dramsys.py",
            ],
        )

    def test_failed_build_skips_simulation_and_keeps_metadata(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            status = root / "output" / "status.json"
            with (
                patch.object(
                    native.subprocess,
                    "check_output",
                    return_value="revision\n",
                ),
                patch.object(
                    native.subprocess,
                    "run",
                    side_effect=[
                        subprocess.CompletedProcess([], 0),
                        subprocess.CompletedProcess([], 9),
                    ],
                ) as runner,
            ):
                self.assertEqual(
                    native.run("systemc", True, 4, status, root), 9
                )
            self.assertEqual(runner.call_count, 2)
            data = json.loads(status.read_text())
            self.assertEqual(data["revision"], "revision")
            self.assertEqual(data["group"], "systemc")
            self.assertEqual(data["status"], "failure")
            self.assertEqual(data["stages"]["build-binary"], "failure")
            self.assertEqual(data["stages"]["run-tests"], "skipped")
            self.assertEqual(data["stages"]["run-continue"], "skipped")

    def test_missing_command_keeps_failure_record(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            status = root / "status.json"
            with (
                patch.object(
                    native.subprocess, "check_output", return_value="revision"
                ),
                patch.object(
                    native.subprocess,
                    "run",
                    side_effect=FileNotFoundError("scons"),
                ),
            ):
                self.assertEqual(
                    native.run("unittests-opt", False, 1, status, root), 1
                )
            data = json.loads(status.read_text())
            self.assertEqual(data["stages"]["unit-tests"], "failure")
            self.assertEqual(data["error"], "scons")

    def test_counters_cleared_only_before_simulation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            build = root / "build" / "ARM"
            build.mkdir(parents=True)
            for name in ["test.gcda", "test.py.gcno", "test.gcno", "test.cc"]:
                (build / name).write_text("counter")
            calls = []

            def execute(command, **kwargs):
                calls.append(command)
                if command[0].startswith("./build/"):
                    self.assertFalse((build / "test.gcda").exists())
                    self.assertFalse((build / "test.py.gcno").exists())
                    self.assertTrue((build / "test.gcno").exists())
                    self.assertTrue((build / "test.cc").exists())
                return subprocess.CompletedProcess(command, 0)

            with (
                patch.object(
                    native.subprocess, "check_output", return_value="revision"
                ),
                patch.object(native.subprocess, "run", side_effect=execute),
            ):
                self.assertEqual(
                    native.run("systemc", True, 4, root / "status.json", root),
                    0,
                )
            self.assertEqual(
                json.loads((root / "status.json").read_text())["status"],
                "success",
            )
            self.assertEqual(len(calls), 8)

    def test_new_catalog_entry_requires_an_explicit_implementation(self):
        for group in ["future-integration", "unittests-future"]:
            with (
                self.subTest(group=group),
                patch.object(
                    native, "groups", return_value=[{"group": group}]
                ),
                patch.object(native.subprocess, "run") as command,
            ):
                with self.assertRaisesRegex(ValueError, "no implementation"):
                    native.run(group, False, 1, Path("unused-status.json"))
                command.assert_not_called()

    def test_reject_unknown_groups_and_invalid_jobs(self):
        with self.assertRaises(ValueError):
            native.stages("arbitrary-shell-command")
        with self.assertRaises(ValueError):
            native.stages("systemc", jobs=0)


if __name__ == "__main__":
    unittest.main()
