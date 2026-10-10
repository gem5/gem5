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

"""Verify separate SystemC coverage build roots and ordinary job behavior."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / ".github/workflows/daily-tests.yaml"


class SystemCCoverageWorkflowTest(unittest.TestCase):
    def run_commands(self, coverage):
        workflow = yaml.safe_load(WORKFLOW.read_text())
        job = workflow["jobs"]["systemc-test"]
        expression = job["env"]["SYSTEMC_LIBRARY_BUILD"]
        self.assertEqual(
            expression,
            "${{ inputs.coverage && 'ARM-systemc-library' || 'ARM' }}",
        )
        library = "ARM-systemc-library" if coverage else "ARM"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tools = root / "tools"
            tools.mkdir()
            log = root / "commands.jsonl"
            recorder = (
                f"#!{sys.executable}\nimport json, os, sys\n"
                "with open(os.environ['COMMAND_LOG'], 'a') as stream:\n"
                "    stream.write(json.dumps({'argv': sys.argv, 'library': os.environ.get('LD_LIBRARY_PATH')}) + '\\n')\n"
            )
            for name in ("scons", "make"):
                path = tools / name
                path.write_text(recorder)
                path.chmod(0o755)
            nproc = tools / "nproc"
            nproc.write_text("#!/bin/sh\necho 2\n")
            nproc.chmod(0o755)
            for name in (
                "build/ARM/gem5.opt",
                "util/systemc/gem5_within_systemc/gem5.opt.sc",
            ):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(recorder)
                path.chmod(0o755)
            environment = dict(
                os.environ,
                PATH=str(tools) + os.pathsep + os.environ["PATH"],
                COMMAND_LOG=str(log),
                SYSTEMC_LIBRARY_BUILD=library,
                GCOV_FLAGS="--gcov" if coverage else "",
            )
            selected = {
                "Build ARM/gem5.opt",
                "disable systemc",
                "Build ARM/libgem5_opt.so",
                "Compile gem5 withing SystemC",
                "Run gem5 within SystemC test",
                "Continue gem5 within SystemC test",
            }
            for step in job["steps"]:
                if step.get("name") in selected:
                    subprocess.run(
                        ["bash", "-eu", "-c", step["run"]],
                        cwd=root,
                        env=environment,
                        check=True,
                    )
            return [json.loads(line) for line in log.read_text().splitlines()]

    def test_coverage_build_and_runtime_use_distinct_roots(self):
        calls = self.run_commands(True)
        scons = [
            item["argv"][1:]
            for item in calls
            if Path(item["argv"][0]).name == "scons"
        ]
        binary = next(
            args[0] for args in scons if args[0].endswith("gem5.opt")
        )
        library = next(
            args[0] for args in scons if args[0].endswith("libgem5_opt.so")
        )
        self.assertNotEqual(Path(binary).parent, Path(library).parent)
        self.assertIn(
            [
                "defconfig",
                "build/ARM-systemc-library",
                "build_opts/ARM",
                "--ignore-style",
            ],
            scons,
        )
        make = next(
            item for item in calls if Path(item["argv"][0]).name == "make"
        )
        self.assertEqual(make["argv"][1:], ["ARCH=ARM-systemc-library"])
        self.assertEqual(
            calls[-1]["library"],
            "build/ARM-systemc-library/:/opt/systemc/lib-linux64/",
        )
        dry_run = subprocess.check_output(
            ["make", "-Bn", "ARCH=ARM-systemc-library"],
            cwd=ROOT / "util/systemc/gem5_within_systemc",
            text=True,
        )
        self.assertIn("-I../../../build/ARM-systemc-library", dry_run)
        self.assertIn("-L../../../build/ARM-systemc-library", dry_run)

    def test_ordinary_job_keeps_original_build_and_library_paths(self):
        calls = self.run_commands(False)
        scons = [
            item["argv"][1:]
            for item in calls
            if Path(item["argv"][0]).name == "scons"
        ]
        self.assertEqual(sum(args[0] == "defconfig" for args in scons), 1)
        self.assertIn("build/ARM/libgem5_opt.so", [args[0] for args in scons])
        self.assertFalse(any("--gcov" in args for args in scons))
        self.assertEqual(
            calls[-1]["library"], "build/ARM/:/opt/systemc/lib-linux64/"
        )


if __name__ == "__main__":
    unittest.main()
