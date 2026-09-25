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

"""Execute ordinary and coverage workflow commands with a recording harness."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


class WorkloadCommandsTest(unittest.TestCase):
    def test_ordinary_and_coverage_selection(self):
        root = Path(__file__).parents[3]
        workloads = (
            ("quick-tests.yaml", "testlib-quick-execution"),
            ("daily-tests.yaml", "testlib-long-tests"),
            ("weekly-tests.yaml", "testlib-very-long-tests"),
        )
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            main = work / "main.py"
            main.write_text(
                "#!" + sys.executable + "\n"
                "import json, sys\nprint(json.dumps(sys.argv[1:]))\n"
            )
            main.chmod(0o755)
            nproc = work / "nproc"
            nproc.write_text("#!/bin/sh\necho 8\n")
            nproc.chmod(0o755)
            for filename, job in workloads:
                workflow = yaml.safe_load(
                    (root / ".github/workflows" / filename).read_text()
                )
                script = next(
                    step["run"]
                    for step in workflow["jobs"][job]["steps"]
                    if step.get("id") == "run-tests"
                )
                for enabled in (False, True):
                    for testdir in (
                        "gem5/gpu",
                        "gem5/x86_boot_tests",
                        "gem5/cpu_tests",
                    ):
                        with self.subTest(
                            workload=filename,
                            coverage=enabled,
                            testdir=testdir,
                        ):
                            env = dict(
                                os.environ,
                                PATH=str(work)
                                + os.pathsep
                                + os.environ["PATH"],
                                GCOV_FLAGS="--gcov" if enabled else "",
                                TEST_DIR=testdir,
                                GEM5_RESOURCE_DIR="/shared",
                                RUNNER_TEMP=str(work),
                                GITHUB_STEP_SUMMARY=str(work / "summary"),
                            )
                            command = script.replace(
                                "${{ matrix.test-type }}", testdir
                            ).replace("${{ matrix.test-dir }}", testdir)
                            result = subprocess.run(
                                ["bash", "-e"],
                                input=command,
                                cwd=work,
                                env=env,
                                text=True,
                                check=True,
                                capture_output=True,
                            )
                            args = json.loads(result.stdout.splitlines()[-1])
                            excluded = (
                                filename == "weekly-tests.yaml"
                                and testdir == "gem5/x86_boot_tests"
                            )
                            self.assertEqual(
                                "--gcov=per-test" in args,
                                enabled and not excluded,
                            )
                            self.assertEqual(
                                "--skip-build" in args,
                                filename != "weekly-tests.yaml"
                                or (enabled and not excluded),
                            )
                            if filename != "weekly-tests.yaml":
                                self.assertEqual(
                                    args[args.index("-t") + 1],
                                    "1" if enabled else "8",
                                )
                            if filename == "quick-tests.yaml" and not enabled:
                                self.assertNotIn("--bin-path", args)
                                continue
                            resource = args[args.index("--bin-path") + 1]
                            self.assertEqual(
                                resource,
                                (
                                    str(work / "gem5-testlib-downloads")
                                    if testdir == "gem5/cpu_tests"
                                    else (
                                        "/gem5-resource-cache"
                                        if filename == "quick-tests.yaml"
                                        else "/shared"
                                    )
                                ),
                            )


if __name__ == "__main__":
    unittest.main()
