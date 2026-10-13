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

"""Verify the coverage runner budget and shared ordinary workloads."""

import importlib.util
import json
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[3]
WORKFLOWS = ROOT / ".github/workflows"


def workflow(name):
    return yaml.safe_load((WORKFLOWS / name).read_text())


def commands(job):
    return "\n".join(step.get("run", "") for step in job.get("steps", []))


class CoverageWorkflows(unittest.TestCase):
    def test_exactly_three_sequential_shared_runner_matrices(self):
        jobs = workflow("codecov.yaml")["jobs"]

        def pool_job(job):
            selection = job.get("runs-on")
            if selection is None and "uses" in job:
                return False
            if isinstance(selection, dict):
                return True
            labels = selection if isinstance(selection, list) else [selection]
            if "self-hosted" in [str(label).lower() for label in labels]:
                return True
            if all(label == "ubuntu-24.04" for label in labels):
                return False
            raise AssertionError(f"Unknown runner selection: {selection!r}")

        self_hosted = {name for name, job in jobs.items() if pool_job(job)}
        self.assertEqual(self_hosted, {"builds", "native", "tests"})
        for name in self_hosted:
            job = jobs[name]
            self.assertNotIn("uses", job)
            self.assertEqual(job["runs-on"], ["self-hosted", "linux", "x64"])
            self.assertEqual(job["strategy"]["max-parallel"], 4)
            self.assertFalse(job["strategy"]["fail-fast"])
        self.assertEqual(jobs["builds"]["needs"], "plan")
        self.assertEqual(jobs["native"]["needs"], ["plan", "builds"])
        self.assertEqual(jobs["tests"]["needs"], ["plan", "native"])
        self.assertIn("!cancelled()", jobs["native"]["if"])
        self.assertIn("!cancelled()", jobs["tests"]["if"])
        self.assertEqual(
            workflow("codecov.yaml")["concurrency"]["cancel-in-progress"],
            False,
        )

    def test_native_catalog_used_by_plan_and_execution(self):
        catalog = json.loads((ROOT / "util/ci/native.json").read_text())
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
        job = workflow("codecov.yaml")["jobs"]["native"]
        self.assertIn(
            "fromJSON(needs.plan.outputs.native)",
            job["strategy"]["matrix"]["include"],
        )
        self.assertIn("util/ci/native.py", commands(job))
        self.assertIn("--coverage", commands(job))
        self.assertEqual(job["container"]["image"], "${{ matrix.image }}")

    def test_ordinary_native_workloads_use_same_executor(self):
        ci = workflow("ci-tests.yaml")["jobs"]
        daily = workflow("daily-tests.yaml")["jobs"]
        unit = ci["unittests-all-fast-opt"]
        self.assertEqual(unit["strategy"]["matrix"]["type"], ["fast", "opt"])
        self.assertEqual(unit["needs"], ["pre-commit"])
        self.assertIn("draft == false", unit["if"])
        self.assertEqual(
            daily["unittests-debug"]["strategy"]["matrix"]["type"], ["debug"]
        )
        for job in [
            unit,
            daily["unittests-debug"],
            daily["sst-test"],
            daily["systemc-test"],
            daily["dramsys-tests"],
        ]:
            self.assertIn("util/ci/native.py", commands(job))
            self.assertIn('--jobs "$(nproc)"', commands(job))
            self.assertNotIn("--coverage", commands(job))
        for name in ["sst-test", "dramsys-tests"]:
            self.assertEqual(daily[name]["needs"], "get-date")
            self.assertTrue(
                any(
                    step.get("uses", "").startswith("actions/cache")
                    for step in daily[name]["steps"]
                )
            )
        self.assertFalse(
            any(
                step.get("uses", "").startswith("actions/cache")
                for step in daily["systemc-test"]["steps"]
            )
        )

    def test_shared_testlib_commands_keep_ordinary_parallelism(self):
        pairs = [
            (
                "ci-tests.yaml",
                "testlib-quick-matrix",
                "testlib-quick-execution",
                "quick",
                True,
                360,
            ),
            (
                "daily-tests.yaml",
                "get-testlib-long-dirs",
                "testlib-long-tests",
                "long",
                True,
                1440,
            ),
            (
                "weekly-tests.yaml",
                "get-testlib-very-long-dirs",
                "testlib-very-long-tests",
                "very-long",
                False,
                4320,
            ),
        ]
        for filename, discovery, run, length, skip_build, timeout in pairs:
            with self.subTest(filename=filename):
                jobs = workflow(filename)["jobs"]
                self.assertIn(
                    "util/ci/testlib.py discover", commands(jobs[discovery])
                )
                text = commands(jobs[run])
                self.assertIn("util/ci/testlib.py run", text)
                self.assertIn(f"--length {length}", text)
                self.assertEqual("--skip-build" in text, skip_build)
                self.assertNotIn("--coverage", text)
                self.assertNotIn("gcov", text)
                self.assertEqual(jobs[run]["timeout-minutes"], timeout)
                self.assertIn(discovery, jobs[run]["needs"])
        weekly = workflow("weekly-tests.yaml")["jobs"][
            "testlib-very-long-tests"
        ]
        self.assertEqual(weekly["strategy"]["max-parallel"], 5)
        self.assertNotIn(
            "max-parallel",
            workflow("daily-tests.yaml")["jobs"]["testlib-long-tests"][
                "strategy"
            ],
        )
        self.assertNotIn(
            "max-parallel",
            workflow("ci-tests.yaml")["jobs"]["testlib-quick-execution"][
                "strategy"
            ],
        )
        coverage = commands(workflow("codecov.yaml")["jobs"]["tests"])
        self.assertIn("util/ci/testlib.py run", coverage)
        self.assertIn("--coverage --skip-build", coverage)

    def test_ordinary_workflows_have_no_coverage_orchestration(self):
        for filename in [
            "ci-tests.yaml",
            "daily-tests.yaml",
            "weekly-tests.yaml",
        ]:
            with self.subTest(filename=filename):
                text = (WORKFLOWS / filename).read_text()
                self.assertNotIn("coverage-phase", text)
                self.assertNotIn("inputs.coverage", text)
                self.assertNotIn("GCOV_FLAGS", text)
                self.assertNotIn("codecov/codecov-action", text)
        self.assertFalse((WORKFLOWS / "ci-daily-codecov.yaml").exists())
        self.assertNotIn(
            "ci-daily-codecov", (WORKFLOWS / "scheduler.yaml").read_text()
        )

    def test_plan_retains_known_exclusion_and_does_not_build_its_targets(self):
        spec = importlib.util.spec_from_file_location(
            "coverage_workflow_plan", ROOT / "util/coverage/plan.py"
        )
        plan = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(plan)
        calls = []

        def discover(length, coverage=False, directory="gem5"):
            calls.append((length, coverage, directory))
            if directory != "gem5":
                return {
                    "targets": ["build/ALL/gem5.opt"],
                    "suites": {directory: ["SuiteUID:example"]},
                }
            selected = {"gem5/example": [f"SuiteUID:{length}:example"]}
            if length == "very-long":
                selected["gem5/x86_boot_tests"] = ["SuiteUID:known-failure"]
            return {"suites": selected, "targets": ["build/ALL/gem5.opt"]}

        catalog = json.loads((ROOT / "util/ci/native.json").read_text())
        images = {
            item["image"]: item["image"] + "@sha256:fixed" for item in catalog
        }
        with (
            patch.object(plan, "discover", side_effect=discover),
            patch.object(
                plan.subprocess, "check_output", return_value="revision"
            ),
        ):
            result = plan.plan(images)
        self.assertEqual(
            {task["length"] for task in result["tests"]},
            {"quick", "long", "very-long"},
        )
        self.assertEqual(result["targets"], ["build/ALL/gem5.opt"])
        self.assertEqual(len(result["exclusions"]), 1)
        self.assertEqual(
            result["exclusions"][0]["suites"], ["SuiteUID:known-failure"]
        )
        self.assertNotIn(("very-long", True, "gem5/x86_boot_tests"), calls)
        self.assertEqual(
            {item["group"] for item in result["native"]},
            {item["group"] for item in catalog},
        )

    def test_draft_infrastructure_check_and_separate_report_recovery(self):
        ci = workflow("ci-tests.yaml")["jobs"]
        checks = ci["coverage-tools"]
        self.assertNotIn("if", checks)
        self.assertEqual(checks["runs-on"], "ubuntu-24.04")
        self.assertIn("coverage-tools", ci["ci-tests"]["needs"])
        report = workflow("coverage-report.yaml")["jobs"]
        self.assertEqual(set(report), {"report"})
        self.assertEqual(report["report"]["runs-on"], "ubuntu-24.04")
        recovery = workflow("coverage-recovery.yaml")["jobs"]
        self.assertEqual(set(recovery), {"report"})
        self.assertEqual(
            recovery["report"]["uses"],
            "./.github/workflows/coverage-report.yaml",
        )
        self.assertEqual(
            workflow("codecov.yaml")["jobs"]["report"]["uses"],
            recovery["report"]["uses"],
        )


if __name__ == "__main__":
    unittest.main()
