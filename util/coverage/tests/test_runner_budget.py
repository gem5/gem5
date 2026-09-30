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

"""Check the campaign's runner budget and ordinary workflow behavior."""

import copy
import io
import math
import re
import tokenize
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).parents[3]


def workflow(name):
    return yaml.safe_load((ROOT / ".github/workflows" / name).read_text())


def expression(value, coverage, phase="testlib", needs=None, cancelled=False):
    if not isinstance(value, str) or not value.startswith("${{"):
        return value
    source = value[3:-2].strip()
    source = re.sub(r"inputs\.([\w-]+)", r'inputs["\1"]', source)
    source = re.sub(
        r"needs\.([\w-]+)\.outputs\.([\w-]+)",
        r'needs["\1"]["outputs"]["\2"]',
        source,
    )
    source = re.sub(
        r"needs\.([\w-]+)\.result", r'needs["\1"]["result"]', source
    )
    source = source.replace("&&", " and ").replace("||", " or ")
    source = re.sub(r"!(?!=)", "not ", source)
    source = tokenize.untokenize(
        (
            token._replace(
                string={"true": "True", "false": "False"}.get(
                    token.string, token.string
                )
            )
            if token.type == tokenize.NAME
            else token
        )
        for token in tokenize.generate_tokens(io.StringIO(source).readline)
    )
    return eval(
        source,
        {"__builtins__": {}},
        {
            "inputs": {"coverage": coverage, "coverage-phase": phase},
            "needs": needs or {},
            "cancelled": lambda: cancelled,
            "always": lambda: True,
            "success": lambda: True,
        },
    )


def dependencies(job):
    value = job.get("needs", [])
    return [value] if isinstance(value, str) else value


def active_jobs(data, coverage, phase):
    needs = {
        name: {"result": "success", "outputs": {"allowed": "true"}}
        for name in data["jobs"]
    }
    # Resolve skipped producer jobs before their downstream conditions.
    # Explicit status functions let coverage consumers use shared builds.
    for _ in range(len(needs)):
        changed = False
        for name, job in data["jobs"].items():
            condition = job.get("if", True)
            enabled = bool(expression(condition, coverage, phase, needs))
            if not isinstance(condition, str) or not re.search(
                r"\b(cancelled|always|success|failure)\(", condition
            ):
                enabled &= all(
                    needs[parent]["result"] == "success"
                    for parent in dependencies(job)
                )
            result = "success" if enabled else "skipped"
            changed |= result != needs[name]["result"]
            needs[name]["result"] = result
        if not changed:
            break
    return {
        name: data["jobs"][name]
        for name, state in needs.items()
        if state["result"] == "success"
    }


def width(job, coverage):
    strategy = job.get("strategy", {})
    axes = strategy.get("matrix", {})
    # A large discovered matrix makes max-parallel the binding limit.
    count = math.prod(
        len(axis) if isinstance(axis, list) else 256 for axis in axes.values()
    )
    return min(count, expression(strategy.get("max-parallel", 256), coverage))


def self_hosted(job):
    selection = job.get("runs-on")
    if selection is None and "uses" in job:
        return False
    if isinstance(selection, dict):
        # Conservatively count runner groups as consuming the shared budget.
        return True
    labels = selection if isinstance(selection, list) else [selection]
    if "self-hosted" in [str(label).lower() for label in labels]:
        return True
    hosted = {"ubuntu-24.04", "macos-26"}
    if all(label in hosted for label in labels):
        return False
    raise AssertionError(f"Unclassified runner selection: {selection!r}")


class RunnerBudgetTest(unittest.TestCase):
    def test_entire_campaign_has_four_runner_bound(self):
        coordinator = workflow("codecov.yaml")
        builds = coordinator["jobs"]["coverage-builds"]
        self.assertEqual(width(builds, True), 4)
        phases = ("quick-native", "quick", "daily-native", "daily", "weekly")
        self.assertEqual(
            {
                name
                for name, job in coordinator["jobs"].items()
                if self_hosted(job)
            },
            {"coverage-builds"},
        )
        self.assertEqual(
            {
                name
                for name, job in coordinator["jobs"].items()
                if "uses" in job
            },
            set(phases),
        )
        previous = "coverage-builds"
        for name in phases:
            with self.subTest(phase=name):
                call = coordinator["jobs"][name]
                self.assertIn(previous, dependencies(call))
                self.assertTrue(call["with"]["coverage"])
                data = workflow(call["uses"].rsplit("/", 1)[-1])
                phase = call["with"].get("coverage-phase", "testlib")
                active = active_jobs(data, True, phase)
                pool_jobs = {
                    key: job for key, job in active.items() if self_hosted(job)
                }
                total = sum(width(job, True) for job in pool_jobs.values())
                self.assertGreater(total, 0)
                self.assertLessEqual(total, 4, pool_jobs.keys())
                self.assertEqual(total, 2 if name == "quick-native" else 4)
                previous = name
        concurrency = coordinator["concurrency"]
        self.assertEqual(
            concurrency["group"], "codecov-${{ github.repository }}"
        )
        self.assertFalse(concurrency["cancel-in-progress"])
        self.assertEqual(concurrency["queue"], "max")

    def test_extra_runner_labels_and_groups_cannot_escape_the_budget(self):
        original = workflow("daily-tests.yaml")
        for selection in (
            ["self-hosted", "linux", "x64", "gpu"],
            "self-hosted",
            {"group": "shared", "labels": "gpu"},
        ):
            data = copy.deepcopy(original)
            extra = copy.deepcopy(data["jobs"]["systemc-test"])
            extra["runs-on"] = selection
            data["jobs"]["additional-native-job"] = extra
            active = active_jobs(data, True, "native")
            total = sum(
                width(job, True) for job in active.values() if self_hosted(job)
            )
            self.assertEqual(total, 5)
        with self.assertRaisesRegex(AssertionError, "Unclassified runner"):
            self_hosted({"runs-on": "${{ inputs.runner }}"})

    def test_each_coverage_job_runs_in_exactly_one_phase(self):
        expected = {
            "quick-tests.yaml": {
                "native": {"unittests-all-fast-opt"},
                "testlib": {"testlib-quick-execution"},
            },
            "daily-tests.yaml": {
                "native": {
                    "unittests-debug",
                    "sst-test",
                    "systemc-test",
                    "dramsys-tests",
                },
                "testlib": {"testlib-long-tests"},
            },
        }
        for name, phases in expected.items():
            data = workflow(name)
            self.assertEqual(
                data[True]["workflow_call"]["inputs"]["coverage-phase"][
                    "default"
                ],
                "testlib",
            )
            for phase, jobs in phases.items():
                active = active_jobs(data, True, phase)
                actual = {
                    key for key, job in active.items() if self_hosted(job)
                }
                self.assertEqual(actual, jobs)
                discovery = (
                    "testlib-quick-matrix"
                    if name.startswith("quick")
                    else "get-testlib-long-dirs"
                )
                self.assertEqual(discovery in active, phase == "testlib")

    def test_ordinary_runs_ignore_the_phase_and_keep_parallelism(self):
        expected = {
            "quick-tests.yaml": {
                "unittests-all-fast-opt",
                "testlib-quick-gem5-builds",
                "testlib-quick-execution",
            },
            "daily-tests.yaml": {
                "build-clang-gem5-fast",
                "build-testlib-gem5",
                "unittests-debug",
                "testlib-long-tests",
                "sst-test",
                "systemc-test",
                "dramsys-tests",
            },
            "weekly-tests.yaml": {"testlib-very-long-tests"},
        }
        for name, jobs in expected.items():
            data = workflow(name)
            for phase in ("native", "testlib", "ignored"):
                active = active_jobs(data, False, phase)
                actual = {
                    key for key, job in active.items() if self_hosted(job)
                }
                self.assertEqual(actual, jobs)
            testlib = next(
                job
                for key, job in data["jobs"].items()
                if key
                in {
                    "testlib-quick-execution",
                    "testlib-long-tests",
                    "testlib-very-long-tests",
                }
            )
            expected_width = 5 if name.startswith("weekly") else 256
            self.assertEqual(width(testlib, False), expected_width)
        self.assertEqual(
            workflow("daily-tests.yaml")["jobs"]["build-testlib-gem5"][
                "strategy"
            ]["max-parallel"],
            2,
        )
        self.assertEqual(
            dependencies(
                workflow("quick-tests.yaml")["jobs"]["testlib-quick-execution"]
            ),
            ["testlib-quick-matrix", "testlib-quick-gem5-builds"],
        )
        self.assertEqual(
            dependencies(
                workflow("daily-tests.yaml")["jobs"]["testlib-long-tests"]
            ),
            ["build-testlib-gem5", "get-testlib-long-dirs", "get-date"],
        )

    def test_failed_phase_does_not_suppress_later_collection(self):
        coordinator = workflow("codecov.yaml")
        for name in (
            "quick-native",
            "quick",
            "daily-native",
            "daily",
            "weekly",
        ):
            call = coordinator["jobs"][name]
            needs = {
                parent: {"result": "success", "outputs": {"allowed": "true"}}
                for parent in dependencies(call)
            }
            predecessor = dependencies(call)[-1]
            for result in ("failure", "skipped"):
                needs[predecessor]["result"] = result
                self.assertTrue(expression(call["if"], True, needs=needs))
                self.assertFalse(
                    expression(call["if"], True, needs=needs, cancelled=True)
                )
        report = coordinator["jobs"]["report"]
        self.assertIn("quick-native", dependencies(report))
        self.assertIn("daily-native", dependencies(report))


if __name__ == "__main__":
    unittest.main()
