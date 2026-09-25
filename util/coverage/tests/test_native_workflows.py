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

"""Keep native report plans aligned with their reusable workflow jobs."""

import json
import re
import unittest
from pathlib import Path

import yaml


class NativeWorkflowPlanTest(unittest.TestCase):
    def test_all_collected_groups_and_execution_stages_match_the_plan(self):
        root = Path(__file__).parents[3]
        plan = json.loads((root / ".github/coverage-native.json").read_text())
        expected = {
            item["group"]: set(item["stages"]) for item in plan["groups"]
        }
        actual = {}
        for filename in (
            "quick-tests.yaml",
            "daily-tests.yaml",
            "weekly-tests.yaml",
        ):
            workflow = yaml.safe_load(
                (root / ".github/workflows" / filename).read_text()
            )
            for job in workflow["jobs"].values():
                steps = job.get("steps", [])
                for step in steps:
                    if step.get("uses") != (
                        "./.github/actions/collect-native-coverage"
                    ):
                        continue
                    values = step["with"]
                    group = values["group"]
                    variants = [None]
                    if "${{ matrix.type }}" in group:
                        variants = job["strategy"]["matrix"]["type"]
                    outcomes = values["outcomes"]
                    referenced = re.findall(
                        r"steps\.([a-zA-Z0-9_-]+)\.outcome", outcomes
                    )
                    available = {
                        s.get("id") for s in steps[: steps.index(step)]
                    }
                    self.assertTrue(set(referenced) <= available)
                    stages = set(
                        json.loads(
                            re.sub(r"\$\{\{.*?\}\}", "success", outcomes)
                        )
                    )
                    self.assertEqual(len(stages), len(referenced))
                    for variant in variants:
                        name = group.replace(
                            "${{ matrix.type }}", str(variant)
                        )
                        self.assertNotIn(name, actual)
                        actual[name] = stages
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
