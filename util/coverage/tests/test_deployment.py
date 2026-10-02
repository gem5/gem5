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

"""Exercise deployment comparisons against actual Git revisions."""

import importlib.util
import subprocess
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "coverage_deployment", Path(__file__).parents[1] / "deployment.py"
)
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)


class DeploymentTest(unittest.TestCase):
    def test_reports_changed_added_and_missing_runtime_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def git(*args):
                return subprocess.check_output(["git", *args], cwd=root)

            git("init", "-q")
            git("config", "user.name", "Coverage test")
            git("config", "user.email", "coverage@example.invalid")
            files = {
                ".github/workflows/codecov.yaml": "name: coverage\n",
                "util/coverage/report.py": "report = 1\n",
                "util/coverage/old.py": "old = 1\n",
                "util/coverage/test_old.py": "test = 1\n",
            }
            for path, content in files.items():
                target = root / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(content)
            git("add", ".")
            git("-c", "core.hooksPath=/dev/null", "commit", "-qm", "fixture")
            self.assertTrue(deployment.check(root, "HEAD")["synchronized"])
            (root / "util/coverage/report.py").write_text("report = 2\n")
            (root / "util/coverage/test_old.py").write_text("test = 2\n")
            (root / "util/coverage/old.py").unlink()
            (root / "util/coverage/new.py").write_text("new = 1\n")
            git("add", ".")
            result = deployment.check(root, "HEAD")
            self.assertFalse(result["synchronized"])
            self.assertEqual(
                {item["path"] for item in result["differences"]},
                {
                    "util/coverage/report.py",
                    "util/coverage/old.py",
                    "util/coverage/new.py",
                },
            )


if __name__ == "__main__":
    unittest.main()
