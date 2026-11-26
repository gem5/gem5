# Copyright (c) 2026 The Regents of the University of California
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

"""Exercise the formatter wrapper in temporary Git repositories."""

import os
import subprocess
import sysconfig
import tempfile
import unittest
from pathlib import Path


class GitClangFormatTest(unittest.TestCase):
    wrapper = (
        Path(__file__).resolve().parents[2] / "util/run-git-clang-format.py"
    )
    # sys.executable may be gem5; use its matching Python installation.
    python = Path(sysconfig.get_config_var("BINDIR")) / (
        f"python{sysconfig.get_python_version()}"
    )

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.args_file = self.root / "formatter-args"
        self.env = os.environ.copy()
        for name in (
            "GEM5_NO_GIT_CLANG_FORMAT",
            "GEM5_GIT_CLANG_FORMAT_IGNORE",
            "GEM5_CI_PR_BASE_COMMIT",
        ):
            self.env.pop(name, None)
        self.env["GIT_AUTHOR_NAME"] = "Test"
        self.env["GIT_AUTHOR_EMAIL"] = "test@example.com"
        self.env["GIT_COMMITTER_NAME"] = "Test"
        self.env["GIT_COMMITTER_EMAIL"] = "test@example.com"
        self.env["FORMAT_ARGS"] = str(self.args_file)
        self.env["PATH"] = (
            str(self.root / "bin") + os.pathsep + self.env["PATH"]
        )
        self.git("init", "-q")
        self.write("src/with space.cc", "int value;\n")
        self.write("ext/vendor.cc", "int vendor;\n")
        self.git("add", ".")
        self.git("commit", "-qm", "Baseline")
        self.base = self.git("rev-parse", "HEAD").strip()
        formatter = self.write(
            "bin/git-clang-format",
            '#!/bin/sh\nprintf \'%s\\0\' "$@" > "$FORMAT_ARGS"\n',
        )
        formatter.chmod(0o755)

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def git(self, *args):
        return subprocess.check_output(
            [
                "git",
                "-c",
                "commit.gpgsign=false",
                "-c",
                f"core.hooksPath={os.devnull}",
                *args,
            ],
            cwd=self.root,
            env=self.env,
            text=True,
        )

    def run_wrapper(self, *args):
        result = subprocess.run(
            [str(self.python), str(self.wrapper), *args],
            cwd=self.root,
            env=self.env,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        if not self.args_file.exists():
            return None
        return self.args_file.read_text().rstrip("\0").split("\0")

    def test_pre_commit_uses_only_supplied_files(self):
        self.assertEqual(
            self.run_wrapper("--pre-commit", "src/with space.cc"),
            ["--staged", "--quiet", "--style=file", "--", "src/with space.cc"],
        )

    def test_ci_uses_supplied_files(self):
        self.env["GEM5_CI_PR_BASE_COMMIT"] = self.base
        self.assertEqual(
            self.run_wrapper("--pre-commit", "src/with space.cc"),
            [
                "--style=file",
                f"--commit={self.base}",
                "--",
                "src/with space.cc",
            ],
        )

    def test_empty_pre_commit_selection_skips_formatter(self):
        self.assertIsNone(self.run_wrapper("--pre-commit"))

    def test_all_supplied_files_ignored_skips_formatter(self):
        self.assertIsNone(
            self.run_wrapper(
                "--pre-commit", "--ignore", "src", "src/with space.cc"
            )
        )

    def test_environment_ignore_keeps_other_supplied_files(self):
        self.env["GEM5_GIT_CLANG_FORMAT_IGNORE"] = "ext/vendor.cc"
        self.assertEqual(
            self.run_wrapper(
                "--pre-commit", "src/with space.cc", "ext/vendor.cc"
            )[-2:],
            ["--", "src/with space.cc"],
        )

    def stage_changes(self):
        self.write("src/with space.cc", "int changed;\n")
        self.write("ext/vendor.cc", "int changed_vendor;\n")
        self.git("add", "src", "ext")

    def test_manual_ignore_discovers_staged_files(self):
        self.stage_changes()
        self.assertEqual(
            self.run_wrapper("--ignore", "ext"),
            ["--staged", "--quiet", "--style=file", "--", "src/with space.cc"],
        )

    def test_ci_ignore_discovers_committed_files(self):
        self.stage_changes()
        self.git("commit", "-qm", "Changes")
        self.assertEqual(
            self.run_wrapper(
                "--ci-pr-base-commit", self.base, "--ignore", "ext"
            ),
            [
                "--style=file",
                f"--commit={self.base}",
                "--",
                "src/with space.cc",
            ],
        )

    def test_manual_without_filenames_keeps_staged_mode(self):
        self.assertEqual(
            self.run_wrapper(), ["--staged", "--quiet", "--style=file"]
        )

    def test_global_disable_skips_formatter(self):
        self.env["GEM5_NO_GIT_CLANG_FORMAT"] = "1"
        self.assertIsNone(
            self.run_wrapper("--pre-commit", "src/with space.cc")
        )


if __name__ == "__main__":
    unittest.main()
