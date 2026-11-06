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

"""Check that coverage deployment files match a tested source revision."""

import argparse
import fnmatch
import json
import subprocess
from pathlib import Path

PATTERNS = (
    ".github/workflows/codecov.yaml",
    ".github/workflows/quick-tests.yaml",
    ".github/workflows/daily-tests.yaml",
    ".github/workflows/weekly-tests.yaml",
    ".github/workflows/scheduler.yaml",
    ".github/codecov.yml",
    ".github/coverage-native.json",
    "util/coverage/*.py",
    "ext/testlib/coverage.py",
)


def required(path):
    return any(
        fnmatch.fnmatchcase(path, pattern) for pattern in PATTERNS
    ) and (not Path(path).name.startswith("test_") and "/tests/" not in path)


def check(root, reference):
    root = Path(root).resolve()

    def git(*arguments):
        return subprocess.check_output(["git", *arguments], cwd=root)

    revision = (
        git(
            "rev-parse",
            "--verify",
            "--end-of-options",
            reference + "^{commit}",
        )
        .decode()
        .strip()
    )
    current = {
        path
        for path in git("ls-files", "-z").decode().split("\0")
        if path and required(path)
    }
    expected = {
        path
        for path in git("ls-tree", "-r", "--name-only", "-z", revision)
        .decode()
        .split("\0")
        if path and required(path)
    }
    differences = []
    for path in sorted(current | expected):
        local = root / path
        if path not in expected:
            reason = "absent from reference"
        elif not local.is_file():
            reason = "absent from checkout"
        elif local.read_bytes() != git("show", f"{revision}:{path}"):
            reason = "contents differ"
        else:
            continue
        differences.append({"path": path, "reason": reason})
    return {
        "reference": revision,
        "synchronized": not differences,
        "differences": differences,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--reference", required=True)
    args = parser.parse_args()
    result = check(args.root, args.reference)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["synchronized"] else 1)


if __name__ == "__main__":
    main()
