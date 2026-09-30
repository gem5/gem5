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

"""Shared discovery and execution of ordinary and instrumented TestLib tests."""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LENGTHS = ("quick", "long", "very-long")
LEGACY_DOWNLOADS = {
    "gem5/cpu_tests",
    "gem5/fdp_tests",
    "gem5/m5threads_test_atomic",
}


def list_command(length, coverage=False, directory="gem5", targets=False):
    command = [
        sys.executable,
        "main.py",
        "list",
        directory,
        f"--length={length}",
        "-q",
    ]
    command.append("--build-targets" if targets else "--suites")
    if coverage:
        command.extend(
            ["--host=x86_64", "--gcov=per-test", "--python-coverage"]
        )
    elif targets and length == "long":
        command.append("--host=x86_64")
    return command


def discover(length, coverage=False, directory="gem5", root=ROOT):
    suites = subprocess.check_output(
        list_command(length, coverage, directory),
        cwd=root / "tests",
        text=True,
    )
    targets = subprocess.check_output(
        list_command(length, coverage, directory, True),
        cwd=root / "tests",
        text=True,
    )
    groups = {}
    for uid in suites.splitlines():
        if not uid.startswith("SuiteUID:"):
            continue
        filename = Path(uid.split(":")[1])
        if not filename.is_absolute():
            filename = root / filename
        name = filename.parent.relative_to(root / "tests").as_posix()
        groups.setdefault(name, []).append(uid)
    if not groups:
        raise ValueError(f"No TestLib suites found for {length}")
    return {
        "directories": sorted(groups),
        "suites": groups,
        "targets": sorted(set(targets.splitlines())),
    }


def run_command(directory, length, jobs, coverage=False, skip_build=False):
    command = [sys.executable, "main.py", "run", directory]
    if skip_build:
        command.append("--skip-build")
    if length == "quick" and not coverage:
        command.extend(["-t", str(jobs)])
    else:
        command.extend([f"--length={length}", f"-j{jobs}", "-vv"])
        if coverage or length == "long":
            command.extend(["-t", "1" if coverage else str(jobs)])
        resource = os.environ.get("GEM5_RESOURCE_DIR", "/gem5-resource-cache")
        if directory in LEGACY_DOWNLOADS:
            resource = str(
                Path(os.environ["RUNNER_TEMP"]) / "gem5-testlib-downloads"
            )
        command.extend(["--bin-path", resource])
    if coverage:
        command.extend(
            ["--host=x86_64", "--gcov=per-test", "--python-coverage"]
        )
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("discover")
    running = commands.add_parser("run")
    for child in (listing, running):
        child.add_argument("--length", choices=LENGTHS, required=True)
        child.add_argument("--coverage", action="store_true")
        child.add_argument("--directory", default="gem5")
    running.add_argument("--skip-build", action="store_true")
    running.add_argument("--jobs", type=int, default=os.cpu_count())
    running.add_argument("--task-id")
    running.add_argument("--status", type=Path)
    args = parser.parse_args()
    if args.command == "discover":
        print(json.dumps(discover(args.length, args.coverage, args.directory)))
        return
    status = {
        "revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "task_id": args.task_id,
        "stages": {"test": "failure"},
    }
    try:
        result = subprocess.run(
            run_command(
                args.directory,
                args.length,
                args.jobs,
                args.coverage,
                args.skip_build,
            ),
            cwd=ROOT / "tests",
        )
        status["stages"]["test"] = (
            "success" if result.returncode == 0 else "failure"
        )
    finally:
        if args.status:
            args.status.parent.mkdir(parents=True, exist_ok=True)
            args.status.write_text(json.dumps(status, indent=2) + "\n")
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
