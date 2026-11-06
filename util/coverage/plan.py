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

"""Freeze the expected workloads for one coverage campaign."""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ci"))
from testlib import (
    LENGTHS,
    ROOT,
    discover,
)


def plan(images):
    tasks, targets, exclusions = [], set(), []
    for length in LENGTHS:
        selected = discover(length, coverage=True)
        for directory, suites in selected["suites"].items():
            if length == "very-long" and directory == "gem5/x86_boot_tests":
                exclusions.append(
                    {
                        "length": length,
                        "test_dir": directory,
                        "suites": suites,
                        "reason": "Known GCC instrumentation failure",
                    }
                )
                continue
            identity = hashlib.sha256(
                f"{length}:{directory}".encode()
            ).hexdigest()[:16]
            tasks.append(
                {
                    "id": identity,
                    "length": length,
                    "test_dir": directory,
                    "suites": suites,
                }
            )
            # Avoid building configurations only used by excluded workloads.
            targets.update(discover(length, True, directory)["targets"])
    catalog = json.loads((ROOT / "util/ci/native.json").read_text())
    native = [dict(item, image=images[item["image"]]) for item in catalog]
    targets = sorted(
        (
            Path(target).resolve().relative_to(ROOT).as_posix()
            if Path(target).is_absolute()
            else target
        )
        for target in targets
    )
    if not tasks or not targets:
        raise ValueError("Coverage discovery produced an empty test plan")
    return {
        "revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "image": images["ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest"],
        "targets": targets,
        "native": native,
        "tests": tasks,
        "exclusions": exclusions,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    value = plan(json.loads(args.images.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(value, indent=2) + "\n")
    # GitHub matrix include has a documented 256-job limit.
    if len(value["tests"]) > 256:
        raise ValueError("Coverage plan exceeds GitHub's matrix limit")


if __name__ == "__main__":
    main()
