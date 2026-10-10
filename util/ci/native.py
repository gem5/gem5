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

"""Run the native test groups shared by ordinary CI and coverage."""

import argparse
import json
import os
import subprocess
from dataclasses import (
    dataclass,
    field,
)
from pathlib import Path

CATALOG = Path(__file__).with_name("native.json")
ROOT = Path(__file__).resolve().parents[2]


@dataclass
class Stage:
    name: str
    command: list
    directory: str = "."
    environment: dict = field(default_factory=dict)


def groups():
    return json.loads(CATALOG.read_text())


def stages(group, coverage=False, jobs=1):
    """Keep integration-specific configuration beside its commands."""
    if group not in {item["group"] for item in groups()}:
        raise ValueError(f"Unknown native group: {group}")
    if jobs < 1:
        raise ValueError("jobs must be positive")

    def build(name, target, *flags):
        return Stage(
            name,
            [
                "scons",
                target,
                *flags,
                *(["--gcov"] if coverage else []),
                "-j",
                str(jobs),
            ],
        )

    clear = [Stage("clear-counters", [])] if coverage else []
    if group in {"unittests-fast", "unittests-opt", "unittests-debug"}:
        variant = group.removeprefix("unittests-")
        return [build("unit-tests", f"build/ALL/unittests.{variant}")]
    if group == "sst":
        return [
            Stage(
                "configure",
                [
                    "scons",
                    "defconfig",
                    "build/RISCV",
                    "build_opts/RISCV",
                    "--ignore-style",
                ],
            ),
            build(
                "build-library",
                "build/RISCV/libgem5_opt.so",
                "--without-tcmalloc",
                "--duplicate-sources",
                "--ignore-style",
            ),
            Stage(
                "prepare-makefile",
                ["mv", "Makefile.linux", "Makefile"],
                "ext/sst",
            ),
            Stage("build-integration", ["make", "-j", str(jobs)], "ext/sst"),
            *clear,
            Stage(
                "run-tests",
                ["sst", "--add-lib-path=./", "sst/example.py"],
                "ext/sst",
            ),
        ]
    if group == "systemc":
        # .o and .os outputs otherwise overwrite the same GCC .gcno files.
        library = "ARM-systemc-library" if coverage else "ARM"
        library_config = (
            [
                Stage(
                    "configure-library",
                    [
                        "scons",
                        "defconfig",
                        f"build/{library}",
                        "build_opts/ARM",
                        "--ignore-style",
                    ],
                )
            ]
            if coverage
            else []
        )
        return [
            Stage(
                "configure",
                [
                    "scons",
                    "defconfig",
                    "build/ARM",
                    "build_opts/ARM",
                    "--ignore-style",
                ],
            ),
            build(
                "build-binary",
                "build/ARM/gem5.opt",
                "--ignore-style",
                "--duplicate-sources",
            ),
            *library_config,
            Stage(
                "disable-systemc",
                [
                    "scons",
                    "setconfig",
                    f"build/{library}",
                    "--ignore-style",
                    "USE_SYSTEMC=n",
                ],
            ),
            build(
                "build-library",
                f"build/{library}/libgem5_opt.so",
                "--with-cxx-config",
                "--without-python",
                "--without-tcmalloc",
                "--duplicate-sources",
            ),
            Stage(
                "build-integration",
                ["make", f"ARCH={library}"],
                "util/systemc/gem5_within_systemc",
            ),
            *clear,
            Stage(
                "run-tests",
                [
                    "./build/ARM/gem5.opt",
                    "configs/deprecated/example/se.py",
                    "-c",
                    "tests/test-progs/hello/bin/arm/linux/hello",
                ],
            ),
            Stage(
                "run-continue",
                [
                    "./util/systemc/gem5_within_systemc/gem5.opt.sc",
                    "m5out/config.ini",
                ],
                environment={
                    "LD_LIBRARY_PATH": f"build/{library}/:/opt/systemc/lib-linux64/"
                },
            ),
        ]
    if group == "dramsys":
        return [
            Stage(
                "checkout-dramsys",
                [
                    "git",
                    "clone",
                    "https://github.com/tukl-msd/DRAMSys",
                    "--branch",
                    "v5.6.0",
                    "--depth",
                    "1",
                    "DRAMSys",
                ],
                "ext/dramsys",
            ),
            Stage(
                "configure",
                ["scons", "defconfig", "build/ALL", "build_opts/ALL"],
            ),
            build("build-binary", "build/ALL/gem5.opt"),
            *clear,
            *[
                Stage(name, ["./build/ALL/gem5.opt", config])
                for name, config in [
                    (
                        "arm-hello",
                        "configs/example/gem5_library/dramsys/"
                        "arm-hello-dramsys.py",
                    ),
                    (
                        "traffic",
                        "configs/example/gem5_library/dramsys/"
                        "dramsys-traffic.py",
                    ),
                    ("example", "configs/example/dramsys.py"),
                ]
            ],
        ]
    raise ValueError(f"Native group has no implementation: {group}")


def run(group, coverage, jobs, status, root=ROOT):
    """Stop on a failed build or test and retain explicit stage outcomes."""
    plan = stages(group, coverage, jobs)
    record = {
        "group": group,
        "coverage": coverage,
        "revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True
        ).strip(),
        "status": "running",
        "stages": {stage.name: "pending" for stage in plan},
    }
    status = Path(status)
    status.parent.mkdir(parents=True, exist_ok=True)

    def save():
        temporary = status.with_suffix(status.suffix + ".tmp")
        temporary.write_text(json.dumps(record, indent=2) + "\n")
        temporary.replace(status)

    save()
    result = 0
    try:
        for stage in plan:
            if result:
                record["stages"][stage.name] = "skipped"
                continue
            print(f"::group::{stage.name}", flush=True)
            if stage.name == "clear-counters":
                for path in (root / "build").rglob("*"):
                    if path.is_file() and (
                        path.suffix == ".gcda"
                        or path.name.endswith(".py.gcno")
                    ):
                        path.unlink()
            else:
                result = subprocess.run(
                    stage.command,
                    cwd=root / stage.directory,
                    env={**os.environ, **stage.environment},
                    check=False,
                ).returncode
            record["stages"][stage.name] = "failure" if result else "success"
            print("::endgroup::", flush=True)
            save()
    except (OSError, KeyboardInterrupt) as error:
        record["stages"][stage.name] = "failure"
        record["error"] = str(error)
        result = 1
    finally:
        record["stages"] = {
            name: "skipped" if outcome == "pending" else outcome
            for name, outcome in record["stages"].items()
        }
        record["status"] = "failure" if result else "success"
        save()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", choices=[item["group"] for item in groups()])
    parser.add_argument("--coverage", action="store_true")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    parser.add_argument("--status", type=Path, required=True)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    return run(args.group, args.coverage, args.jobs, args.status)


if __name__ == "__main__":
    raise SystemExit(main())
