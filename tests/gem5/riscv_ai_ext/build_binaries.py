#!/usr/bin/env python3

# Copyright (c) 2026
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

import argparse
import os
import shutil
import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR / "src"
BIN_DIR = SCRIPT_DIR / "bin"
PROGRAMS = ("ai_ops_smoke", "p_lw_smoke", "hwloop_smoke")


def find_compiler() -> str:
    prefixes = []
    if os.environ.get("CROSS_COMPILE"):
        prefixes.append(os.environ["CROSS_COMPILE"])
    prefixes.extend(
        [
            "riscv64-linux-gnu-",
            "riscv64-unknown-linux-gnu-",
            "riscv64-unknown-elf-",
        ]
    )

    seen = set()
    for prefix in prefixes:
        if prefix in seen:
            continue
        seen.add(prefix)
        compiler = shutil.which(f"{prefix}gcc")
        if compiler:
            return compiler

    raise SystemExit(
        "No RISC-V cross-compiler found.\n"
        "Set CROSS_COMPILE or install one of:\n"
        "  riscv64-linux-gnu-gcc\n"
        "  riscv64-unknown-linux-gnu-gcc\n"
        "  riscv64-unknown-elf-gcc"
    )


def build_program(compiler: str, program: str) -> None:
    source = SRC_DIR / f"{program}.S"
    output = BIN_DIR / program
    command = [
        compiler,
        "-nostdlib",
        "-nostartfiles",
        "-static",
        "-no-pie",
        "-march=rv64gc",
        "-mabi=lp64d",
        "-I",
        str(SRC_DIR),
        "-Wl,--build-id=none",
        "-o",
        str(output),
        str(source),
    ]
    subprocess.run(command, check=True)
    print(f"Built {output}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build local RISC-V custom ISA test binaries."
    )
    parser.add_argument(
        "--clean", action="store_true", help="Remove built binaries and exit."
    )
    args = parser.parse_args()

    if args.clean:
        if BIN_DIR.exists():
            shutil.rmtree(BIN_DIR)
            print(f"Removed {BIN_DIR}")
        return

    BIN_DIR.mkdir(parents=True, exist_ok=True)
    compiler = find_compiler()
    print(f"Using compiler: {compiler}")

    for program in PROGRAMS:
        build_program(compiler, program)


if __name__ == "__main__":
    main()
