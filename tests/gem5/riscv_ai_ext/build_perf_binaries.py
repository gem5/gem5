#!/usr/bin/env python3

import argparse
import os
import shutil
import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR / "perf_src"
BIN_DIR = SCRIPT_DIR / "perf_bin"
PROGRAMS = (
    "dot4_pipeline_scalar",
    "dot4_pipeline_custom",
    "mac_pipeline_scalar",
    "mac_pipeline_custom",
)


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
    source = SRC_DIR / f"{program}.cpp"
    output = BIN_DIR / program
    command = [
        compiler,
        "-x",
        "c++",
        "-std=c++17",
        "-O3",
        "-ffreestanding",
        "-fno-builtin",
        "-fno-exceptions",
        "-fno-rtti",
        "-fno-threadsafe-statics",
        "-fno-use-cxa-atexit",
        "-fno-stack-protector",
        "-nostdlib",
        "-nostartfiles",
        "-static",
        "-no-pie",
        "-march=rv64gc",
        "-mabi=lp64d",
        "-I",
        str(SRC_DIR),
        "-Wl,--build-id=none",
        "-Wl,-e,_start",
        "-o",
        str(output),
        str(source),
    ]
    subprocess.run(command, check=True)
    print(f"Built {output}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build local RISC-V performance benchmark binaries."
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
