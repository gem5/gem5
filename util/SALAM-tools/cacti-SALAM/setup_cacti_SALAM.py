#!/usr/bin/env python3

# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

import os
import stat
import subprocess
import sys
import time

M5_PATH_ENV_VAR = "M5_PATH"
ACC_BENCH_PATH_ENV_VAR = "ACC_BENCH_PATH"
TARGET_SUBDIR = "ext/mcpat/cacti"
CACTI_INFILE = "cache.cfg"


def run_command(command_list, working_dir):
    command_str = " ".join(command_list)
    print(f"Running: '{command_str}' in '{working_dir}'")
    result = subprocess.run(
        command_list, cwd=working_dir, capture_output=True, text=True
    )
    return result


def main():
    m5_path = os.environ.get(M5_PATH_ENV_VAR)
    acc_bench_path = os.environ.get(ACC_BENCH_PATH_ENV_VAR)
    if not m5_path or not acc_bench_path:
        print(
            f"Error: Environment variables M5_PATH and/or "
            f"ACC_BENCH_PATH not set.",
            file=os.sys.stderr,
        )
        exit(1)

    cacti_dir = os.path.join(m5_path, TARGET_SUBDIR)
    print(f"CACTI directory: {cacti_dir}")

    # Build cacti
    run_command(["make", "clean"], working_dir=cacti_dir)
    run_command(["make", "all"], working_dir=cacti_dir)
    time.sleep(1)

    # Run cacti
    cacti_executable = os.path.join(
        "./cacti"
    )  # Relative path within cacti_dir
    cacti_result = run_command(
        [cacti_executable, "-infile", CACTI_INFILE], working_dir=cacti_dir
    )

    if cacti_result.stdout:
        print(cacti_result.stdout.strip())
    if cacti_result.stderr:
        print(cacti_result.stderr.strip(), file=sys.stderr)

    print(f"CACTI exited with code: {cacti_result.returncode}\n")

    print("==================================================================")
    print("Next, to run cacti-SALAM use run_cacti_salam.py\n")
    print("Usage: ./run_cacti_salam.py <path to benchmark configs list>")
    print(
        "Each line of benchmark configs list contains"
        " </path/to/config> <benchmark name> <config>"
    )
    print("<benchmark name> and <config> are only used for grouping")
    print("==================================================================")


if __name__ == "__main__":
    main()
