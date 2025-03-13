# Copyright (c) 2025 The Regents of the University of California
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
import subprocess

"""
This script will run the provided workload with several different gem5 configurations.
All the configurations run the workload till the Kernel is booted (hypercall 1).
This tests are run to make sure that the gem5 can boot the full system workloads with
various CPU types and cache hierarchies.
"""


# Path to gem5 binary
GEM5_BINARY = "./build/ALL/gem5.opt"

# Path to the config script
GEM5_CONFIG = "util/disk-image-validator/config_tester.py"

# List of configuration tests to run
CONFIG_TESTS = [
    "KVM_test",
    "MESI_cache_test",
    "O3_test",
    "MINOR_test",
    "ATOMIC_test",
]


def run_config_test(test_name, workload_id, resource_version):
    """
    Runs a specific configuration test using gem5.
    """
    output_dir = (
        f"m5out/{workload_id}-{test_name}"  # Output directory for gem5
    )
    os.makedirs(output_dir, exist_ok=True)  # Ensure the directory exists

    print(f"Running {test_name} with gem5...")

    try:
        subprocess.run(
            [
                GEM5_BINARY,
                "-d",
                output_dir,
                "-re",
                GEM5_CONFIG,
                "--config",
                test_name,
                "--workload",
                workload_id,
                "--resource_version",
                resource_version,
            ],
            check=True,
        )
        print(f"{test_name} PASSED\n")
        return True
    except subprocess.CalledProcessError:
        print(f"{test_name} FAILED\n")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Run gem5 configuration tests."
    )
    parser.add_argument("--workload", required=True, help="The workload ID")
    parser.add_argument(
        "--resource_version", required=True, help="The resource version"
    )

    args = parser.parse_args()

    print(
        f"Running gem5 configuration tests for Workload ID: {args.workload}, Version: {args.resource_version}\n"
    )

    # Run configuration tests using gem5
    results = {
        test_name: run_config_test(
            test_name, args.workload, args.resource_version
        )
        for test_name in CONFIG_TESTS
    }

    # Print summary
    print("\n===== Test Summary =====")
    for test, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"{test}: {status}")

    # Exit with an error code if any test fails
    if not all(results.values()):
        print("\nSome configuration tests failed!")
        exit(1)
    else:
        print("\nAll configuration tests passed successfully!")
        exit(0)


if __name__ == "__main__":
    main()
