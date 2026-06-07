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

import argparse
import csv
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

# Configuration variables
M5_PATH_ENV = os.environ.get("M5_PATH")
ACC_BENCH_PATH_ENV = os.environ.get("ACC_BENCH_PATH")
if not M5_PATH_ENV or not ACC_BENCH_PATH_ENV:
    sys.exit(
        "ERROR: M5_PATH and/or ACC_BENCH_PATH environment variables not set."
    )

M5_PATH = Path(M5_PATH_ENV).resolve()
ACC_BENCH_PATH = Path(ACC_BENCH_PATH_ENV).resolve()
CACTI_DIR = M5_PATH / "ext" / "mcpat" / "cacti"
CACTI_EXE = CACTI_DIR / "cacti"
SCRIPT_DIR = Path(__file__).parent.resolve()
RESULTS_DIR = SCRIPT_DIR / "results"
DEFAULT_DELAY_S = 1.0
MIN_CACTI_SIZE = 2048
BENCH_LIST = ACC_BENCH_PATH / "benchmarks.list"

# Cacti SALAM default args
CACTI_DEFAULT_FILE = SCRIPT_DIR / "cacti_salam.cfg"

# Define output/log file paths
STDOUT_LOG = RESULTS_DIR / "stdout.log"
STDERR_LOG = RESULTS_DIR / "stderr.log"
CACTI_OUT_CSV = CACTI_DIR / "out.csv"
INTERMEDIATE_CSV = RESULTS_DIR / "out.csv"
MASTER_CSV = RESULTS_DIR / "SALAM-out.csv"


# Recursive function to find memory objects
def find_mem_objects(data):
    spms_found = []
    if isinstance(data, dict):
        obj_type = data.get("Type")
        is_potential_mem = all(k in data for k in ("Name", "Size", "Ports"))
        if obj_type == "SPM" and is_potential_mem:
            print(
                f"  Found SPM: {data['Name']} (Size: {data['Size']},"
                f" Ports: {data['Ports']})"
            )
            spms_found.append(data)
        elif obj_type and obj_type != "SPM" and is_potential_mem:
            print(
                f"  INFO: Ignoring non-SPM Var Type='{obj_type}'"
                f" Name='{data.get('Name', 'N/A')}'"
            )
        if not (obj_type and is_potential_mem):
            for value in data.values():
                spms_found.extend(find_mem_objects(value))
    elif isinstance(data, list):
        for item in data:
            spms_found.extend(find_mem_objects(item))
    return spms_found


def main():
    parser = argparse.ArgumentParser(description="Run CACTI for SALAM SPMs.")
    parser.add_argument(
        "--bench-list",
        default=BENCH_LIST,
        type=Path,
        help="Input file listing configs.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=DEFAULT_DELAY_S,
        help="Delay between configs (sec).",
    )
    args = parser.parse_args()

    if not CACTI_EXE.is_file():
        sys.exit(
            f"ERROR: CACTI executable not found at {CACTI_EXE}."
            "Run setup script."
        )
    if not args.bench_list.is_file():
        sys.exit(f"ERROR: Benchmark list file not found: {args.bench_list}")
    if not CACTI_DEFAULT_FILE.is_file():
        sys.exit(
            f"ERROR: CACTI template file not found at {CACTI_DEFAULT_FILE}"
        )

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Starting CACTI Sweep from {args.bench_list}")
    print(f"Using CACTI template: {CACTI_DEFAULT_FILE}\n")
    try:
        with open(args.bench_list) as f:
            bench_lines = f.readlines()
    except OSError as e:
        sys.exit(
            f"ERROR: Cannot read benchmark list file {args.bench_list}: {e}"
        )

    # Process each benchmark config
    for line_num, line in enumerate(bench_lines, 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        parts = line.split()
        if len(parts) != 3:
            print(
                f"Warning: Skipping malformed line {line_num}: '{line}'",
                file=sys.stderr,
            )
            continue

        config_file, bench_name, sub_name = parts
        config_path = Path(config_file)
        if not config_path.is_absolute() and not config_path.exists():
            potential_path = ACC_BENCH_PATH / config_path
            if potential_path.exists():
                config_path = potential_path

        print(f"\nProcessing: {bench_name}/{sub_name} ({config_path})...")

        memobjects = []
        try:
            with open(config_path) as yf:
                yaml_data = yaml.safe_load(yf)
            if isinstance(yaml_data, dict):
                memobjects = find_mem_objects(yaml_data.get("acc_cluster", []))
            else:
                print(
                    f"Warning: Invalid YAML format in {config_path},"
                    " expected a dictionary.",
                    file=sys.stderr,
                )
        except FileNotFoundError:
            print(
                f"ERROR: YAML file not found: {config_path}", file=sys.stderr
            )
            continue  # Skip this config
        except yaml.YAMLError as e:
            print(
                f"ERROR: Failed to parse YAML {config_path}: {e}",
                file=sys.stderr,
            )
            continue  # Skip this config
        except Exception as e:
            print(
                f"ERROR: Unexpected error processing YAML {config_path}: {e}",
                file=sys.stderr,
            )
            continue  # Skip this config

        if not memobjects:
            print("INFO: No SPMs found in config. Skipping CACTI run.")
            if args.delay > 0 and line_num < len(bench_lines):
                time.sleep(args.delay)
            continue

        # Cleanup intermediate files
        CACTI_OUT_CSV.unlink(missing_ok=True)
        INTERMEDIATE_CSV.unlink(missing_ok=True)
        STDOUT_LOG.unlink(missing_ok=True)
        STDERR_LOG.unlink(missing_ok=True)

        # Read cacti salam args (default)
        try:
            with open(CACTI_DEFAULT_FILE) as f_template:
                template_lines = f_template.readlines()
        except OSError as e:
            print(
                f"ERROR: Cannot read CACTI template {CACTI_DEFAULT_FILE}: {e}",
                file=sys.stderr,
            )
            continue

        # Run CACTI for each SPM
        spm_names_processed = []
        overall_cacti_success = True
        with open(STDOUT_LOG, "a") as f_out, open(STDERR_LOG, "a") as f_err:
            for spm in memobjects:
                spm_name = spm["Name"]
                spm_size = int(spm["Size"])
                spm_ports = int(spm["Ports"])
                run_size = max(spm_size, MIN_CACTI_SIZE)

                print(
                    f"  Running CACTI for"
                    f"{spm_name} (Size={run_size}, Ports={spm_ports})"
                )
                # Generate temp config from template
                modified_cfg_lines = []
                size_replaced = False
                ports_replaced = False
                for t_line in template_lines:
                    stripped_line = t_line.strip()
                    # Replace specific lines, keep others
                    if stripped_line.startswith("-size (bytes)"):
                        modified_cfg_lines.append(
                            f"-size (bytes) {run_size}\n"
                        )
                        size_replaced = True
                    elif stripped_line.startswith("-read-write port"):
                        modified_cfg_lines.append(
                            f"-read-write port {spm_ports}\n"
                        )
                        ports_replaced = True
                    else:
                        modified_cfg_lines.append(
                            t_line
                        )  # Keep original template line

                if not size_replaced:
                    print(
                        "Warning: '-size (bytes)' not"
                        " found/replaced in template.",
                        file=sys.stderr,
                    )
                if not ports_replaced:
                    print(
                        "Warning: '-read-write port' not"
                        " found/replaced in template.",
                        file=sys.stderr,
                    )
                tmp_cfg_path = None
                cacti_run_ok = False
                try:
                    with tempfile.NamedTemporaryFile(
                        mode="w+",
                        suffix=".cfg",
                        prefix="cacti_salam_",
                        dir=CACTI_DIR,
                        delete=False,
                    ) as tmp_cfg:
                        tmp_cfg_path = Path(tmp_cfg.name)
                        tmp_cfg.writelines(modified_cfg_lines)
                        tmp_cfg.flush()

                    cmd = [str(CACTI_EXE), "-infile", str(tmp_cfg_path.name)]
                    process = subprocess.run(
                        cmd,
                        cwd=CACTI_DIR,
                        capture_output=True,
                        text=True,
                        check=False,
                        timeout=300,
                    )
                    f_out.write(
                        f"# Output: {bench_name}/{sub_name}/{spm_name}\n"
                        f"{process.stdout or '<No stdout>'}\n"
                    )
                    if process.stderr:
                        f_err.write(
                            f"# Stderr: {bench_name}/{sub_name}/{spm_name}\n"
                            f"{process.stderr}\n"
                        )

                    # More robust check for CACTI success
                    if (
                        process.returncode != 0
                        or "ERROR:" in process.stdout
                        or not CACTI_OUT_CSV.exists()
                    ):
                        print(
                            f"  ERROR: CACTI failed for {spm_name}."
                            f" Code: {process.returncode}. Check logs.",
                            file=sys.stderr,
                        )
                        overall_cacti_success = False
                    else:
                        spm_names_processed.append(spm_name)
                        cacti_run_ok = True
                except subprocess.TimeoutExpired:
                    print(
                        f"  ERROR: CACTI timed out for {spm_name}",
                        file=sys.stderr,
                    )
                    overall_cacti_success = False
                except Exception as e:
                    print(
                        f"  ERROR: Exception during CACTI run for"
                        f" {spm_name}: {e}",
                        file=sys.stderr,
                    )
                    overall_cacti_success = False
                finally:
                    if tmp_cfg_path and tmp_cfg_path.exists():
                        tmp_cfg_path.unlink()

        # Move results
        if not overall_cacti_success or not spm_names_processed:
            print("INFO: Skipping results processing due to CACTI failures.")
            if args.delay > 0 and line_num < len(bench_lines):
                time.sleep(args.delay)
            continue
        if not CACTI_OUT_CSV.exists():
            print(
                f"ERROR: CACTI output {CACTI_OUT_CSV} missing before move.",
                file=sys.stderr,
            )
            if args.delay > 0 and line_num < len(bench_lines):
                time.sleep(args.delay)
            continue
        try:
            shutil.move(str(CACTI_OUT_CSV), str(INTERMEDIATE_CSV))
        except Exception as e:
            print(f"ERROR: Failed to move CACTI output: {e}", file=sys.stderr)
            if args.delay > 0 and line_num < len(bench_lines):
                time.sleep(args.delay)
            continue

        # Process results
        results_csv = []
        try:
            with open(INTERMEDIATE_CSV, newline="") as infile:
                reader = csv.reader(infile)
                header = next(reader, None)
                if not header:
                    raise ValueError("CSV file is empty")
                results_csv.append(["Benchmark", "Config", "Acc"] + header)
                num_results = 0
                for i, row in enumerate(reader):
                    num_results += 1
                    spm_name = (
                        spm_names_processed[i]
                        if i < len(spm_names_processed)
                        else f"UNKNOWN_SPM_{i+1}"
                    )
                    results_csv.append([bench_name, sub_name, spm_name] + row)
            if num_results != len(spm_names_processed):
                print(
                    f"  Warning: Mismatch between result rows ({num_results})"
                    f" and successful SPMs ({len(spm_names_processed)})",
                    file=sys.stderr,
                )
        except Exception as e:
            print(
                f"ERROR: Failed to process intermediate CSV"
                f" {INTERMEDIATE_CSV}: {e}",
                file=sys.stderr,
            )
            if args.delay > 0 and line_num < len(bench_lines):
                time.sleep(args.delay)
            continue

        # Append to master CSV
        write_header = not MASTER_CSV.exists()
        try:
            with open(MASTER_CSV, "a", newline="") as outfile:
                writer = csv.writer(outfile)
                if write_header:
                    writer.writerow(results_csv[0])
                writer.writerows(results_csv[1:])
            print(f"  Appended {len(results_csv)-1} rows to {MASTER_CSV}")
        except Exception as e:
            print(
                f"ERROR: Failed to write to master CSV {MASTER_CSV}: {e}",
                file=sys.stderr,
            )

        # Delay
        if args.delay > 0 and line_num < len(bench_lines):
            time.sleep(args.delay)

    print("\nCACTI Sweep Finished.")


if __name__ == "__main__":
    main()
