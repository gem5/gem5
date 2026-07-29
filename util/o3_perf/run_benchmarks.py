#!/usr/bin/env python3

# Copyright (c) 2026 Magnushst
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
import csv
import hashlib
import math
import os
import re
import statistics
import subprocess
import time
from collections import defaultdict
from pathlib import Path

WORKLOADS = {
    "arithmetic": (150_000, None),
    "branch": (80_000, None),
    "stream": (1, 32_768),
    "pointer": (30_000, 16_384),
    "mixed": (25_000, 8_192),
    "queue": (80_000, 4),
}

PERF_EVENTS = (
    "instructions",
    "cycles",
    "branches",
    "branch-misses",
    "cache-references",
    "cache-misses",
)

RAW_FIELDS = [
    "sequence",
    "session",
    "label",
    "baseline_commit",
    "gem5_binary",
    "gem5_sha256",
    "build_type",
    "build_flags",
    "target_binary",
    "target_sha256",
    "workload",
    "iterations",
    "parameter",
    "cpu_config",
    "target_isa",
    "seed",
    "affinity",
    "wall_seconds",
    "user_seconds",
    "system_seconds",
    "peak_rss_kib",
    "major_page_faults",
    "minor_page_faults",
    "voluntary_context_switches",
    "involuntary_context_switches",
    "host_instructions",
    "host_cycles",
    "host_ipc",
    "host_branches",
    "host_branch_misses",
    "host_cache_references",
    "host_cache_misses",
    "simulated_instructions",
    "simulated_cycles",
    "simulated_ipc",
    "simulation_ticks",
    "simulated_insts_per_host_second",
    "simulated_cycles_per_host_second",
    "exit_code",
    "exit_cause",
    "target_output",
    "counter_status",
    "output_directory",
]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_key_values(path):
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def parse_stats(path):
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 2 or fields[0].startswith("-"):
            continue
        try:
            values[fields[0]] = float(fields[1])
        except ValueError:
            continue
    return values


def parse_perf_stat(path):
    values = {}
    if not path.exists():
        return values
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        fields = line.split(";")
        if len(fields) < 3:
            continue
        value = fields[0].strip()
        event = fields[2].strip()
        if value.startswith("<"):
            values[event] = math.nan
            continue
        try:
            values[event] = float(value)
        except ValueError:
            continue
    return values


def metric(stats, *names):
    for name in names:
        if name in stats:
            return stats[name]
    return math.nan


def label_sequence(labels, runs):
    if len(labels) == 1:
        return labels * runs
    if len(labels) != 2:
        raise ValueError("exactly one or two gem5 binaries are supported")
    left, right = labels
    sequence = []
    counts = {left: 0, right: 0}
    blocks = ([left, right, right, left], [right, left, left, right])
    block_index = 0
    while any(count < runs for count in counts.values()):
        for label in blocks[block_index % 2]:
            if counts[label] < runs:
                sequence.append(label)
                counts[label] += 1
        block_index += 1
    return sequence


def run_case(args, binaries, label, workload, cpu_config, sequence):
    iterations, parameter = WORKLOADS[workload]
    output = (
        args.output_root
        / "runs"
        / workload
        / cpu_config
        / f"{sequence:03d}-{label}"
    )
    output.mkdir(parents=True)
    time_file = output / "time.txt"
    stdout_file = output / "stdout.txt"
    stderr_file = output / "stderr.txt"
    perf_file = output / "perf-stat.txt"
    gem5 = binaries[label]

    time_command = [
        "/usr/bin/time",
        "--output",
        str(time_file),
        "--format",
        (
            "elapsed=%e\nuser=%U\nsystem=%S\npeak_rss_kib=%M\n"
            "major_page_faults=%F\nminor_page_faults=%R\n"
            "voluntary_context_switches=%w\n"
            "involuntary_context_switches=%c"
        ),
    ]
    gem5_command = [
        "taskset",
        "--cpu-list",
        args.affinity,
        str(gem5),
        "--outdir",
        str(output),
        "--listener-mode=off",
        str(args.config_script),
        "--binary",
        str(args.target),
        "--workload",
        workload,
        "--iterations",
        str(iterations),
        "--cpu-config",
        cpu_config,
        "--seed",
        str(args.seed),
    ]
    if parameter is not None:
        gem5_command.extend(["--parameter", str(parameter)])
    if args.counter_mode == "perf":
        gem5_command = [
            "perf",
            "stat",
            "--output",
            str(perf_file),
            "--field-separator",
            ";",
            "--event",
            ",".join(PERF_EVENTS),
            *gem5_command,
        ]
    command = [*time_command, *gem5_command]

    started = time.perf_counter()
    with stdout_file.open("w", encoding="utf-8") as stdout, stderr_file.open(
        "w", encoding="utf-8"
    ) as stderr:
        completed = subprocess.run(
            command,
            check=False,
            stdout=stdout,
            stderr=stderr,
            text=True,
            env={**os.environ, "LC_ALL": "C"},
        )
    harness_wall = time.perf_counter() - started

    timing = parse_key_values(time_file)
    host_counters = parse_perf_stat(perf_file)
    stats_path = output / "stats.txt"
    stats = parse_stats(stats_path) if stats_path.exists() else {}
    stdout_text = stdout_file.read_text(encoding="utf-8")
    exit_match = re.search(
        r"Exiting @ tick \d+ because (.+)",
        stdout_text,
    )
    output_match = re.search(
        rf"^{re.escape(workload)}:(\d+)$",
        stdout_text,
        re.MULTILINE,
    )
    wall_seconds = float(timing.get("elapsed", harness_wall))
    simulated_instructions = metric(stats, "simInsts")
    simulated_cycles = metric(stats, "system.cpu.numCycles")
    simulated_ipc = (
        simulated_instructions / simulated_cycles
        if simulated_cycles
        else math.nan
    )
    host_instructions = host_counters.get("instructions", math.nan)
    host_cycles = host_counters.get("cycles", math.nan)
    host_ipc = (
        host_instructions / host_cycles
        if host_cycles and not math.isnan(host_cycles)
        else math.nan
    )
    collected_counters = sum(
        not math.isnan(host_counters.get(event, math.nan))
        for event in PERF_EVENTS
    )
    if args.counter_mode == "none":
        counter_status = "not collected"
    elif collected_counters == len(PERF_EVENTS):
        counter_status = "collected"
    elif collected_counters:
        counter_status = "partially supported"
    else:
        counter_status = "unsupported"

    return {
        "sequence": sequence,
        "session": args.session,
        "label": label,
        "baseline_commit": args.baseline_commit,
        "gem5_binary": str(gem5.resolve()),
        "gem5_sha256": args.gem5_hashes[label],
        "build_type": args.build_type,
        "build_flags": args.build_flags,
        "target_binary": str(args.target.resolve()),
        "target_sha256": args.target_hash,
        "workload": workload,
        "iterations": iterations,
        "parameter": "" if parameter is None else parameter,
        "cpu_config": cpu_config,
        "target_isa": "X86",
        "seed": args.seed,
        "affinity": args.affinity,
        "wall_seconds": wall_seconds,
        "user_seconds": float(timing.get("user", "nan")),
        "system_seconds": float(timing.get("system", "nan")),
        "peak_rss_kib": int(timing.get("peak_rss_kib", "0")),
        "major_page_faults": int(timing.get("major_page_faults", "0")),
        "minor_page_faults": int(timing.get("minor_page_faults", "0")),
        "voluntary_context_switches": int(
            timing.get("voluntary_context_switches", "0")
        ),
        "involuntary_context_switches": int(
            timing.get("involuntary_context_switches", "0")
        ),
        "host_instructions": host_instructions,
        "host_cycles": host_cycles,
        "host_ipc": host_ipc,
        "host_branches": host_counters.get("branches", math.nan),
        "host_branch_misses": host_counters.get("branch-misses", math.nan),
        "host_cache_references": host_counters.get(
            "cache-references", math.nan
        ),
        "host_cache_misses": host_counters.get("cache-misses", math.nan),
        "simulated_instructions": simulated_instructions,
        "simulated_cycles": simulated_cycles,
        "simulated_ipc": simulated_ipc,
        "simulation_ticks": metric(stats, "simTicks"),
        "simulated_insts_per_host_second": (
            simulated_instructions / wall_seconds
        ),
        "simulated_cycles_per_host_second": (simulated_cycles / wall_seconds),
        "exit_code": completed.returncode,
        "exit_cause": exit_match.group(1) if exit_match else "",
        "target_output": output_match.group(1) if output_match else "",
        "counter_status": counter_status,
        "output_directory": str(output.resolve()),
    }


def percentile(values, probability):
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (
        position - lower
    )


def summarise(rows, output):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["label"], row["workload"], row["cpu_config"])].append(
            float(row["wall_seconds"])
        )
    with output.open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "label",
            "workload",
            "cpu_config",
            "runs",
            "median_wall_seconds",
            "p25_wall_seconds",
            "p75_wall_seconds",
            "minimum_wall_seconds",
            "maximum_wall_seconds",
            "coefficient_of_variation",
            "median_simulated_insts_per_host_second",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for (label, workload, cpu_config), values in sorted(groups.items()):
            mean = statistics.mean(values)
            cv = (
                statistics.stdev(values) / mean
                if len(values) > 1 and mean
                else 0.0
            )
            writer.writerow(
                {
                    "label": label,
                    "workload": workload,
                    "cpu_config": cpu_config,
                    "runs": len(values),
                    "median_wall_seconds": statistics.median(values),
                    "p25_wall_seconds": percentile(values, 0.25),
                    "p75_wall_seconds": percentile(values, 0.75),
                    "minimum_wall_seconds": min(values),
                    "maximum_wall_seconds": max(values),
                    "coefficient_of_variation": cv,
                    "median_simulated_insts_per_host_second": (
                        statistics.median(
                            float(row["simulated_insts_per_host_second"])
                            for row in rows
                            if row["label"] == label
                            and row["workload"] == workload
                            and row["cpu_config"] == cpu_config
                        )
                    ),
                }
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gem5",
        action="append",
        required=True,
        metavar="LABEL=PATH",
    )
    parser.add_argument("--config-script", required=True, type=Path)
    parser.add_argument("--target", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--baseline-commit", required=True)
    parser.add_argument("--build-flags", required=True)
    parser.add_argument("--build-type", required=True)
    parser.add_argument("--session", default="session-1")
    parser.add_argument("--affinity", default="2")
    parser.add_argument("--seed", default=1, type=int)
    parser.add_argument("--runs", default=8, type=int)
    parser.add_argument(
        "--counter-mode",
        choices=("none", "perf"),
        default="none",
    )
    parser.add_argument(
        "--workloads",
        nargs="+",
        choices=WORKLOADS,
        default=list(WORKLOADS),
    )
    parser.add_argument(
        "--cpu-configs",
        nargs="+",
        choices=("o3-small", "o3-medium", "o3-large"),
        default=("o3-small", "o3-medium", "o3-large"),
    )
    args = parser.parse_args()

    binaries = {}
    for specification in args.gem5:
        label, separator, path = specification.partition("=")
        if not label or not separator or not path:
            parser.error(f"invalid --gem5 value: {specification}")
        binaries[label] = Path(path)
    labels = list(binaries)
    args.gem5_hashes = {
        label: sha256(binary) for label, binary in binaries.items()
    }
    args.target_hash = sha256(args.target)
    args.output_root.mkdir(parents=True)

    rows = []
    raw_path = args.output_root / "raw_runs.csv"
    sequence = 0
    for workload in args.workloads:
        for cpu_config in args.cpu_configs:
            for label in label_sequence(labels, args.runs):
                sequence += 1
                row = run_case(
                    args,
                    binaries,
                    label,
                    workload,
                    cpu_config,
                    sequence,
                )
                rows.append(row)
                with raw_path.open(
                    "w",
                    newline="",
                    encoding="utf-8",
                ) as stream:
                    writer = csv.DictWriter(stream, fieldnames=RAW_FIELDS)
                    writer.writeheader()
                    writer.writerows(rows)
                print(
                    f"{sequence}: {label} {workload} {cpu_config} "
                    f"{row['wall_seconds']:.3f}s exit={row['exit_code']}",
                    flush=True,
                )
                if (
                    row["exit_code"] != 0
                    or not row["exit_cause"]
                    or not row["target_output"]
                ):
                    raise SystemExit(
                        "gem5 run failed or produced incomplete output"
                    )
    summarise(rows, args.output_root / "summary.csv")


if __name__ == "__main__":
    main()
