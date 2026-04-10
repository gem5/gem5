#!/usr/bin/env python3

import argparse
import csv
import json
import re
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
UPDATE_REPO = SCRIPT_DIR.parents[2]
DEFAULT_BASELINE_REPO = Path("/home/duydonv/gem5_baseline")
BIN_DIR = SCRIPT_DIR / "perf_bin"
BUILD_SCRIPT = SCRIPT_DIR / "build_perf_binaries.py"
CONFIG_SCRIPT = SCRIPT_DIR / "configs" / "perf_binary_run.py"

BENCHMARKS = {
    "dot4_pipeline": {
        "scalar_binary": BIN_DIR / "dot4_pipeline_scalar",
        "custom_binary": BIN_DIR / "dot4_pipeline_custom",
    },
    "mac_pipeline": {
        "scalar_binary": BIN_DIR / "mac_pipeline_scalar",
        "custom_binary": BIN_DIR / "mac_pipeline_custom",
    },
}

CASE_LAYOUT = (
    ("baseline_scalar", "baseline", "scalar_binary"),
    ("update_scalar", "update", "scalar_binary"),
    ("update_custom", "update", "custom_binary"),
)

STAT_PATHS = {
    "simTicks": ("simTicks",),
    "simSeconds": ("simSeconds",),
    "hostSeconds": ("hostSeconds",),
    "simInsts": ("simInsts",),
    "numCycles": ("board.processor.cores.core.numCycles",),
    "ipc": ("board.processor.cores.core.ipc",),
    "cpi": ("board.processor.cores.core.cpi",),
    "branchLookups": ("board.processor.cores.core.branchPred.lookups_0::total",),
    "condIncorrect": ("board.processor.cores.core.branchPred.condIncorrect",),
    "BTBLookups": ("board.processor.cores.core.branchPred.BTBLookups",),
    "BTBHits": ("board.processor.cores.core.branchPred.BTBHits",),
    "branchMispredicts": (
        "board.processor.cores.core.commit.branchMispredicts",
        "board.processor.cores.core.iew.branchMispredicts",
    ),
    "nonControlRedirects": ("board.processor.cores.core.iew.nonControlRedirects",),
}

BENCH_REGEX = re.compile(r"BENCH_RESULT\s+(\S+)\s+(0x[0-9a-fA-F]+)")
STAT_REGEX = re.compile(r"^(\S+)\s+([^\s#]+)")


def parse_number(token: str) -> Any:
    if token.lower() in {"nan", "inf", "-inf"}:
        return float(token)
    if any(ch in token for ch in ".eE"):
        return float(token)
    return int(token)


def parse_stats(stats_path: Path) -> dict[str, Any]:
    raw_stats: dict[str, Any] = {}
    with stats_path.open("r", encoding="utf-8") as stats_file:
        for line in stats_file:
            if not line or line.startswith("-"):
                continue
            match = STAT_REGEX.match(line)
            if match is None:
                continue
            raw_stats[match.group(1)] = parse_number(match.group(2))

    selected: dict[str, Any] = {}
    for output_name, candidates in STAT_PATHS.items():
        selected[output_name] = None
        for candidate in candidates:
            if candidate in raw_stats:
                selected[output_name] = raw_stats[candidate]
                break
    return selected


def extract_bench_result(stdout: str, stderr: str) -> tuple[str, str]:
    match = BENCH_REGEX.search(stdout)
    if match is None:
        match = BENCH_REGEX.search(stderr)
    if match is None:
        raise RuntimeError("Benchmark checksum line not found in gem5 output.")
    return match.group(1), match.group(2).lower()


def run_command(command: list[str], cwd: Path, output_dir: Path) -> subprocess.CompletedProcess:
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )

    (output_dir / "stdout.log").write_text(completed.stdout, encoding="utf-8")
    (output_dir / "stderr.log").write_text(completed.stderr, encoding="utf-8")

    if completed.returncode != 0:
        raise RuntimeError(
            f"Command failed with exit code {completed.returncode}:\n"
            f"{' '.join(command)}\n\n"
            f"{completed.stdout}\n{completed.stderr}"
        )

    return completed


def format_cell(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def print_table(rows: list[dict[str, Any]]) -> None:
    headers = (
        "case",
        "cycles",
        "ticks",
        "insts",
        "ipc",
        "cpi",
        "br_miss",
        "pc_redir",
        "cond_bad",
        "btb_hit",
        "btb_lookup",
        "checksum",
    )
    widths = {header: len(header) for header in headers}

    rendered_rows = []
    for row in rows:
        rendered = {
            "case": row["case_name"],
            "cycles": format_cell(row["stats"]["numCycles"]),
            "ticks": format_cell(row["stats"]["simTicks"]),
            "insts": format_cell(row["stats"]["simInsts"]),
            "ipc": format_cell(row["stats"]["ipc"]),
            "cpi": format_cell(row["stats"]["cpi"]),
            "br_miss": format_cell(row["stats"]["branchMispredicts"]),
            "pc_redir": format_cell(row["stats"]["nonControlRedirects"]),
            "cond_bad": format_cell(row["stats"]["condIncorrect"]),
            "btb_hit": format_cell(row["stats"]["BTBHits"]),
            "btb_lookup": format_cell(row["stats"]["BTBLookups"]),
            "checksum": row["checksum"],
        }
        for key, value in rendered.items():
            widths[key] = max(widths[key], len(value))
        rendered_rows.append(rendered)

    header_line = "  ".join(header.ljust(widths[header]) for header in headers)
    print(header_line)
    print("  ".join("-" * widths[header] for header in headers))
    for rendered in rendered_rows:
        print("  ".join(rendered[header].ljust(widths[header]) for header in headers))


def speedup(base: Any, improved: Any) -> Any:
    if base in (None, 0) or improved in (None, 0):
        return None
    return float(base) / float(improved)


def percent_delta(reference: Any, candidate: Any) -> Any:
    if reference in (None, 0) or candidate is None:
        return None
    return (float(candidate) - float(reference)) * 100.0 / float(reference)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build and compare scalar/custom RISC-V AI extension benchmarks."
    )
    parser.add_argument(
        "--baseline-repo",
        type=Path,
        default=DEFAULT_BASELINE_REPO,
        help="Path to the baseline gem5 repository.",
    )
    parser.add_argument(
        "--update-repo",
        type=Path,
        default=UPDATE_REPO,
        help="Path to the updated gem5 repository.",
    )
    parser.add_argument(
        "--cpu",
        default="o3",
        help="CPU model used for the comparison. Default: o3.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=SCRIPT_DIR / "perf_results" / datetime.now().strftime("%Y%m%d_%H%M%S"),
        help="Directory where comparison outputs will be stored.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip rebuilding the performance binaries.",
    )
    parser.add_argument(
        "--no-cache",
        action="store_true",
        help="Forward --no-cache to perf_binary_run.py (direct-to-memory, no L1/L2).",
    )
    parser.add_argument(
        "--no-o3-fdp",
        action="store_true",
        help="Forward --no-o3-fdp to perf_binary_run.py for coupled O3 runs.",
    )
    args = parser.parse_args()

    baseline_repo = args.baseline_repo.resolve()
    update_repo = args.update_repo.resolve()
    out_dir = args.out_dir.resolve()

    baseline_gem5 = baseline_repo / "build" / "RISCV" / "gem5.opt"
    update_gem5 = update_repo / "build" / "RISCV" / "gem5.opt"

    if not baseline_gem5.is_file():
        raise FileNotFoundError(f"Baseline gem5 binary not found: {baseline_gem5}")
    if not update_gem5.is_file():
        raise FileNotFoundError(f"Updated gem5 binary not found: {update_gem5}")
    if not CONFIG_SCRIPT.is_file():
        raise FileNotFoundError(f"Run config not found: {CONFIG_SCRIPT}")

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.skip_build:
        build_output_dir = out_dir / "build_logs"
        build_output_dir.mkdir(parents=True, exist_ok=True)
        run_command(
            ["python3", str(BUILD_SCRIPT)],
            cwd=update_repo,
            output_dir=build_output_dir,
        )

    summary: dict[str, Any] = {
        "cpu": args.cpu,
        "no_cache": args.no_cache,
        "no_o3_fdp": args.no_o3_fdp,
        "baseline_repo": str(baseline_repo),
        "update_repo": str(update_repo),
        "generated_at": datetime.now().isoformat(),
        "benchmarks": {},
    }

    mismatches: list[str] = []

    for benchmark_name, benchmark_files in BENCHMARKS.items():
        print(f"\n## {benchmark_name}")
        benchmark_output_dir = out_dir / benchmark_name
        benchmark_output_dir.mkdir(parents=True, exist_ok=True)

        rows: list[dict[str, Any]] = []
        for case_name, repo_kind, binary_key in CASE_LAYOUT:
            case_output_dir = benchmark_output_dir / case_name
            case_output_dir.mkdir(parents=True, exist_ok=True)

            binary_path = benchmark_files[binary_key].resolve()
            if not binary_path.is_file():
                raise FileNotFoundError(f"Benchmark binary not found: {binary_path}")

            gem5_binary = baseline_gem5 if repo_kind == "baseline" else update_gem5
            gem5_repo = baseline_repo if repo_kind == "baseline" else update_repo

            command = [
                str(gem5_binary),
                "-d",
                str(case_output_dir),
                str(CONFIG_SCRIPT),
                str(binary_path),
                args.cpu,
            ]
            if args.no_cache:
                command.append("--no-cache")
            if args.no_o3_fdp:
                command.append("--no-o3-fdp")
            completed = run_command(command, cwd=gem5_repo, output_dir=case_output_dir)

            stats_path = case_output_dir / "stats.txt"
            if not stats_path.is_file():
                raise FileNotFoundError(f"stats.txt missing: {stats_path}")

            bench_label, checksum = extract_bench_result(
                completed.stdout, completed.stderr
            )
            row = {
                "case_name": case_name,
                "repo_kind": repo_kind,
                "binary": str(binary_path),
                "bench_label": bench_label,
                "checksum": checksum,
                "stats": parse_stats(stats_path),
            }
            rows.append(row)

        baseline_row = next(row for row in rows if row["case_name"] == "baseline_scalar")
        update_scalar_row = next(row for row in rows if row["case_name"] == "update_scalar")
        update_custom_row = next(row for row in rows if row["case_name"] == "update_custom")

        expected_checksum = baseline_row["checksum"]
        for row in rows:
            row["checksum_match"] = row["checksum"] == expected_checksum
            if not row["checksum_match"]:
                mismatches.append(
                    f"{benchmark_name}:{row['case_name']} expected {expected_checksum} got {row['checksum']}"
                )

        derived = {
            "speedup_vs_baseline_cycles": speedup(
                baseline_row["stats"]["numCycles"],
                update_custom_row["stats"]["numCycles"],
            ),
            "speedup_vs_baseline_ticks": speedup(
                baseline_row["stats"]["simTicks"],
                update_custom_row["stats"]["simTicks"],
            ),
            "control_delta_cycles_pct": percent_delta(
                baseline_row["stats"]["numCycles"],
                update_scalar_row["stats"]["numCycles"],
            ),
            "control_delta_ticks_pct": percent_delta(
                baseline_row["stats"]["simTicks"],
                update_scalar_row["stats"]["simTicks"],
            ),
            "custom_ipc_delta_pct": percent_delta(
                baseline_row["stats"]["ipc"], update_custom_row["stats"]["ipc"]
            ),
            "custom_branch_mispredict_delta": None
            if baseline_row["stats"]["branchMispredicts"] is None
            or update_custom_row["stats"]["branchMispredicts"] is None
            else update_custom_row["stats"]["branchMispredicts"]
            - baseline_row["stats"]["branchMispredicts"],
        }

        print_table(rows)
        print(
            "speedup(cycles): {}x | speedup(ticks): {}x | control cycles delta: {}%".format(
                format_cell(derived["speedup_vs_baseline_cycles"]),
                format_cell(derived["speedup_vs_baseline_ticks"]),
                format_cell(derived["control_delta_cycles_pct"]),
            )
        )

        summary["benchmarks"][benchmark_name] = {
            "rows": rows,
            "derived": derived,
        }

    summary_json = out_dir / "summary.json"
    summary_csv = out_dir / "summary.csv"

    summary_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    with summary_csv.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "benchmark",
                "case",
                "repo_kind",
                "checksum",
                "checksum_match",
                "simTicks",
                "simSeconds",
                "hostSeconds",
                "simInsts",
                "numCycles",
                "ipc",
                "cpi",
                "branchLookups",
                "condIncorrect",
                "BTBLookups",
                "BTBHits",
                "branchMispredicts",
                "nonControlRedirects",
            ]
        )
        for benchmark_name, benchmark_summary in summary["benchmarks"].items():
            for row in benchmark_summary["rows"]:
                stats = row["stats"]
                writer.writerow(
                    [
                        benchmark_name,
                        row["case_name"],
                        row["repo_kind"],
                        row["checksum"],
                        row["checksum_match"],
                        stats["simTicks"],
                        stats["simSeconds"],
                        stats["hostSeconds"],
                        stats["simInsts"],
                        stats["numCycles"],
                        stats["ipc"],
                        stats["cpi"],
                        stats["branchLookups"],
                        stats["condIncorrect"],
                        stats["BTBLookups"],
                        stats["BTBHits"],
                        stats["branchMispredicts"],
                        stats["nonControlRedirects"],
                    ]
                )

    print(f"\nSaved summary: {summary_json}")
    print(f"Saved CSV: {summary_csv}")

    if mismatches:
        mismatch_text = "\n".join(mismatches)
        raise SystemExit(f"Checksum mismatch detected:\n{mismatch_text}")


if __name__ == "__main__":
    main()
