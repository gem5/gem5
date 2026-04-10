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

from hwloop_expectations import KERNEL_ITERATIONS, expected_loop_backs


SCRIPT_DIR = Path(__file__).resolve().parent
UPDATE_REPO = SCRIPT_DIR.parents[2]
BIN_DIR = SCRIPT_DIR / "hwloop_perf_bin"
BUILD_SCRIPT = SCRIPT_DIR / "build_hwloop_perf_binaries.py"
CONFIG_SCRIPT = SCRIPT_DIR / "configs" / "perf_binary_run.py"
BASELINE_DIR = SCRIPT_DIR / "hwloop_baseline"
DEFAULT_BASELINE_FILE = BASELINE_DIR / "reference_summary.json"

BENCHMARKS = {
    "small": {
        "outer_repeats": 1024,
        "ref": BIN_DIR / "hwloop_redirect_ref_small",
        "swloop": BIN_DIR / "hwloop_redirect_swloop_small",
        "hwloop": BIN_DIR / "hwloop_redirect_hwloop_small",
    },
    "medium": {
        "outer_repeats": 4096,
        "ref": BIN_DIR / "hwloop_redirect_ref_medium",
        "swloop": BIN_DIR / "hwloop_redirect_swloop_medium",
        "hwloop": BIN_DIR / "hwloop_redirect_hwloop_medium",
    },
    "large": {
        "outer_repeats": 16384,
        "ref": BIN_DIR / "hwloop_redirect_ref_large",
        "swloop": BIN_DIR / "hwloop_redirect_swloop_large",
        "hwloop": BIN_DIR / "hwloop_redirect_hwloop_large",
    },
}

CASE_LAYOUT = ("ref", "swloop", "hwloop")

STAT_PATHS = {
    "simTicks": ("simTicks",),
    "simSeconds": ("simSeconds",),
    "hostSeconds": ("hostSeconds",),
    "simInsts": ("simInsts",),
    "numCycles": ("board.processor.cores.core.numCycles",),
    "ipc": ("board.processor.cores.core.ipc",),
    "cpi": ("board.processor.cores.core.cpi",),
    "branchMispredicts": (
        "board.processor.cores.core.commit.branchMispredicts",
        "board.processor.cores.core.iew.branchMispredicts",
    ),
    "nonControlRedirects": ("board.processor.cores.core.iew.nonControlRedirects",),
    "condIncorrect": ("board.processor.cores.core.branchPred.condIncorrect",),
    "bpMispredicted": ("board.processor.cores.core.branchPred.mispredicted_0::total",),
    "predictorMispredicted": (
        "board.processor.cores.core.branchPred.mispredictDueToPredictor_0::total",
    ),
    "btbMispredicted": (
        "board.processor.cores.core.branchPred.mispredictDueToBTBMiss_0::total",
    ),
    "targetWrong": ("board.processor.cores.core.branchPred.targetWrong_0::total",),
    "dispSquashedInsts": ("board.processor.cores.core.iew.dispSquashedInsts",),
    "commitSquashedInsts": ("board.processor.cores.core.commit.commitSquashedInsts",),
    "icacheSquashes": ("board.processor.cores.core.fetch.icacheSquashes",),
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


def delta(candidate: Any, reference: Any) -> Any:
    if candidate is None or reference is None:
        return None
    return float(candidate) - float(reference)


def speedup(base: Any, improved: Any) -> Any:
    if base in (None, 0) or improved in (None, 0):
        return None
    return float(base) / float(improved)


def per_redirect(delta_cycles: Any, redirect_count: Any) -> Any:
    if delta_cycles is None or redirect_count in (None, 0):
        return None
    return float(delta_cycles) / float(redirect_count)


def git_head(repo: Path) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            text=True,
            capture_output=True,
            check=False,
        )
        if completed.returncode == 0:
            return completed.stdout.strip()
    except OSError:
        pass
    return None


def compare_summary_to_baseline(
    new_summary: dict[str, Any],
    baseline: dict[str, Any],
    max_hw_cycle_regression: float,
) -> list[str]:
    """
    Regression checks vs a saved reference_summary.json.
    Requires identical ref-case checksums per scale; allows hwloop cycle count
    to improve or stay within max_hw_cycle_regression (e.g. 1.03 = 3% slack).
    """
    issues: list[str] = []
    for scale, base_scale in baseline.get("scales", {}).items():
        if scale not in new_summary.get("scales", {}):
            issues.append(f"baseline scale {scale!r} missing in new summary")
            continue
        new_scale = new_summary["scales"][scale]
        base_rows = {r["case_name"]: r for r in base_scale["rows"]}
        new_rows = {r["case_name"]: r for r in new_scale["rows"]}
        for case in ("ref", "swloop", "hwloop"):
            if case not in base_rows or case not in new_rows:
                issues.append(f"{scale}: missing case {case!r}")
                continue
            b_chk = base_rows[case]["checksum"]
            n_chk = new_rows[case]["checksum"]
            if b_chk != n_chk:
                issues.append(
                    f"{scale}:{case} checksum drift vs baseline "
                    f"(baseline {b_chk} new {n_chk})"
                )
        br = base_rows.get("hwloop")
        nr = new_rows.get("hwloop")
        if br and nr:
            bc = br["stats"].get("numCycles")
            nc = nr["stats"].get("numCycles")
            if isinstance(bc, (int, float)) and isinstance(nc, (int, float)):
                limit = float(bc) * max_hw_cycle_regression
                if float(nc) > limit + 0.5:
                    issues.append(
                        f"{scale}:hwloop cycles regression "
                        f"(baseline {bc} new {nc}, limit {limit:.1f})"
                    )
    return issues


def print_table(rows: list[dict[str, Any]]) -> None:
    headers = (
        "case",
        "cycles",
        "insts",
        "br_miss",
        "bp_miss",
        "pred_bad",
        "target_bad",
        "pc_redir",
        "disp_sq",
        "commit_sq",
        "icache_sq",
        "checksum",
    )
    widths = {header: len(header) for header in headers}

    rendered_rows = []
    for row in rows:
        rendered = {
            "case": row["case_name"],
            "cycles": format_cell(row["stats"]["numCycles"]),
            "insts": format_cell(row["stats"]["simInsts"]),
            "br_miss": format_cell(row["stats"]["branchMispredicts"]),
            "bp_miss": format_cell(row["stats"]["bpMispredicted"]),
            "pred_bad": format_cell(row["stats"]["predictorMispredicted"]),
            "target_bad": format_cell(row["stats"]["targetWrong"]),
            "pc_redir": format_cell(row["stats"]["nonControlRedirects"]),
            "disp_sq": format_cell(row["stats"]["dispSquashedInsts"]),
            "commit_sq": format_cell(row["stats"]["commitSquashedInsts"]),
            "icache_sq": format_cell(row["stats"]["icacheSquashes"]),
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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a fast O3 microbenchmark focused on hardware-loop redirect cost."
    )
    parser.add_argument(
        "--update-repo",
        type=Path,
        default=UPDATE_REPO,
        help="Path to the gem5 repository used for the benchmark.",
    )
    parser.add_argument(
        "--cpu",
        default="o3",
        help="CPU model used for the comparison. Default: o3.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=SCRIPT_DIR / "hwloop_perf_results" / datetime.now().strftime("%Y%m%d_%H%M%S"),
        help="Directory where microbenchmark outputs will be stored.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip rebuilding the hardware-loop microbenchmark binaries.",
    )
    parser.add_argument(
        "--save-baseline",
        action="store_true",
        help=f"After a successful run, write {DEFAULT_BASELINE_FILE} for regression compares.",
    )
    parser.add_argument(
        "--compare-baseline",
        type=Path,
        nargs="?",
        const=DEFAULT_BASELINE_FILE,
        help="After run, compare to this JSON (default: hwloop_baseline/reference_summary.json).",
    )
    parser.add_argument(
        "--max-hw-cycle-regression",
        type=float,
        default=1.03,
        help="With --compare-baseline, fail if hwloop numCycles exceeds baseline * this factor.",
    )
    parser.add_argument(
        "--no-cache",
        action="store_true",
        help="Forward --no-cache to perf_binary_run.py (DRAM-only).",
    )
    parser.add_argument(
        "--no-o3-fdp",
        action="store_true",
        help="Forward --no-o3-fdp to perf_binary_run.py for coupled O3 runs.",
    )
    args = parser.parse_args()

    update_repo = args.update_repo.resolve()
    out_dir = args.out_dir.resolve()
    update_gem5 = update_repo / "build" / "RISCV" / "gem5.opt"

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
        "update_repo": str(update_repo),
        "generated_at": datetime.now().isoformat(),
        "kernel_iterations": KERNEL_ITERATIONS,
        "scales": {},
    }

    mismatches: list[str] = []

    for scale_name, benchmark_config in BENCHMARKS.items():
        print(f"\n## {scale_name}")
        scale_output_dir = out_dir / scale_name
        scale_output_dir.mkdir(parents=True, exist_ok=True)

        rows: list[dict[str, Any]] = []
        for case_name in CASE_LAYOUT:
            case_output_dir = scale_output_dir / case_name
            case_output_dir.mkdir(parents=True, exist_ok=True)

            binary_path = benchmark_config[case_name].resolve()
            if not binary_path.is_file():
                raise FileNotFoundError(f"Benchmark binary not found: {binary_path}")

            command = [
                str(update_gem5),
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
            completed = run_command(command, cwd=update_repo, output_dir=case_output_dir)

            stats_path = case_output_dir / "stats.txt"
            if not stats_path.is_file():
                raise FileNotFoundError(f"stats.txt missing: {stats_path}")

            bench_label, checksum = extract_bench_result(completed.stdout, completed.stderr)
            rows.append(
                {
                    "case_name": case_name,
                    "binary": str(binary_path),
                    "bench_label": bench_label,
                    "checksum": checksum,
                    "stats": parse_stats(stats_path),
                }
            )

        ref_row = next(row for row in rows if row["case_name"] == "ref")
        swloop_row = next(row for row in rows if row["case_name"] == "swloop")
        hwloop_row = next(row for row in rows if row["case_name"] == "hwloop")

        expected_checksum = ref_row["checksum"]
        for row in rows:
            row["checksum_match"] = row["checksum"] == expected_checksum
            if not row["checksum_match"]:
                mismatches.append(
                    f"{scale_name}:{row['case_name']} expected {expected_checksum} got {row['checksum']}"
                )

        expected_redirects = expected_loop_backs(benchmark_config["outer_repeats"])
        observed_redirects = hwloop_row["stats"]["nonControlRedirects"]
        early_elided_redirects = None
        redirect_elision_ratio = None
        if observed_redirects is not None:
            early_elided_redirects = max(expected_redirects - observed_redirects, 0)
            if expected_redirects:
                redirect_elision_ratio = early_elided_redirects / float(expected_redirects)

        # IEW nonControlRedirects counts execute-time redirects only. BAC fast
        # path elides most loop-backs at fetch, so observed << expected is normal.
        redirect_accounting_ok = None
        if observed_redirects is not None:
            redirect_accounting_ok = observed_redirects <= expected_redirects

        derived = {
            "expected_non_control_redirects": expected_redirects,
            "observed_non_control_redirects": observed_redirects,
            "early_elided_redirects": early_elided_redirects,
            "redirect_elision_ratio": redirect_elision_ratio,
            "swloop_cycles_over_ref": delta(
                swloop_row["stats"]["numCycles"], ref_row["stats"]["numCycles"]
            ),
            "hwloop_cycles_over_ref": delta(
                hwloop_row["stats"]["numCycles"], ref_row["stats"]["numCycles"]
            ),
            "hwloop_cycles_per_observed_redirect": per_redirect(
                delta(hwloop_row["stats"]["numCycles"], ref_row["stats"]["numCycles"]),
                observed_redirects,
            ),
            "hwloop_cycles_per_expected_loopback": per_redirect(
                delta(hwloop_row["stats"]["numCycles"], ref_row["stats"]["numCycles"]),
                expected_redirects,
            ),
            "hwloop_speedup_vs_swloop": speedup(
                swloop_row["stats"]["numCycles"], hwloop_row["stats"]["numCycles"]
            ),
            "redirect_accounting_ok": redirect_accounting_ok,
        }

        print_table(rows)
        print(
            "logical loop-backs: {} | IEW nonControlRedirects: {} | elided-at-fetch (est.): {} | "
            "elision ratio: {} | accounting ok (obs<=logical): {}".format(
                format_cell(derived["expected_non_control_redirects"]),
                format_cell(derived["observed_non_control_redirects"]),
                format_cell(derived["early_elided_redirects"]),
                format_cell(derived["redirect_elision_ratio"]),
                derived["redirect_accounting_ok"],
            )
        )
        print(
            "swloop delta vs ref: {} cycles | hwloop delta vs ref: {} cycles | "
            "hwloop cycles/observed-redirect: {} | hwloop cycles/expected-loopback: {} | "
            "hwloop speedup vs swloop: {}x".format(
                format_cell(derived["swloop_cycles_over_ref"]),
                format_cell(derived["hwloop_cycles_over_ref"]),
                format_cell(derived["hwloop_cycles_per_observed_redirect"]),
                format_cell(derived["hwloop_cycles_per_expected_loopback"]),
                format_cell(derived["hwloop_speedup_vs_swloop"]),
            )
        )

        summary["scales"][scale_name] = {
            "outer_repeats": benchmark_config["outer_repeats"],
            "rows": rows,
            "derived": derived,
        }

    summary_json = out_dir / "summary.json"
    summary_csv = out_dir / "summary.csv"

    summary["meta"] = {
        "kernel_iterations": KERNEL_ITERATIONS,
        "expected_loop_backs_formula": "outer_repeats * (kernel_iterations - 1)",
        "git_head": git_head(update_repo),
    }

    summary_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    with summary_csv.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "scale",
                "outer_repeats",
                "case",
                "checksum",
                "checksum_match",
                "simTicks",
                "simSeconds",
                "hostSeconds",
                "simInsts",
                "numCycles",
                "ipc",
                "cpi",
                "branchMispredicts",
                "nonControlRedirects",
                "condIncorrect",
                "bpMispredicted",
                "predictorMispredicted",
                "btbMispredicted",
                "targetWrong",
                "dispSquashedInsts",
                "commitSquashedInsts",
                "icacheSquashes",
            ]
        )
        for scale_name, scale_summary in summary["scales"].items():
            for row in scale_summary["rows"]:
                stats = row["stats"]
                writer.writerow(
                    [
                        scale_name,
                        scale_summary["outer_repeats"],
                        row["case_name"],
                        row["checksum"],
                        row["checksum_match"],
                        stats["simTicks"],
                        stats["simSeconds"],
                        stats["hostSeconds"],
                        stats["simInsts"],
                        stats["numCycles"],
                        stats["ipc"],
                        stats["cpi"],
                        stats["branchMispredicts"],
                        stats["nonControlRedirects"],
                        stats["condIncorrect"],
                        stats["bpMispredicted"],
                        stats["predictorMispredicted"],
                        stats["btbMispredicted"],
                        stats["targetWrong"],
                        stats["dispSquashedInsts"],
                        stats["commitSquashedInsts"],
                        stats["icacheSquashes"],
                    ]
                )

    print(f"\nSaved summary: {summary_json}")
    print(f"Saved CSV: {summary_csv}")

    if mismatches:
        mismatch_text = "\n".join(mismatches)
        raise SystemExit(f"Checksum mismatch detected:\n{mismatch_text}")

    if args.save_baseline:
        BASELINE_DIR.mkdir(parents=True, exist_ok=True)
        DEFAULT_BASELINE_FILE.write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        print(f"Saved baseline: {DEFAULT_BASELINE_FILE}")

    if args.compare_baseline is not None:
        baseline_path = args.compare_baseline.resolve()
        if not baseline_path.is_file():
            raise SystemExit(f"Baseline file not found: {baseline_path}")
        baseline_data = json.loads(baseline_path.read_text(encoding="utf-8"))
        regressions = compare_summary_to_baseline(
            summary, baseline_data, args.max_hw_cycle_regression
        )
        if regressions:
            raise SystemExit(
                "Baseline compare failed:\n" + "\n".join(regressions)
            )
        print(f"Baseline compare OK ({baseline_path})")


if __name__ == "__main__":
    main()
