#!/usr/bin/env python3

# Run DAWG and baseline gem5 configs and analyze the results

import argparse
import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

GEM5_BIN = REPO_ROOT / "build/X86/gem5.opt"
DAWG_CONFIG = REPO_ROOT / "configs/dawg/two_proc_poc.py"
BASELINE_CONFIG = REPO_ROOT / "configs/dawg/two_proc_poc_baseline.py"
DAWG_OUTDIR = REPO_ROOT / "m5out-dawg"
BASELINE_OUTDIR = REPO_ROOT / "m5out-baseline"
LLC_NAME = "system.l3cache"


def run_cmd(cmd, cwd: Path) -> int:
    proc = subprocess.run(cmd, cwd=str(cwd))
    if proc.returncode != 0:
        print(
            f"Command failed with exit code {proc.returncode}: {' '.join(cmd)}\n"
        )
    return proc.returncode


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Run baseline and DAWG configs and analyze them",
    )
    p.add_argument(
        "--no-baseline",
        action="store_true",
        help="Skip baseline run and only run DAWG config",
    )
    p.add_argument(
        "--no-dawg",
        action="store_true",
        help="Skip DAWG run and only run baseline config",
    )
    return p.parse_args()


def prepare_outdir(outdir: Path) -> None:
    if outdir.exists():
        shutil.rmtree(outdir)
    outdir.mkdir(parents=True, exist_ok=True)


def run_sim(gem5: Path, config: Path, outdir: Path) -> int:
    simout = outdir / "simout.log"
    cmd = [
        str(gem5),
        f"--outdir={outdir}",
        str(config),
    ]

    print(f"\nRunning {config} -> {outdir} (simout: {simout})\n")
    with simout.open("w") as f:
        proc = subprocess.run(
            cmd,
            cwd=str(REPO_ROOT),
            stdout=f,
            stderr=subprocess.STDOUT,
        )

    if proc.returncode != 0:
        print(f"Simulation failed (rc={proc.returncode}) for {config}\n")

    return proc.returncode


def run_analysis(outdir: Path, llc_name: str) -> int:
    stats = outdir / "stats.txt"
    simout = outdir / "simout.log"
    analyze = REPO_ROOT / "util/dawg_analyze.py"

    cmd = [
        "python3",
        str(analyze),
        "--stats",
        str(stats),
        "--simout",
        str(simout),
        "--llc-name",
        llc_name,
    ]

    proc = subprocess.run(cmd, cwd=str(REPO_ROOT))
    if proc.returncode != 0:
        print(
            f"dawg_analyze.py failed (rc={proc.returncode})\n"
            f"for outdir {outdir}"
        )

    return proc.returncode


def main() -> None:
    args = parse_args()

    if not GEM5_BIN.is_file():
        raise SystemExit(f"gem5 binary not found: {GEM5_BIN}")

    # DAWG run
    if not args.no_dawg:
        prepare_outdir(DAWG_OUTDIR)
        if run_sim(GEM5_BIN, DAWG_CONFIG, DAWG_OUTDIR) != 0:
            raise SystemExit("DAWG simulation failed; aborting.")
        run_analysis(DAWG_OUTDIR, LLC_NAME)

    # Baseline run
    if not args.no_baseline:
        prepare_outdir(BASELINE_OUTDIR)
        if run_sim(GEM5_BIN, BASELINE_CONFIG, BASELINE_OUTDIR) != 0:
            raise SystemExit("Baseline simulation failed; aborting.")
        run_analysis(BASELINE_OUTDIR, LLC_NAME)


if __name__ == "__main__":
    main()
