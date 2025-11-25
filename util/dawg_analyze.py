#!/usr/bin/env python3

# Summarize DAWG behavior from gem5 logs

import argparse
import re
from pathlib import Path
from typing import (
    Dict,
    Optional,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Summarize DAWG behavior from gem5 logs"
    )
    p.add_argument(
        "--stats",
        type=Path,
        default=Path("m5out/stats.txt"),
        help="Path to stats.txt (default: m5out/stats.txt)",
    )
    p.add_argument(
        "--simout",
        type=Path,
        default=Path("m5out/simout"),
        help="Path to simout / console log (default: m5out/simout).\n"
        "If this file does not exist, a suitable .log file"
        " near stats.txt will be auto-detected.",
    )
    p.add_argument(
        "--llc-name",
        type=str,
        default="system.l3cache",
        help="Name of the LLC object (currently unused, kept for future stats parsing)",
    )
    return p.parse_args()


def find_simout(stats_path: Path, explicit_simout: Optional[Path]) -> Path:

    if explicit_simout is not None and explicit_simout.is_file():
        return explicit_simout

    stats_dir = stats_path.parent

    default_simout = stats_dir / "simout"
    if default_simout.is_file():
        return default_simout

    cand_logs = list(stats_dir.glob("*.log"))
    if cand_logs:
        cand_logs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        return cand_logs[0]

    raise FileNotFoundError(
        f"could not find simout: no explicit path, no 'simout', and no .log files in {stats_dir}"
    )


def read_simout(simout_path: Path) -> Dict[str, int]:

    if not simout_path.is_file():
        raise FileNotFoundError(f"simout file not found: {simout_path}")

    counts = {
        "filter": 0,
        "install": 0,
        "fallback": 0,
        "assert_fail": 0,
    }

    with simout_path.open() as f:
        for line in f:
            if "DAWG-FILTER" in line:
                counts["filter"] += 1
            if "DAWG-INSTALL" in line:
                counts["install"] += 1
            if "DAWG-FALLBACK" in line:
                counts["fallback"] += 1
            if "DAWG-ASSERT-FAIL" in line:
                counts["assert_fail"] += 1

    return counts


def parse_dawg_stats(
    stats_path: Path, llc_name: str
) -> Dict[str, Dict[str, float]]:

    result: Dict[str, Dict[str, float]] = {
        "dawgFilteredCandidatesPerDomain": {},
        "dawgInstallsPerDomain": {},
    }

    if not stats_path.is_file():
        return result

    prefix = f"{llc_name}.tags."
    wanted = set(result.keys())

    line_re = re.compile(r"^(?P<name>\S+)\s+(?P<value>\S+)")

    with stats_path.open() as f:
        for line in f:
            m = line_re.match(line.strip())
            if not m:
                continue
            full = m.group("name")
            raw_val = m.group("value")
            if not full.startswith(prefix):
                continue
            rest = full[len(prefix) :]
            if "::" in rest:
                base, sub = rest.split("::", 1)
            else:
                base, sub = rest, "all"
            if base not in wanted:
                continue
            try:
                val = float(raw_val)
            except ValueError:
                continue
            result[base][sub] = val

    return result


def parse_performance_stats(stats_path: Path) -> Dict[str, float]:

    keys = {
        "simSeconds": r"^simSeconds\s+(?P<val>\S+)",
        "simInsts": r"^simInsts\s+(?P<val>\S+)",
        "simOps": r"^simOps\s+(?P<val>\S+)",
        "system.cpu0.ipc": r"^system\.cpu0\.ipc\s+(?P<val>\S+)",
        "system.cpu0.cpi": r"^system\.cpu0\.cpi\s+(?P<val>\S+)",
        "system.cpu1.ipc": r"^system\.cpu1\.ipc\s+(?P<val>\S+)",
        "system.cpu1.cpi": r"^system\.cpu1\.cpi\s+(?P<val>\S+)",
        "system.l3cache.demandMisses.total": (
            r"^system\.l3cache\.demandMisses::total\s+(?P<val>\S+)"
        ),
        "system.l3cache.overallMisses.total": (
            r"^system\.l3cache\.overallMisses::total\s+(?P<val>\S+)"
        ),
        "system.l3cache.replacements": (
            r"^system\.l3cache\.replacements\s+(?P<val>\S+)"
        ),
    }

    compiled = {name: re.compile(pat) for name, pat in keys.items()}
    out: Dict[str, float] = {}

    if not stats_path.is_file():
        return out

    with stats_path.open() as f:
        for raw_line in f:
            line = raw_line.lstrip()
            for name, cre in compiled.items():
                m = cre.search(line)
                if not m:
                    continue
                try:
                    val = float(m.group("val"))
                except ValueError:
                    continue
                out[name] = val

    return out


def format_domain_table(title: str, data: Dict[str, float]) -> str:
    if not data:
        return f"{title}: (no data)"
    lines = [title]
    for sub in sorted(data.keys()):
        lines.append(f"  {sub:20s} {data[sub]:.0f}")
    return "\n".join(lines)


def gather_analysis(
    stats_path: Path, simout_path: Optional[Path], llc_name: str
) -> None:
    if not stats_path.is_file():
        print(f"Warning: stats file not found: {stats_path}")

    try:
        real_simout = find_simout(stats_path, simout_path)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return

    log_counts = read_simout(real_simout)
    dawg_stats = parse_dawg_stats(stats_path, llc_name)
    performance_stats = parse_performance_stats(stats_path)

    print(f"DAWG analysis for LLC '{llc_name}'")
    print(f"  stats:  {stats_path}")
    print(f"  simout: {real_simout}")
    print()

    print("=== simout: DAWG debug log events ===")
    print(f"  DAWG-FILTER   lines: {log_counts['filter']}")
    print(f"  DAWG-INSTALL  lines: {log_counts['install']}")
    print(f"  DAWG-FALLBACK lines: {log_counts['fallback']}")
    print(f"  DAWG-ASSERT-FAIL lines: {log_counts['assert_fail']}")

    if log_counts["assert_fail"] > 0:
        print(
            "\nWARNING: DAWG-ASSERT-FAIL detected; policy violated at least once."
        )

    print("\n=== stats.txt: DAWG per-domain stats ===")
    print(
        format_domain_table(
            "Filtered candidates per domain",
            dawg_stats["dawgFilteredCandidatesPerDomain"],
        )
    )
    print()
    print(
        format_domain_table(
            "Installs per domain",
            dawg_stats["dawgInstallsPerDomain"],
        )
    )

    print("\n=== stats.txt: performance summary ===")
    if performance_stats:
        if "simSeconds" in performance_stats:
            print(
                f"  simSeconds        : {performance_stats['simSeconds']:.6f} s"
            )
        if "simInsts" in performance_stats:
            print(
                f"  simInsts          : {performance_stats['simInsts']:.0f} insts"
            )
        if "simOps" in performance_stats:
            print(
                f"  simOps            : {performance_stats['simOps']:.0f} ops"
            )
        if "system.cpu0.ipc" in performance_stats:
            print(
                f"  cpu0.ipc / cpi    : {performance_stats['system.cpu0.ipc']:.6f} / {performance_stats.get('system.cpu0.cpi', float('nan')):.6f}"
            )
        if "system.cpu1.ipc" in performance_stats:
            print(
                f"  cpu1.ipc / cpi    : {performance_stats['system.cpu1.ipc']:.6f} / {performance_stats.get('system.cpu1.cpi', float('nan')):.6f}"
            )
        if "system.l3cache.demandMisses.total" in performance_stats:
            print(
                f"  l3.demandMisses   : {performance_stats['system.l3cache.demandMisses.total']:.0f} misses"
            )
        if "system.l3cache.overallMisses.total" in performance_stats:
            print(
                f"  l3.overallMisses  : {performance_stats['system.l3cache.overallMisses.total']:.0f} misses"
            )
        if "system.l3cache.replacements" in performance_stats:
            print(
                f"  l3.replacements   : {performance_stats['system.l3cache.replacements']:.0f}"
            )
    else:
        print("  (no performance stats parsed)")


def main() -> None:
    args = parse_args()
    gather_analysis(args.stats, args.simout, args.llc_name)


if __name__ == "__main__":
    main()
