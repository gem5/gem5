# Copyright (c) 2026 Akanksha Chaudhari
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
import json
import math
from pathlib import Path


def windows(path):
    result = []
    current = None
    for line in Path(path).read_text().splitlines():
        if "Begin Simulation Statistics" in line:
            current = {}
        elif "End Simulation Statistics" in line:
            if current is None:
                raise ValueError("statistics end without a begin")
            result.append(current)
            current = None
        elif current is not None:
            fields = line.split()
            if len(fields) >= 2 and not fields[0].startswith("#"):
                try:
                    current[fields[0]] = float(fields[1])
                except ValueError:
                    pass
    if current is not None or not result:
        raise ValueError("no complete statistics window")
    return result


def models(config):
    result = {}

    def visit(node, owner=None):
        if isinstance(node, dict):
            if node.get("type") == "SALAMPowerModel":
                path = node["path"]
                if path in result:
                    raise ValueError(f"duplicate physical model path {path}")
                result[path] = dict(
                    node,
                    owner_type=(owner or {}).get("type"),
                    owner_path=(owner or {}).get("path"),
                )
            else:
                for value in node.values():
                    visit(value, node)
        elif isinstance(node, list):
            for value in node:
                visit(value, owner)

    visit(config)
    return result


def aggregate(stats, config, prefix=""):
    components = []
    disabled = []
    for path, model in sorted(models(config).items()):
        if prefix and not (path == prefix or path.startswith(prefix + ".")):
            continue
        if not model.get("enabled", True):
            disabled.append(path)
            continue

        def value(name):
            key = path + "." + name
            if key not in stats or not math.isfinite(stats[key]):
                raise ValueError(f"missing or invalid statistic {key}")
            return stats[key]

        def flag(name):
            number = value(name)
            if number not in (0, 1):
                raise ValueError(f"invalid coverage flag {path}.{name}")
            return bool(number)

        def count(name):
            number = value(name)
            if number < 0 or number != int(number):
                raise ValueError(f"invalid event count {path}.{name}")
            return int(number)

        duration = value("elapsedSeconds")
        if duration < 0:
            raise ValueError("negative reporting duration")
        component = {
            "identity": path,
            "scope": "device/statistics window",
            "owner_type": model.get("owner_type"),
            "owner_path": model.get("owner_path"),
            "elapsed_s": duration,
            "known_area_um2": value("knownAreaUm2"),
            "known_dynamic_j": value("knownDynamicEnergyJ"),
            "known_leakage_j": value("knownLeakageEnergyJ"),
            "coverage": {
                key: flag(key + "Complete")
                for key in ("area", "dynamic", "leakage")
            },
            "approximate": flag("approximate"),
            "fu_timing_available": flag("timingCompatibilityAvailable"),
            "fu_timing_compatible": flag("timingCompatible"),
            "missing_event_count": count("missingEventCount"),
            "known_read_words": count("readWords"),
            "known_write_words": count("writeWords"),
            "memory_word_counts_available": flag("memoryWordCountsAvailable"),
            "memory_timing_available": model.get("memory_timing_available"),
            "characterization_ids": model.get("ids", []),
            "characterization_keys": model.get("keys", []),
            "characterization_sources": [
                json.loads(s) for s in model.get("sources", [])
            ],
        }
        component["fu_timing_status"] = (
            "incompatible"
            if not component["fu_timing_compatible"]
            else (
                "unavailable"
                if not component["fu_timing_available"]
                else "compatible"
            )
        )
        for key in ("known_area_um2", "known_dynamic_j", "known_leakage_j"):
            if component[key] < 0:
                raise ValueError(f"negative component quantity {path}:{key}")
        component["known_total_j"] = (
            component["known_dynamic_j"] + component["known_leakage_j"]
        )
        if not math.isclose(
            component["known_total_j"],
            value("knownTotalEnergyJ"),
            rel_tol=1e-12,
            abs_tol=1e-24,
        ):
            raise ValueError(f"inconsistent component total {path}")
        component["known_average_w"] = (
            component["known_total_j"] / duration if duration else None
        )
        components.append(component)
    if not components:
        return {
            "scope": "cluster/statistics window",
            "status": "disabled" if disabled else "unavailable",
            "components": [],
            "disabled": disabled,
        }
    intervals = {component["elapsed_s"] for component in components}
    if len(intervals) != 1:
        raise ValueError("cannot aggregate different observation intervals")
    duration = intervals.pop()
    coverage = {
        key: all(c["coverage"][key] for c in components)
        for key in ("area", "dynamic", "leakage")
    }
    result = {
        "scope": "cluster/statistics window",
        "status": (
            "complete"
            if all(coverage.values()) and not disabled
            else "incomplete"
        ),
        "elapsed_s": duration,
        "coverage": coverage,
        "approximate": any(c["approximate"] for c in components),
        "disabled": disabled,
        "components": components,
        "excluded": [
            "controllers",
            "DMA",
            "interconnect",
            "clock trees",
            "caches",
            "streams",
            "external memory",
        ],
    }
    for key in (
        "known_area_um2",
        "known_dynamic_j",
        "known_leakage_j",
        "known_total_j",
    ):
        result[key] = math.fsum(component[key] for component in components)
    result["fu_timing_status"] = (
        "incompatible"
        if any(c["fu_timing_status"] == "incompatible" for c in components)
        else (
            "unavailable"
            if disabled
            or any(c["fu_timing_status"] == "unavailable" for c in components)
            else "compatible"
        )
    )
    result["known_average_w"] = (
        result["known_total_j"] / duration if duration else None
    )
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Report SALAM physical components over one statistics window"
    )
    parser.add_argument("--stats", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--window", type=int, default=-1)
    parser.add_argument("--prefix", default="")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        result = aggregate(
            windows(args.stats)[args.window],
            json.loads(args.config.read_text()),
            args.prefix,
        )
    except (ValueError, KeyError, IndexError) as exc:
        parser.error(str(exc))
    result["stats_file"] = str(args.stats)
    result["config_file"] = str(args.config)
    result["window_index"] = args.window
    args.output.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    if args.strict and result["status"] != "complete":
        parser.error("modeled PPA coverage is incomplete; see output record")


if __name__ == "__main__":
    main()
