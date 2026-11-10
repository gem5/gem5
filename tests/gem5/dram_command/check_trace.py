# Copyright (c) 2026 CkingX256
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

"""Validate actual row state in command-start order, with no DRAMPower parser."""

import argparse
import csv
import json
from pathlib import Path


def simulation_stats(path):
    """Retain simulated statistics; exclude host time and memory usage."""
    values = {}
    for line in Path(path).read_text().splitlines():
        columns = line.split()
        if len(columns) < 2 or columns[0].startswith(("#", "-", "host")):
            continue
        values[columns[0]] = columns[1]
    assert "simTicks" in values, "Missing simulation statistics"
    return values


def check_trace(path, policy):
    with Path(path).open(encoding="utf-8", newline="") as stream:
        records = list(csv.DictReader(stream))
    assert records, "Empty DRAM command trace"
    commands = {record["command"] for record in records}
    expected = {"ACT", "PRE", "RD", "WR", "REF"}
    if policy == "open":
        expected.add("PREA")
    assert expected <= commands, (expected, commands)
    scheduled = [int(record["scheduled_at"]) for record in records]
    assert scheduled == sorted(scheduled)
    assert [int(record["sequence"]) for record in records] == list(
        range(len(records))
    )
    opened = {}
    implicit = 0
    future = 0
    for record in sorted(
        records, key=lambda r: (int(r["command_at"]), int(r["sequence"]))
    ):
        now, when = int(record["scheduled_at"]), int(record["command_at"])
        assert when >= now
        future += when > now
        command, rank = record["command"], int(record["rank"])
        assert rank in (0, 1)
        if command in {"REF", "PREA"}:
            assert record["bank"] == record["row"] == ""
            if command == "REF":
                assert not any(key[0] == rank for key in opened)
            opened = {
                key: row for key, row in opened.items() if key[0] != rank
            }
        else:
            bank = int(record["bank"])
            assert 0 <= bank < 8
            key = (rank, bank)
            if command == "PRE":
                assert record["row"] == "" and key in opened
                del opened[key]
            else:
                row = int(record["row"])
                assert row >= 0
                if command == "ACT":
                    assert key not in opened
                    opened[key] = row
                else:
                    assert command in {"RD", "WR"}
                    assert opened.get(key) == row, record
        if record["implicit"] == "1":
            assert command == "PRE"
            implicit += 1
        else:
            assert record["implicit"] == "0"
    assert future > 0, "No future command start recorded"
    if policy == "close":
        assert implicit > 0, "Auto-precharge was not distinguished"
    return {
        "records": len(records),
        "commands": sorted(commands),
        "future_starts": future,
        "implicit_precharges": implicit,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--policy", choices=("open", "close"), default="open")
    parser.add_argument("--compare-stats", nargs=2, type=Path)
    args = parser.parse_args()
    if args.compare_stats:
        assert simulation_stats(args.compare_stats[0]) == simulation_stats(
            args.compare_stats[1]
        ), "Observation changed simulated statistics"
    print(json.dumps(check_trace(args.trace, args.policy)))
