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
import json
import math
from pathlib import Path


def parse_stats(path):
    statistics = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 2 or fields[0].startswith("-"):
            continue
        try:
            value = float(fields[1])
        except ValueError:
            continue
        statistics[fields[0]] = value
    return statistics


def values_equal(left, right):
    if math.isnan(left) and math.isnan(right):
        return True
    return left == right


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    baseline = parse_stats(args.baseline)
    candidate = parse_stats(args.candidate)
    result = {
        "baseline": str(Path(args.baseline).resolve()),
        "candidate": str(Path(args.candidate).resolve()),
        "compared": 0,
        "missing_from_baseline": [],
        "missing_from_candidate": [],
        "host_only_differences": [],
        "simulated_semantic_differences": [],
    }

    baseline_names = set(baseline)
    candidate_names = set(candidate)
    result["missing_from_baseline"] = sorted(candidate_names - baseline_names)
    result["missing_from_candidate"] = sorted(baseline_names - candidate_names)

    for name in sorted(baseline_names & candidate_names):
        result["compared"] += 1
        if values_equal(baseline[name], candidate[name]):
            continue
        difference = {
            "name": name,
            "baseline": baseline[name],
            "candidate": candidate[name],
        }
        if name.startswith("host"):
            result["host_only_differences"].append(difference)
        else:
            result["simulated_semantic_differences"].append(difference)

    result["match"] = not (
        result["missing_from_baseline"]
        or result["missing_from_candidate"]
        or result["simulated_semantic_differences"]
    )
    Path(args.output).write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if result["match"] else 1)


if __name__ == "__main__":
    main()
