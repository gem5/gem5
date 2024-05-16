# Copyright (c) 2024 The Regents of the University of California
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
import sys

import m5
from m5.objects import (
    Root,
    SparseHistStatTester,
)
from m5.stats.gem5stats import get_simstat

parser = argparse.ArgumentParser(
    description="Tests the output of a SparseHist Pystat."
)
parser.add_argument(
    "samples",
    help="delimited list representing the samples for the distributed "
    "histogram.",
    type=lambda s: [float(item) for item in s.split(",")],
)

parser.add_argument(
    "--name",
    type=str,
    default="sparse_hist",
    required=False,
    help="The name of the Sparse Histogram statistic.",
)

parser.add_argument(
    "--description",
    type=str,
    default=None,
    required=False,
    help="The description of the Sparse Histogram statistic.",
)

args = parser.parse_args()

stat_tester = SparseHistStatTester(
    name=args.name, description=args.description, samples=args.samples
)

root = Root(full_system=False, system=stat_tester)
m5.instantiate()
m5.simulate()

simstats = get_simstat(stat_tester)
output = simstats.to_json()


value_dict = {}
for sample in args.samples:
    value_dict[sample] = (
        1 if sample not in value_dict else value_dict[sample] + 1
    )

scaler_dict = {}
for key in value_dict:
    scaler_dict[key] = {
        "unit": "Count",
        "type": "Scalar",
        "description": None,
        "value": value_dict[key],
        "datatype": "f64",
    }

expected_output = {
    "type": "SimObject",
    "name": "system",
    "time_conversion": None,
    args.name: {
        "value": scaler_dict,
        "type": "SparseHist",
        "description": str(args.description),
    },
}

# Remove the time related fields from the outputs if they exist.
# `creation_time` is not deterministic, and `simulated_begin_time` and
# simulated_end_time are not under test here.
for field in ["creation_time", "simulated_begin_time", "simulated_end_time"]:
    for map in [output, expected_output]:
        if field in map:
            del map[field]

if output != expected_output:
    print("Output statistics do not match expected:", file=sys.stderr)
    print("", file=sys.stderr)
    print("Expected:", file=sys.stderr)
    print(json.dumps(expected_output, indent=4), file=sys.stderr)
    print("", file=sys.stderr)
    print("Actual:", file=sys.stderr)
    print(json.dumps(output, indent=4), file=sys.stderr)
    sys.exit(1)
