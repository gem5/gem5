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
import sys

import m5
from m5.objects import (
    Root,
    VectorStatTester,
)
from m5.stats.gem5stats import get_simstat

"""This script is used for checking that the Vector statistics set in "
the simulation are correctly parsed through to the python Pystats.
"""

parser = argparse.ArgumentParser(
    description="Tests the output of a Vector PyStat."
)

parser.add_argument(
    "value",
    help="Comma delimited list representing the vector.",
    type=lambda s: [float(item) for item in s.split(",")],
)

parser.add_argument(
    "--name",
    type=str,
    default="vector",
    required=False,
    help="Name of the vector statistic.",
)

parser.add_argument(
    "--description",
    type=str,
    default="",
    required=False,
    help="Description of the vector statistic.",
)

parser.add_argument(
    "--subnames",
    help="Comma delimited list representing the vector subnames.",
    type=str,
)

parser.add_argument(
    "--subdescs",
    help="Comma delimited list representing the vector subdescs",
    type=str,
)

args = parser.parse_args()

stat_tester = VectorStatTester()
stat_tester.name = args.name
stat_tester.description = args.description
stat_tester.values = args.value

stat_tester.subnames = []
if args.subnames:
    stat_tester.subnames = [str(item) for item in args.subnames.split(",")]

stat_tester.subdescs = []
if args.subdescs:
    stat_tester.subdescs = [str(item) for item in args.subdescs.split(",")]

value_dict = {}
for i in range(len(args.value)):
    i_name = i
    description = args.description
    if stat_tester.subnames and i < len(stat_tester.subnames):
        i_name = stat_tester.subnames[i]
    if stat_tester.subdescs and i < len(stat_tester.subdescs):
        description = stat_tester.subdescs[i]

    value_dict[i_name] = {
        "value": args.value[i],
        "type": "Scalar",
        "unit": "Count",
        "description": description,
        "datatype": "f64",
    }

expected_output = {
    "type": "Group",
    "time_conversion": None,
    args.name: {
        "value": value_dict,
        "type": "Vector",
        "description": args.description,
    },
}

root = Root(full_system=False, system=stat_tester)

m5.instantiate()
m5.simulate()

simstats = get_simstat(stat_tester)
output = simstats.to_json()["system"]

if output != expected_output:
    print("Output statistics do not match expected:", file=sys.stderr)
    print("", file=sys.stderr)
    print("Expected:", file=sys.stderr)
    print(expected_output, file=sys.stderr)
    print("", file=sys.stderr)
    print("Actual:", file=sys.stderr)
    print(output, file=sys.stderr)
    sys.exit(1)
