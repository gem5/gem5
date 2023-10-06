#! /usr/bin/env python3
# Copyright (c) 2010 Advanced Micro Devices, Inc.
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
#
# Author: Steve Reinhardt
#
# Basic test script for checkpointing.
#
# Given an M5 command and an interval (in ticks), this script will:
# 1. Run the command, dumping periodic checkpoints at the given interval.
# 2. Rerun the command for each pair of adjacent checkpoints:
#    a. Restore from checkpoint N
#    b. Run until the timestamp of checkpoint N+1
#    c. Dump a checkpoint and end the simulation
#    d. Diff the new checkpoint with the original checkpoint N+1
#
# Note that '--' must be used to separate the script args from the
# M5 command line.
#
# Caveats:
#
# - This script relies on the checkpoint options implemented in
#   configs/common/Simulation.py, so it works with commands based on
#   the se.py and fs.py scripts in configs/example, but does not work
#   directly with the existing regression tests.
# - Interleaving simulator and program output can cause discrepancies
#   in the file position checkpoint information since different runs
#   have different amount of simulator output.
# - Probably lots more issues we don't even know about yet.
#
# Examples:
#
# util/checkpoint-tester.py -i 400000 -- build/<ISA>/gem5.opt \
#      configs/example/se.py -c tests/test-progs/hello/bin/<isa>/linux/hello \
#      --output=progout --errout=progerr
#
# util/checkpoint-tester.py -i 200000000000 -- build/<ISA>/gem5.opt \
#      configs/example/fs.py --script configs/boot/halt.sh
#
import argparse
import os
import re
import subprocess
import sys

parser = argparse.ArgumentParser()

parser.add_argument("-i", "--interval", type=int)
parser.add_argument("-d", "--directory", default="checkpoint-test")
parser.add_argument("cmdline", nargs="+", help="gem5 command line")

args = parser.parse_args()

interval = args.interval

if os.path.exists(args.directory):
    print("Error: test directory", args.directory, "exists")
    print("       Tester needs to create directory from scratch")
    sys.exit(1)

top_dir = args.directory
os.mkdir(top_dir)

cmd_echo = open(os.path.join(top_dir, "command"), "w")
print(" ".join(sys.argv), file=cmd_echo)
cmd_echo.close()

m5_binary = args.cmdline[0]

args = args.cmdline[1:]

checkpoint_args = ["--take-checkpoints", "%d,%d" % (interval, interval)]

cptdir = os.path.join(top_dir, "m5out")

print("===> Running initial simulation.")
subprocess.call([m5_binary] + ["-red", cptdir] + args + checkpoint_args)

dirs = os.listdir(cptdir)
expr = re.compile("cpt\.([0-9]*)")
cpts = []
for dir in dirs:
    match = expr.match(dir)
    if match:
        cpts.append(int(match.group(1)))

cpts.sort()

# We test by loading checkpoint N, simulating to (and dumping at)
# checkpoint N+1, then comparing the resulting checkpoint with the
# original checkpoint N+1.  Thus the number of tests we can run is one
# less than tha number of checkpoints.
for i in range(1, len(cpts)):
    print("===> Running test %d of %d." % (i, len(cpts) - 1))
    checkpoint_args = ["--take-checkpoints", "%d,%d" % (cpts[i], interval)]
    mydir = os.path.join(top_dir, "test.%d" % i)
    subprocess.call(
        [m5_binary]
        + ["-red", mydir]
        + args
        + checkpoint_args
        + [
            "--max-checkpoints",
            "1",
            "--checkpoint-dir",
            cptdir,
            "--checkpoint-restore",
            str(i),
        ],
    )
    cpt_name = "cpt.%d" % cpts[i]
    diff_name = os.path.join(mydir, "diffout")
    diffout = open(diff_name, "w")
    subprocess.call(
        [
            "diff",
            "-ru",
            "-I",
            "^##.*",
            f"{cptdir}/{cpt_name}",
            f"{mydir}/{cpt_name}",
        ],
        stdout=diffout,
    )
    diffout.close()
    # print out the diff
    diffout = open(diff_name)
    print(diffout.read(), end=" ")
    diffout.close()
