#!/usr/bin/env python3
# Copyright (c) 2015 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

try:
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    import numpy as np
except ImportError:
    print("Failed to import matplotlib and numpy")
    exit(-1)

import sys
import re


# This script is intended to post process and plot the output from
# running configs/dram/lat_mem_rd.py, as such it parses the simout.txt and
# stats.txt to get the relevant data points.
def main():
    if len(sys.argv) != 2:
        print("Usage: ", sys.argv[0], "<simout directory>")
        exit(-1)

    try:
        stats = open(sys.argv[1] + "/stats.txt", "r")
    except IOError:
        print("Failed to open ", sys.argv[1] + "/stats.txt", " for reading")
        exit(-1)

    try:
        simout = open(sys.argv[1] + "/simout.txt", "r")
    except IOError:
        print("Failed to open ", sys.argv[1] + "/simout.txt", " for reading")
        exit(-1)

    # Get the address ranges
    got_ranges = False
    ranges = []

    iterations = 1

    for line in simout:
        if got_ranges:
            ranges.append(int(line) / 1024)

        match = re.match("lat_mem_rd with (\d+) iterations, ranges:.*", line)
        if match:
            got_ranges = True
            iterations = int(match.groups(0)[0])

    simout.close()

    if not got_ranges:
        print("Failed to get address ranges, ensure simout.txt is up-to-date")
        exit(-1)

    # Now parse the stats
    raw_rd_lat = []

    for line in stats:
        match = re.match(".*readLatencyHist::mean\s+(.+)\s+#.*", line)
        if match:
            raw_rd_lat.append(float(match.groups(0)[0]) / 1000)
    stats.close()

    # The stats also contain the warming, so filter the latency stats
    i = 0
    filtered_rd_lat = []
    for l in raw_rd_lat:
        if i % (iterations + 1) == 0:
            pass
        else:
            filtered_rd_lat.append(l)
        i = i + 1

    # Next we need to take care of the iterations
    rd_lat = []
    for i in range(iterations):
        rd_lat.append(filtered_rd_lat[i::iterations])

    final_rd_lat = [min(p) for p in zip(*rd_lat)]

    # Sanity check
    if not (len(ranges) == len(final_rd_lat)):
        print(
            "Address ranges (%d) and read latency (%d) do not match"
            % (len(ranges), len(final_rd_lat))
        )
        exit(-1)

    for r, l in zip(ranges, final_rd_lat):
        print(r, round(l, 2))

    # lazy version to check if an integer is a power of two
    def is_pow2(num):
        return num != 0 and ((num & (num - 1)) == 0)

    plt.semilogx(ranges, final_rd_lat)

    # create human readable labels
    xticks_locations = [r for r in ranges if is_pow2(r)]
    xticks_labels = []
    for x in xticks_locations:
        if x < 1024:
            xticks_labels.append("%d kB" % x)
        else:
            xticks_labels.append("%d MB" % (x / 1024))
    plt.xticks(xticks_locations, xticks_labels, rotation=-45)

    plt.minorticks_off()
    plt.xlim((xticks_locations[0], xticks_locations[-1]))
    plt.ylabel("Latency (ns)")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
