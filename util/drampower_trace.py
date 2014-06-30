#!/usr/bin/env python

# Copyright (c) 2014 ARM Limited
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
#
# Authors: Andreas Hansson

import sys
import re
import math

# This utility script parses through the debug output looking for
# lines printed by the DRAMPowerTrace debug flag. For all such lines,
# it sorts the entries based on the timestamp and outputs it to a new
# file in a format that DRAMPower accepts.
def main():
    if len(sys.argv) != 4:
        print "Usage: ", sys.argv[0], " <simout> <rank> <command output>"
        exit(-1)

    try:
        sim_in = open(sys.argv[1], 'r')
    except IOError:
        print "Failed to open ", sys.argv[1], " for reading"
        exit(-1)

    try:
        rank = int(sys.argv[2])
    except ValueError:
        print "Failed to determine rank from ", sys.argv[2]
        exit(-1)

    try:
        cmd_out = open(sys.argv[3], 'w')
    except IOError:
        print "Failed to open ", sys.argv[3], " for writing"
        exit(-1)

    # Due to the out-of-order printing from the event-based DRAM
    # controller model, we need to sort the commands by time stamp. We
    # use the refresh as the ordering point
    history = []

    # For each line in the debug trace output
    for line in sim_in:
        # Search for a line with the right format. At the moment this
        # is rather dangerous, and could be made more robust.
        if re.search('\d+: .*: \d+,[A-Z]+,\d+,\d+', line):
            cmd = line.split(' ')[-1]
            cmd_cycle, cmd_name, cmd_bank, cmd_rank = cmd.split(',')

            # only look at the commands for the specific rank
            if (int(cmd_rank) == rank):

                # Do the conversion to clock cycles so we can use it to
                # sort
                cmd_cycle = int(cmd_cycle)

                # Append this to the history
                history.append((cmd_cycle, cmd_name, cmd_bank))

                # If we are refreshing, print the history and flush it
                if cmd_name == "REF":
                    history.sort(key = lambda tup: tup[0])
                    for (c, n, b) in history:
                        cmd_entry = "%d,%s,%s\n" % (c, n, b)
                        cmd_out.write(cmd_entry)

                    history = []

    # We're done
    sim_in.close()
    cmd_out.close()

if __name__ == "__main__":
    main()

