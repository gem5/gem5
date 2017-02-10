#! /usr/bin/env python2

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
#
# Authors: Andreas Hansson

import optparse
import subprocess
import sys

parser = optparse.OptionParser()

# This script lets the user run a soak test using the false-sharing
# memtest.py example script. It runs a number of iterations with
# random tree topologies generated for each run, and runs for a number
# of ticks. Both the iteration count and the ticks for each run can be
# set on the command line.

parser.add_option('-c', '--count', type='int', default=100)
parser.add_option('-t', '--ticks', type='int', default=100000000000)

(options, args) = parser.parse_args()

if len(args) != 1:
    print "Error: Expecting a single argument specifying the gem5 binary"
    sys.exit(1)

gem5_binary = args[0]

for i in range(options.count):
    status = subprocess.call([gem5_binary, 'configs/example/memtest.py',
                              '-r', '-m %d' % (options.ticks)])
    if status != 0:
        print "Error: memtest run failed\n"
        sys.exit(1)

print "memtest soak finished without errors"
