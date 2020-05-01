# Copyright (c) 2016 Jason Lowe-Power
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
import os

import m5
import m5.ticks
from m5.objects import *

sys.path.append(os.path.dirname(__file__) + '/system')
sys.path.append(os.path.dirname(__file__) + '/../../../configs/common/')
from system import *

parser = argparse.ArgumentParser(description="")
parser.add_argument('--kernel', type=str)
parser.add_argument('--disk', type=str)
parser.add_argument('--cpu-type', choices=['atomic', 'kvm', 'o3', 'simple',])
parser.add_argument('--num-cpus', type=int)
parser.add_argument('--boot-type', choices=['init', 'systemd',])

#(options, args) = parser.parse_args()
args = parser.parse_args()

# create the system we are going to simulate
system = MySystem(args.kernel, args.disk, args.cpu_type, args.num_cpus)

if args.boot_type == "init":
    # Simply run "exit.sh"
    system.workload.command_line += ' init=/root/exit.sh'
else:
    if args.boot_type != "systemd":
        m5.fatal("Bad option for boot_type. init or systemd.")

# set up the root SimObject and start the simulation
root = Root(full_system = True, system = system)

if system.getHostParallel():
    # Required for running kvm on multiple host cores.
    # Uses gem5's parallel event queue feature
    # Note: The simulator is quite picky about this number!
    root.sim_quantum = int(1e9) # 1 ms

# instantiate all of the objects we've created above
m5.instantiate()

print("Running the simulation")
exit_event = m5.simulate()

if exit_event.getCause() != "m5_exit instruction encountered":
    print("Failed to exit correctly")
    exit(1)
else:
    print("Success!")
    exit(0)
