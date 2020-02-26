# Copyright (c) 2020 The Regents of the University of California
# All Rights Reserved.
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

import m5
from m5.objects import *
import sys
import argparse

parser = argparse.ArgumentParser(description='m5threads atomic tester')
parser.add_argument('--cpu-type', default='DerivO3CPU')
parser.add_argument('--num-cores', default='8')
parser.add_argument('--cmd')

args = parser.parse_args()

root = Root(full_system = False)
root.system = System()

root.system.clk_domain = SrcClockDomain()
root.system.clk_domain.clock = '3GHz'
root.system.clk_domain.voltage_domain = VoltageDomain()
root.system.mem_mode = 'timing'

if args.cpu_type == 'DerivO3CPU':
    root.system.cpu = [DerivO3CPU(cpu_id = i)
                       for i in range (int(args.num_cores))]
elif args.cpu_type == 'TimingSimpleCPU':
    root.system.cpu = [TimingSimpleCPU(cpu_id=i)
                       for i in range(int(args.num_cores))]
else:
    print("ERROR: CPU Type '" + args.cpu_type + "' not supported")
    sys.exit(1)

process = Process(executable = args.cmd,
                  cmd = [args.cmd, str(args.num_cores)])

for i in range(int(args.num_cores)):
    root.system.cpu[i].workload = process

m5.instantiate()
exit_event = m5.simulate()
