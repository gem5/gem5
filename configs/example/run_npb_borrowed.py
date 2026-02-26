# Copyright (c) 2019 The Regents of the University of California.
# All rights reserved.
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
# Authors: Jason Lowe-Power, Ayaz Akram
""" Script to run NAS parallel benchmarks with gem5.
    The script expects kernel, diskimage, mem_sys,
    cpu (kvm, atomic, or timing), benchmark to run
    and number of cpus as arguments.
    If your application has ROI annotations, this script will count the total
    number of instructions executed in the ROI. It also tracks how much
    wallclock and simulated time.
"""
import errno
import os
import sys
import time

import m5
import m5.ticks
from m5.objects import *

sys.path.append("gem5/configs/common/")  # For the next line...
import SimpleOpts

from system import *


def writeBenchScript(dir, bench):
    """
    This method creates a script in dir which will be eventually
    passed to the simulated system (to run a specific benchmark
    at bootup).
    """
    file_name = f"{dir}/run_{bench}"
    bench_file = open(file_name, "w+")
    bench_file.write(f"/home/gem5/NPB3.3-OMP/bin/{bench} \n")
    # sleeping for sometime (5 seconds here) makes sure
    # that the benchmark's output has been
    # printed to the console
    bench_file.write("sleep 5 \n")
    bench_file.write("m5 exit \n")
    bench_file.close()
    return file_name


if __name__ == "__m5_main__":
    (opts, args) = SimpleOpts.parse_args()
    kernel, disk, cpu, mem_sys, benchmark, num_cpus = args
    if not cpu in ["atomic", "kvm", "timing"]:
        m5.fatal("cpu not supported")
    # create the system we are going to simulate
    system = MySystem(kernel, disk, int(num_cpus), opts, no_kvm=False)
    ruby_protocols = ["MI_example", "MESI_Two_Level", "MOESI_CMP_directory"]
    if mem_sys == "classic":
        system = MySystem(kernel, disk, int(num_cpus), opts, no_kvm=False)
    elif mem_sys in ruby_protocols:
        system = MyRubySystem(kernel, disk, mem_sys, int(num_cpus), opts)
    else:
        m5.fatal("Bad option for mem_sys")
    # Exit from guest on workbegin/workend
    system.exit_on_work_items = True
    # Create and pass a script to the simulated system to run the reuired
    # benchmark
    system.readfile = writeBenchScript(m5.options.outdir, benchmark)
    # set up the root SimObject and start the simulation
    root = Root(full_system=True, system=system)
    if system.getHostParallel():
        # Required for running kvm on multiple host cores.
        # Uses gem5's parallel event queue feature
        # Note: The simulator is quite picky about this number!
        root.sim_quantum = int(1e9)  # 1 ms
    # needed for long running jobs
    m5.disableAllListeners()
    # instantiate all of the objects we've created above
    m5.instantiate()
    globalStart = time.time()
    print("Running the simulation")
    print(f"Using cpu: {cpu}")
    exit_event = m5.simulate()
    if exit_event.getCause() == "workbegin":
        print("Done booting Linux")
        # Reached the start of ROI
        # start of ROI is marked by an
        # m5_work_begin() call
        print("Resetting stats at the start of ROI!")
        m5.stats.reset()
        start_tick = m5.curTick()
        start_insts = system.totalInsts()
        # switching cpu if argument cpu == atomic or timing
        if cpu == "atomic":
            system.switchCpus(system.cpu, system.atomicCpu)
        if cpu == "timing":
            system.switchCpus(system.cpu, system.timingCpu)
    else:
        print("Unexpected termination of simulation !")
        exit()
    # Simulate the ROI
    exit_event = m5.simulate()
    # Reached the end of ROI
    # Finish executing the benchmark
    print("Dump stats at the end of the ROI!")
    m5.stats.dump()
    end_tick = m5.curTick()
    end_insts = system.totalInsts()
    m5.stats.reset()
    # Switching back to KVM does not work
    # with Ruby mem protocols, so not
    # switching back to simulate the remaining
    # part
    if mem_sys in ruby_protocols:
        print("Ruby Mem: Not Switching back to KVM!")
    if mem_sys == "classic":
        # switch cpu back to kvm if atomic/timing was used for ROI
        if cpu == "atomic":
            system.switchCpus(system.atomicCpu, system.cpu)
        if cpu == "timing":
            system.switchCpus(system.timingCpu, system.cpu)
        # Simulate the remaning part of the benchmark
        exit_event = m5.simulate()
    print("Done with the simulation")
    print()
    print("Performance statistics:")
    print("Simulated time in ROI: %.2fs" % ((end_tick - start_tick) / 1e12))
    print("Instructions executed in ROI: %d" % (end_insts - start_insts))
    print("Ran a total of", m5.curTick() / 1e12, "simulated seconds")
    print(
        "Total wallclock time: %.2fs, %.2f min"
        % (time.time() - globalStart, (time.time() - globalStart) / 60)
    )
