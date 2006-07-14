# Simple test script
#
# Alpha: "m5 test.py"
# MIPS: "m5 test.py -c hello_mips"

import m5
import os, optparse, sys
m5.AddToPath('../common')
from SEConfig import *

this_dir = os.path.dirname(__file__)

process = LiveProcess()
process.executable = os.path.join(this_dir, options.cmd)
process.cmd = options.cmd + " " + options.options
if options.input != "":
    process.input = options.input

if options.detailed:
    #check for SMT workload
    workloads = options.cmd.split(';')
    if len(workloads) > 1:
        process = []
        smt_idx = 0
        inputs = []

        if options.input != "":
            inputs = options.input.split(';')

        for wrkld in workloads:
            smt_process = LiveProcess()
            smt_process.executable = os.path.join(this_dir, wrkld)
            smt_process.cmd = wrkld + " " + options.options
            if inputs and inputs[smt_idx]:
                smt_process.input = inputs[smt_idx]
            process += [smt_process, ]
            smt_idx += 1

cpu.workload = process

if options.timing or options.detailed:
    system.mem_mode = 'timing'



# instantiate configuration
m5.instantiate(root)

# simulate until program terminates
if options.maxtick:
    exit_event = m5.simulate(options.maxtick)
else:
    exit_event = m5.simulate()

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()

