# Simple test script
#
# Alpha: "m5 test.py"
# MIPS: "m5 test.py -a Mips -c hello_mips"

import os, optparse, sys
import m5
from m5.objects import *
from FullO3Config import *

# parse command-line arguments
parser = optparse.OptionParser(option_list=m5.standardOptions)

parser.add_option("-c", "--cmd", default="hello")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-f", "--full", action="store_true")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# build configuration
this_dir = os.path.dirname(__file__)

process = LiveProcess()
process.executable = os.path.join(this_dir, options.cmd)
process.cmd = options.cmd

magicbus = Bus()
mem = PhysicalMemory()

if options.timing:
    cpu = TimingSimpleCPU()
elif options.full:
    cpu = DetailedCPU()
else:
    cpu = AtomicSimpleCPU()
cpu.workload = process
cpu.mem = magicbus

system = System(physmem = mem, cpu = cpu)
system.c1 =  Connector(side_a = mem, side_b = magicbus)
root = Root(system = system)

# instantiate configuration
m5.instantiate(root)

# simulate until program terminates
exit_event = m5.simulate()

print 'Exiting @ cycle', m5.curTick(), 'because', exit_event.getCause()

