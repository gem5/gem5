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
parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-m", "--maxtick", type="int")

(options, args) = parser.parse_args()
m5.setStandardOptions(options)

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
elif options.detailed:
    cpu = DetailedO3CPU()
else:
    cpu = AtomicSimpleCPU()
cpu.workload = process
cpu.mem = magicbus

system = System(physmem = mem, cpu = cpu)
mem.port = magicbus.port
root = Root(system = system)

# instantiate configuration
m5.instantiate(root)

# simulate until program terminates
if options.maxtick:
    exit_event = m5.simulate(options.maxtick)
else:
    exit_event = m5.simulate()

print 'Exiting @ cycle', m5.curTick(), 'because', exit_event.getCause()

