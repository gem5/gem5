import os, optparse, sys
import m5
from m5.objects import *

parser = optparse.OptionParser(option_list=m5.standardOptions)

parser.add_option("-t", "--timing", action="store_true")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

this_dir = os.path.dirname(__file__)

process = AlphaLiveProcess()
process.executable = os.path.join(this_dir, 'hello')
process.cmd = 'hello'

magicbus = Bus()
mem = PhysicalMemory()

if options.timing:
    cpu = TimingSimpleCPU()
else:
    cpu = AtomicSimpleCPU()
cpu.workload = process
cpu.mem = magicbus

system = System(physmem = mem, cpu = cpu)
system.c1 =  Connector(side_a = mem, side_b = magicbus)
root = Root(system = system)

m5.instantiate(root)

exit_event = m5.simulate()

print 'Exiting @', m5.curTick(), 'because', exit_event.getCause()

