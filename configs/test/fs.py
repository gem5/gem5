import optparse, os, sys

import m5
from m5.objects import *
m5.AddToPath('../common')
from FSConfig import *
from SysPaths import *
from Util import *

parser = optparse.OptionParser()

parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-m", "--maxtick", type="int")
parser.add_option("--maxtime", type="float")
parser.add_option("--dual", help="Run full system using dual systems",
                  action="store_true")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

if options.detailed:
    cpu = DetailedO3CPU()
    cpu2 = DetailedO3CPU()
    mem_mode = 'Timing'
elif options.timing:
    cpu = TimingSimpleCPU()
    cpu2 = TimingSimpleCPU()
    mem_mode = 'Timing'
else:
    cpu = AtomicSimpleCPU()
    cpu2 = AtomicSimpleCPU()
    mem_mode = 'Atomic'

if options.dual:
    root = DualRoot(
        MyLinuxAlphaSystem(cpu, mem_mode, linux_image),
        MyLinuxAlphaSystem(cpu2, mem_mode, linux_image))
    root.client.readfile = script('netperf-stream-nt-client.rcS')
    root.server.readfile = script('netperf-server.rcS')
else:
    root = TsunamiRoot(clock = '1THz', system = MyLinuxAlphaSystem(cpu, mem_mode, linux_image))

m5.instantiate(root)

#exit_event = m5.simulate(2600000000000)
#if exit_event.getCause() != "user interrupt received":
#    m5.checkpoint(root, 'cpt')
#    exit_event = m5.simulate(300000000000)
#    if exit_event.getCause() != "user interrupt received":
#        m5.checkpoint(root, 'cptA')


if options.maxtick:
    exit_event = m5.simulate(options.maxtick)
elif options.maxtime:
    simtime = int(options.maxtime * root.clock.value)
    print "simulating for: ", simtime
    exit_event = m5.simulate(simtime)
else:
    exit_event = m5.simulate()

print 'Exiting @ cycle', m5.curTick(), 'because', exit_event.getCause()
