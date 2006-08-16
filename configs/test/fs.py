import optparse, os, sys

import m5
from m5.objects import *
m5.AddToPath('../common')
from FSConfig import *
from SysPaths import *
from Benchmarks import *

parser = optparse.OptionParser()

parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-m", "--maxtick", type="int")
parser.add_option("--maxtime", type="float")
parser.add_option("--dual", action="store_true",
                  help="Simulate two systems attached with an ethernet link")
parser.add_option("-b", "--benchmark", action="store", type="string",
                  dest="benchmark",
                  help="Specify the benchmark to run. Available benchmarks: %s"\
                          % DefinedBenchmarks)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

if options.detailed:
    cpu = DetailedO3CPU()
    cpu2 = DetailedO3CPU()
    mem_mode = 'timing'
elif options.timing:
    cpu = TimingSimpleCPU()
    cpu2 = TimingSimpleCPU()
    mem_mode = 'timing'
else:
    cpu = AtomicSimpleCPU()
    cpu2 = AtomicSimpleCPU()
    mem_mode = 'atomic'


if options.benchmark:
    if options.benchmark not in Benchmarks:
        print "Error benchmark %s has not been defined." % options.benchmark
        print "Valid benchmarks are: %s" % DefinedBenchmarks
        sys.exit(1)

    bm = Benchmarks[options.benchmark]

    if len(bm) == 2:
        s1 = makeLinuxAlphaSystem(mem_mode, bm[0])
        s2 = makeLinuxAlphaSystem(mem_mode, bm[1])
        cpu.connectMemPorts(s1.membus)
        cpu2.connectMemPorts(s2.membus)
        root = makeDualRoot(s1, s2)
    elif len(bm) == 1:
        root = Root(clock = '1THz',
                    system = makeLinuxAlphaSystem(mem_mode, bm[0]))
        cpu.connectMemPorts(root.system.membus)
    else:
        print "Error I don't know how to create more than 2 systems."
        sys.exit(1)

else:
    if options.dual:
        root = makeDualRoot(
            makeLinuxAlphaSystem(cpu, mem_mode, Machine()),
            makeLinuxAlphaSystem(cpu2, mem_mode, Machine()))
    else:
        root = Root(clock = '1THz',
                    system = makeLinuxAlphaSystem(cpu, mem_mode, Machine()))

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
