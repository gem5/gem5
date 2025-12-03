import argparse

import m5
from m5.objects import *


# parse the arguments
def argument_parse():
    parser = argparse.ArgumentParser(description="DSE arguments")

    # location of the binary file
    parser.add_argument(
        "--binary",
        type=str,
        default="/home/abg309/PhD/RCL/gem5-dse/workload/defaultGait/x86/controller_code_cpp",
        help="Location of the binary file",
    )

    # input array
    parser.add_argument(
        "--input",
        type=str,
        nargs="+",
        default=["1.0", "2.0", "3.0"],
        help="Input array",
    )

    args = parser.parse_args()

    return args


# Create a system
args = argument_parse()


system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "3.4GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "atomic"
system.mem_ranges = [AddrRange("6MB")]

# system.cpu = X86TimingSimpleCPU()
system.cpu = X86AtomicSimpleCPU()


system.membus = SystemXBar()

system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports
system.cpu.createInterruptController()
system.cpu.interrupts[0].pio = system.membus.mem_side_ports
system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

binary = args.binary

cmd = [binary]
cmd.extend(args.input)


# for gem5 V21 and beyond
system.workload = SEWorkload.init_compatible(binary)


process = Process()
process.cmd = cmd
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system=False, system=system)
m5.instantiate()

exit_event = m5.simulate()
