import optparse
import sys
import subprocess

import m5
from m5.objects import *
from m5.util import addToPath

addToPath('../')

from common import MemConfig
from common import HMC

parser = optparse.OptionParser()

# Use a HMC_2500_x32 by default
parser.add_option("--mem-type", type = "choice", default = "HMC_2500_x32",
                  choices = MemConfig.mem_names(),
                  help = "type of memory to use")

parser.add_option("--ranks", "-r", type = "int", default = 1,
                  help = "Number of ranks to iterate across")

parser.add_option("--rd_perc", type ="int", default=100,
                  help = "Percentage of read commands")

parser.add_option("--mode", type ="choice", default ="DRAM",
                  choices = ["DRAM", "DRAM_ROTATE", "RANDOM"],
                  help = "DRAM: Random traffic; \
                          DRAM_ROTATE: Traffic rotating across banks and ranks"
                          )

parser.add_option("--addr_map", type ="int", default = 1,
                  help = "0: RoCoRaBaCh; 1: RoRaBaCoCh/RoRaBaChCo")

parser.add_option("--arch", type = "choice", default = "distributed",
                  choices = ["same", "distributed", "mixed"],
                  help = "same: HMC-4 links with same range\
                  distributed: HMC-4 links with distributed range\
                  mixed: mixed with same & distributed range")

parser.add_option("--linkaggr", type = "int", default = 0,
                  help = "1: enable link crossbar, 0: disable link crossbar")

parser.add_option("--num_cross", type = "int", default = 4,
                  help = "1: number of crossbar in HMC=1;\
                  4: number of crossbar = 4")

parser.add_option("--tlm-memory", type = "string",
                  help="use external port for SystemC TLM cosimulation")

parser.add_option("--elastic-trace-en", action ="store_true",
                  help = """Enable capture of data dependency and instruction
                  fetch traces using elastic trace probe.""")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

system = System()
system.clk_domain = SrcClockDomain(clock='100GHz',
                                   voltage_domain=
                                   VoltageDomain(voltage = '1V'))
# Create additional crossbar for arch1
if options.arch == "distributed" or options.arch == "mixed" :
    system.membus = NoncoherentXBar( width=8 )
    system.membus.badaddr_responder = BadAddr()
    system.membus.default = Self.badaddr_responder.pio
    system.membus.width = 8
    system.membus.frontend_latency = 3
    system.membus.forward_latency = 4
    system.membus.response_latency = 2

    system.membus.clk_domain = SrcClockDomain(clock='100GHz', voltage_domain=
            VoltageDomain(voltage = '1V'))

# we are considering 4GB HMC device with following parameters
# hmc_device_size = '4GB'
# hmc_num_vaults = 16
# hmc_vault_size = '256MB'
# hmc_stack_size = 8
# hmc_bank_in_stack = 2
# hmc_bank_size = '16MB'
# hmc_bank_in_vault = 16

# determine the burst length in bytes
burst_size = 256
num_serial_links = 4
num_vault_ctrl = 16
options.mem_channels = 1
options.external_memory_system = 0
options.mem_ranks=1
stride_size = burst_size
system.cache_line_size = burst_size

# Enable performance monitoring
options.enable_global_monitor = True
options.enable_link_monitor = False

# Bytes used for calculations
oneGBytes = 1024 * 1024 * 1024
oneMBytes = 1024 * 1024

# Memory ranges of 16 vault controller - Total_HMC_size / 16
mem_range_vault = [ AddrRange(i * 256 * oneMBytes, ((i + 1) * 256 * oneMBytes)
    - 1)
        for i in range(num_vault_ctrl)]

# Memmory ranges of serial link for arch-0
# Same as the ranges of vault controllers - 4 vault - to - 1 serial link
if options.arch == "same":
    ser_range  = [ AddrRange(0, (4 * oneGBytes) - 1)
            for i in range(num_serial_links)]
    options.ser_ranges = ser_range

# Memmory ranges of serial link for arch-1
# Distributed range accross links
if options.arch == "distributed":
    ser_range  = [ AddrRange(i * oneGBytes, ((i + 1) * oneGBytes) - 1)
            for i in range(num_serial_links)]
    options.ser_ranges = ser_range

# Memmory ranges of serial link for arch-2
# "Mixed" address distribution over links
if options.arch == "mixed":
    ser_range0  = AddrRange(0              , (1 * oneGBytes) - 1)
    ser_range1  = AddrRange(1 * oneGBytes  , (2 * oneGBytes) - 1)
    ser_range2  = AddrRange(0              , (4 * oneGBytes) - 1)
    ser_range3  = AddrRange(0              , (4 * oneGBytes) - 1)
    options.ser_ranges = [ser_range0, ser_range1, ser_range2, ser_range3]

# Assign ranges of vault controller to system ranges
system.mem_ranges = mem_range_vault

# open traffic generator
cfg_file_name = "./tests/quick/se/70.tgen/traffic.cfg"
cfg_file = open(cfg_file_name, 'r')

# number of traffic generator
np = 4
# create a traffic generator, and point it to the file we just created
system.tgen = [ TrafficGen(config_file = cfg_file_name) for i in xrange(np)]

# Config memory system with given HMC arch
MemConfig.config_mem(options, system)

if options.arch == "distributed":
    for i in xrange(np):
        system.tgen[i].port = system.membus.slave
    # connect the system port even if it is not used in this example
    system.system_port = system.membus.slave

if options.arch == "mixed":
    for i in xrange(int(np/2)):
        system.tgen[i].port = system.membus.slave
    # connect the system port even if it is not used in this example
    system.system_port = system.membus.slave


# run Forrest, run!
root = Root(full_system = False, system = system)
root.system.mem_mode = 'timing'

m5.instantiate()
m5.simulate(10000000000)

m5.stats.dump()

print "Done!"
