from __future__ import print_function

import sys
import argparse
import subprocess
from pprint import pprint

import m5
from m5.objects import *
from m5.util import *

addToPath('../')

from common import MemConfig
from common import HMC


def add_options(parser):
    parser.add_argument("--external-memory-system", default=0, action="store",
                        type=int, help="External memory system")
    # TLM related options, currently optional in configs/common/MemConfig.py
    parser.add_argument("--tlm-memory", action="store_true", help="use\
                        external port for SystemC TLM co-simulation. Default:\
                        no")
    # Elastic traces related options, currently optional in
    # configs/common/MemConfig.py
    parser.add_argument("--elastic-trace-en", action="store_true",
                        help="enable capture of data dependency and\
                        instruction fetch traces using elastic trace\
                        probe.\nDefault: no")
    # Options related to traffic generation
    parser.add_argument("--num-tgen", default=4, action="store", type=int,
                        choices=[4], help="number of traffic generators.\
                        Right now this script supports only 4.\nDefault: 4")
    parser.add_argument("--tgen-cfg-file",
                        default="./configs/example/hmc_tgen.cfg",
                        type=str, help="Traffic generator(s) configuration\
                        file. Note: this script uses the same configuration\
                        file for all traffic generators")


# considering 4GB HMC device with following parameters
# hmc_device_size = '4GB'
# hmc_vault_size = '256MB'
# hmc_stack_size = 8
# hmc_bank_in_stack = 2
# hmc_bank_size = '16MB'
# hmc_bank_in_vault = 16
def build_system(options):
    # create the system we are going to simulate
    system = System()
    # use timing mode for the interaction between master-slave ports
    system.mem_mode = 'timing'
    # set the clock fequency of the system
    clk = '100GHz'
    vd = VoltageDomain(voltage='1V')
    system.clk_domain = SrcClockDomain(clock=clk, voltage_domain=vd)
    # add traffic generators to the system
    system.tgen = [TrafficGen(config_file=options.tgen_cfg_file) for i in
                   range(options.num_tgen)]
    # Config memory system with given HMC arch
    MemConfig.config_mem(options, system)
    # Connect the traffic generatiors
    if options.arch == "distributed":
        for i in range(options.num_tgen):
            system.tgen[i].port = system.membus.slave
        # connect the system port even if it is not used in this example
        system.system_port = system.membus.slave
    if options.arch == "mixed":
        for i in range(int(options.num_tgen/2)):
            system.tgen[i].port = system.membus.slave
        hh = system.hmc_host
        if options.enable_global_monitor:
            system.tgen[2].port = hh.lmonitor[2].slave
            hh.lmonitor[2].master = hh.seriallink[2].slave
            system.tgen[3].port = hh.lmonitor[3].slave
            hh.lmonitor[3].master = hh.seriallink[3].slave
        else:
            system.tgen[2].port = hh.seriallink[2].slave
            system.tgen[3].port = hh.seriallink[3].slave
        # connect the system port even if it is not used in this example
        system.system_port = system.membus.slave
    if options.arch == "same":
        hh = system.hmc_host
        for i in range(options.num_links_controllers):
            if options.enable_global_monitor:
                system.tgen[i].port = hh.lmonitor[i].slave
            else:
                system.tgen[i].port = hh.seriallink[i].slave
    # set up the root SimObject
    root = Root(full_system=False, system=system)
    return root


def main():
    parser = argparse.ArgumentParser(description="Simple system using HMC as\
                                     main memory")
    HMC.add_options(parser)
    add_options(parser)
    options = parser.parse_args()
    # build the system
    root = build_system(options)
    # instantiate all of the objects we've created so far
    m5.instantiate()
    print("Beginning simulation!")
    event = m5.simulate(10000000000)
    m5.stats.dump()
    print('Exiting @ tick %i because %s (exit code is %i)' % (m5.curTick(),
                                                              event.getCause(),
                                                              event.getCode()))
    print("Done")


if __name__ == "__m5_main__":
    main()
