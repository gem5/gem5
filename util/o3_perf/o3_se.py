#!/usr/bin/env python3

# Copyright (c) 2026 Magnushst
# All rights reserved.
#
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

import argparse

import m5
from m5.objects import (
    X86O3CPU,
    AddrRange,
    Cache,
    DDR3_1600_8x8,
    L2XBar,
    MemCtrl,
    Process,
    Root,
    SEWorkload,
    SrcClockDomain,
    System,
    SystemXBar,
    VoltageDomain,
)


class L1Cache(Cache):
    assoc = 2
    data_latency = 2
    mshrs = 4
    response_latency = 2
    size = "32KiB"
    tag_latency = 2
    tgts_per_mshr = 20


class L2Cache(Cache):
    assoc = 8
    data_latency = 20
    mshrs = 16
    response_latency = 20
    size = "256KiB"
    tag_latency = 20
    tgts_per_mshr = 12


CPU_CONFIGS = {
    "o3-small": {
        "fetchWidth": 4,
        "decodeWidth": 4,
        "renameWidth": 4,
        "dispatchWidth": 4,
        "issueWidth": 4,
        "wbWidth": 4,
        "commitWidth": 4,
        "numROBEntries": 64,
        "iq_entries": 32,
        "LQEntries": 16,
        "SQEntries": 16,
        "numPhysIntRegs": 128,
    },
    "o3-medium": {
        "fetchWidth": 8,
        "decodeWidth": 8,
        "renameWidth": 8,
        "dispatchWidth": 8,
        "issueWidth": 8,
        "wbWidth": 8,
        "commitWidth": 8,
        "numROBEntries": 192,
        "iq_entries": 64,
        "LQEntries": 32,
        "SQEntries": 32,
        "numPhysIntRegs": 256,
    },
    "o3-large": {
        "fetchWidth": 12,
        "decodeWidth": 12,
        "renameWidth": 12,
        "dispatchWidth": 12,
        "issueWidth": 12,
        "wbWidth": 12,
        "commitWidth": 12,
        "numROBEntries": 384,
        "iq_entries": 128,
        "LQEntries": 64,
        "SQEntries": 64,
        "numPhysIntRegs": 512,
    },
}


def configure_cpu(cpu, name):
    parameters = CPU_CONFIGS[name]
    for parameter, value in parameters.items():
        if parameter != "iq_entries":
            setattr(cpu, parameter, value)
    cpu.instQueues[0].numEntries = parameters["iq_entries"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--workload", required=True)
    parser.add_argument("--iterations", required=True, type=int)
    parser.add_argument("--parameter", type=int)
    parser.add_argument(
        "--cpu-config",
        choices=CPU_CONFIGS,
        default="o3-medium",
    )
    parser.add_argument("--seed", default=1, type=int)
    args = parser.parse_args()
    m5.core.seedRandom(args.seed)

    system = System()
    system.clk_domain = SrcClockDomain(
        clock="1GHz", voltage_domain=VoltageDomain()
    )
    system.cpu_voltage_domain = VoltageDomain()
    system.cpu_clk_domain = SrcClockDomain(
        clock="3GHz", voltage_domain=system.cpu_voltage_domain
    )
    system.mem_mode = "timing"
    system.mem_ranges = [AddrRange("8GiB")]

    system.cpu = X86O3CPU(
        cpu_id=0,
        clk_domain=system.cpu_clk_domain,
    )
    configure_cpu(system.cpu, args.cpu_config)

    system.cpu.icache = L1Cache()
    system.cpu.dcache = L1Cache()
    system.l2bus = L2XBar()
    system.l2cache = L2Cache()
    system.membus = SystemXBar()

    system.cpu.icache.cpu_side = system.cpu.icache_port
    system.cpu.dcache.cpu_side = system.cpu.dcache_port
    system.cpu.icache.mem_side = system.l2bus.cpu_side_ports
    system.cpu.dcache.mem_side = system.l2bus.cpu_side_ports
    system.l2cache.cpu_side = system.l2bus.mem_side_ports
    system.l2cache.mem_side = system.membus.cpu_side_ports

    system.cpu.createInterruptController()
    system.cpu.interrupts[0].pio = system.membus.mem_side_ports
    system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR3_1600_8x8()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports
    system.system_port = system.membus.cpu_side_ports

    command = [args.binary, args.workload, str(args.iterations)]
    if args.parameter is not None:
        command.append(str(args.parameter))
    process = Process(pid=100)
    process.cmd = command
    process.executable = args.binary
    system.cpu.workload = process
    system.cpu.createThreads()
    system.workload = SEWorkload.init_compatible(args.binary)

    root = Root(full_system=False, system=system)
    m5.instantiate()
    exit_event = m5.simulate()
    print(f"Exiting @ tick {m5.curTick()} because " f"{exit_event.getCause()}")


if __name__ == "__m5_main__":
    main()
