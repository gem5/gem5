# Copyright (c) 2016-2017, 2020-2022, 2025-2026 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
import os
import sys
from typing import List

import m5
from m5.objects import (
    ArmFsLinux,
    CowDiskImage,
    MinorCPU,
    NonCachingSimpleCPU,
    PciVirtIO,
    Root,
    VExpress_GEM5_Foundation,
    VirtIOBlock,
)
from m5.util import addToPath

addToPath("../..")

import devices
from common import SysPaths
from common.cores.arm import (
    HPI,
    O3_ARM_v7a,
)

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.cachehierarchies.chi.nodes.abstract_node import (
    CHI_NodeType,
)
from gem5.components.cachehierarchies.ruby.topologies.custom_mesh import (
    load_topology_config,
)
from gem5.components.memory.dram_interfaces.ddr4 import DDR4_2400_16x4
from gem5.components.memory.memory import ChanneledMemory
from gem5.isas import ISA
from gem5.utils.requires import requires

default_kernel = "vmlinux.arm64"
default_disk = "linaro-minimal-aarch64.img"
default_root_device = "/dev/vda1"

cpu_types = {
    "noncaching": NonCachingSimpleCPU,
    "minor": MinorCPU,
    "hpi": HPI.HPI,
    "o3": O3_ARM_v7a.O3_ARM_v7a_3,
}


def create_cow_image(name):
    image = CowDiskImage()
    image.child.image_file = SysPaths.disk(name)
    return image


class _LegacyCoreAdapter:
    def __init__(self, cpu):
        self._cpu = cpu

    def requires_send_evicts(self) -> bool:
        # Mirror stdlib ARM behavior.
        return True

    def connect_icache(self, port):
        self._cpu.icache_port = port

    def connect_dcache(self, port):
        self._cpu.dcache_port = port

    def connect_walker_ports(self, port1, port2):
        self._cpu.mmu.connectWalkerPorts(port1, port2)

    def connect_interrupt(self):
        self._cpu.createInterruptController()


class _LegacyProcessorAdapter:
    def __init__(self, cluster):
        self._cluster = cluster
        self._cores = [_LegacyCoreAdapter(cpu) for cpu in cluster]

    def get_cores(self):
        return self._cores

    def get_isa(self):
        return ISA.ARM

    def get_clock_domain(self):
        return self._cluster.clk_domain


class _LegacyBoardAdapter:
    def __init__(self, system, processor_adapter, memory):
        self._system = system
        self._system.memory = memory
        self._processor = processor_adapter

    def get_processor(self):
        return self._processor

    def get_mem_ranges(self):
        return self._system.memory.get_uninterleaved_range() + [
            self._system.realview.bootmem.range
        ]

    def get_mem_ports(self):
        all_ports = [
            (
                self._system.realview.bootmem.range,
                self._system.realview.bootmem.port,
            ),
        ] + self._system.memory.get_mem_ports()

        return all_ports

    def has_dma_ports(self):
        return len(self._system._dma_ports) > 0

    def get_dma_ports(self):
        return self._system._dma_ports

    def has_io_bus(self):
        return True

    def get_io_bus(self):
        return self._system.iobus

    def get_cache_line_size(self):
        return self._system.cache_line_size

    def get_clock_domain(self):
        return self._system.clk_domain

    def connect_system_port(self, port):
        self._system.system_port = port


def create(args):
    requires(
        isa_required=ISA.ARM,
        coherence_protocol_required=CoherenceProtocol.CHI,
    )

    if args.script and not os.path.isfile(args.script):
        print(f"Error: Bootscript {args.script} does not exist")
        sys.exit(1)

    cpu_class = cpu_types[args.cpu]
    mem_mode = cpu_class.memory_mode()

    system = devices.ArmRubySystem(
        args.mem_size,
        platform=VExpress_GEM5_Foundation(),
        mem_mode=mem_mode,
        workload=ArmFsLinux(object_file=SysPaths.binary(args.kernel)),
        readfile=args.script,
    )

    system.cpu_cluster = devices.ArmCpuCluster(
        system,
        args.num_cpus,
        args.cpu_freq,
        "1.0V",
        cpu_class,
        None,
        None,
        None,
    )

    topology_config = load_topology_config(args.topology_config)

    # Import the cache hierarchy
    cache_hierarchy = topology_config.cache_hierarchy_type
    # Import the topology node parameters
    get_node_params = topology_config.get_node_params
    rnf_in_mesh = get_node_params(CHI_NodeType.CHI_RNF)
    snf_mainmem_in_mesh = get_node_params(CHI_NodeType.CHI_SNF_MainMem)

    if args.num_cpus != rnf_in_mesh.num_nodes():
        raise ValueError(
            f"--num-cpus ({args.num_cpus}) must match topology-config "
            f"RNF node count ({rnf_in_mesh.num_nodes()})."
        )

    system.pci_devices = [
        PciVirtIO(vio=VirtIOBlock(image=create_cow_image(args.disk_image)))
    ]
    for dev in system.pci_devices:
        system.attach_pci(dev)

    system.connect()

    # Setup the system memory.
    num_channels = snf_mainmem_in_mesh.num_nodes()
    memory = ChanneledMemory(
        dram_interface_class=DDR4_2400_16x4,
        num_channels=num_channels,
        interleaving_size=64,
        size=args.mem_size,
    )
    if len(system.mem_ranges) != 1:
        raise ValueError(
            "This example requires a single contiguous system mem range."
        )
    memory.set_memory_range(system.mem_ranges)

    proc_adapter = _LegacyProcessorAdapter(system.cpu_cluster)
    board_adapter = _LegacyBoardAdapter(system, proc_adapter, memory)

    system.mesh_cache = cache_hierarchy()
    system.mesh_cache.add_default_nodes()
    system.mesh_cache.incorporate_cache(board_adapter)

    system.realview.setupBootLoader(system, SysPaths.binary)
    system.workload.dtb_filename = os.path.join(
        m5.options.outdir, "system.dtb"
    )
    system.generateDtb(system.workload.dtb_filename)
    system.workload.command_line = " ".join(
        [
            "console=ttyAMA0",
            "lpj=19988480",
            "norandmaps",
            f"root={args.root_device}",
            "rw",
            f"mem={args.mem_size}",
        ]
    )

    return system


def run():
    event = m5.simulate()
    print(event.getCause(), " @ ", m5.curTick())
    sys.exit(event.getCode())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kernel", type=str, default=default_kernel)
    parser.add_argument("--disk-image", type=str, default=default_disk)
    parser.add_argument("--root-device", type=str, default=default_root_device)
    parser.add_argument("--script", type=str, default="")
    parser.add_argument(
        "--cpu", choices=list(cpu_types.keys()), default="minor"
    )
    parser.add_argument("--cpu-freq", type=str, default="4GHz")
    parser.add_argument("-n", "--num-cpus", type=int, default=4)
    parser.add_argument("--mem-size", type=str, default="2GiB")
    parser.add_argument(
        "--topology-config",
        type=str,
        required=True,
        help=(
            "Path to a Python topology config module that defines "
            "cache_hierarchy and get_node_params(node_type: CHI_NodeType) "
            "-> Node_Params."
        ),
    )
    args = parser.parse_args()

    root = Root(full_system=True)
    root.system = create(args)
    m5.instantiate()
    run()


if __name__ == "__m5_main__":
    main()
