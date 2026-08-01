# Copyright (c) 2024-2026 Arm Limited
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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2009 Advanced Micro Devices, Inc.
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
import importlib.util
import os
import sys

import m5
from m5.objects import (
    RubyPortProxy,
    RubySystem,
    SrcClockDomain,
    SubSystem,
    TlmController,
    TlmGenerator,
    VoltageDomain,
)
from m5.params import (
    AllMemory,
    Port,
)

from gem5.components.boards.test_board import TestBoard
from gem5.components.cachehierarchies.chi.nodes.abstract_node import (
    CacheController,
)
from gem5.components.cachehierarchies.chi.nodes.directory import (
    SimpleDirectory,
)
from gem5.components.cachehierarchies.chi.nodes.memory_controller import (
    MemoryController,
)
from gem5.components.cachehierarchies.chi.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.topologies.simple_pt2pt import (
    SimplePt2Pt,
)
from gem5.components.memory.simple import SingleChannelSimpleMemory
from gem5.components.processors.abstract_generator import AbstractGenerator
from gem5.components.processors.abstract_generator_core import (
    AbstractGeneratorCore,
)
from gem5.isas import ISA
from gem5.utils.override import overrides


class TlmGeneratorCore(AbstractGeneratorCore):
    """Stdlib core wrapper for a CHI TLM generator.

    TlmGenerator does not use the usual stdlib CPU-side cache port path. The
    hierarchy connects it to a TlmController using CHI TLM ports instead.
    """

    def __init__(
        self,
        cpu_id: int,
        max_pending_tran: int,
        clk_domain: SrcClockDomain,
    ):
        super().__init__()
        self.generator = TlmGenerator(
            cpu_id=cpu_id,
            max_pending_tran=max_pending_tran,
            clk_domain=clk_domain,
        )

    @overrides(AbstractGeneratorCore)
    def connect_dcache(self, port: Port) -> None:
        self.port_end.req_ports = port

    @overrides(AbstractGeneratorCore)
    def start_traffic(self) -> None:
        pass

    def get_tlm_generator(self) -> TlmGenerator:
        return self.generator


class TlmGeneratorProcessor(AbstractGenerator):
    """Stdlib processor wrapper owning one TlmGeneratorCore per RN-F."""

    def __init__(
        self,
        num_cores: int,
        max_pending_tran: int,
        clk_domain: SrcClockDomain,
    ):
        super().__init__(
            cores=[
                TlmGeneratorCore(
                    cpu_id=i,
                    max_pending_tran=max_pending_tran,
                    clk_domain=clk_domain,
                )
                for i in range(num_cores)
            ]
        )

    @overrides(AbstractGenerator)
    def start_traffic(self) -> None:
        pass

    @overrides(AbstractGenerator)
    def get_isa(self) -> ISA:
        return ISA.NULL

    def get_tlm_generators(self) -> list[TlmGenerator]:
        return [core.get_tlm_generator() for core in self.get_cores()]


class TlmPrivateL1PrivateL2CacheHierarchy(PrivateL1PrivateL2CacheHierarchy):
    """PrivateL1PrivateL2-derived CHI hierarchy for TLM RN-F tests.

    The base class provides the stdlib CHI hierarchy shape and version reset
    logic. This subclass keeps the Ruby system, point-to-point network,
    directory, and memory-controller construction, but replaces each CPU
    cluster with a single TlmController RN-F and no RubySequencers, L1s, or
    L2s.
    """

    def __init__(
        self,
        ruby_clock: str,
        voltage_domain: VoltageDomain,
    ):
        super().__init__(
            l1i_size="1B",
            l1i_assoc=1,
            l1d_size="1B",
            l1d_assoc=1,
            l2_size="1B",
            l2_assoc=1,
        )
        self._ruby_clock = ruby_clock
        self._voltage_domain = voltage_domain

    def _create_core_cluster(
        self,
        core: TlmGeneratorCore,
        core_num: int,
        board: TestBoard,
    ) -> SubSystem:
        """Create a one-controller TLM RN-F cluster."""

        cluster = SubSystem()
        cluster.rnf = TlmController(
            version=CacheController.versionCount(),
            ruby_system=self.ruby_system,
        )
        cluster.rnf.data_channel_size = 32
        cluster.rnf.connectQueues(self.ruby_system.network)

        generator = core.get_tlm_generator()
        generator.out_port = cluster.rnf.in_port
        generator.in_port = cluster.rnf.out_port

        cluster.rnf.ruby_system = self.ruby_system

        cluster.rnf.downstream_destinations = [self.directory]

        return cluster

    @overrides(PrivateL1PrivateL2CacheHierarchy)
    def incorporate_cache(self, board: TestBoard) -> None:
        """Build the CHI Ruby system around TLM RN-F controllers."""

        self._reset_version_numbers()
        self.ruby_system = RubySystem()

        self.ruby_system.clk_domain = SrcClockDomain(
            clock=self._ruby_clock,
            voltage_domain=self._voltage_domain,
        )
        self.ruby_system.randomization = False
        self.ruby_system.network = SimplePt2Pt(self.ruby_system)
        self.ruby_system.number_of_virtual_networks = 4
        self.ruby_system.network.number_of_virtual_networks = 4

        self.directory = SimpleDirectory(
            self.ruby_system.network,
            cache_line_size=board.get_cache_line_size(),
            clk_domain=board.get_clock_domain(),
            addr_ranges=[AllMemory],
        )
        self.directory.ruby_system = self.ruby_system

        self.core_clusters = [
            self._create_core_cluster(core, i, board)
            for i, core in enumerate(board.get_processor().get_cores())
        ]

        self.memory_controllers = [
            MemoryController(self.ruby_system.network, rng, port)
            for rng, port in board.get_mem_ports()
        ]
        for controller in self.memory_controllers:
            controller.ruby_system = self.ruby_system

        self.directory.downstream_destinations = self.memory_controllers

        self.ruby_system.num_of_sequencers = 0
        self.ruby_system.network.connectControllers(
            [cluster.rnf for cluster in self.core_clusters]
            + self.memory_controllers
            + [self.directory]
            + (self.dma_controllers if board.has_dma_ports() else [])
        )
        self.ruby_system.network.setup_buffers()

        self.ruby_system.sys_port_proxy = RubyPortProxy(
            ruby_system=self.ruby_system
        )
        board.connect_system_port(self.ruby_system.sys_port_proxy.in_ports)


def suite_importer(file_path):
    """
    Used to import the suite file.
    :param file_path: Path of the file to import
    """
    module_name = os.path.basename(file_path)
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument(
    "suite",
    type=str,
    help="Path to the suite file",
)
parser.add_argument("-n", "--num-cpus", type=int, default=1)
parser.add_argument(
    "--sys-voltage",
    type=str,
    default="1.0V",
    help="Top-level voltage",
)
parser.add_argument(
    "--sys-clock",
    type=str,
    default="1GHz",
    help="Top-level clock",
)
parser.add_argument(
    "--ruby-clock",
    type=str,
    default="2GHz",
    help="Ruby clock",
)
parser.add_argument(
    "--tester-clock",
    type=str,
    default="3GHz",
    help="Tester clock frequency",
)
parser.add_argument(
    "--tester-max-pending",
    type=int,
    default=64,
    help="Maximum number of pending transactions",
)
parser.add_argument("--mem-size", type=str, default="4GiB")
parser.add_argument("--cacheline-size", type=int, default=64)
parser.add_argument(
    "-m",
    "--abs-max-tick",
    type=int,
    default=m5.MaxTick,
    metavar="TICKS",
    help="Run to absolute simulated tick",
)

args = parser.parse_args()

suite = suite_importer(args.suite)

voltage_domain = VoltageDomain(voltage=args.sys_voltage)
tester_clk_domain = SrcClockDomain(
    clock=args.tester_clock,
    voltage_domain=voltage_domain,
)

processor = TlmGeneratorProcessor(
    num_cores=args.num_cpus,
    max_pending_tran=args.tester_max_pending,
    clk_domain=tester_clk_domain,
)

memory = SingleChannelSimpleMemory(
    latency="30ns",
    latency_var="0ns",
    bandwidth="12.8GiB/s",
    size=args.mem_size,
)

cache_hierarchy = TlmPrivateL1PrivateL2CacheHierarchy(
    ruby_clock=args.ruby_clock,
    voltage_domain=voltage_domain,
)

board = TestBoard(
    clk_freq=args.sys_clock,
    generator=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)
board.cache_line_size = args.cacheline_size

root = board._pre_instantiate()
m5.instantiate()

for verifier in suite.test_all(processor.get_tlm_generators()):
    while True:
        exit_event = m5.simulate(args.abs_max_tick)

        print(
            "Exiting @ tick",
            m5.curTick(),
            "because",
            exit_event.getCause(),
        )

        if exit_event.getCause() != "TlmGenerator done" or not any(
            c.isActive() for c in processor.get_tlm_generators()
        ):
            break

    m5.drain()

    verifier(board)
