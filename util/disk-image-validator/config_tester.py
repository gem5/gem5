# Copyright (c) 2025 The Regents of the University of California
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
import sys

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.exit_handler import ExitHandler
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires


# Define test functions
def MESI_cache_test(workload_id, resource_version):
    print("in mesi test")
    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1d_size="16KiB",
        l1d_assoc=8,
        l1i_size="16KiB",
        l1i_assoc=8,
        l2_size="256KiB",
        l2_assoc=16,
        num_l2_banks=1,
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=2
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with MESI cache. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def KVM_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.KVM, isa=ISA.X86, num_cores=1
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with KVM CPU. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def O3_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(cpu_type=CPUTypes.O3, isa=ISA.X86, num_cores=1)

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with O3 CPU. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def MINOR_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.MINOR, isa=ISA.X86, num_cores=1
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with MINOR CPU. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def TIMING_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.TIMING, isa=ISA.X86, num_cores=1
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with TIMING CPU. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def ATOMIC_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=1
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with ATOMIC CPU. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def ATOMIC_2_core_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=2
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with ATOMIC CPU, 2 cores. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def ATOMIC_4_core_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=4
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with ATOMIC CPU, 4 cores. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


def ATOMIC_8_core_test(workload_id, resource_version):
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="64KiB", l1i_size="64KiB", l2_size="8MiB"
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=8
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_workload(
        obtain_resource(workload_id, resource_version=resource_version)
    )

    class KernelBootedExit(ExitHandler, hypercall_num=1):
        def _process(self, simulator: "Simulator") -> None:
            print("Kernel Booted with ATOMIC CPU, 8 cores. Test Passed")

        def _exit_simulation(self) -> bool:
            return True

    simulator = Simulator(board=board)
    simulator.run()
    return True


TEST_FUNCTIONS = {
    "MESI_cache_test": MESI_cache_test,
    "KVM_test": KVM_test,
    "O3_test": O3_test,
    "MINOR_test": MINOR_test,
    "TIMING_test": TIMING_test,
    "ATOMIC_test": ATOMIC_test,
    "ATOMIC_2_core_test": ATOMIC_2_core_test,
    "ATOMIC_4_core_test": ATOMIC_4_core_test,
    "ATOMIC_8_core_test": ATOMIC_8_core_test,
}


def main():
    parser = argparse.ArgumentParser(
        description="Run a gem5 configuration test"
    )
    parser.add_argument(
        "--config",
        required=True,
        help="The configuration test to run (e.g., MESI_cache_test, KVM_test, O3_test, etc.)",
    )
    parser.add_argument("--workload", required=True, help="The workload ID")
    parser.add_argument(
        "--resource_version", required=True, help="The resource version"
    )

    args = parser.parse_args()
    test_name = args.config
    workload = args.workload
    resource_version = args.resource_version

    # Call the correct function dynamically
    test_func = TEST_FUNCTIONS.get(test_name)
    if test_func:
        try:
            result = test_func(workload, resource_version)
            if result:
                print("PASS")
                sys.exit(0)
            else:
                print("FAIL")
                sys.exit(1)
        except Exception as e:
            print(f" Error running {test_name}: {e}")
            sys.exit(1)
    else:
        print(f"Error: Test {test_name} not found")
        sys.exit(1)


if __name__ == "__main__" or __name__ == "__m5_main__":
    main()
