# Copyright (c) 2026 The Regents of the University of California
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

"""
Multi-CPU X86 SoC with Ubuntu boot and hello world workload.

This simulation creates a configurable multi-core X86 System-on-Chip that:
1. Boots Ubuntu Linux using KVM acceleration for fast boot
2. Runs a "hello world" workload after system boot
3. Exits cleanly via `m5 exit` mechanism

Configuration options:
- Core count (default: 2)
- Boot CPU type (default: KVM)
- Execution CPU type (default: Timing)
- Cache sizes (L1D, L1I, L2)
- Memory size
- Clock frequency

Usage
-----

```
scons build/X86/gem5.opt -j`nproc`
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py
```

For custom configuration:
```
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 4 \
    --boot-cpu atomic \
    --exec-cpu timing \
    --memory-size 8GiB
```
"""

import argparse

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires


def parse_args():
    """Parse command line arguments for multi-CPU SoC configuration."""
    parser = argparse.ArgumentParser(
        description="Multi-CPU X86 SoC with Ubuntu boot and hello world workload"
    )

    # Core configuration
    parser.add_argument(
        "--num-cores",
        type=int,
        default=2,
        help="Number of CPU cores (default: 2)",
    )

    # CPU type configuration
    cpu_type_choices = ["kvm", "atomic", "timing", "o3", "minor"]
    parser.add_argument(
        "--boot-cpu",
        type=str,
        default="kvm",
        choices=cpu_type_choices,
        help="CPU type for boot phase (default: kvm)",
    )

    parser.add_argument(
        "--exec-cpu",
        type=str,
        default="timing",
        choices=cpu_type_choices,
        help="CPU type for execution phase (default: timing)",
    )

    parser.add_argument(
        "--no-switch",
        action="store_true",
        help="Disable CPU switching (use boot CPU for entire simulation)",
    )

    # Cache configuration
    parser.add_argument(
        "--l1d-size",
        type=str,
        default="32KiB",
        help="L1 data cache size (default: 32KiB)",
    )

    parser.add_argument(
        "--l1i-size",
        type=str,
        default="32KiB",
        help="L1 instruction cache size (default: 32KiB)",
    )

    parser.add_argument(
        "--l2-size",
        type=str,
        default="512KiB",
        help="L2 cache size (default: 512KiB)",
    )

    # Memory configuration
    parser.add_argument(
        "--memory-size",
        type=str,
        default="3GiB",
        help="Main memory size (default: 3GiB)",
    )

    # Clock frequency
    parser.add_argument(
        "--clk-freq",
        type=str,
        default="3GHz",
        help="System clock frequency (default: 3GHz)",
    )

    # KVM perf setting
    parser.add_argument(
        "--kvm-perf",
        action="store_true",
        help="Enable perf for KVM CPUs (default: disabled)",
    )

    return parser.parse_args()


def get_cpu_type(cpu_str):
    """Map string CPU type to CPUTypes enum."""
    cpu_map = {
        "kvm": CPUTypes.KVM,
        "atomic": CPUTypes.ATOMIC,
        "timing": CPUTypes.TIMING,
        "o3": CPUTypes.O3,
        "minor": CPUTypes.MINOR,
    }
    return cpu_map[cpu_str.lower()]


def configure_system(args):
    """Configure the multi-CPU SoC based on command line arguments."""

    # Validate requirements
    requires(
        isa_required=ISA.X86,
        coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
        kvm_required=(args.boot_cpu == "kvm" or args.exec_cpu == "kvm"),
    )

    # Configure cache hierarchy
    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1d_size=args.l1d_size,
        l1d_assoc=8,
        l1i_size=args.l1i_size,
        l1i_assoc=8,
        l2_size=args.l2_size,
        l2_assoc=16,
        num_l2_banks=1,
    )

    # Configure memory
    memory = SingleChannelDDR4_2400(size=args.memory_size)

    # Configure processor
    if args.no_switch:
        # Use single CPU type throughout
        processor = SimpleSwitchableProcessor(
            starting_core_type=get_cpu_type(args.boot_cpu),
            switch_core_type=get_cpu_type(args.boot_cpu),  # Same as starting
            isa=ISA.X86,
            num_cores=args.num_cores,
        )
    else:
        # Switch from boot CPU to execution CPU
        processor = SimpleSwitchableProcessor(
            starting_core_type=get_cpu_type(args.boot_cpu),
            switch_core_type=get_cpu_type(args.exec_cpu),
            isa=ISA.X86,
            num_cores=args.num_cores,
        )

    # Disable perf for KVM CPUs if not explicitly enabled
    if not args.kvm_perf:
        for proc in processor.start:
            if hasattr(proc.core, "usePerf"):
                proc.core.usePerf = False

    # Configure board
    board = X86Board(
        clk_freq=args.clk_freq,
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    # Set workload
    workload = obtain_resource("x86-ubuntu-24.04-boot-with-systemd")
    board.set_workload(workload)

    return board, processor


def create_exit_event_handler(processor, args):
    """Create exit event handler for boot and workload execution phases."""

    def exit_event_handler():
        # Phase 1: Kernel boot complete
        print(
            f"[Phase 1] Ubuntu kernel booted with {args.num_cores} {args.boot_cpu.upper()} cores"
        )
        yield False  # Continue to systemd startup

        # Phase 2: Systemd started, ready for workload
        print("[Phase 2] Systemd started, ready for workload execution")

        if not args.no_switch and args.boot_cpu != args.exec_cpu:
            print(
                f"[Phase 2] Switching from {args.boot_cpu.upper()} to {args.exec_cpu.upper()} CPUs"
            )
            processor.switch()

        yield False  # Continue to workload execution

        # Phase 3: Workload complete, exit simulation
        print("[Phase 3] Workload execution complete")
        print("[Phase 3] Exiting simulation via m5 exit")
        yield True  # Terminate simulation

    return exit_event_handler


def run_simulation(board, processor, args):
    """Run the simulation with configured exit events."""

    simulator = Simulator(
        board=board,
        on_exit_event={
            ExitEvent.EXIT: create_exit_event_handler(processor, args)()
        },
    )

    print(f"\n{'='*60}")
    print("Starting Multi-CPU X86 SoC Simulation")
    print(f"{'='*60}")
    print(f"Configuration:")
    print(f"  Cores: {args.num_cores}")
    print(f"  Boot CPU: {args.boot_cpu.upper()}")
    print(f"  Exec CPU: {args.exec_cpu.upper()}")
    print(f"  CPU Switch: {'Disabled' if args.no_switch else 'Enabled'}")
    print(f"  L1D Cache: {args.l1d_size}")
    print(f"  L1I Cache: {args.l1i_size}")
    print(f"  L2 Cache: {args.l2_size}")
    print(f"  Memory: {args.memory_size}")
    print(f"  Clock: {args.clk_freq}")
    print(f"{'='*60}\n")

    simulator.run()

    print(f"\n{'='*60}")
    print("Simulation Complete")
    print(f"{'='*60}")


def create_hello_world_workload():
    """Create a simple hello world workload for testing."""
    import os
    import tempfile

    # Create a simple shell script that prints hello and exits
    script_content = """#!/bin/bash
echo "========================================"
echo "Hello from gem5 Multi-CPU X86 SoC!"
echo "Running on Ubuntu in full-system mode"
echo "Number of CPU cores configured: $1"
echo "========================================"
m5 exit
"""

    # Create temporary file
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".sh", delete=False
    ) as f:
        f.write(script_content)
        script_path = f.name

    # Make executable
    os.chmod(script_path, 0o755)

    return script_path


def main():
    """Main function to run the multi-CPU SoC simulation."""

    # Parse command line arguments
    args = parse_args()

    try:
        # Configure system
        board, processor = configure_system(args)

        # Create hello world workload
        workload_path = create_hello_world_workload()
        print(f"[Info] Created hello world workload at: {workload_path}")
        print("[Info] Pass this file via 'm5 readfile' in the guest")
        print(
            "[Info] The Ubuntu after_boot.sh script will automatically execute it"
        )

        # Run simulation
        run_simulation(board, processor, args)

        # Cleanup
        import os

        os.unlink(workload_path)

    except Exception as e:
        print(f"\n{'='*60}")
        print("Simulation Failed")
        print(f"{'='*60}")
        print(f"Error: {e}")
        print(f"\nCommon issues:")
        print("1. Ensure gem5 is built with X86 and MESI_TWO_LEVEL support")
        print(
            "2. For KVM: Ensure host has KVM enabled and user has permissions"
        )
        print("3. Check internet connection for resource download")
        print(f"{'='*60}")
        raise


if __name__ == "__main__":
    main()
