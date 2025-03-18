# Copyright (c) 2021-2025 The Regents of the University of California
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

import gem5.utils.multisim as multisim
from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    DiskImageResource,
    KernelResource,
    obtain_resource,
)
from gem5.simulate.simulator import Simulator

NUM_PROCESSES = 10

multisim.set_num_processes(NUM_PROCESSES)

# RISCV boot using Atomic cores, with and without systemd. Medium length
# workloads.
for no_systemd in [True, False]:
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB",
        l1i_size="16KiB",
        l2_size="256KiB",
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.RISCV, num_cores=1
    )

    board = RiscvBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    if not no_systemd:
        board.set_kernel_disk_workload(
            kernel=KernelResource(
                "/projects/gem5/new-base-imgs-w-hypercalls/riscv-disk-image-24-04/vmlinux"
            ),
            disk_image=DiskImageResource(
                "/projects/gem5/new-base-imgs-w-hypercalls/riscv-disk-image-24-04/riscv-ubuntu",
                root_partition="1",
            ),
            bootloader=obtain_resource("riscv-bootloader-opensbi-1.3.1"),
            kernel_args=[
                "console=ttyS0",
                "root=/dev/vda1",
                "rw",
            ],
        )
    else:
        board.set_kernel_disk_workload(
            kernel=KernelResource(
                "/projects/gem5/new-base-imgs-w-hypercalls/riscv-disk-image-24-04/vmlinux"
            ),
            disk_image=DiskImageResource(
                "/projects/gem5/new-base-imgs-w-hypercalls/riscv-disk-image-24-04/riscv-ubuntu",
                root_partition="1",
            ),
            bootloader=obtain_resource("riscv-bootloader-opensbi-1.3.1"),
            kernel_args=[
                "console=ttyS0",
                "root=/dev/vda1",
                "rw",
                "no_systemd=true",
            ],
        )
    if no_systemd:
        multisim.add_simulator(
            Simulator(
                board=board, id=f"process_riscv-atomic-24-04-boot-no-systemd"
            )
        )

    else:
        multisim.add_simulator(
            Simulator(
                board=board, id=f"process_riscv-atomic-24-04-boot-with-systemd"
            )
        )


# Run cg, is, and mg size S and A. These are the NPB workloads with the
# shortest wallclock times on size A, but they still take more time
# to finish compared to the Ubuntu boot workloads.
for npb_workload in ["cg", "is", "mg"]:  # "bt", "ep", "ft", "lu", "sp", "ua"
    for size in ["S", "A"]:
        cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
            l1d_size="16KiB",
            l1i_size="16KiB",
            l2_size="256KiB",
        )
        memory = SingleChannelDDR3_1600(size="3GiB")
        processor = SimpleProcessor(
            cpu_type=CPUTypes.TIMING, isa=ISA.X86, num_cores=1
        )
        board = X86Board(
            clk_freq="1GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

        board.set_kernel_disk_workload(
            kernel=KernelResource(
                "/home/bees/gem5-resources/src/ubuntu-generic-diskimages/x86-disk-image-24-04/vmlinux-x86-ubuntu"
            ),
            disk_image=DiskImageResource(
                "/home/bees/gem5-resources/src/npb-24.04-imgs/disk-image-x86-npb/x86-ubuntu-npb"
            ),
            kernel_args=[
                "earlyprintk=ttyS0",
                "console=ttyS0",
                "lpj=7999923",
                "root=/dev/sda2",
            ],
            readfile_contents=f"/home/gem5/NPB3.4-OMP/bin/{npb_workload}.{size}.x; sleep 5;",
        )

        multisim.add_simulator(
            Simulator(
                board=board,
                id=f"process_x86-timing-npb-{npb_workload}-{size.lower()}",
            )
        )


# Run X86 and Arm hello world binaries. The shortest workloads.
for isa in [ISA.X86, ISA.ARM]:
    cache_hierarchy = NoCache()

    memory = SingleChannelDDR3_1600(size="32MiB")

    processor = SimpleProcessor(cpu_type=CPUTypes.TIMING, isa=isa, num_cores=1)

    board = SimpleBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    if isa == ISA.X86:
        board.set_se_binary_workload(
            obtain_resource("x86-hello64-static", resource_version="1.0.0")
        )
        multisim.add_simulator(Simulator(board=board, id=f"process_x86-hello"))
    else:
        board.set_se_binary_workload(
            obtain_resource("arm-hello64-static", resource_version="1.0.0")
        )
        multisim.add_simulator(Simulator(board=board, id=f"process_arm-hello"))
