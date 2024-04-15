# -*- mode:python -*-

# Copyright (c) 2016 RISC-V Foundation
# Copyright (c) 2016 The University of Virginia
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

from m5.objects.System import System
from m5.objects.Workload import (
    KernelWorkload,
    Workload,
)
from m5.params import *


class RiscvBareMetal(Workload):
    type = "RiscvBareMetal"
    cxx_class = "gem5::RiscvISA::BareMetal"
    cxx_header = "arch/riscv/bare_metal/fs_workload.hh"

    bootloader = Param.String("File, that contains the bootloader code")
    bare_metal = Param.Bool(True, "Using Bare Metal Application?")
    reset_vect = Param.Addr(0x0, "Reset vector")
    auto_reset_vect = Param.Bool(
        True,
        "Use bootloader entry point as reset vector. "
        "If `auto_reset_vect` is true, the `reset_vect` parameter is ignored.",
    )


class RiscvLinux(KernelWorkload):
    type = "RiscvLinux"
    cxx_class = "gem5::RiscvISA::FsLinux"
    cxx_header = "arch/riscv/linux/fs_workload.hh"

    dtb_filename = Param.String(
        "", "File that contains the Device Tree Blob. Don't use DTB if empty."
    )
    dtb_addr = Param.Addr(0x87E00000, "DTB address")

    # gem5 event upon guest's kernel panic
    # Default to false because when the kernel is compiled into the bootloader
    # it will not have symbols
    exit_on_kernel_panic = Param.Bool(
        False, "Generate gem5 panic upon the guest's kernel panic."
    )
    exit_on_kernel_oops = Param.Bool(
        False, "Generate gem5 panic upon the guest's kernel oops."
    )


class RiscvBootloaderKernelWorkload(Workload):
    type = "RiscvBootloaderKernelWorkload"
    cxx_class = "gem5::RiscvISA::BootloaderKernelWorkload"
    cxx_header = "arch/riscv/linux/fs_workload.hh"

    bootloader_filename = Param.String(
        "", "File that contains the bootloader. Don't use bootloader if empty."
    )
    bootloader_addr = Param.Addr(
        0x0, "Where to place the bootloader in memory."
    )
    object_file = Param.String("", "vmlinux file. Don't use kernel if empty.")
    kernel_addr = Param.Addr(
        0x80200000,
        "Where to place the kernel in memory. Typically, after the first "
        "stage of booting is done, the bootloader will jump to where the "
        "`start` symbol of the kernel is.",
    )
    entry_point = Param.Addr(
        0x80000000, "Where to find the first instruction to execute."
    )
    dtb_filename = Param.String(
        "", "File that contains the Device Tree Blob. Don't use DTB if empty."
    )
    dtb_addr = Param.Addr(0x87E00000, "Where to place the DTB in memory.")

    # booting parameters
    command_line = Param.String(
        "", "Booting arguments, to be passed to the kernel."
    )

    # gem5 event upon guest's kernel panic
    # Note that if the kernel doesn't have symbols there will be a warning and
    # gem5 will not exit
    exit_on_kernel_panic = Param.Bool(
        True, "Generate gem5 exit upon the guest's kernel panic."
    )
    exit_on_kernel_oops = Param.Bool(
        False, "Generate gem5 exit upon the guest's kernel oops."
    )

    # Note: Duplicated from KernelWorkload for now
    on_panic = Param.KernelPanicOopsBehaviour(
        "DumpDmesgAndExit",
        "Define how gem5 should behave after a Linux Kernel Panic. "
        "Handler might not be implemented for all architectures.",
    )

    on_oops = Param.KernelPanicOopsBehaviour(
        "DumpDmesgAndExit",
        "Define how gem5 should behave after a Linux Kernel Oops. "
        "Handler might not be implemented for all architectures.",
    )
