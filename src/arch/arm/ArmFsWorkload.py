# Copyright (c) 2009, 2012-2013, 2015-2020 ARM Limited
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

from m5.params import *
from m5.options import *
from m5.SimObject import *
from m5.objects.Workload import KernelWorkload


class ArmMachineType(Enum):
    map = {"VExpress_EMM": 2272, "VExpress_EMM64": 2272, "DTOnly": -1}


class ArmFsWorkload(KernelWorkload):
    type = "ArmFsWorkload"
    cxx_header = "arch/arm/fs_workload.hh"
    cxx_class = "gem5::ArmISA::FsWorkload"

    boot_loader = VectorParam.String(
        [],
        "File that contains the boot loader code. Zero or more files may be "
        "specified. The first boot loader that matches the kernel's "
        "architecture will be used.",
    )

    dtb_filename = Param.String(
        "", "File that contains the Device Tree Blob. Don't use DTB if empty."
    )
    dtb_addr = Param.Addr(0, "DTB or ATAGS address")
    initrd_filename = Param.String(
        "",
        "File that contains the initial ramdisk. Don't use initrd if empty.",
    )
    initrd_addr = Param.Addr(0, "initrd/initramfs address")
    cpu_release_addr = Param.Addr(0, "cpu-release-addr property")

    machine_type = Param.ArmMachineType(
        "DTOnly",
        "Machine id from http://www.arm.linux.org.uk/developer/machines/",
    )
    early_kernel_symbols = Param.Bool(
        False, "enable early kernel symbol tables before MMU"
    )
    enable_context_switch_stats_dump = Param.Bool(
        False, "enable stats/task info dumping at context switch boundaries"
    )

    panic_on_panic = Param.Bool(
        False, "Trigger a gem5 panic if the guest kernel panics"
    )
    panic_on_oops = Param.Bool(
        False, "Trigger a gem5 panic if the guest kernel oopses"
    )


class ArmFsLinux(ArmFsWorkload):
    type = "ArmFsLinux"
    cxx_header = "arch/arm/linux/fs_workload.hh"
    cxx_class = "gem5::ArmISA::FsLinux"

    load_addr_mask = 0

    @cxxMethod
    def dumpDmesg(self):
        """Dump dmesg from the simulated kernel to standard out"""
        pass


class ArmFsFreebsd(ArmFsWorkload):
    type = "ArmFsFreebsd"
    cxx_header = "arch/arm/freebsd/fs_workload.hh"
    cxx_class = "gem5::ArmISA::FsFreebsd"
