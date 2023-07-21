# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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

from m5.objects.E820 import X86E820Table, X86E820Entry
from m5.objects.SMBios import X86SMBiosSMBiosTable
from m5.objects.IntelMP import X86IntelMPFloatingPointer, X86IntelMPConfigTable
from m5.objects.ACPI import X86ACPIRSDP
from m5.objects.Workload import KernelWorkload, Workload


class X86BareMetalWorkload(Workload):
    type = "X86BareMetalWorkload"
    cxx_header = "arch/x86/bare_metal/workload.hh"
    cxx_class = "gem5::X86ISA::BareMetalWorkload"


class X86FsWorkload(KernelWorkload):
    type = "X86FsWorkload"
    cxx_header = "arch/x86/fs_workload.hh"
    cxx_class = "gem5::X86ISA::FsWorkload"

    smbios_table = Param.X86SMBiosSMBiosTable(
        X86SMBiosSMBiosTable(), "table of smbios/dmi information"
    )
    intel_mp_pointer = Param.X86IntelMPFloatingPointer(
        X86IntelMPFloatingPointer(), "intel mp spec floating pointer structure"
    )
    intel_mp_table = Param.X86IntelMPConfigTable(
        X86IntelMPConfigTable(), "intel mp spec configuration table"
    )
    acpi_description_table_pointer = Param.X86ACPIRSDP(
        X86ACPIRSDP(), "ACPI root description pointer structure"
    )
    enable_osxsave = Param.Bool(False, "Enable OSXSAVE in CR4 register")


class X86FsLinux(X86FsWorkload):
    type = "X86FsLinux"
    cxx_header = "arch/x86/linux/fs_workload.hh"
    cxx_class = "gem5::X86ISA::FsLinux"

    e820_table = Param.X86E820Table(
        X86E820Table(), "E820 map of physical memory"
    )
