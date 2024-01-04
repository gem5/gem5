# Copyright (c) 2012 ARM Limited
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
# Copyright (c) 2005-2008 The Regents of The University of Michigan
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

from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class AbstractMemory(ClockedObject):
    type = "AbstractMemory"
    abstract = True
    cxx_header = "mem/abstract_mem.hh"
    cxx_class = "gem5::memory::AbstractMemory"

    # A default memory size of 128 MiB (starting at 0) is used to
    # simplify the regressions
    range = Param.AddrRange(
        "128MiB", "Address range (potentially interleaved)"
    )
    null = Param.Bool(False, "Do not store data, always return zero")

    # All memories are passed to the global physical memory, and
    # certain memories may be excluded from the global address map,
    # e.g. by the testers that use shadow memories as a reference
    in_addr_map = Param.Bool(True, "Memory part of the global address map")

    # When KVM acceleration is used, memory is mapped into the guest process
    # address space and accessed directly. Some memories may need to be
    # excluded from this mapping if they overlap with other memory ranges or
    # are not accessible by the CPU.
    kvm_map = Param.Bool(True, "Should KVM map this memory for the guest")

    # Should the bootloader include this memory when passing
    # configuration information about the physical memory layout to
    # the kernel, e.g. using ATAG or ACPI
    conf_table_reported = Param.Bool(True, "Report to configuration table")

    # Image file to load into this memory as its initial contents. This is
    # particularly useful for ROMs.
    image_file = Param.String(
        "", "Image to load into memory as its initial contents"
    )

    writeable = Param.Bool(True, "Allow writes to this memory")

    collect_stats = Param.Bool(
        True,
        "Collect statistics per requestor for "
        "each type of access. Set this to `False` if "
        "requestors may be unknown or when running "
        "with multiple `System` objects without a "
        "`SysBridge`.",
    )
