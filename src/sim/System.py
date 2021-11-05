# Copyright (c) 2017, 2019 ARM Limited
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
# Copyright (c) 2005-2007 The Regents of The University of Michigan
# Copyright (c) 2011 Regents of the University of California
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

from m5.SimObject import *
from m5.defines import buildEnv
from m5.params import *
from m5.proxy import *

from m5.objects.DVFSHandler import *
from m5.objects.SimpleMemory import *
from m5.objects.Workload import StubWorkload

class MemoryMode(Enum): vals = ['invalid', 'atomic', 'timing',
                                'atomic_noncaching']

class System(SimObject):
    type = 'System'
    cxx_header = "sim/system.hh"
    cxx_class = 'gem5::System'

    system_port = RequestPort("System port")

    cxx_exports = [
        PyBindMethod("getMemoryMode"),
        PyBindMethod("setMemoryMode"),
    ]

    memories = VectorParam.AbstractMemory(Self.all,
                                          "All memories in the system")
    mem_mode = Param.MemoryMode('atomic', "The mode the memory system is in")

    thermal_model = Param.ThermalModel(NULL, "Thermal model")
    thermal_components = VectorParam.SimObject([],
            "A collection of all thermal components in the system.")

    # When reserving memory on the host, we have the option of
    # reserving swap space or not (by passing MAP_NORESERVE to
    # mmap). By enabling this flag, we accommodate cases where a large
    # (but sparse) memory is simulated.
    mmap_using_noreserve = Param.Bool(False, "mmap the backing store " \
                                          "without reserving swap")

    # The memory ranges are to be populated when creating the system
    # such that these can be passed from the I/O subsystem through an
    # I/O bridge or cache
    mem_ranges = VectorParam.AddrRange([], "Ranges that constitute main memory")

    # The ranges backed by a shadowed ROM
    shadow_rom_ranges = VectorParam.AddrRange([], "Ranges  backed by a " \
                                                  "shadowed ROM")

    shared_backstore = Param.String("", "backstore's shmem segment filename, "
        "use to directly address the backstore from another host-OS process. "
        "Leave this empty to unset the MAP_SHARED flag.")

    cache_line_size = Param.Unsigned(64, "Cache line size in bytes")

    redirect_paths = VectorParam.RedirectPath([], "Path redirections")

    exit_on_work_items = Param.Bool(False, "Exit from the simulation loop when "
                                    "encountering work item annotations.")
    work_item_id = Param.Int(-1, "specific work item id")
    num_work_ids = Param.Int(16, "Number of distinct work item types")
    work_begin_cpu_id_exit = Param.Int(-1,
        "work started on specific id, now exit simulation")
    work_begin_ckpt_count = Param.Counter(0,
        "create checkpoint when work items begin count value is reached")
    work_begin_exit_count = Param.Counter(0,
        "exit simulation when work items begin count value is reached")
    work_end_ckpt_count = Param.Counter(0,
        "create checkpoint when work items end count value is reached")
    work_end_exit_count = Param.Counter(0,
        "exit simulation when work items end count value is reached")
    work_cpus_ckpt_count = Param.Counter(0,
        "create checkpoint when active cpu count value is reached")

    workload = Param.Workload(StubWorkload(), "Workload to run on this system")
    init_param = Param.UInt64(0, "numerical value to pass into simulator")
    readfile = Param.String("", "file to read startup script from")
    symbolfile = Param.String("", "file to get the symbols from")

    multi_thread = Param.Bool(False,
            "Supports multi-threaded CPUs? Impacts Thread/Context IDs")

    # Dynamic voltage and frequency handler for the system, disabled by default
    # Provide list of domains that need to be controlled by the handler
    dvfs_handler = DVFSHandler()

    # SE mode doesn't use the ISA System subclasses, and so we need to set an
    # ISA specific value in this class directly.
    m5ops_base = Param.Addr(0, "Base of the 64KiB PA range used for "
       "memory-mapped m5ops. Set to 0 to disable.")

    if buildEnv['USE_KVM']:
        kvm_vm = Param.KvmVM(NULL, 'KVM VM (i.e., shared memory domain)')
