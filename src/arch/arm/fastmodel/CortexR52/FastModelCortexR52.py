# Copyright 2020 Google, Inc.
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
from m5.proxy import *
from m5.SimObject import SimObject

from m5.objects.ArmInterrupts import ArmInterrupts
from m5.objects.ArmISA import ArmISA
from m5.objects.FastModel import AmbaInitiatorSocket
from m5.objects.IntPin import VectorIntSinkPin
from m5.objects.Iris import IrisBaseCPU
from m5.objects.SystemC import SystemC_ScModule

class FastModelCortexR52(IrisBaseCPU):
    type = 'FastModelCortexR52'
    cxx_class = 'gem5::fastmodel::CortexR52'
    cxx_header = 'arch/arm/fastmodel/CortexR52/cortex_r52.hh'

    evs = Parent.evs

    ppi = VectorIntSinkPin('PPI inputs (0-8)')

    llpp = AmbaInitiatorSocket(64, 'Low Latency Peripheral Port')
    flash = AmbaInitiatorSocket(64, 'Flash')
    amba = AmbaInitiatorSocket(64, 'AMBA initiator socket')

    CFGEND = Param.Bool(False, "Endianness configuration at reset.  0, " \
            "little endian. 1, big endian.")
    CFGTE = Param.Bool(False, "Equivalent to CFGTHUMBEXCEPTIONS")
    RVBARADDR = Param.UInt32(0, "Equivalent to CFGVECTABLE")
    ase_present = Param.Bool(True, "Set whether the model has been built " \
            "with NEON support")
    dcache_size = Param.UInt16(0x8000, "L1 D-Cache size in bytes")
    flash_enable = Param.Bool(False, "Equivalent to CFGFLASHEN")
    icache_size = Param.UInt16(0x8000, "L1 I-Cache size in bytes")
    llpp_base = Param.UInt32(0, "Equivalent to CFGLLPPBASEADDR")
    llpp_size = Param.UInt32(0x1000, "Equivalent to CFGLLPPSIZE")
    max_code_cache_mb = Param.UInt64(0x100, "Maximum size of the " \
            "simulation code cache (MiB). For platforms with more than 2 " \
            "cores this limit will be scaled down. (e.g 1/8 for 16 or more " \
            "cores).")
    min_sync_level = Param.UInt8(0, "Force minimum syncLevel " \
            "(0=off=default,1=syncState,2=postInsnIO,3=postInsnAll)")
    semihosting_A32_HLT = Param.UInt16(0xf000, "A32 HLT number for " \
            "semihosting calls.")
    semihosting_ARM_SVC = Param.UInt32(0x123456, "A32 SVC number for " \
            "semihosting calls.")
    semihosting_T32_HLT = Param.UInt8(60, "T32 HLT number for semihosting " \
            "calls.")
    semihosting_Thumb_SVC = Param.UInt8(171, "T32 SVC number for " \
            "semihosting calls.")
    semihosting_cmd_line = Param.String("", "Command line available to " \
            "semihosting calls.")
    semihosting_cwd = Param.String("", "Base directory for semihosting " \
            "file access.")
    semihosting_enable = Param.Bool(True, "Enable semihosting SVC/HLT traps.")
    semihosting_heap_base = Param.UInt32(0, "Virtual address of heap base.")
    semihosting_heap_limit = Param.UInt32(0xf000000, "Virtual address of " \
            "top of heap.")
    semihosting_stack_base = Param.UInt32(0x10000000, "Virtual address of " \
            "base of descending stack.")
    semihosting_stack_limit = Param.UInt32(0xf000000, "Virtual address of " \
            "stack limit.")
    tcm_a_enable = Param.Bool(False, "Equivalent to CFGTCMBOOT")
    tcm_a_size = Param.UInt32(0x4000, "Sets the size of the ATCM(in bytes)")
    tcm_b_size = Param.UInt32(0x4000, "Sets the size of the BTCM(in bytes)")
    tcm_c_size = Param.UInt32(0x2000, "Sets the size of the CTCM(in bytes)")
    vfp_dp_present = Param.Bool(True, "Whether double-precision floating " \
            "point feature is implemented")
    vfp_enable_at_reset = Param.Bool(False, "Enable VFP in CPACR, CPPWR, " \
            "NSACR at reset. Warning: Arm recommends going through the "
            "implementation's suggested VFP power-up sequence!")

class FastModelCortexR52Cluster(SimObject):
    type = 'FastModelCortexR52Cluster'
    cxx_class = 'gem5::fastmodel::CortexR52Cluster'
    cxx_header = 'arch/arm/fastmodel/CortexR52/cortex_r52.hh'

    cores = VectorParam.FastModelCortexR52(
            'Core in a given cluster of CortexR52s')

    evs = Param.SystemC_ScModule(
            "Fast mo0del exported virtual subsystem holding cores")

    spi = VectorIntSinkPin('SPI inputs (0-959)')

    CLUSTER_ID = Param.UInt16(0, "CLUSTER_ID[15:8] equivalent to " \
            "CFGMPIDRAFF2, CLUSTER_ID[7:0] equivalent to CFGMPIDRAFF1")
    DBGROMADDR = Param.UInt32(0, "Equivalent to CFGDBGROMADDR")
    DBGROMADDRV = Param.Bool(False, "If true, set bits[1:0] of the CP15 " \
            "DBGDRAR to indicate that the address is valid")
    PERIPHBASE = Param.UInt32(0x13080000, "Equivalent to CFGPERIPHBASE")
    cluster_utid = Param.UInt8(0, "Equivalent to CFGCLUSTERUTID")
    cpi_div = Param.UInt32(1, "Divider for calculating CPI " \
            "(Cycles Per Instruction)")
    cpi_mul = Param.UInt32(1, "Multiplier for calculating CPI " \
            "(Cycles Per Instruction)")
    dcache_prefetch_enabled = Param.Bool(False, "Enable simulation of data " \
            "cache prefetching.  This is only used when " \
            "dcache-state_modelled=true")
    dcache_read_access_latency = Param.UInt64(0, "L1 D-Cache timing " \
            "annotation latency for read accesses given in ticks per access " \
            "(of size dcache-read_bus_width_in_bytes).  If this parameter " \
            "is non-zero, per-access latencies will be used instead of " \
            "per-byte even if dcache-read_latency is set. This is in " \
            "addition to the hit or miss latency, and intended to " \
            "correspond to the time taken to transfer across the cache " \
            "upstream bus, this is only used when dcache-state_modelled=true.")
    dcache_state_modelled = Param.Bool(False, "Set whether D-cache has " \
            "stateful implementation")
    dcache_write_access_latency = Param.UInt64(0, "L1 D-Cache timing " \
            "annotation latency for write accesses given in ticks per " \
            "access (of size dcache-write_bus_width_in_bytes). If this " \
            "parameter is non-zero, per-access latencies will be used " \
            "instead of per-byte even if dcache-write_latency is set. This " \
            "is only used when dcache-state_modelled=true.")
    flash_protection_enable_at_reset = Param.Bool(False, "Equivalent to " \
            "CFGFLASHPROTEN")
    has_flash_protection = Param.Bool(True, "Equivalent to CFGFLASHPROTIMP")
    icache_prefetch_enabled = Param.Bool(False, "Enable simulation of " \
            "instruction cache prefetching. This is only used when " \
            "icache-state_modelled=true.")
    icache_read_access_latency = Param.UInt64(0, "L1 I-Cache timing " \
            "annotation latency for read accesses given in ticks per access " \
            "(of size icache-read_bus_width_in_bytes).  If this parameter " \
            "is non-zero, per-access latencies will be used instead of " \
            "per-byte even if icache-read_latency is set. This is in " \
            "addition to the hit or miss latency, and intended to " \
            "correspond to the time taken to transfer across the cache " \
            "upstream bus, this is only used when icache-state_modelled=true.")
    icache_state_modelled = Param.Bool(False, "Set whether I-cache has " \
            "stateful implementation")
    memory_ext_slave_base = Param.UInt32(0, "Equivalent to CFGAXISTCMBASEADDR")
    memory_flash_base = Param.UInt32(0, "Equivalent to CFGFLASHBASEADDR")
    memory_flash_size = Param.UInt32(0x4000000, "Equivalent to CFGFLASHIMP. " \
            "memory.flash_size = 0 => CFGFLASHIMP = false")
    num_protection_regions_s1 = Param.UInt8(16, "Number of v8-R stage1 " \
            "protection regions")
    num_protection_regions_s2 = Param.UInt8(16, "Number of v8-R hyp " \
            "protection regions")
    num_spi = Param.UInt16(960, "Number of interrupts (SPI) into the " \
            "internal GIC controller")
    ram_protection_enable_at_reset = Param.Bool(False, "Equivalent to " \
            "CFGRAMPROTEN")
    has_export_m_port = Param.Bool(True, "The interrupt distributor has an " \
            "optional interrupt export port for routing interrupts to an " \
            "external device")

class FastModelScxEvsCortexR52x1(SystemC_ScModule):
    type = 'FastModelScxEvsCortexR52x1'
    cxx_class = 'gem5::fastmodel::ScxEvsCortexR52<' \
                    'gem5::fastmodel::ScxEvsCortexR52x1Types>'
    cxx_template_params = [ 'class Types' ]
    cxx_header = 'arch/arm/fastmodel/CortexR52/evs.hh'

class FastModelCortexR52x1(FastModelCortexR52Cluster):
    cores = [ FastModelCortexR52(thread_paths=[ 'core.cpu0' ]) ]

    evs = FastModelScxEvsCortexR52x1()

class FastModelScxEvsCortexR52x2(SystemC_ScModule):
    type = 'FastModelScxEvsCortexR52x2'
    cxx_class = 'gem5::fastmodel::ScxEvsCortexR52<' \
                    'gem5::fastmodel::ScxEvsCortexR52x2Types>'
    cxx_template_params = [ 'class Types' ]
    cxx_header = 'arch/arm/fastmodel/CortexR52/evs.hh'

class FastModelCortexR52x2(FastModelCortexR52Cluster):
    cores = [ FastModelCortexR52(thread_paths=[ 'core.cpu0' ]),
              FastModelCortexR52(thread_paths=[ 'core.cpu1' ]) ]

    evs = FastModelScxEvsCortexR52x2()

class FastModelScxEvsCortexR52x3(SystemC_ScModule):
    type = 'FastModelScxEvsCortexR52x3'
    cxx_class = 'gem5::fastmodel::ScxEvsCortexR52<' \
                    'gem5::fastmodel::ScxEvsCortexR52x3Types>'
    cxx_template_params = [ 'class Types' ]
    cxx_header = 'arch/arm/fastmodel/CortexR52/evs.hh'

class FastModelCortexR52x3(FastModelCortexR52Cluster):
    cores = [ FastModelCortexR52(thread_paths=[ 'core.cpu0' ]),
              FastModelCortexR52(thread_paths=[ 'core.cpu1' ]),
              FastModelCortexR52(thread_paths=[ 'core.cpu2' ]) ]

    evs = FastModelScxEvsCortexR52x3()

class FastModelScxEvsCortexR52x4(SystemC_ScModule):
    type = 'FastModelScxEvsCortexR52x4'
    cxx_class = 'gem5::fastmodel::ScxEvsCortexR52<' \
                    'gem5::fastmodel::ScxEvsCortexR52x4Types>'
    cxx_template_params = [ 'class Types' ]
    cxx_header = 'arch/arm/fastmodel/CortexR52/evs.hh'

class FastModelCortexR52x4(FastModelCortexR52Cluster):
    cores = [ FastModelCortexR52(thread_paths=[ 'core.cpu0' ]),
              FastModelCortexR52(thread_paths=[ 'core.cpu1' ]),
              FastModelCortexR52(thread_paths=[ 'core.cpu2' ]),
              FastModelCortexR52(thread_paths=[ 'core.cpu3' ]) ]

    evs = FastModelScxEvsCortexR52x4()
