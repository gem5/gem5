# Copyright 2019 Google, Inc.
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
#
# Authors: Gabe Black

from m5.params import *
from m5.SimObject import SimObject

from m5.objects.ArmInterrupts import ArmInterrupts
from m5.objects.ArmISA import ArmISA
from m5.objects.FastModel import AmbaInitiatorSocket, AmbaTargetSocket
from m5.objects.FastModel import ScMasterPort
from m5.objects.FastModelArch import FastModelArmCPU
from m5.objects.FastModelGIC import Gicv3CommsInitiatorSocket
from m5.objects.FastModelGIC import Gicv3CommsTargetSocket
from m5.objects.SystemC import SystemC_ScModule

class FastModelCortexA76x1(SystemC_ScModule):
    type = 'FastModelCortexA76x1'
    cxx_class = 'FastModel::CortexA76x1'
    cxx_header = 'arch/arm/fastmodel/CortexA76x1/cortex_a76x1.hh'

    _core_paths = [ 'core.cpu0' ]
    cpu_wrapper = FastModelArmCPU(
            core_paths=_core_paths,

            cntfrq = 0x1800000,

            # We shouldn't need these, but gem5 gets mad without them.
            interrupts = [ ArmInterrupts() ],
            isa = [ ArmISA() ],
    )

    amba = AmbaInitiatorSocket(64, 'AMBA initiator socket')
    redistributor_m = Gicv3CommsInitiatorSocket('GIC communication initiator')
    redistributor_s = Gicv3CommsTargetSocket('GIC communication target')
    cnthpirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    cnthvirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    cntpsirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    cntvirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    commirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    ctidbgirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    pmuirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    vcpumntirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")
    cntpnsirq = ScMasterPort("Master port for CPU-to-GIC signal", "bool")

    # These parameters are described in "Fast Models Reference Manual" section
    # 3.4.19, "ARMCortexA7x1CT".
    BROADCASTATOMIC = Param.Bool(True, "Enable broadcasting of atomic "\
            "operation. The broadcastatomic signal will override this value "\
            "if used")
    BROADCASTCACHEMAINT = Param.Bool(True, "Enable broadcasting of cache "\
            "maintenance operations to downstream caches. The "\
            "broadcastcachemaint signal will override this value if used.")
    BROADCASTOUTER = Param.Bool(True, "Enable broadcasting of Outer "\
            "Shareable transactions. The broadcastouter signal will override "\
            "this value if used.")
    BROADCASTPERSIST = Param.Bool(True, "Enable broadcasting  of cache clean "\
            "to the point of persistence operations. The broadcastpersist "\
            "signal will override this value if used")
    CLUSTER_ID = Param.UInt16(0x0, "Processor cluster ID value")
    GICDISABLE = Param.Bool(True, "Disable the new style GICv3 CPU interface "\
            "in each core model. Should be left enabled unless the platform "\
            "contains a GICv3 distributor.")
    cpi_div = Param.UInt32(1,
            "Divider for calculating CPI (Cycles Per Instruction)")
    cpi_mul = Param.UInt32(1,
            "Multiplier for calculating CPI (Cycles Per Instruction)")
    dcache_hit_latency = Param.UInt64(0, "L1 D-Cache timing annotation "\
            "latency for hit.  Intended to model the tag-lookup time.  This "\
            "is only used when dcache-state_modelled=true.")
    dcache_maintenance_latency = Param.UInt64(0, "L1 D-Cache timing "\
            "annotation latency for cache maintenance operations given in "\
            "total ticks. This is only used when dcache-state_modelled=true.")
    dcache_miss_latency = Param.UInt64(0, "L1 D-Cache timing annotation "\
            "latency for miss.  Intended to model the time for failed "\
            "tag-lookup and allocation of intermediate buffers.  This is "\
            "only used when dcache-state_modelled=true.")
    dcache_prefetch_enabled = Param.Bool(False, "Enable simulation of data "\
            "cache prefetching.  This is only used when "\
            "dcache-state_modelled=true")
    dcache_read_access_latency = Param.UInt64(0, "L1 D-Cache timing "\
            "annotation latency for read accesses given in ticks per access "\
            "(of size dcache-read_bus_width_in_bytes).  If this parameter is "\
            "non-zero, per-access latencies will be used instead of per-byte "\
            "even if dcache-read_latency is set. This is in addition to the "\
            "hit or miss latency, and intended to correspond to the time "\
            "taken to transfer across the cache upstream bus, this is only "\
            "used when dcache-state_modelled=true.")
    dcache_read_latency = Param.UInt64(0, "L1 D-Cache timing annotation "\
            "latency for read accesses given in ticks per byte "\
            "accessed.dcache-read_access_latency must be set to 0 for "\
            "per-byte latencies to be applied.  This is in addition to the "\
            "hit or miss latency, and intended to correspond to the time "\
            "taken to transfer across the cache upstream bus. This is only "\
            "used when dcache-state_modelled=true.")
    dcache_snoop_data_transfer_latency = Param.UInt64(0, "L1 D-Cache timing "\
        "annotation latency for received snoop accesses that perform a data "\
        "transfer given in ticks per byte accessed. This is only used when "\
        "dcache-state_modelled=true.")
    dcache_state_modelled = Param.Bool(False,
        "Set whether D-cache has stateful implementation")
    dcache_write_access_latency = Param.UInt64(0, "L1 D-Cache timing "\
        "annotation latency for write accesses given in ticks per access "\
        "(of size dcache-write_bus_width_in_bytes). If this parameter is "\
        "non-zero, per-access latencies will be used instead of per-byte "\
        "even if dcache-write_latency is set. This is only used when "\
        "dcache-state_modelled=true.")
    dcache_write_latency = Param.UInt64(0, "L1 D-Cache timing annotation "\
        "latency for write accesses given in ticks per byte accessed. "\
        "dcache-write_access_latency must be set to 0 for per-byte latencies "\
        "to be applied. This is only used when dcache-state_modelled=true.")
    default_opmode = Param.Unsigned(4, "Operating mode of DynamIQ coming out "\
            "of reset. 0: SFONLY ON, 1: 1/4 CACHE ON, 2: 1/2 CACHE ON, 3: "\
            "3/4 CACHE ON, 4: FULL CACHE ON")
    diagnostics = Param.Bool(False, "Enable DynamIQ diagnostic messages")
    enable_simulation_performance_optimizations = Param.Bool(True,
            "With this option enabled, the model will run more quickly, but "\
            "be less accurate to exact CPU behavior. The model will still be "\
            "functionally accurate for software, but may increase "\
            "differences seen between hardware behavior and model behavior "\
            "for certain workloads (it changes the micro-architectural value "\
            "of stage12_tlb_size parameter to 1024).")
    ext_abort_device_read_is_sync = Param.Bool(False,
            "Synchronous reporting of device-nGnRE read external aborts")
    ext_abort_device_write_is_sync = Param.Bool(False,
            "Synchronous reporting of device-nGnRE write external aborts")
    ext_abort_so_read_is_sync = Param.Bool(False,
            "Synchronous reporting of device-nGnRnE read external aborts")
    ext_abort_so_write_is_sync = Param.Bool(False,
            "Synchronous reporting of device-nGnRnE write external aborts")
    gicv3_cpuintf_mmap_access_level = Param.Unsigned(0, "Allowed values are: "\
            "0-mmap access is supported for GICC,GICH,GICV registers. 1-mmap "\
            "access is supported only for GICV registers. 2-mmap access is "\
            "not supported.")
    has_peripheral_port = Param.Bool(False,
            "If true, additional AXI peripheral port is configured.")
    has_statistical_profiling = Param.Bool(True,
            "Whether Statistical Based Profiling is implemented")
    icache_hit_latency = Param.UInt64(0, "L1 I-Cache timing annotation "\
            "latency for hit.  Intended to model the tag-lookup time.  This "\
            "is only used when icache-state_modelled=true.")
    icache_maintenance_latency = Param.UInt64(0, "L1 I-Cache timing "\
            "annotation latency for cache maintenance operations given in "\
            "total ticks. This is only used when icache-state_modelled=true.")
    icache_miss_latency = Param.UInt64(0, "L1 I-Cache timing annotation "\
            "latency for miss.  Intended to model the time for failed "\
            "tag-lookup and allocation of intermediate buffers.  This is "\
            "only used when icache-state_modelled=true.")
    icache_prefetch_enabled = Param.Bool(False, "Enable simulation of "\
            "instruction cache prefetching. This is only used when "\
            "icache-state_modelled=true.")
    icache_read_access_latency = Param.UInt64(0, "L1 I-Cache timing "\
            "annotation latency for read accesses given in ticks per access "\
            "(of size icache-read_bus_width_in_bytes).  If this parameter is "\
            "non-zero, per-access latencies will be used instead of per-byte "\
            "even if icache-read_latency is set. This is in addition to the "\
            "hit or miss latency, and intended to correspond to the time "\
            "taken to transfer across the cache upstream bus, this is only "\
            "used when icache-state_modelled=true.")
    icache_read_latency = Param.UInt64(0, "L1 I-Cache timing annotation "\
            "latency for read accesses given in ticks per byte "\
            "accessed.icache-read_access_latency must be set to 0 for "\
            "per-byte latencies to be applied.  This is in addition to the "\
            "hit or miss latency, and intended to correspond to the time "\
            "taken to transfer across the cache upstream bus. This is only "\
            "used when icache-state_modelled=true.")
    icache_state_modelled = Param.Bool(False,
            "Set whether I-cache has stateful implementation")
    l3cache_hit_latency = Param.UInt64(0, "L3 Cache timing annotation "\
            "latency for hit.  Intended to model the tag-lookup time.  This "\
            "is only used when l3cache-state_modelled=true.")
    l3cache_maintenance_latency = Param.UInt64(0, "L3 Cache timing "\
            "annotation latency for cache maintenance operations given in "\
            "total ticks. This is only used when dcache-state_modelled=true.")
    l3cache_miss_latency = Param.UInt64(0, "L3 Cache timing annotation "\
            "latency for miss.  Intended to model the time for failed "\
            "tag-lookup and allocation of intermediate buffers.  This is "\
            "only used when l3cache-state_modelled=true.")
    l3cache_read_access_latency = Param.UInt64(0, "L3 Cache timing "\
            "annotation latency for read accesses given in ticks per access "\
            "(of size l3cache-read_bus_width_in_bytes).  If this parameter "\
            "is non-zero, per-access latencies will be used instead of "\
            "per-byte even if l3cache-read_latency is set. This is in "\
            "addition to the hit or miss latency, and intended to correspond "\
            "to the time taken to transfer across the cache upstream bus, "\
            "this is only used when l3cache-state_modelled=true.")
    l3cache_read_latency = Param.UInt64(0, "L3 Cache timing annotation "\
            "latency for read accesses given in ticks per byte "\
            "accessed.l3cache-read_access_latency must be set to 0 for "\
            "per-byte latencies to be applied.  This is in addition to the "\
            "hit or miss latency, and intended to correspond to the time "\
            "taken to transfer across the cache upstream bus. This is only "\
            "used when l3cache-state_modelled=true.")
    l3cache_size = Param.MemorySize('0x100000', "L3 Cache size in bytes.")
    l3cache_snoop_data_transfer_latency = Param.UInt64(0, "L3 Cache timing "\
            "annotation latency for received snoop accesses that perform a "\
            "data transfer given in ticks per byte accessed. This is only "\
            "used when dcache-state_modelled=true.")
    l3cache_snoop_issue_latency = Param.UInt64(0, "L3 Cache timing "\
            "annotation latency for snoop accesses issued by this cache in "\
            "total ticks. This is only used when dcache-state_modelled=true.")
    l3cache_write_access_latency = Param.UInt64(0, "L3 Cache timing "\
            "annotation latency for write accesses given in ticks per access "\
            "(of size l3cache-write_bus_width_in_bytes). If this parameter "\
            "is non-zero, per-access latencies will be used instead of "\
            "per-byte even if l3cache-write_latency is set. This is only "\
            "used when l3cache-state_modelled=true.")
    l3cache_write_latency = Param.UInt64(0, "L3 Cache timing annotation "\
            "latency for write accesses given in ticks per byte accessed. "\
            "l3cache-write_access_latency must be set to 0 for per-byte "\
            "latencies to be applied. This is only used when "\
            "l3cache-state_modelled=true.")
    pchannel_treat_simreset_as_poreset = Param.Bool(False,
            "Register core as ON state to cluster with simulation reset.")
    periph_address_end = Param.Addr(0x0, "End address for peripheral port "\
            "address range exclusive(corresponds to AENDMP input signal).")
    periph_address_start = Param.Addr(0x0, "Start address for peripheral "\
            "port address range inclusive(corresponds to ASTARTMP input "\
            "signal).")
    ptw_latency = Param.UInt64(0, "Page table walker latency for TA "\
            "(Timing Annotation), expressed in simulation ticks")
    tlb_latency = Param.UInt64(0, "TLB latency for TA (Timing Annotation), "\
            "expressed in simulation ticks")
    treat_dcache_cmos_to_pou_as_nop = Param.Bool(False, "Whether dcache "\
            "invalidation to the point of unification is required for "\
            "instruction to data coherence. true - Invalidate operations not "\
            "required")
    walk_cache_latency = Param.UInt64(0, "Walk cache latency for TA (Timing "\
            "Annotation), expressed in simulation ticks")

    cpu0_CFGEND = Param.Bool(False, "Endianness configuration at reset.  "\
            "0, little endian. 1, big endian.")
    cpu0_CFGTE = Param.Bool(False, "Instruction set state when resetting "\
            "into AArch32.  0, A32. 1, T32.")
    cpu0_CRYPTODISABLE = Param.Bool(False, "Disable cryptographic features.")
    cpu0_RVBARADDR = Param.Addr(0x0, "Value of RVBAR_ELx register.")
    cpu0_VINITHI = Param.Bool(False, "Reset value of SCTLR.V.")
    cpu0_enable_trace_special_hlt_imm16 = Param.Bool(False,
            "Enable usage of parameter trace_special_hlt_imm16")
    cpu0_l2cache_hit_latency = Param.UInt64(0, "L2 Cache timing annotation "\
            "latency for hit.  Intended to model the tag-lookup time.  This "\
            "is only used when l2cache-state_modelled=true.")
    cpu0_l2cache_maintenance_latency = Param.UInt64(0, "L2 Cache timing "\
            "annotation latency for cache maintenance operations given in "\
            "total ticks. This is only used when dcache-state_modelled=true.")
    cpu0_l2cache_miss_latency = Param.UInt64(0, "L2 Cache timing annotation "\
            "latency for miss.  Intended to model the time for failed "\
            "tag-lookup and allocation of intermediate buffers.  This is "\
            "only used when l2cache-state_modelled=true.")
    cpu0_l2cache_read_access_latency = Param.UInt64(0, "L2 Cache timing "\
            "annotation latency for read accesses given in ticks per "\
            "access.  If this parameter is non-zero, per-access latencies "\
            "will be used instead of per-byte even if l2cache-read_latency "\
            "is set. This is in addition to the hit or miss latency, and "\
            "intended to correspond to the time taken to transfer across the "\
            "cache upstream bus, this is only used when "\
            "l2cache-state_modelled=true.")
    cpu0_l2cache_read_latency = Param.UInt64(0, "L2 Cache timing annotation "\
            "latency for read accesses given in ticks per byte "\
            "accessed.l2cache-read_access_latency must be set to 0 for "\
            "per-byte latencies to be applied.  This is in addition to the "\
            "hit or miss latency, and intended to correspond to the time "\
            "taken to transfer across the cache upstream bus. This is only "\
            "used when l2cache-state_modelled=true.")
    cpu0_l2cache_size = Param.MemorySize32('0x80000',
            "L2 Cache size in bytes.")
    cpu0_l2cache_snoop_data_transfer_latency = Param.UInt64(0, "L2 Cache "\
            "timing annotation latency for received snoop accesses that "\
            "perform a data transfer given in ticks per byte accessed. This "\
            "is only used when dcache-state_modelled=true.")
    cpu0_l2cache_snoop_issue_latency = Param.UInt64(0, "L2 Cache timing "\
            "annotation latency for snoop accesses issued by this cache in "\
            "total ticks. This is only used when dcache-state_modelled=true.")
    cpu0_l2cache_write_access_latency = Param.UInt64(0, "L2 Cache timing "\
            "annotation latency for write accesses given in ticks per "\
            "access. If this parameter is non-zero, per-access latencies "\
            "will be used instead of per-byte even if l2cache-write_latency "\
            "is set. This is only used when l2cache-state_modelled=true.")
    cpu0_l2cache_write_latency = Param.UInt64(0, "L2 Cache timing annotation "\
            "latency for write accesses given in ticks per byte accessed. "\
            "l2cache-write_access_latency must be set to 0 for per-byte "\
            "latencies to be applied. This is only used when "\
            "l2cache-state_modelled=true.")
    cpu0_max_code_cache_mb = Param.MemorySize32('0x100', "Maximum size of "\
            "the simulation code cache (MiB). For platforms with more than 2 "\
            "cores this limit will be scaled down. (e.g 1/8 for 16 or more "\
            "cores)")
    cpu0_min_sync_level = Param.Unsigned(0, "Force minimum syncLevel "\
            "(0=off=default,1=syncState,2=postInsnIO,3=postInsnAll)")
    cpu0_semihosting_A32_HLT = Param.UInt16(0xf000,
            "A32 HLT number for semihosting calls.")
    cpu0_semihosting_A64_HLT = Param.UInt16(0xf000,
            "A64 HLT number for semihosting calls.")
    cpu0_semihosting_ARM_SVC = Param.UInt32(0x123456,
            "A32 SVC number for semihosting calls.")
    cpu0_semihosting_T32_HLT = Param.Unsigned(60,
            "T32 HLT number for semihosting calls.")
    cpu0_semihosting_Thumb_SVC = Param.Unsigned(171,
            "T32 SVC number for semihosting calls.")
    cpu0_semihosting_cmd_line = Param.String("",
            "Command line available to semihosting calls.")
    cpu0_semihosting_cwd = Param.String("",
            "Base directory for semihosting file access.")
    cpu0_semihosting_enable = Param.Bool(True,
            "Enable semihosting SVC/HLT traps.")
    cpu0_semihosting_heap_base = Param.Addr(0x0,
            "Virtual address of heap base.")
    cpu0_semihosting_heap_limit = Param.Addr(0xf000000,
            "Virtual address of top of heap.")
    cpu0_semihosting_stack_base = Param.Addr(0x10000000,
            "Virtual address of base of descending stack.")
    cpu0_semihosting_stack_limit = Param.Addr(0xf000000,
            "Virtual address of stack limit.")
    cpu0_trace_special_hlt_imm16 = Param.UInt16(0xf000, "For this HLT "\
            "number, IF enable_trace_special_hlt_imm16=true, skip performing "\
            "usual HLT execution but call MTI trace if registered")
    cpu0_vfp_enable_at_reset = Param.Bool(False, "Enable VFP in CPACR, "\
            "CPPWR, NSACR at reset. Warning: Arm recommends going through "\
            "the implementation's suggested VFP power-up sequence!")
