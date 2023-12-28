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

from m5.objects.ArmInterrupts import ArmInterrupts
from m5.objects.ArmISA import ArmISA
from m5.objects.FastModel import (
    AmbaInitiatorSocket,
    AmbaTargetSocket,
)
from m5.objects.FastModelGIC import Gicv3CommsTargetSocket
from m5.objects.Gic import ArmPPI
from m5.objects.IntPin import IntSinkPin
from m5.objects.Iris import IrisBaseCPU
from m5.objects.ResetPort import ResetResponsePort
from m5.objects.SystemC import SystemC_ScModule
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util.fdthelper import (
    FdtNode,
    FdtPropertyWords,
)


class FastModelCortexA76(IrisBaseCPU):
    type = "FastModelCortexA76"
    cxx_class = "gem5::fastmodel::CortexA76"
    cxx_header = "arch/arm/fastmodel/CortexA76/cortex_a76.hh"

    cntfrq = Param.UInt64(0x1800000, "Value for the CNTFRQ timer register")

    evs = Parent.evs

    redistributor = Gicv3CommsTargetSocket("GIC communication target")
    core_reset = IntSinkPin(
        "Raising this signal will put the core into reset mode."
    )
    poweron_reset = IntSinkPin(
        "Power on reset. Initializes all the "
        "processor logic, including debug logic."
    )

    CFGEND = Param.Bool(
        False,
        "Endianness configuration at reset.  "
        "0, little endian. 1, big endian.",
    )
    CFGTE = Param.Bool(
        False,
        "Instruction set state when resetting "
        "into AArch32.  0, A32. 1, T32.",
    )
    CRYPTODISABLE = Param.Bool(False, "Disable cryptographic features.")
    RVBARADDR = Param.Addr(0x0, "Value of RVBAR_ELx register.")
    VINITHI = Param.Bool(False, "Reset value of SCTLR.V.")
    enable_trace_special_hlt_imm16 = Param.Bool(
        False, "Enable usage of parameter trace_special_hlt_imm16"
    )
    l2cache_hit_latency = Param.UInt64(
        0,
        "L2 Cache timing annotation "
        "latency for hit.  Intended to model the tag-lookup time.  This "
        "is only used when l2cache-state_modelled=true.",
    )
    l2cache_maintenance_latency = Param.UInt64(
        0,
        "L2 Cache timing "
        "annotation latency for cache maintenance operations given in "
        "total ticks. This is only used when dcache-state_modelled=true.",
    )
    l2cache_miss_latency = Param.UInt64(
        0,
        "L2 Cache timing annotation "
        "latency for miss.  Intended to model the time for failed "
        "tag-lookup and allocation of intermediate buffers.  This is "
        "only used when l2cache-state_modelled=true.",
    )
    l2cache_read_access_latency = Param.UInt64(
        0,
        "L2 Cache timing "
        "annotation latency for read accesses given in ticks per "
        "access.  If this parameter is non-zero, per-access latencies "
        "will be used instead of per-byte even if l2cache-read_latency "
        "is set. This is in addition to the hit or miss latency, and "
        "intended to correspond to the time taken to transfer across the "
        "cache upstream bus, this is only used when "
        "l2cache-state_modelled=true.",
    )
    l2cache_read_latency = Param.UInt64(
        0,
        "L2 Cache timing annotation "
        "latency for read accesses given in ticks per byte "
        "accessed.l2cache-read_access_latency must be set to 0 for "
        "per-byte latencies to be applied.  This is in addition to the "
        "hit or miss latency, and intended to correspond to the time "
        "taken to transfer across the cache upstream bus. This is only "
        "used when l2cache-state_modelled=true.",
    )
    l2cache_size = Param.MemorySize32("0x80000", "L2 Cache size in bytes.")
    l2cache_snoop_data_transfer_latency = Param.UInt64(
        0,
        "L2 Cache "
        "timing annotation latency for received snoop accesses that "
        "perform a data transfer given in ticks per byte accessed. This "
        "is only used when dcache-state_modelled=true.",
    )
    l2cache_snoop_issue_latency = Param.UInt64(
        0,
        "L2 Cache timing "
        "annotation latency for snoop accesses issued by this cache in "
        "total ticks. This is only used when dcache-state_modelled=true.",
    )
    l2cache_write_access_latency = Param.UInt64(
        0,
        "L2 Cache timing "
        "annotation latency for write accesses given in ticks per "
        "access. If this parameter is non-zero, per-access latencies "
        "will be used instead of per-byte even if l2cache-write_latency "
        "is set. This is only used when l2cache-state_modelled=true.",
    )
    l2cache_write_latency = Param.UInt64(
        0,
        "L2 Cache timing annotation "
        "latency for write accesses given in ticks per byte accessed. "
        "l2cache-write_access_latency must be set to 0 for per-byte "
        "latencies to be applied. This is only used when "
        "l2cache-state_modelled=true.",
    )
    max_code_cache_mb = Param.MemorySize32(
        "0x100",
        "Maximum size of "
        "the simulation code cache (MiB). For platforms with more than 2 "
        "cores this limit will be scaled down. (e.g 1/8 for 16 or more "
        "cores)",
    )
    min_sync_level = Param.Unsigned(
        0,
        "Force minimum syncLevel "
        "(0=off=default,1=syncState,2=postInsnIO,3=postInsnAll)",
    )
    semihosting_A32_HLT = Param.UInt16(
        0xF000, "A32 HLT number for semihosting calls."
    )
    semihosting_A64_HLT = Param.UInt16(
        0xF000, "A64 HLT number for semihosting calls."
    )
    semihosting_ARM_SVC = Param.UInt32(
        0x123456, "A32 SVC number for semihosting calls."
    )
    semihosting_T32_HLT = Param.Unsigned(
        60, "T32 HLT number for semihosting calls."
    )
    semihosting_Thumb_SVC = Param.Unsigned(
        171, "T32 SVC number for semihosting calls."
    )
    semihosting_cmd_line = Param.String(
        "", "Command line available to semihosting calls."
    )
    semihosting_cwd = Param.String(
        "", "Base directory for semihosting file access."
    )
    semihosting_enable = Param.Bool(True, "Enable semihosting SVC/HLT traps.")
    semihosting_heap_base = Param.Addr(0x0, "Virtual address of heap base.")
    semihosting_heap_limit = Param.Addr(
        0xF000000, "Virtual address of top of heap."
    )
    semihosting_stack_base = Param.Addr(
        0x10000000, "Virtual address of base of descending stack."
    )
    semihosting_stack_limit = Param.Addr(
        0xF000000, "Virtual address of stack limit."
    )
    trace_special_hlt_imm16 = Param.UInt16(
        0xF000,
        "For this HLT "
        "number, IF enable_trace_special_hlt_imm16=true, skip performing "
        "usual HLT execution but call MTI trace if registered",
    )
    vfp_enable_at_reset = Param.Bool(
        False,
        "Enable VFP in CPACR, "
        "CPPWR, NSACR at reset. Warning: Arm recommends going through "
        "the implementation's suggested VFP power-up sequence!",
    )


class FastModelCortexA76Cluster(SimObject):
    type = "FastModelCortexA76Cluster"
    cxx_class = "gem5::fastmodel::CortexA76Cluster"
    cxx_header = "arch/arm/fastmodel/CortexA76/cortex_a76.hh"

    cores = VectorParam.FastModelCortexA76(
        "Core in a given cluster of CortexA76s"
    )

    evs = Param.SystemC_ScModule(
        "Fast mo0del exported virtual subsystem holding cores"
    )

    cnthpirq = Param.ArmInterruptPin(
        ArmPPI(num=10), "EL2 physical timer event"
    )
    cnthvirq = Param.ArmInterruptPin(ArmPPI(num=12), "EL2 virtual timer event")
    cntpsirq = Param.ArmInterruptPin(
        ArmPPI(num=13), "EL1 Secure physical timer event"
    )
    cntvirq = Param.ArmInterruptPin(ArmPPI(num=11), "Virtual timer event")
    commirq = Param.ArmInterruptPin(
        ArmPPI(num=6), "Interrupt signal from debug communications channel"
    )
    ctidbgirq = Param.ArmInterruptPin(
        ArmPPI(num=8), "Cross Trigger Interface (CTI) interrupt trigger output"
    )
    pmuirq = Param.ArmInterruptPin(
        ArmPPI(num=7), "Interrupt from performance monitoring unit"
    )
    vcpumntirq = Param.ArmInterruptPin(
        ArmPPI(num=9), "Interrupt signal for virtual CPU maintenance IRQ"
    )
    cntpnsirq = Param.ArmInterruptPin(
        ArmPPI(num=14), "Non-secure physical timer event"
    )

    amba = AmbaInitiatorSocket(64, "AMBA initiator socket")
    top_reset = IntSinkPin(
        "A single cluster-wide power on reset signal for "
        "all resettable registers in DynamIQ."
    )
    dbg_reset = IntSinkPin(
        "Initialize the shared debug APB, Cross Trigger "
        "Interface (CTI), and Cross Trigger Matrix (CTM) logic."
    )
    model_reset = ResetResponsePort("A reset port to reset the whole cluster.")

    # These parameters are described in "Fast Models Reference Manual" section
    # 3.4.19, "ARMCortexA7x1CT".
    BROADCASTATOMIC = Param.Bool(
        True,
        "Enable broadcasting of atomic "
        "operation. The broadcastatomic signal will override this value "
        "if used",
    )
    BROADCASTCACHEMAINT = Param.Bool(
        True,
        "Enable broadcasting of cache "
        "maintenance operations to downstream caches. The "
        "broadcastcachemaint signal will override this value if used.",
    )
    BROADCASTOUTER = Param.Bool(
        True,
        "Enable broadcasting of Outer "
        "Shareable transactions. The broadcastouter signal will override "
        "this value if used.",
    )
    BROADCASTPERSIST = Param.Bool(
        True,
        "Enable broadcasting  of cache clean "
        "to the point of persistence operations. The broadcastpersist "
        "signal will override this value if used",
    )
    CLUSTER_ID = Param.UInt16(0x0, "Processor cluster ID value")
    GICDISABLE = Param.Bool(
        True,
        "Disable the new style GICv3 CPU interface "
        "in each core model. Should be left enabled unless the platform "
        "contains a GICv3 distributor.",
    )
    cpi_div = Param.UInt32(
        1, "Divider for calculating CPI (Cycles Per Instruction)"
    )
    cpi_mul = Param.UInt32(
        1, "Multiplier for calculating CPI (Cycles Per Instruction)"
    )
    dcache_hit_latency = Param.UInt64(
        0,
        "L1 D-Cache timing annotation "
        "latency for hit.  Intended to model the tag-lookup time.  This "
        "is only used when dcache-state_modelled=true.",
    )
    dcache_maintenance_latency = Param.UInt64(
        0,
        "L1 D-Cache timing "
        "annotation latency for cache maintenance operations given in "
        "total ticks. This is only used when dcache-state_modelled=true.",
    )
    dcache_miss_latency = Param.UInt64(
        0,
        "L1 D-Cache timing annotation "
        "latency for miss.  Intended to model the time for failed "
        "tag-lookup and allocation of intermediate buffers.  This is "
        "only used when dcache-state_modelled=true.",
    )
    dcache_prefetch_enabled = Param.Bool(
        False,
        "Enable simulation of data "
        "cache prefetching.  This is only used when "
        "dcache-state_modelled=true",
    )
    dcache_read_access_latency = Param.UInt64(
        0,
        "L1 D-Cache timing "
        "annotation latency for read accesses given in ticks per access "
        "(of size dcache-read_bus_width_in_bytes).  If this parameter is "
        "non-zero, per-access latencies will be used instead of per-byte "
        "even if dcache-read_latency is set. This is in addition to the "
        "hit or miss latency, and intended to correspond to the time "
        "taken to transfer across the cache upstream bus, this is only "
        "used when dcache-state_modelled=true.",
    )
    dcache_read_latency = Param.UInt64(
        0,
        "L1 D-Cache timing annotation "
        "latency for read accesses given in ticks per byte "
        "accessed.dcache-read_access_latency must be set to 0 for "
        "per-byte latencies to be applied.  This is in addition to the "
        "hit or miss latency, and intended to correspond to the time "
        "taken to transfer across the cache upstream bus. This is only "
        "used when dcache-state_modelled=true.",
    )
    dcache_snoop_data_transfer_latency = Param.UInt64(
        0,
        "L1 D-Cache timing "
        "annotation latency for received snoop accesses that perform a data "
        "transfer given in ticks per byte accessed. This is only used when "
        "dcache-state_modelled=true.",
    )
    dcache_state_modelled = Param.Bool(
        False, "Set whether D-cache has stateful implementation"
    )
    dcache_write_access_latency = Param.UInt64(
        0,
        "L1 D-Cache timing "
        "annotation latency for write accesses given in ticks per access "
        "(of size dcache-write_bus_width_in_bytes). If this parameter is "
        "non-zero, per-access latencies will be used instead of per-byte "
        "even if dcache-write_latency is set. This is only used when "
        "dcache-state_modelled=true.",
    )
    dcache_write_latency = Param.UInt64(
        0,
        "L1 D-Cache timing annotation "
        "latency for write accesses given in ticks per byte accessed. "
        "dcache-write_access_latency must be set to 0 for per-byte latencies "
        "to be applied. This is only used when dcache-state_modelled=true.",
    )
    default_opmode = Param.Unsigned(
        4,
        "Operating mode of DynamIQ coming out "
        "of reset. 0: SFONLY ON, 1: 1/4 CACHE ON, 2: 1/2 CACHE ON, 3: "
        "3/4 CACHE ON, 4: FULL CACHE ON",
    )
    diagnostics = Param.Bool(False, "Enable DynamIQ diagnostic messages")
    enable_simulation_performance_optimizations = Param.Bool(
        True,
        "With this option enabled, the model will run more quickly, but "
        "be less accurate to exact CPU behavior. The model will still be "
        "functionally accurate for software, but may increase "
        "differences seen between hardware behavior and model behavior "
        "for certain workloads (it changes the micro-architectural value "
        "of stage12_tlb_size parameter to 1024).",
    )
    ext_abort_device_read_is_sync = Param.Bool(
        False, "Synchronous reporting of device-nGnRE read external aborts"
    )
    ext_abort_device_write_is_sync = Param.Bool(
        False, "Synchronous reporting of device-nGnRE write external aborts"
    )
    ext_abort_so_read_is_sync = Param.Bool(
        False, "Synchronous reporting of device-nGnRnE read external aborts"
    )
    ext_abort_so_write_is_sync = Param.Bool(
        False, "Synchronous reporting of device-nGnRnE write external aborts"
    )
    gicv3_cpuintf_mmap_access_level = Param.Unsigned(
        0,
        "Allowed values are: "
        "0-mmap access is supported for GICC,GICH,GICV registers. 1-mmap "
        "access is supported only for GICV registers. 2-mmap access is "
        "not supported.",
    )
    has_peripheral_port = Param.Bool(
        False, "If true, additional AXI peripheral port is configured."
    )
    has_statistical_profiling = Param.Bool(
        True, "Whether Statistical Based Profiling is implemented"
    )
    icache_hit_latency = Param.UInt64(
        0,
        "L1 I-Cache timing annotation "
        "latency for hit.  Intended to model the tag-lookup time.  This "
        "is only used when icache-state_modelled=true.",
    )
    icache_maintenance_latency = Param.UInt64(
        0,
        "L1 I-Cache timing "
        "annotation latency for cache maintenance operations given in "
        "total ticks. This is only used when icache-state_modelled=true.",
    )
    icache_miss_latency = Param.UInt64(
        0,
        "L1 I-Cache timing annotation "
        "latency for miss.  Intended to model the time for failed "
        "tag-lookup and allocation of intermediate buffers.  This is "
        "only used when icache-state_modelled=true.",
    )
    icache_prefetch_enabled = Param.Bool(
        False,
        "Enable simulation of "
        "instruction cache prefetching. This is only used when "
        "icache-state_modelled=true.",
    )
    icache_read_access_latency = Param.UInt64(
        0,
        "L1 I-Cache timing "
        "annotation latency for read accesses given in ticks per access "
        "(of size icache-read_bus_width_in_bytes).  If this parameter is "
        "non-zero, per-access latencies will be used instead of per-byte "
        "even if icache-read_latency is set. This is in addition to the "
        "hit or miss latency, and intended to correspond to the time "
        "taken to transfer across the cache upstream bus, this is only "
        "used when icache-state_modelled=true.",
    )
    icache_read_latency = Param.UInt64(
        0,
        "L1 I-Cache timing annotation "
        "latency for read accesses given in ticks per byte "
        "accessed.icache-read_access_latency must be set to 0 for "
        "per-byte latencies to be applied.  This is in addition to the "
        "hit or miss latency, and intended to correspond to the time "
        "taken to transfer across the cache upstream bus. This is only "
        "used when icache-state_modelled=true.",
    )
    icache_state_modelled = Param.Bool(
        False, "Set whether I-cache has stateful implementation"
    )
    l3cache_hit_latency = Param.UInt64(
        0,
        "L3 Cache timing annotation "
        "latency for hit.  Intended to model the tag-lookup time.  This "
        "is only used when l3cache-state_modelled=true.",
    )
    l3cache_maintenance_latency = Param.UInt64(
        0,
        "L3 Cache timing "
        "annotation latency for cache maintenance operations given in "
        "total ticks. This is only used when dcache-state_modelled=true.",
    )
    l3cache_miss_latency = Param.UInt64(
        0,
        "L3 Cache timing annotation "
        "latency for miss.  Intended to model the time for failed "
        "tag-lookup and allocation of intermediate buffers.  This is "
        "only used when l3cache-state_modelled=true.",
    )
    l3cache_read_access_latency = Param.UInt64(
        0,
        "L3 Cache timing "
        "annotation latency for read accesses given in ticks per access "
        "(of size l3cache-read_bus_width_in_bytes).  If this parameter "
        "is non-zero, per-access latencies will be used instead of "
        "per-byte even if l3cache-read_latency is set. This is in "
        "addition to the hit or miss latency, and intended to correspond "
        "to the time taken to transfer across the cache upstream bus, "
        "this is only used when l3cache-state_modelled=true.",
    )
    l3cache_read_latency = Param.UInt64(
        0,
        "L3 Cache timing annotation "
        "latency for read accesses given in ticks per byte "
        "accessed.l3cache-read_access_latency must be set to 0 for "
        "per-byte latencies to be applied.  This is in addition to the "
        "hit or miss latency, and intended to correspond to the time "
        "taken to transfer across the cache upstream bus. This is only "
        "used when l3cache-state_modelled=true.",
    )
    l3cache_size = Param.MemorySize("0x100000", "L3 Cache size in bytes.")
    l3cache_snoop_data_transfer_latency = Param.UInt64(
        0,
        "L3 Cache timing "
        "annotation latency for received snoop accesses that perform a "
        "data transfer given in ticks per byte accessed. This is only "
        "used when dcache-state_modelled=true.",
    )
    l3cache_snoop_issue_latency = Param.UInt64(
        0,
        "L3 Cache timing "
        "annotation latency for snoop accesses issued by this cache in "
        "total ticks. This is only used when dcache-state_modelled=true.",
    )
    l3cache_write_access_latency = Param.UInt64(
        0,
        "L3 Cache timing "
        "annotation latency for write accesses given in ticks per access "
        "(of size l3cache-write_bus_width_in_bytes). If this parameter "
        "is non-zero, per-access latencies will be used instead of "
        "per-byte even if l3cache-write_latency is set. This is only "
        "used when l3cache-state_modelled=true.",
    )
    l3cache_write_latency = Param.UInt64(
        0,
        "L3 Cache timing annotation "
        "latency for write accesses given in ticks per byte accessed. "
        "l3cache-write_access_latency must be set to 0 for per-byte "
        "latencies to be applied. This is only used when "
        "l3cache-state_modelled=true.",
    )
    pchannel_treat_simreset_as_poreset = Param.Bool(
        False, "Register core as ON state to cluster with simulation reset."
    )
    periph_address_end = Param.Addr(
        0x0,
        "End address for peripheral port "
        "address range exclusive(corresponds to AENDMP input signal).",
    )
    periph_address_start = Param.Addr(
        0x0,
        "Start address for peripheral "
        "port address range inclusive(corresponds to ASTARTMP input "
        "signal).",
    )
    ptw_latency = Param.UInt64(
        0,
        "Page table walker latency for TA "
        "(Timing Annotation), expressed in simulation ticks",
    )
    tlb_latency = Param.UInt64(
        0,
        "TLB latency for TA (Timing Annotation), "
        "expressed in simulation ticks",
    )
    treat_dcache_cmos_to_pou_as_nop = Param.Bool(
        False,
        "Whether dcache "
        "invalidation to the point of unification is required for "
        "instruction to data coherence. true - Invalidate operations not "
        "required",
    )
    walk_cache_latency = Param.UInt64(
        0,
        "Walk cache latency for TA (Timing "
        "Annotation), expressed in simulation ticks",
    )

    def generateDeviceTree(self, state):
        node = FdtNode("timer")

        node.appendCompatible(
            ["arm,cortex-a15-timer", "arm,armv7-timer", "arm,armv8-timer"]
        )
        node.append(
            FdtPropertyWords(
                "interrupts",
                [
                    1,
                    int(self.cntpsirq.num),
                    0xF08,
                    1,
                    int(self.cntpnsirq.num),
                    0xF08,
                    1,
                    int(self.cntvirq.num),
                    0xF08,
                    1,
                    int(self.cnthpirq.num),
                    0xF08,
                ],
            )
        )

        yield node


class FastModelScxEvsCortexA76x1(SystemC_ScModule):
    type = "FastModelScxEvsCortexA76x1"
    cxx_class = (
        "gem5::fastmodel::ScxEvsCortexA76<"
        "gem5::fastmodel::ScxEvsCortexA76x1Types>"
    )
    cxx_template_params = ["class Types"]
    cxx_header = "arch/arm/fastmodel/CortexA76/evs.hh"


class FastModelCortexA76x1(FastModelCortexA76Cluster):
    cores = [FastModelCortexA76(thread_paths=["core.cpu0"])]

    evs = FastModelScxEvsCortexA76x1()


class FastModelScxEvsCortexA76x2(SystemC_ScModule):
    type = "FastModelScxEvsCortexA76x2"
    cxx_class = (
        "gem5::fastmodel::ScxEvsCortexA76<"
        "gem5::fastmodel::ScxEvsCortexA76x2Types>"
    )
    cxx_template_params = ["class Types"]
    cxx_header = "arch/arm/fastmodel/CortexA76/evs.hh"


class FastModelCortexA76x2(FastModelCortexA76Cluster):
    cores = [
        FastModelCortexA76(thread_paths=["core.cpu0"]),
        FastModelCortexA76(thread_paths=["core.cpu1"]),
    ]

    evs = FastModelScxEvsCortexA76x2()


class FastModelScxEvsCortexA76x3(SystemC_ScModule):
    type = "FastModelScxEvsCortexA76x3"
    cxx_class = (
        "gem5::fastmodel::ScxEvsCortexA76<"
        "gem5::fastmodel::ScxEvsCortexA76x3Types>"
    )
    cxx_template_params = ["class Types"]
    cxx_header = "arch/arm/fastmodel/CortexA76/evs.hh"


class FastModelCortexA76x3(FastModelCortexA76Cluster):
    cores = [
        FastModelCortexA76(thread_paths=["core.cpu0"]),
        FastModelCortexA76(thread_paths=["core.cpu1"]),
        FastModelCortexA76(thread_paths=["core.cpu2"]),
    ]

    evs = FastModelScxEvsCortexA76x3()


class FastModelScxEvsCortexA76x4(SystemC_ScModule):
    type = "FastModelScxEvsCortexA76x4"
    cxx_class = (
        "gem5::fastmodel::ScxEvsCortexA76<"
        "gem5::fastmodel::ScxEvsCortexA76x4Types>"
    )
    cxx_template_params = ["class Types"]
    cxx_header = "arch/arm/fastmodel/CortexA76/evs.hh"


class FastModelCortexA76x4(FastModelCortexA76Cluster):
    cores = [
        FastModelCortexA76(thread_paths=["core.cpu0"]),
        FastModelCortexA76(thread_paths=["core.cpu1"]),
        FastModelCortexA76(thread_paths=["core.cpu2"]),
        FastModelCortexA76(thread_paths=["core.cpu3"]),
    ]

    evs = FastModelScxEvsCortexA76x4()
