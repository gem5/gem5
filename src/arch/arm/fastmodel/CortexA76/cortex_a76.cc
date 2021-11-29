/*
 * Copyright 2019 Google, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/arm/fastmodel/CortexA76/cortex_a76.hh"

#include "arch/arm/fastmodel/iris/cpu.hh"
#include "arch/arm/regs/misc.hh"
#include "base/logging.hh"
#include "dev/arm/base_gic.hh"
#include "systemc/tlm_bridge/gem5_to_tlm.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

void
CortexA76::initState()
{
    for (auto *tc : threadContexts)
        tc->setMiscRegNoEffect(ArmISA::MISCREG_CNTFRQ_EL0, params().cntfrq);

    evs_base_cpu->setSysCounterFrq(params().cntfrq);
}

void
CortexA76::setCluster(CortexA76Cluster *_cluster, int _num)
{
    cluster = _cluster;
    num = _num;

    set_evs_param("CFGEND", params().CFGEND);
    set_evs_param("CFGTE", params().CFGTE);
    set_evs_param("CRYPTODISABLE", params().CRYPTODISABLE);
    set_evs_param("RVBARADDR", params().RVBARADDR);
    set_evs_param("VINITHI", params().VINITHI);
    set_evs_param("enable_trace_special_hlt_imm16",
                  params().enable_trace_special_hlt_imm16);
    set_evs_param("l2cache-hit_latency", params().l2cache_hit_latency);
    set_evs_param("l2cache-maintenance_latency",
                  params().l2cache_maintenance_latency);
    set_evs_param("l2cache-miss_latency", params().l2cache_miss_latency);
    set_evs_param("l2cache-read_access_latency",
                  params().l2cache_read_access_latency);
    set_evs_param("l2cache-read_latency", params().l2cache_read_latency);
    set_evs_param("l2cache-size", params().l2cache_size);
    set_evs_param("l2cache-snoop_data_transfer_latency",
                  params().l2cache_snoop_data_transfer_latency);
    set_evs_param("l2cache-snoop_issue_latency",
                  params().l2cache_snoop_issue_latency);
    set_evs_param("l2cache-write_access_latency",
                  params().l2cache_write_access_latency);
    set_evs_param("l2cache-write_latency", params().l2cache_write_latency);
    set_evs_param("max_code_cache_mb", params().max_code_cache_mb);
    set_evs_param("min_sync_level", params().min_sync_level);
    set_evs_param("semihosting-A32_HLT", params().semihosting_A32_HLT);
    set_evs_param("semihosting-A64_HLT", params().semihosting_A64_HLT);
    set_evs_param("semihosting-ARM_SVC", params().semihosting_ARM_SVC);
    set_evs_param("semihosting-T32_HLT", params().semihosting_T32_HLT);
    set_evs_param("semihosting-Thumb_SVC", params().semihosting_Thumb_SVC);
    set_evs_param("semihosting-cmd_line", params().semihosting_cmd_line);
    set_evs_param("semihosting-cwd", params().semihosting_cwd);
    set_evs_param("semihosting-enable", params().semihosting_enable);
    set_evs_param("semihosting-heap_base", params().semihosting_heap_base);
    set_evs_param("semihosting-heap_limit", params().semihosting_heap_limit);
    set_evs_param("semihosting-stack_base", params().semihosting_stack_base);
    set_evs_param("semihosting-stack_limit", params().semihosting_stack_limit);
    set_evs_param("trace_special_hlt_imm16", params().trace_special_hlt_imm16);
    set_evs_param("vfp-enable_at_reset", params().vfp_enable_at_reset);
}

void
CortexA76::setResetAddr(Addr addr, bool secure)
{
    evs_base_cpu->setResetAddr(num, addr, secure);
}

Port &
CortexA76::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "redistributor")
        return cluster->getEvs()->gem5_getPort(if_name, num);
    else
        return Base::getPort(if_name, idx);
}

CortexA76Cluster::CortexA76Cluster(const Params &p) :
    SimObject(p), cores(p.cores), evs(p.evs)
{
    for (int i = 0; i < p.cores.size(); i++)
        p.cores[i]->setCluster(this, i);

    Iris::BaseCpuEvs *e = dynamic_cast<Iris::BaseCpuEvs *>(evs);
    panic_if(!e, "EVS should be of type Iris::BaseCpuEvs");
    e->setCluster(this);

    set_evs_param("core.BROADCASTATOMIC", p.BROADCASTATOMIC);
    set_evs_param("core.BROADCASTCACHEMAINT", p.BROADCASTCACHEMAINT);
    set_evs_param("core.BROADCASTOUTER", p.BROADCASTOUTER);
    set_evs_param("core.BROADCASTPERSIST", p.BROADCASTPERSIST);
    set_evs_param("core.CLUSTER_ID", p.CLUSTER_ID);
    set_evs_param("core.GICDISABLE", p.GICDISABLE);
    set_evs_param("core.cpi_div", p.cpi_div);
    set_evs_param("core.cpi_mul", p.cpi_mul);
    set_evs_param("core.dcache-hit_latency", p.dcache_hit_latency);
    set_evs_param("core.dcache-maintenance_latency",
                  p.dcache_maintenance_latency);
    set_evs_param("core.dcache-miss_latency", p.dcache_miss_latency);
    set_evs_param("core.dcache-prefetch_enabled",
                  p.dcache_prefetch_enabled);
    set_evs_param("core.dcache-read_access_latency",
                  p.dcache_read_access_latency);
    set_evs_param("core.dcache-read_latency", p.dcache_read_latency);
    set_evs_param("core.dcache-snoop_data_transfer_latency",
                  p.dcache_snoop_data_transfer_latency);
    set_evs_param("core.dcache-state_modelled", p.dcache_state_modelled);
    set_evs_param("core.dcache-write_access_latency",
                  p.dcache_write_access_latency);
    set_evs_param("core.dcache-write_latency", p.dcache_write_latency);
    set_evs_param("core.default_opmode", p.default_opmode);
    set_evs_param("core.diagnostics", p.diagnostics);
    set_evs_param("core.enable_simulation_performance_optimizations",
                  p.enable_simulation_performance_optimizations);
    set_evs_param("core.ext_abort_device_read_is_sync",
                  p.ext_abort_device_read_is_sync);
    set_evs_param("core.ext_abort_device_write_is_sync",
                  p.ext_abort_device_write_is_sync);
    set_evs_param("core.ext_abort_so_read_is_sync",
                  p.ext_abort_so_read_is_sync);
    set_evs_param("core.ext_abort_so_write_is_sync",
                  p.ext_abort_so_write_is_sync);
    set_evs_param("core.gicv3.cpuintf-mmap-access-level",
                  p.gicv3_cpuintf_mmap_access_level);
    set_evs_param("core.has_peripheral_port", p.has_peripheral_port);
    set_evs_param("core.has_statistical_profiling",
                  p.has_statistical_profiling);
    set_evs_param("core.icache-hit_latency", p.icache_hit_latency);
    set_evs_param("core.icache-maintenance_latency",
                  p.icache_maintenance_latency);
    set_evs_param("core.icache-miss_latency", p.icache_miss_latency);
    set_evs_param("core.icache-prefetch_enabled",
                  p.icache_prefetch_enabled);
    set_evs_param("core.icache-read_access_latency",
                  p.icache_read_access_latency);
    set_evs_param("core.icache-read_latency", p.icache_read_latency);
    set_evs_param("core.icache-state_modelled", p.icache_state_modelled);
    set_evs_param("core.l3cache-hit_latency", p.l3cache_hit_latency);
    set_evs_param("core.l3cache-maintenance_latency",
                  p.l3cache_maintenance_latency);
    set_evs_param("core.l3cache-miss_latency", p.l3cache_miss_latency);
    set_evs_param("core.l3cache-read_access_latency",
                  p.l3cache_read_access_latency);
    set_evs_param("core.l3cache-read_latency", p.l3cache_read_latency);
    set_evs_param("core.l3cache-size", p.l3cache_size);
    set_evs_param("core.l3cache-snoop_data_transfer_latency",
                  p.l3cache_snoop_data_transfer_latency);
    set_evs_param("core.l3cache-snoop_issue_latency",
                  p.l3cache_snoop_issue_latency);
    set_evs_param("core.l3cache-write_access_latency",
                  p.l3cache_write_access_latency);
    set_evs_param("core.l3cache-write_latency", p.l3cache_write_latency);
    set_evs_param("core.pchannel_treat_simreset_as_poreset",
                  p.pchannel_treat_simreset_as_poreset);
    set_evs_param("core.periph_address_end", p.periph_address_end);
    set_evs_param("core.periph_address_start", p.periph_address_start);
    set_evs_param("core.ptw_latency", p.ptw_latency);
    set_evs_param("core.tlb_latency", p.tlb_latency);
    set_evs_param("core.treat-dcache-cmos-to-pou-as-nop",
                  p.treat_dcache_cmos_to_pou_as_nop);
    set_evs_param("core.walk_cache_latency", p.walk_cache_latency);
}

Port &
CortexA76Cluster::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "amba") {
        return evs->gem5_getPort(if_name, idx);
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

} // namespace fastmodel
} // namespace gem5
