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
 *
 * Authors: Gabe Black
 */

#include "arch/arm/fastmodel/CortexA76x1/cortex_a76x1.hh"

#include "arch/arm/fastmodel/arm/cpu.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"
#include "base/logging.hh"
#include "dev/arm/base_gic.hh"
#include "sim/core.hh"
#include "systemc/tlm_bridge/gem5_to_tlm.hh"

namespace FastModel
{

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

Port &
CortexA76::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "redistributor")
        return cluster->getEvs()->gem5_getPort(if_name, num);
    else
        return ArmCPU::getPort(if_name, idx);
}

CortexA76Cluster::CortexA76Cluster(Params &p) :
    SimObject(&p), _params(p), cores(p.cores), evs(p.evs)
{
    for (int i = 0; i < p.cores.size(); i++)
        p.cores[i]->setCluster(this, i);

    sc_core::sc_attr_base *base;

    base = evs->get_attribute(Iris::Gem5CpuClusterAttributeName);
    auto *gem5_cluster_attr =
        dynamic_cast<sc_core::sc_attribute<CortexA76Cluster *> *>(base);
    panic_if(base && !gem5_cluster_attr,
             "The EVS gem5 CPU cluster attribute was not of type "
             "sc_attribute<FastModel::CortexA76Cluster *>.");
    if (gem5_cluster_attr)
        gem5_cluster_attr->value = this;

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

void
ScxEvsCortexA76x1::clockChangeHandler()
{
    clockRateControl->set_mul_div(SimClock::Int::s, clockPeriod.value);
}

ScxEvsCortexA76x1::ScxEvsCortexA76x1(const sc_core::sc_module_name &mod_name,
        const Params &p) :
    scx_evs_CortexA76x1(mod_name),
    amba(scx_evs_CortexA76x1::amba, p.name + ".amba", -1),
    redist {
      new TlmGicTarget(redistributor[0],
              csprintf("%s.redistributor[%d]", name(), 0), 0)
    },
    cnthpirq("cnthpirq"), cnthvirq("cnthvirq"), cntpsirq("cntpsirq"),
    cntvirq("cntvirq"), commirq("commirq"), ctidbgirq("ctidbgirq"),
    pmuirq("pmuirq"), vcpumntirq("vcpumntirq"), cntpnsirq("cntpnsirq"),
    clockChanged(Iris::ClockEventName.c_str()),
    clockPeriod(Iris::PeriodAttributeName.c_str()),
    gem5CpuCluster(Iris::Gem5CpuClusterAttributeName.c_str()),
    sendFunctional(Iris::SendFunctionalAttributeName.c_str()),
    params(p)
{
    clockRateControl.bind(clock_rate_s);

    add_attribute(gem5CpuCluster);
    add_attribute(clockPeriod);
    SC_METHOD(clockChangeHandler);
    dont_initialize();
    sensitive << clockChanged;

    scx_evs_CortexA76x1::cnthpirq[0].bind(cnthpirq.signal_in);
    scx_evs_CortexA76x1::cnthvirq[0].bind(cnthvirq.signal_in);
    scx_evs_CortexA76x1::cntpsirq[0].bind(cntpsirq.signal_in);
    scx_evs_CortexA76x1::cntvirq[0].bind(cntvirq.signal_in);
    scx_evs_CortexA76x1::commirq[0].bind(commirq.signal_in);
    scx_evs_CortexA76x1::ctidbgirq[0].bind(ctidbgirq.signal_in);
    scx_evs_CortexA76x1::pmuirq[0].bind(pmuirq.signal_in);
    scx_evs_CortexA76x1::vcpumntirq[0].bind(vcpumntirq.signal_in);
    scx_evs_CortexA76x1::cntpnsirq[0].bind(cntpnsirq.signal_in);

    sendFunctional.value = [this](PacketPtr pkt) { sendFunc(pkt); };
    add_attribute(sendFunctional);
}

void
ScxEvsCortexA76x1::sendFunc(PacketPtr pkt)
{
    auto *trans = sc_gem5::packet2payload(pkt);
    panic_if(scx_evs_CortexA76x1::amba->transport_dbg(*trans) !=
            trans->get_data_length(), "Didn't send entire functional packet!");
    trans->release();
}

void
ScxEvsCortexA76x1::before_end_of_elaboration()
{
    scx_evs_CortexA76x1::before_end_of_elaboration();

    auto *cluster = gem5CpuCluster.value;

    auto set_on_change = [cluster](
            SignalReceiver &recv, ArmInterruptPinGen *gen, int num)
    {
        auto *pin = gen->get(cluster->getCore(num)->getContext(0));
        auto handler = [pin](bool status)
        {
            status ? pin->raise() : pin->clear();
        };
        recv.onChange(handler);
    };

    set_on_change(cnthpirq, cluster->params().cnthpirq, 0);
    set_on_change(cnthvirq, cluster->params().cnthvirq, 0);
    set_on_change(cntpsirq, cluster->params().cntpsirq, 0);
    set_on_change(cntvirq, cluster->params().cntvirq, 0);
    set_on_change(commirq, cluster->params().commirq, 0);
    set_on_change(ctidbgirq, cluster->params().ctidbgirq, 0);
    set_on_change(pmuirq, cluster->params().pmuirq, 0);
    set_on_change(vcpumntirq, cluster->params().vcpumntirq, 0);
    set_on_change(cntpnsirq, cluster->params().cntpnsirq, 0);
}

Port &
ScxEvsCortexA76x1::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "redistributor")
        return *redist.at(idx);
    else if (if_name == "amba")
        return amba;
    else
        return scx_evs_CortexA76x1::gem5_getPort(if_name, idx);
}

} // namespace FastModel

FastModel::CortexA76 *
FastModelCortexA76Params::create()
{
    return new FastModel::CortexA76(*this);
}

FastModel::CortexA76Cluster *
FastModelCortexA76ClusterParams::create()
{
    return new FastModel::CortexA76Cluster(*this);
}

FastModel::ScxEvsCortexA76x1 *
FastModelScxEvsCortexA76x1Params::create()
{
    return new FastModel::ScxEvsCortexA76x1(name.c_str(), *this);
}
