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

#include "arch/arm/fastmodel/iris/cpu.hh"
#include "base/logging.hh"
#include "params/FastModelCortexA76x1.hh"
#include "sim/core.hh"

namespace FastModel
{

void
CortexA76x1::clockChangeHandler()
{
    clockRateControl->set_mul_div(SimClock::Int::s, clockPeriod.value);
}

CortexA76x1::CortexA76x1(const sc_core::sc_module_name &mod_name,
        const FastModelCortexA76x1Params &params)
    : scx_evs_CortexA76x1(mod_name),
      amba(scx_evs_CortexA76x1::amba, params.name + ".amba", -1),
      redistributorM(redistributor_m, params.name + ".redistributor_m", -1),
      redistributorS(redistributor_s, params.name + ".redistributor_s", -1),
      cnthpirqWrapper(cnthpirq, params.name + ".cnthpirq", -1),
      cnthvirqWrapper(cnthvirq, params.name + ".cnthvirq", -1),
      cntpsirqWrapper(cntpsirq, params.name + ".cntpsirq", -1),
      cntvirqWrapper(cntvirq, params.name + ".cntvirq", -1),
      commirqWrapper(commirq, params.name + ".commirq", -1),
      ctidbgirqWrapper(ctidbgirq, params.name + ".ctidbgirq", -1),
      pmuirqWrapper(pmuirq, params.name + ".pmuirq", -1),
      vcpumntirqWrapper(vcpumntirq, params.name + ".vcpumntirq", -1),
      cntpnsirqWrapper(cntpnsirq, params.name + ".cntpnsirq", -1),
      clockChanged(Iris::ClockEventName.c_str()),
      clockPeriod(Iris::PeriodAttributeName.c_str()),
      gem5Cpu(Iris::Gem5CpuAttributeName.c_str())
{
    clockRateControl.bind(clock_rate_s);

    set_parameter("core.BROADCASTATOMIC", params.BROADCASTATOMIC);
    set_parameter("core.BROADCASTCACHEMAINT", params.BROADCASTCACHEMAINT);
    set_parameter("core.BROADCASTOUTER", params.BROADCASTOUTER);
    set_parameter("core.BROADCASTPERSIST", params.BROADCASTPERSIST);
    set_parameter("core.CLUSTER_ID", params.CLUSTER_ID);
    set_parameter("core.GICDISABLE", params.GICDISABLE);
    set_parameter("core.cpi_div", params.cpi_div);
    set_parameter("core.cpi_mul", params.cpi_mul);
    set_parameter("core.dcache-hit_latency", params.dcache_hit_latency);
    set_parameter("core.dcache-maintenance_latency",
                  params.dcache_maintenance_latency);
    set_parameter("core.dcache-miss_latency", params.dcache_miss_latency);
    set_parameter("core.dcache-prefetch_enabled",
                  params.dcache_prefetch_enabled);
    set_parameter("core.dcache-read_access_latency",
                  params.dcache_read_access_latency);
    set_parameter("core.dcache-read_latency", params.dcache_read_latency);
    set_parameter("core.dcache-snoop_data_transfer_latency",
                  params.dcache_snoop_data_transfer_latency);
    set_parameter("core.dcache-state_modelled", params.dcache_state_modelled);
    set_parameter("core.dcache-write_access_latency",
                  params.dcache_write_access_latency);
    set_parameter("core.dcache-write_latency", params.dcache_write_latency);
    set_parameter("core.default_opmode", params.default_opmode);
    set_parameter("core.diagnostics", params.diagnostics);
    set_parameter("core.enable_simulation_performance_optimizations",
                  params.enable_simulation_performance_optimizations);
    set_parameter("core.ext_abort_device_read_is_sync",
                  params.ext_abort_device_read_is_sync);
    set_parameter("core.ext_abort_device_write_is_sync",
                  params.ext_abort_device_write_is_sync);
    set_parameter("core.ext_abort_so_read_is_sync",
                  params.ext_abort_so_read_is_sync);
    set_parameter("core.ext_abort_so_write_is_sync",
                  params.ext_abort_so_write_is_sync);
    set_parameter("core.gicv3.cpuintf-mmap-access-level",
                  params.gicv3_cpuintf_mmap_access_level);
    set_parameter("core.has_peripheral_port", params.has_peripheral_port);
    set_parameter("core.has_statistical_profiling",
                  params.has_statistical_profiling);
    set_parameter("core.icache-hit_latency", params.icache_hit_latency);
    set_parameter("core.icache-maintenance_latency",
                  params.icache_maintenance_latency);
    set_parameter("core.icache-miss_latency", params.icache_miss_latency);
    set_parameter("core.icache-prefetch_enabled",
                  params.icache_prefetch_enabled);
    set_parameter("core.icache-read_access_latency",
                  params.icache_read_access_latency);
    set_parameter("core.icache-read_latency", params.icache_read_latency);
    set_parameter("core.icache-state_modelled", params.icache_state_modelled);
    set_parameter("core.l3cache-hit_latency", params.l3cache_hit_latency);
    set_parameter("core.l3cache-maintenance_latency",
                  params.l3cache_maintenance_latency);
    set_parameter("core.l3cache-miss_latency", params.l3cache_miss_latency);
    set_parameter("core.l3cache-read_access_latency",
                  params.l3cache_read_access_latency);
    set_parameter("core.l3cache-read_latency", params.l3cache_read_latency);
    set_parameter("core.l3cache-size", params.l3cache_size);
    set_parameter("core.l3cache-snoop_data_transfer_latency",
                  params.l3cache_snoop_data_transfer_latency);
    set_parameter("core.l3cache-snoop_issue_latency",
                  params.l3cache_snoop_issue_latency);
    set_parameter("core.l3cache-write_access_latency",
                  params.l3cache_write_access_latency);
    set_parameter("core.l3cache-write_latency", params.l3cache_write_latency);
    set_parameter("core.pchannel_treat_simreset_as_poreset",
                  params.pchannel_treat_simreset_as_poreset);
    set_parameter("core.periph_address_end", params.periph_address_end);
    set_parameter("core.periph_address_start", params.periph_address_start);
    set_parameter("core.ptw_latency", params.ptw_latency);
    set_parameter("core.tlb_latency", params.tlb_latency);
    set_parameter("core.treat-dcache-cmos-to-pou-as-nop",
                  params.treat_dcache_cmos_to_pou_as_nop);
    set_parameter("core.walk_cache_latency", params.walk_cache_latency);

    set_parameter("core.cpu0.CFGEND", params.cpu0_CFGEND);
    set_parameter("core.cpu0.CFGTE", params.cpu0_CFGTE);
    set_parameter("core.cpu0.CRYPTODISABLE", params.cpu0_CRYPTODISABLE);
    set_parameter("core.cpu0.RVBARADDR", params.cpu0_RVBARADDR);
    set_parameter("core.cpu0.VINITHI", params.cpu0_VINITHI);
    set_parameter("core.cpu0.enable_trace_special_hlt_imm16",
                  params.cpu0_enable_trace_special_hlt_imm16);
    set_parameter("core.cpu0.l2cache-hit_latency",
                  params.cpu0_l2cache_hit_latency);
    set_parameter("core.cpu0.l2cache-maintenance_latency",
                  params.cpu0_l2cache_maintenance_latency);
    set_parameter("core.cpu0.l2cache-miss_latency",
                  params.cpu0_l2cache_miss_latency);
    set_parameter("core.cpu0.l2cache-read_access_latency",
                  params.cpu0_l2cache_read_access_latency);
    set_parameter("core.cpu0.l2cache-read_latency",
                  params.cpu0_l2cache_read_latency);
    set_parameter("core.cpu0.l2cache-size", params.cpu0_l2cache_size);
    set_parameter("core.cpu0.l2cache-snoop_data_transfer_latency",
                  params.cpu0_l2cache_snoop_data_transfer_latency);
    set_parameter("core.cpu0.l2cache-snoop_issue_latency",
                  params.cpu0_l2cache_snoop_issue_latency);
    set_parameter("core.cpu0.l2cache-write_access_latency",
                  params.cpu0_l2cache_write_access_latency);
    set_parameter("core.cpu0.l2cache-write_latency",
                  params.cpu0_l2cache_write_latency);
    set_parameter("core.cpu0.max_code_cache_mb",
                  params.cpu0_max_code_cache_mb);
    set_parameter("core.cpu0.min_sync_level", params.cpu0_min_sync_level);
    set_parameter("core.cpu0.semihosting-A32_HLT",
                  params.cpu0_semihosting_A32_HLT);
    set_parameter("core.cpu0.semihosting-A64_HLT",
                  params.cpu0_semihosting_A64_HLT);
    set_parameter("core.cpu0.semihosting-ARM_SVC",
                  params.cpu0_semihosting_ARM_SVC);
    set_parameter("core.cpu0.semihosting-T32_HLT",
                  params.cpu0_semihosting_T32_HLT);
    set_parameter("core.cpu0.semihosting-Thumb_SVC",
                  params.cpu0_semihosting_Thumb_SVC);
    set_parameter("core.cpu0.semihosting-cmd_line",
                  params.cpu0_semihosting_cmd_line);
    set_parameter("core.cpu0.semihosting-cwd", params.cpu0_semihosting_cwd);
    set_parameter("core.cpu0.semihosting-enable",
                  params.cpu0_semihosting_enable);
    set_parameter("core.cpu0.semihosting-heap_base",
                  params.cpu0_semihosting_heap_base);
    set_parameter("core.cpu0.semihosting-heap_limit",
                  params.cpu0_semihosting_heap_limit);
    set_parameter("core.cpu0.semihosting-stack_base",
                  params.cpu0_semihosting_stack_base);
    set_parameter("core.cpu0.semihosting-stack_limit",
                  params.cpu0_semihosting_stack_limit);
    set_parameter("core.cpu0.trace_special_hlt_imm16",
                  params.cpu0_trace_special_hlt_imm16);
    set_parameter("core.cpu0.vfp-enable_at_reset",
                  params.cpu0_vfp_enable_at_reset);

    add_attribute(gem5Cpu);
    add_attribute(clockPeriod);
    SC_METHOD(clockChangeHandler);
    dont_initialize();
    sensitive << clockChanged;
}

Port &
CortexA76x1::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "amba")
        return amba;
    else if (if_name == "redistributor_m")
        return redistributorM;
    else if (if_name == "redistributor_s")
        return redistributorS;
    else if (if_name == "cnthpirq")
        return cnthpirqWrapper;
    else if (if_name == "cnthvirq")
        return cnthvirqWrapper;
    else if (if_name == "cntpsirq")
        return cntpsirqWrapper;
    else if (if_name == "cntvirq")
        return cntvirqWrapper;
    else if (if_name == "commirq")
        return commirqWrapper;
    else if (if_name == "ctidbgirq")
        return ctidbgirqWrapper;
    else if (if_name == "pmuirq")
        return pmuirqWrapper;
    else if (if_name == "vcpumntirq")
        return vcpumntirqWrapper;
    else if (if_name == "cntpnsirq")
        return cntpnsirqWrapper;
    else
        return scx_evs_CortexA76x1::gem5_getPort(if_name, idx);
}

} // namespace FastModel

FastModel::CortexA76x1 *
FastModelCortexA76x1Params::create()
{
    return new FastModel::CortexA76x1(name.c_str(), *this);
}
