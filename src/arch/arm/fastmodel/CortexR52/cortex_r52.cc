/*
 * Copyright 2020 Google, Inc.
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

#include "arch/arm/fastmodel/CortexR52/cortex_r52.hh"

#include "arch/arm/fastmodel/iris/cpu.hh"
#include "base/logging.hh"
#include "dev/arm/base_gic.hh"
#include "systemc/tlm_bridge/gem5_to_tlm.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

void
CortexR52::setCluster(CortexR52Cluster *_cluster, int _num)
{
    cluster = _cluster;
    num = _num;

    set_evs_param("CFGEND", params().CFGEND);
    set_evs_param("CFGTE", params().CFGTE);
    set_evs_param("RVBARADDR", params().RVBARADDR);
    set_evs_param("ase-present", params().ase_present);
    set_evs_param("dcache-size", params().dcache_size);
    set_evs_param("flash.enable", params().flash_enable);
    set_evs_param("icache-size", params().icache_size);
    set_evs_param("llpp.base", params().llpp_base);
    set_evs_param("llpp.size", params().llpp_size);
    set_evs_param("max_code_cache_mb", params().max_code_cache_mb);
    set_evs_param("min_sync_level", params().min_sync_level);
    set_evs_param("semihosting-A32_HLT", params().semihosting_A32_HLT);
    // Use uint32_t, since the model doesn't like setting these as uint8_t.
    set_evs_param<uint32_t>("semihosting-ARM_SVC",
            params().semihosting_ARM_SVC);
    set_evs_param<uint32_t>("semihosting-T32_HLT",
            params().semihosting_T32_HLT);
    set_evs_param<uint32_t>("semihosting-Thumb_SVC",
            params().semihosting_Thumb_SVC);
    set_evs_param("semihosting-cmd_line", params().semihosting_cmd_line);
    set_evs_param("semihosting-cwd", params().semihosting_cwd);
    set_evs_param("semihosting-enable", params().semihosting_enable);
    set_evs_param("semihosting-heap_base", params().semihosting_heap_base);
    set_evs_param("semihosting-heap_limit", params().semihosting_heap_limit);
    set_evs_param("semihosting-stack_base", params().semihosting_stack_base);
    set_evs_param("semihosting-stack_limit", params().semihosting_stack_limit);
    set_evs_param("tcm.a.enable", params().tcm_a_enable);
    set_evs_param("tcm.a.size", params().tcm_a_size);
    set_evs_param("tcm.b.size", params().tcm_b_size);
    set_evs_param("tcm.c.size", params().tcm_c_size);
    set_evs_param("vfp-dp-present", params().vfp_dp_present);
    set_evs_param("vfp-enable_at_reset", params().vfp_enable_at_reset);
}

Port &
CortexR52::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "ppi") {
        // Since PPIs are indexed both by core and by number, modify the name
        // to hold the core number.
        return evs->gem5_getPort(csprintf("%s_%d", if_name, num), idx);
    } else if (if_name == "amba" || if_name == "llpp" || if_name == "flash" ||
               if_name == "core_reset" || if_name == "poweron_reset" ||
               if_name == "halt") {
        // Since these ports are scalar per core, use the core number as the
        // index. Also verify that that index is not being used.
        assert(idx == InvalidPortID);
        return evs->gem5_getPort(if_name, num);
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

CortexR52Cluster::CortexR52Cluster(const Params &p) :
    SimObject(p), cores(p.cores), evs(p.evs)
{
    for (int i = 0; i < p.cores.size(); i++)
        p.cores[i]->setCluster(this, i);

    Iris::BaseCpuEvs *e = dynamic_cast<Iris::BaseCpuEvs *>(evs);
    panic_if(!e, "EVS should be of type Iris::BaseCpuEvs");
    e->setCluster(this);

    set_evs_param("core.CLUSTER_ID", params().CLUSTER_ID);
    set_evs_param("core.DBGROMADDR", params().DBGROMADDR);
    set_evs_param("core.DBGROMADDRV", params().DBGROMADDRV);
    set_evs_param("core.PERIPHBASE", params().PERIPHBASE);
    set_evs_param("core.cluster_utid", params().cluster_utid);
    set_evs_param("core.cpi_div", params().cpi_div);
    set_evs_param("core.cpi_mul", params().cpi_mul);
    set_evs_param("core.dcache-prefetch_enabled",
            params().dcache_prefetch_enabled);
    set_evs_param("core.dcache-read_access_latency",
            params().dcache_read_access_latency);
    set_evs_param("core.dcache-state_modelled",
            params().dcache_state_modelled);
    set_evs_param("core.dcache-write_access_latency",
            params().dcache_write_access_latency);
    set_evs_param("core.flash_protection_enable_at_reset",
            params().flash_protection_enable_at_reset);
    set_evs_param("core.has_flash_protection", params().has_flash_protection);
    set_evs_param("core.icache-prefetch_enabled",
            params().icache_prefetch_enabled);
    set_evs_param("core.icache-read_access_latency",
            params().icache_read_access_latency);
    set_evs_param("core.icache-state_modelled",
            params().icache_state_modelled);
    set_evs_param("core.memory.ext_slave_base",
            params().memory_ext_slave_base);
    set_evs_param("core.memory.flash_base", params().memory_flash_base);
    set_evs_param("core.memory.flash_size", params().memory_flash_size);
    // Use uint32_t, since the model doesn't like setting these as uint8_t.
    set_evs_param<uint32_t>("core.num_protection_regions_s1",
            params().num_protection_regions_s1);
    set_evs_param<uint32_t>("core.num_protection_regions_s2",
            params().num_protection_regions_s2);
    set_evs_param("core.num_spi", params().num_spi);
    set_evs_param("core.ram_protection_enable_at_reset",
            params().ram_protection_enable_at_reset);
    set_evs_param("core.has_export_m_port", params().has_export_m_port);
}

Port &
CortexR52Cluster::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "spi") {
        return evs->gem5_getPort(if_name, idx);
    } else if (if_name == "ext_slave" || if_name == "top_reset") {
        assert(idx == InvalidPortID);
        return evs->gem5_getPort(if_name, idx);
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

} // namespace fastmodel
} // namespace gem5
