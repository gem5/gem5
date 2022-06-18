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

#include "arch/arm/fastmodel/CortexA76/evs.hh"

#include "arch/arm/fastmodel/CortexA76/cortex_a76.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"
#include "base/logging.hh"
#include "dev/arm/base_gic.hh"
#include "sim/core.hh"
#include "systemc/tlm_bridge/gem5_to_tlm.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

template <class Types>
void
ScxEvsCortexA76<Types>::setClkPeriod(Tick clk_period)
{
    clockRateControl->set_mul_div(sim_clock::as_int::s, clk_period);
}

template <class Types>
void
ScxEvsCortexA76<Types>::setSysCounterFrq(uint64_t sys_counter_frq)
{
    periphClockRateControl->set_mul_div(sys_counter_frq, 1);
}

template <class Types>
void
ScxEvsCortexA76<Types>::setCluster(SimObject *cluster)
{
    gem5CpuCluster = dynamic_cast<CortexA76Cluster *>(cluster);
    panic_if(!gem5CpuCluster, "Cluster should be of type CortexA76Cluster");
}

template <class Types>
void
ScxEvsCortexA76<Types>::setResetAddr(int core, Addr addr, bool secure)
{
    this->rvbaraddr[core]->set_state(0, addr);
}

template <class Types>
void
ScxEvsCortexA76<Types>::requestReset()
{
    // Reset all cores.
    for (auto &poweron_reset : this->poweron_reset) {
        poweron_reset->signal_out.set_state(0, true);
        poweron_reset->signal_out.set_state(0, false);
    }
    // Reset DSU.
    this->top_reset.signal_out.set_state(0, true);
    this->top_reset.signal_out.set_state(0, false);
    // Reset debug APB.
    this->dbg_reset.signal_out.set_state(0, true);
    this->dbg_reset.signal_out.set_state(0, false);
}

template <class Types>
ScxEvsCortexA76<Types>::ScxEvsCortexA76(
        const sc_core::sc_module_name &mod_name, const Params &p) :
    Base(mod_name),
    amba(Base::amba, p.name + ".amba", -1),
    top_reset(p.name + ".top_reset", 0),
    dbg_reset(p.name + ".dbg_reset", 0),
    model_reset(p.name + ".model_reset", -1, this),
    params(p)
{
    for (int i = 0; i < CoreCount; i++) {
        redist.emplace_back(new TlmGicTarget(this->redistributor[i],
                    csprintf("%s.redistributor[%d]", name(), i), i));
        cnthpirq.emplace_back(new SignalReceiver(csprintf("cnthpirq[%d]", i)));
        cnthvirq.emplace_back(new SignalReceiver(csprintf("cnthvirq[%d]", i)));
        cntpsirq.emplace_back(new SignalReceiver(csprintf("cntpsirq[%d]", i)));
        cntvirq.emplace_back(new SignalReceiver(csprintf("cntvirq[%d]", i)));
        commirq.emplace_back(new SignalReceiver(csprintf("commirq[%d]", i)));
        ctidbgirq.emplace_back(
                new SignalReceiver(csprintf("ctidbgirq[%d]", i)));
        pmuirq.emplace_back(new SignalReceiver(csprintf("pmuirq[%d]", i)));
        vcpumntirq.emplace_back(
                new SignalReceiver(csprintf("vcpumntirq[%d]", i)));
        cntpnsirq.emplace_back(
                new SignalReceiver(csprintf("cntpnsirq[%d]", i)));
        rvbaraddr.emplace_back(new SignalInitiator<uint64_t>(
                    csprintf("rvbaraddr[%d]", i).c_str()));
        core_reset.emplace_back(
                new SignalSender(csprintf("core_reset[%d]", i), 0));
        poweron_reset.emplace_back(
                new SignalSender(csprintf("poweron_reset[%d]", i), 0));

        Base::cnthpirq[i].bind(cnthpirq[i]->signal_in);
        Base::cnthvirq[i].bind(cnthvirq[i]->signal_in);
        Base::cntpsirq[i].bind(cntpsirq[i]->signal_in);
        Base::cntvirq[i].bind(cntvirq[i]->signal_in);
        Base::commirq[i].bind(commirq[i]->signal_in);
        Base::ctidbgirq[i].bind(ctidbgirq[i]->signal_in);
        Base::pmuirq[i].bind(pmuirq[i]->signal_in);
        Base::vcpumntirq[i].bind(vcpumntirq[i]->signal_in);
        Base::cntpnsirq[i].bind(cntpnsirq[i]->signal_in);
        rvbaraddr[i]->bind(Base::rvbaraddr[i]);
        core_reset[i]->signal_out.bind(Base::core_reset[i]);
        poweron_reset[i]->signal_out.bind(Base::poweron_reset[i]);
    }

    top_reset.signal_out.bind(Base::top_reset);
    dbg_reset.signal_out.bind(Base::dbg_reset);

    clockRateControl.bind(this->clock_rate_s);
    periphClockRateControl.bind(this->periph_clock_rate_s);
}

template <class Types>
void
ScxEvsCortexA76<Types>::sendFunc(PacketPtr pkt)
{
    auto *trans = sc_gem5::packet2payload(pkt);
    panic_if(Base::amba->transport_dbg(*trans) != trans->get_data_length(),
            "Didn't send entire functional packet!");
    trans->release();
}

template <class Types>
void
ScxEvsCortexA76<Types>::before_end_of_elaboration()
{
    Base::before_end_of_elaboration();

    auto set_on_change = [this](
            SignalReceiver &recv, ArmInterruptPinGen *gen, int num)
    {
        auto *pin = gen->get(gem5CpuCluster->getCore(num)->getContext(0));
        auto handler = [pin](bool status)
        {
            status ? pin->raise() : pin->clear();
        };
        recv.onChange(handler);
    };

    for (int i = 0; i < CoreCount; i++) {
        set_on_change(*cnthpirq[i], gem5CpuCluster->params().cnthpirq, i);
        set_on_change(*cnthvirq[i], gem5CpuCluster->params().cnthvirq, i);
        set_on_change(*cntpsirq[i], gem5CpuCluster->params().cntpsirq, i);
        set_on_change(*cntvirq[i], gem5CpuCluster->params().cntvirq, i);
        set_on_change(*commirq[i], gem5CpuCluster->params().commirq, i);
        set_on_change(*ctidbgirq[i], gem5CpuCluster->params().ctidbgirq, i);
        set_on_change(*pmuirq[i], gem5CpuCluster->params().pmuirq, i);
        set_on_change(*vcpumntirq[i], gem5CpuCluster->params().vcpumntirq, i);
        set_on_change(*cntpnsirq[i], gem5CpuCluster->params().cntpnsirq, i);
    }
}

template <class Types>
Port &
ScxEvsCortexA76<Types>::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "redistributor")
        return *redist.at(idx);
    else if (if_name == "core_reset")
        return *core_reset.at(idx);
    else if (if_name == "poweron_reset")
        return *poweron_reset.at(idx);
    else if (if_name == "amba")
        return amba;
    else if (if_name == "top_reset")
        return top_reset;
    else if (if_name == "dbg_reset")
        return dbg_reset;
    else if (if_name == "model_reset")
        return model_reset;
    else
        return Base::gem5_getPort(if_name, idx);
}

template class ScxEvsCortexA76<ScxEvsCortexA76x1Types>;
template class ScxEvsCortexA76<ScxEvsCortexA76x2Types>;
template class ScxEvsCortexA76<ScxEvsCortexA76x3Types>;
template class ScxEvsCortexA76<ScxEvsCortexA76x4Types>;

} // namespace fastmodel
} // namespace gem5
