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

namespace FastModel
{

template <class Types>
void
ScxEvsCortexA76<Types>::clockChangeHandler()
{
    clockRateControl->set_mul_div(SimClock::Int::s, clockPeriod.value);
}

template <class Types>
ScxEvsCortexA76<Types>::ScxEvsCortexA76(
        const sc_core::sc_module_name &mod_name, const Params &p) :
    Base(mod_name), amba(Base::amba, p.name + ".amba", -1),
    clockChanged(Iris::ClockEventName.c_str()),
    clockPeriod(Iris::PeriodAttributeName.c_str()),
    gem5CpuCluster(Iris::Gem5CpuClusterAttributeName.c_str()),
    sendFunctional(Iris::SendFunctionalAttributeName.c_str()),
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

        Base::cnthpirq[i].bind(cnthpirq[i]->signal_in);
        Base::cnthvirq[i].bind(cnthvirq[i]->signal_in);
        Base::cntpsirq[i].bind(cntpsirq[i]->signal_in);
        Base::cntvirq[i].bind(cntvirq[i]->signal_in);
        Base::commirq[i].bind(commirq[i]->signal_in);
        Base::ctidbgirq[i].bind(ctidbgirq[i]->signal_in);
        Base::pmuirq[i].bind(pmuirq[i]->signal_in);
        Base::vcpumntirq[i].bind(vcpumntirq[i]->signal_in);
        Base::cntpnsirq[i].bind(cntpnsirq[i]->signal_in);
    }

    clockRateControl.bind(this->clock_rate_s);

    this->add_attribute(gem5CpuCluster);
    this->add_attribute(clockPeriod);
    SC_METHOD(clockChangeHandler);
    this->dont_initialize();
    this->sensitive << clockChanged;

    sendFunctional.value = [this](PacketPtr pkt) { sendFunc(pkt); };
    this->add_attribute(sendFunctional);
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

    for (int i = 0; i < CoreCount; i++) {
        set_on_change(*cnthpirq[i], cluster->params().cnthpirq, i);
        set_on_change(*cnthvirq[i], cluster->params().cnthvirq, i);
        set_on_change(*cntpsirq[i], cluster->params().cntpsirq, i);
        set_on_change(*cntvirq[i], cluster->params().cntvirq, i);
        set_on_change(*commirq[i], cluster->params().commirq, i);
        set_on_change(*ctidbgirq[i], cluster->params().ctidbgirq, i);
        set_on_change(*pmuirq[i], cluster->params().pmuirq, i);
        set_on_change(*vcpumntirq[i], cluster->params().vcpumntirq, i);
        set_on_change(*cntpnsirq[i], cluster->params().cntpnsirq, i);
    }
}

template <class Types>
Port &
ScxEvsCortexA76<Types>::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "redistributor")
        return *redist.at(idx);
    else if (if_name == "amba")
        return amba;
    else
        return Base::gem5_getPort(if_name, idx);
}

template class ScxEvsCortexA76<ScxEvsCortexA76x1Types>;
template class ScxEvsCortexA76<ScxEvsCortexA76x2Types>;
template class ScxEvsCortexA76<ScxEvsCortexA76x3Types>;
template class ScxEvsCortexA76<ScxEvsCortexA76x4Types>;

} // namespace FastModel

FastModel::ScxEvsCortexA76x1 *
FastModelScxEvsCortexA76x1Params::create()
{
    return new FastModel::ScxEvsCortexA76x1(name.c_str(), *this);
}

FastModel::ScxEvsCortexA76x2 *
FastModelScxEvsCortexA76x2Params::create()
{
    return new FastModel::ScxEvsCortexA76x2(name.c_str(), *this);
}

FastModel::ScxEvsCortexA76x3 *
FastModelScxEvsCortexA76x3Params::create()
{
    return new FastModel::ScxEvsCortexA76x3(name.c_str(), *this);
}

FastModel::ScxEvsCortexA76x4 *
FastModelScxEvsCortexA76x4Params::create()
{
    return new FastModel::ScxEvsCortexA76x4(name.c_str(), *this);
}
