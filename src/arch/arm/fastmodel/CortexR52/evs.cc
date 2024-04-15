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

#include "arch/arm/fastmodel/CortexR52/evs.hh"

#include "arch/arm/fastmodel/CortexR52/cortex_r52.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"
#include "base/logging.hh"
#include "sim/core.hh"
#include "systemc/tlm_bridge/gem5_to_tlm.hh"

namespace gem5
{

namespace fastmodel
{

template <class Types>
void
ScxEvsCortexR52<Types>::setClkPeriod(Tick clk_period)
{
    clockRateControl->set_mul_div(sim_clock::as_int::s, clk_period);
}

template <class Types>
void
ScxEvsCortexR52<Types>::setSysCounterFrq(uint64_t sys_counter_frq)
{
    panic("Not implemented for R52.");
}

template <class Types>
void
ScxEvsCortexR52<Types>::setCluster(SimObject *cluster)
{
    gem5CpuCluster = dynamic_cast<CortexR52Cluster *>(cluster);
    panic_if(!gem5CpuCluster, "Cluster should be of type CortexR52Cluster");
}

template <class Types>
void
ScxEvsCortexR52<Types>::setResetAddr(int core, Addr addr, bool secure)
{
    this->corePins[core]->cfgvectable.set_state(0, addr);
}

template <class Types>
ScxEvsCortexR52<Types>::CorePins::CorePins(Evs *_evs, int _cpu)
    : name(csprintf("%s.cpu%s", _evs->name(), _cpu)),
      evs(_evs),
      cpu(_cpu),
      llpp(evs->llpp[cpu], name + ".llpp", -1),
      flash(evs->flash[cpu], name + ".flash", -1),
      amba(evs->amba[cpu], name + ".amba", -1),
      core_reset(name + ".core_reset", 0),
      poweron_reset(name + ".poweron_reset", 0),
      halt(name + ".halt", 0),
      standbywfi(name + ".standbywfi"),
      cfgvectable((name + "cfgvectable").c_str())
{
    for (int i = 0; i < Evs::PpiCount; i++) {
        ppis.emplace_back(
            new CoreInt(csprintf("%s.ppi[%d]", name, i), i, this));
    }
    core_reset.signal_out.bind(evs->core_reset[cpu]);
    poweron_reset.signal_out.bind(evs->poweron_reset[cpu]);
    halt.signal_out.bind(evs->halt[cpu]);
    evs->standbywfi[cpu].bind(standbywfi.signal_in);
    cfgvectable.bind(evs->cfgvectable[cpu]);
}

template <class Types>
ScxEvsCortexR52<Types>::ScxEvsCortexR52(
    const sc_core::sc_module_name &mod_name, const Params &p)
    : Base(mod_name),
      ext_slave(Base::ext_slave, p.name + ".ext_slave", -1),
      top_reset(p.name + ".top_reset", 0),
      dbg_reset(p.name + ".dbg_reset", 0),
      model_reset(p.name + ".model_reset"),
      params(p)
{
    model_reset.onChange([this](const bool &new_val) {
        // Set reset for all cores.
        for (auto &core_pin : corePins)
            core_pin->poweron_reset.signal_out.set_state(0, new_val);
        // Set reset for L2 system.
        top_reset.signal_out.set_state(0, new_val);
        // Set reset for debug APB.
        dbg_reset.signal_out.set_state(0, new_val);
    });

    for (int i = 0; i < CoreCount; i++)
        corePins.emplace_back(new CorePins(this, i));

    for (int i = 0; i < SpiCount; i++) {
        spis.emplace_back(
            new ClstrInt(csprintf("%s.spi[%d]", name(), i), i, this));
    }

    top_reset.signal_out.bind(Base::top_reset);
    dbg_reset.signal_out.bind(Base::dbg_reset);

    clockRateControl.bind(this->clock_rate_s);
    signalInterrupt.bind(this->signal_interrupt);
}

template <class Types>
Port &
ScxEvsCortexR52<Types>::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "llpp") {
        return this->corePins.at(idx)->llpp;
    } else if (if_name == "flash") {
        return this->corePins.at(idx)->flash;
    } else if (if_name == "amba") {
        return this->corePins.at(idx)->amba;
    } else if (if_name == "core_reset") {
        return this->corePins.at(idx)->core_reset;
    } else if (if_name == "poweron_reset") {
        return this->corePins.at(idx)->poweron_reset;
    } else if (if_name == "halt") {
        return this->corePins.at(idx)->halt;
    } else if (if_name == "ext_slave") {
        return this->ext_slave;
    } else if (if_name == "top_reset") {
        return this->top_reset;
    } else if (if_name == "dbg_reset") {
        return this->dbg_reset;
    } else if (if_name == "model_reset") {
        return this->model_reset;
    } else if (if_name == "spi") {
        return *this->spis.at(idx);
    } else if (if_name.substr(0, 3) == "ppi") {
        int cpu;
        try {
            cpu = std::stoi(if_name.substr(4));
        } catch (const std::invalid_argument &a) {
            panic("Couldn't find CPU number in %s.", if_name);
        }
        return *this->corePins.at(cpu)->ppis.at(idx);
    } else if (if_name.substr(0, 10) == "standbywfi") {
        int cpu;
        try {
            cpu = std::stoi(if_name.substr(11));
        } catch (const std::invalid_argument &a) {
            panic("Couldn't find CPU number in %s.", if_name);
        }
        return this->corePins.at(cpu)->standbywfi.getSignalOut(idx);
    } else {
        return Base::gem5_getPort(if_name, idx);
    }
}

template class ScxEvsCortexR52<ScxEvsCortexR52x1Types>;
template class ScxEvsCortexR52<ScxEvsCortexR52x2Types>;
template class ScxEvsCortexR52<ScxEvsCortexR52x3Types>;
template class ScxEvsCortexR52<ScxEvsCortexR52x4Types>;

} // namespace fastmodel
} // namespace gem5
