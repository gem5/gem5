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

#ifndef __ARCH_ARM_FASTMODEL_CORTEXA76X1_CORETEX_A76X1_HH__
#define __ARCH_ARM_FASTMODEL_CORTEXA76X1_CORETEX_A76X1_HH__

#include <type_traits>

#include "arch/arm/fastmodel/amba_ports.hh"
#include "arch/arm/fastmodel/protocol/exported_clock_rate_control.hh"
#include "params/FastModelCortexA76x1.hh"
#include "scx_evs_CortexA76x1.h"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/sc_port_wrapper.hh"

class BaseCPU;

// This macro is to get the type IF of a sc_export<IF> variable x. It relies on
// the fact that the "operator->()" function returns the "IF*" type and
// std::decay to remove cv-qualifiers and reference.
#define IFACE_TYPE(x) std::decay<decltype(*(x).operator->())>::type

namespace FastModel
{

// The fast model exports a class called scx_evs_CortexA76x1 which represents
// the subsystem described in LISA+. This class specializes it to export gem5
// ports and interface with its peer gem5 CPU. The gem5 CPU inherits from the
// gem5 BaseCPU class and implements its API, while this class actually does
// the work.
class CortexA76x1 : public scx_evs_CortexA76x1
{
  private:
    SC_HAS_PROCESS(CortexA76x1);

    AmbaInitiator amba;
    AmbaInitiator redistributorM;
    AmbaTarget redistributorS;

    ClockRateControlInitiatorSocket clockRateControl;

    sc_gem5::ScPortWrapper<IFACE_TYPE(cnthpirq)> cnthpirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(cnthvirq)> cnthvirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(cntpsirq)> cntpsirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(cntvirq)> cntvirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(commirq)> commirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(ctidbgirq)> ctidbgirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(pmuirq)> pmuirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(vcpumntirq)> vcpumntirqWrapper;
    sc_gem5::ScPortWrapper<IFACE_TYPE(cntpnsirq)> cntpnsirqWrapper;

    sc_core::sc_event clockChanged;
    sc_core::sc_attribute<Tick> clockPeriod;
    sc_core::sc_attribute<::BaseCPU *> gem5Cpu;

    void clockChangeHandler();

  public:
    CortexA76x1(const sc_core::sc_module_name &mod_name,
            const FastModelCortexA76x1Params &params);

    Port &gem5_getPort(const std::string &if_name, int idx=-1) override;

    void
    end_of_elaboration() override
    {
        scx_evs_CortexA76x1::end_of_elaboration();
        scx_evs_CortexA76x1::start_of_simulation();
    }
    void start_of_simulation() override {}
};

} // namespace FastModel

#endif // __ARCH_ARM_FASTMODEL_CORTEXA76X1_CORETEX_A76X1_HH__
