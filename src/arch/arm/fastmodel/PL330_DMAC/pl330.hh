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

#ifndef __ARCH_ARM_FASTMODEL_PL330_PL330_HH__
#define __ARCH_ARM_FASTMODEL_PL330_PL330_HH__

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <amba_pv.h>

#pragma GCC diagnostic pop

#include <array>
#include <vector>

#include "arch/arm/fastmodel/amba_ports.hh"
#include "arch/arm/fastmodel/common/signal_receiver.hh"
#include "arch/arm/fastmodel/common/signal_sender.hh"
#include "arch/arm/fastmodel/protocol/exported_clock_rate_control.hh"
#include "dev/intpin.hh"
#include "params/FastModelPL330.hh"
#include "scx_evs_PL330.h"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/sc_port_wrapper.hh"

namespace gem5
{

namespace fastmodel
{

class PL330 : public scx_evs_PL330
{
  private:
    Tick clockPeriod;

    AmbaInitiator dma;
    AmbaTarget pioS, pioNs;

    ClockRateControlInitiatorSocket clockRateControl;

    using IntSource = IntSourcePin<PL330>;

    std::array<std::vector<std::unique_ptr<IntSource>>, 32> irqPort;
    std::vector<std::unique_ptr<SignalReceiver>> irqReceiver;

    std::vector<std::unique_ptr<IntSource>> irqAbortPort;
    SignalReceiver irqAbortReceiver;

    void allocateIrq(int idx, int count);

    SignalSender resetIn;

  public:
    PL330(const FastModelPL330Params &params, sc_core::sc_module_name _name);
    PL330(const FastModelPL330Params &params) :
        PL330(params, params.name.c_str())
    {}

    gem5::Port &gem5_getPort(const std::string &if_name, int idx=-1) override;

    void
    end_of_elaboration() override
    {
        scx_evs_PL330::end_of_elaboration();
        scx_evs_PL330::start_of_simulation();
    }
    void start_of_simulation() override;
};

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_PL330_PL330_HH__
