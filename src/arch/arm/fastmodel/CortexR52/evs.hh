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

#ifndef __ARCH_ARM_FASTMODEL_CORTEXR52_EVS_HH__
#define __ARCH_ARM_FASTMODEL_CORTEXR52_EVS_HH__

#include <memory>

#include "arch/arm/fastmodel/amba_ports.hh"
#include "arch/arm/fastmodel/common/signal_receiver.hh"
#include "arch/arm/fastmodel/common/signal_sender.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"
#include "arch/arm/fastmodel/protocol/exported_clock_rate_control.hh"
#include "arch/arm/fastmodel/protocol/signal_interrupt.hh"
#include "dev/intpin.hh"
#include "mem/port_proxy.hh"
#include "params/FastModelScxEvsCortexR52x1.hh"
#include "params/FastModelScxEvsCortexR52x2.hh"
#include "params/FastModelScxEvsCortexR52x3.hh"
#include "params/FastModelScxEvsCortexR52x4.hh"
#include "scx_evs_CortexR52x1.h"
#include "scx_evs_CortexR52x2.h"
#include "scx_evs_CortexR52x3.h"
#include "scx_evs_CortexR52x4.h"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/tlm_port_wrapper.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

class CortexR52Cluster;

template <class Types>
class ScxEvsCortexR52 : public Types::Base, public Iris::BaseCpuEvs
{
  private:
    static const int CoreCount = Types::CoreCount;
    static const int PpiCount = 9;
    static const int SpiCount = 960;
    using Base = typename Types::Base;
    using Params = typename Types::Params;
    using Evs = ScxEvsCortexR52<Types>;

    SC_HAS_PROCESS(ScxEvsCortexR52);

    ClockRateControlInitiatorSocket clockRateControl;
    SignalInterruptInitiatorSocket signalInterrupt;

    // A structure to collect per-core connections, and also plumb up PPIs.
    struct CorePins
    {
        using CoreInt = IntSinkPin<CorePins>;
        template <typename T>
        using SignalInitiator = amba_pv::signal_master_port<T>;

        std::string name;
        Evs *evs;
        int cpu;

        CorePins(Evs *_evs, int _cpu);

        void
        raiseInterruptPin(int num)
        {
            evs->signalInterrupt->ppi(cpu, num, true);
        }

        void
        lowerInterruptPin(int num)
        {
            evs->signalInterrupt->ppi(cpu, num, false);
        }

        std::vector<std::unique_ptr<CoreInt>> ppis;

        AmbaInitiator llpp;
        AmbaInitiator flash;
        AmbaInitiator amba;

        SignalSender core_reset;
        SignalSender poweron_reset;
        SignalSender halt;

        SignalInitiator<uint64_t> cfgvectable;
    };

    std::vector<std::unique_ptr<CorePins>> corePins;

    using ClstrInt = IntSinkPin<ScxEvsCortexR52>;

    std::vector<std::unique_ptr<ClstrInt>> spis;

    AmbaTarget ext_slave;

    SignalSender top_reset;

    SignalSender dbg_reset;

    CortexR52Cluster *gem5CpuCluster;

    const Params &params;

  public:
    ScxEvsCortexR52(const Params &p) : ScxEvsCortexR52(p.name.c_str(), p) {}
    ScxEvsCortexR52(const sc_core::sc_module_name &mod_name, const Params &p);

    void
    raiseInterruptPin(int num)
    {
        this->signalInterrupt->spi(num, true);
    }

    void
    lowerInterruptPin(int num)
    {
        this->signalInterrupt->spi(num, false);
    }

    Port &gem5_getPort(const std::string &if_name, int idx) override;

    void
    end_of_elaboration() override
    {
        Base::end_of_elaboration();
        Base::start_of_simulation();
    }
    void start_of_simulation() override {}

    void sendFunc(PacketPtr pkt) override;

    void setClkPeriod(Tick clk_period) override;

    void setSysCounterFrq(uint64_t sys_counter_frq) override;

    void setCluster(SimObject *cluster) override;

    void setResetAddr(int core, Addr addr, bool secure) override;
};

struct ScxEvsCortexR52x1Types
{
    using Base = scx_evs_CortexR52x1;
    using Params = FastModelScxEvsCortexR52x1Params;
    static const int CoreCount = 1;
};
using ScxEvsCortexR52x1 = ScxEvsCortexR52<ScxEvsCortexR52x1Types>;
extern template class ScxEvsCortexR52<ScxEvsCortexR52x1Types>;

struct ScxEvsCortexR52x2Types
{
    using Base = scx_evs_CortexR52x2;
    using Params = FastModelScxEvsCortexR52x2Params;
    static const int CoreCount = 2;
};
using ScxEvsCortexR52x2 = ScxEvsCortexR52<ScxEvsCortexR52x2Types>;
extern template class ScxEvsCortexR52<ScxEvsCortexR52x2Types>;

struct ScxEvsCortexR52x3Types
{
    using Base = scx_evs_CortexR52x3;
    using Params = FastModelScxEvsCortexR52x3Params;
    static const int CoreCount = 3;
};
using ScxEvsCortexR52x3 = ScxEvsCortexR52<ScxEvsCortexR52x3Types>;
extern template class ScxEvsCortexR52<ScxEvsCortexR52x3Types>;

struct ScxEvsCortexR52x4Types
{
    using Base = scx_evs_CortexR52x4;
    using Params = FastModelScxEvsCortexR52x4Params;
    static const int CoreCount = 4;
};
using ScxEvsCortexR52x4 = ScxEvsCortexR52<ScxEvsCortexR52x4Types>;
extern template class ScxEvsCortexR52<ScxEvsCortexR52x4Types>;

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_CORTEXR52_EVS_HH__
