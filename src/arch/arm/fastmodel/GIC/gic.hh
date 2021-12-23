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

#ifndef __ARCH_ARM_FASTMODEL_GIC_GIC_HH__
#define __ARCH_ARM_FASTMODEL_GIC_GIC_HH__

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <amba_pv.h>
#pragma GCC diagnostic pop

#include <memory>

#include "arch/arm/fastmodel/amba_ports.hh"
#include "arch/arm/fastmodel/common/signal_receiver.hh"
#include "dev/arm/base_gic.hh"
#include "dev/intpin.hh"
#include "params/FastModelGIC.hh"
#include "params/SCFastModelGIC.hh"
#include "scx_evs_GIC.h"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/sc_port_wrapper.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

// The fast model exports a class called scx_evs_GIC which represents
// the subsystem described in LISA+. This class specializes it to export gem5
// ports and interface with its peer gem5 GIC. The gem5 GIC inherits from the
// gem5 BaseGic class and implements its API, while this class actually does
// the work.
class SCGIC : public scx_evs_GIC
{
  private:
    // The unconnected CPU ports/sockets still need to be connected for TLM to
    // be happy, so this module finds all unbound sockets, creates pair
    // sockets for them to connect to, binds everything together, and
    // implements the target interface with a dummy stub that will complain
    // and crash gem5 if it ever gets called.
    class Terminator : public sc_core::sc_module,
                            public svp_gicv3_comms::gicv3_comms_fw_if
    {
      protected:
        typedef sc_core::sc_vector<
            svp_gicv3_comms::gicv3_comms_initiator_socket<>> Initiators;
        typedef sc_core::sc_vector<
            svp_gicv3_comms::gicv3_comms_target_socket<>> Targets;

        Targets targets;

        static int countUnbound(const Initiators &inits);

      public:
        Terminator(sc_core::sc_module_name _name, Initiators &inits);

        // Stub out the terminated interface.
        void sendTowardsCPU(uint8_t len, const uint8_t *data) override;
    };

    std::unique_ptr<Terminator> terminator;
    const SCFastModelGICParams &_params;

  public:
    SCGIC(const SCFastModelGICParams &p) : SCGIC(p, p.name.c_str()) {}
    SCGIC(const SCFastModelGICParams &params, sc_core::sc_module_name _name);

    SignalInterruptInitiatorSocket signalInterrupt;

    std::vector<std::unique_ptr<SignalReceiver>> wakeRequests;

    void before_end_of_elaboration() override;

    void
    end_of_elaboration() override
    {
        scx_evs_GIC::end_of_elaboration();
        scx_evs_GIC::start_of_simulation();
    }
    void start_of_simulation() override {}
    PARAMS(SCFastModelGIC);
};

// This class pairs with the one above to implement the receiving end of gem5's
// GIC API. It acts as an interface which passes work to the fast model GIC,
// and lets the fast model GIC interact with the rest of the system.
class GIC : public BaseGic
{
  private:
    typedef sc_gem5::TlmInitiatorBaseWrapper<
        64, svp_gicv3_comms::gicv3_comms_fw_if,
        svp_gicv3_comms::gicv3_comms_bw_if, 1,
        sc_core::SC_ONE_OR_MORE_BOUND> TlmGicInitiator;

    AmbaInitiator ambaM;
    AmbaTarget ambaS;
    std::vector<std::unique_ptr<TlmGicInitiator>> redistributors;
    std::vector<std::unique_ptr<IntSourcePin<GIC>>> wakeRequestPorts;

    SCGIC *scGIC;

  public:
    GIC(const FastModelGICParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void sendInt(uint32_t num) override;
    void clearInt(uint32_t num) override;

    void sendPPInt(uint32_t num, uint32_t cpu) override;
    void clearPPInt(uint32_t num, uint32_t cpu) override;

    bool supportsVersion(GicVersion version) override;

    AddrRangeList getAddrRanges() const override { return AddrRangeList(); }
    Tick read(PacketPtr pkt) override { return 0; }
    Tick write(PacketPtr pkt) override { return 0; }
};

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_GIC_GIC_HH__
