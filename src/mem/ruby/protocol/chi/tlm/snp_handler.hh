/*
 * Copyright (c) 2026 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __MEM_RUBY_PROTOCOL_CHI_TLM_SNP_HANDLER_HH__
#define __MEM_RUBY_PROTOCOL_CHI_TLM_SNP_HANDLER_HH__

#include <ARM/TLM/arm_chi_payload.h>
#include <ARM/TLM/arm_chi_phase.h>

#include "base/statistics.hh"
#include "mem/ruby/protocol/chi/tlm/generator.hh"
#include "params/SnoopHandler.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace tlm::chi
{

class SnoopHandler;
class TlmGenerator;

class SnoopResponse : public TlmGenerator::Transaction
{
  public:
    SnoopResponse(ARM::CHI::Phase &ph);

    void setSnoopHandler(SnoopHandler *handler);

    void setDiscardAfterSnoop(bool discard);
    void runCallbacks() override;

  private:
    bool discardAfterSnoop = true;
    SnoopHandler *handler = nullptr;
};

/**
 * A SnoopHandlder is in charge of responding to any
 * snoop the TlmGenerator might receive.
 *
 * This is not mandatory for a TlmGenerator: there are cases
 * where we don't expect a snoop and in such cases having
 * a snoop handler in un-necessary. Snoops can be handled
 * in many different ways.
 * Someone could potentially write a very simple SnoopHandler
 * which simply assumes there is no line present in the
 * modelled RN. Some others could for example model a rudimentary
 * cache which keeps track of filled lines and return data
 * accordingly.
 */
class SnoopHandler : public SimObject
{
  public:
    PARAMS(SnoopHandler);
    SnoopHandler(const Params &p);

    void setGenerator(TlmGenerator *gen);
    bool snoop(ARM::CHI::Payload *payload, ARM::CHI::Phase *phase);

  protected:
    virtual bool handleSnoop(ARM::CHI::Payload *payload,
                             ARM::CHI::Phase *phase) = 0;

    TlmGenerator *generator = nullptr;

    struct SnoopStats : public statistics::Group
    {
        SnoopStats(statistics::Group *parent);

        void record(ARM::CHI::Phase *phase);

        statistics::Scalar snoopLCrdReturn;
        statistics::Scalar snoopShared;
        statistics::Scalar snoopClean;
        statistics::Scalar snoopOnce;
        statistics::Scalar snoopNotSharedDirty;
        statistics::Scalar snoopUniqueStash;
        statistics::Scalar snoopMakeInvalidStash;
        statistics::Scalar snoopUnique;
        statistics::Scalar snoopCleanShared;
        statistics::Scalar snoopCleanInvalid;
        statistics::Scalar snoopMakeInvalid;
        statistics::Scalar snoopStashUnique;
        statistics::Scalar snoopStashShared;
        statistics::Scalar snoopDvmOp;
        statistics::Scalar snoopQuery;
        statistics::Scalar snoopSharedFwd;
        statistics::Scalar snoopCleanFwd;
        statistics::Scalar snoopOnceFwd;
        statistics::Scalar snoopNotSharedDirtyFwd;
        statistics::Scalar snoopPreferUnique;
        statistics::Scalar snoopPreferUniqueFwd;
        statistics::Scalar snoopUniqueFwd;
    };

    SnoopStats stats;
};

} // namespace tlm::chi

} // namespace gem5

#endif // __MEM_RUBY_PROTOCOL_CHI_TLM_SNP_HANDLER_HH__
