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

#include "mem/ruby/protocol/chi/tlm/snp_handler.hh"

namespace gem5
{

namespace tlm::chi
{

SnoopResponse::SnoopResponse(ARM::CHI::Phase &ph) : Transaction(nullptr, ph)
{}

void
SnoopResponse::setSnoopHandler(SnoopHandler *_handler)
{
    handler = _handler;
}

void
SnoopResponse::setDiscardAfterSnoop(bool discard)
{
    discardAfterSnoop = discard;
}

void
SnoopResponse::runCallbacks()
{
    auto it = actions.begin();
    while (it != actions.end()) {
        const bool is_passing = (*it)->run(this);
        if (!is_passing) {
            passed = false;
        }
        bool wait = (*it)->wait();

        unsigned timeout = (*it)->waitCycles();

        if (discardAfterSnoop) {
            it = actions.erase(it);
        } else {
            ++it;
        }

        if (wait) {
            if (timeout) {
                scheduleEvaluation(timeout);
            }
            break;
        }
    }
}

SnoopHandler::SnoopHandler(const Params &p)
    : SimObject(p), generator(nullptr), stats(this)
{}

void
SnoopHandler::setGenerator(TlmGenerator *gen)
{
    generator = gen;
}

bool
SnoopHandler::snoop(ARM::CHI::Payload *payload, ARM::CHI::Phase *phase)
{
    stats.record(phase);
    return handleSnoop(payload, phase);
}

SnoopHandler::SnoopStats::SnoopStats(statistics::Group *parent)
    : statistics::Group(parent, "snoops"),
      ADD_STAT(snoopLCrdReturn, statistics::units::Count::get(),
               "Number of SnpLCrdReturn snoop requests"),
      ADD_STAT(snoopShared, statistics::units::Count::get(),
               "Number of SnpShared snoop requests"),
      ADD_STAT(snoopClean, statistics::units::Count::get(),
               "Number of SnpClean snoop requests"),
      ADD_STAT(snoopOnce, statistics::units::Count::get(),
               "Number of SnpOnce snoop requests"),
      ADD_STAT(snoopNotSharedDirty, statistics::units::Count::get(),
               "Number of SnpNotSharedDirty snoop requests"),
      ADD_STAT(snoopUniqueStash, statistics::units::Count::get(),
               "Number of SnpUniqueStash snoop requests"),
      ADD_STAT(snoopMakeInvalidStash, statistics::units::Count::get(),
               "Number of SnpMakeInvalidStash snoop requests"),
      ADD_STAT(snoopUnique, statistics::units::Count::get(),
               "Number of SnpUnique snoop requests"),
      ADD_STAT(snoopCleanShared, statistics::units::Count::get(),
               "Number of SnpCleanShared snoop requests"),
      ADD_STAT(snoopCleanInvalid, statistics::units::Count::get(),
               "Number of SnpCleanInvalid snoop requests"),
      ADD_STAT(snoopMakeInvalid, statistics::units::Count::get(),
               "Number of SnpMakeInvalid snoop requests"),
      ADD_STAT(snoopStashUnique, statistics::units::Count::get(),
               "Number of SnpStashUnique snoop requests"),
      ADD_STAT(snoopStashShared, statistics::units::Count::get(),
               "Number of SnpStashShared snoop requests"),
      ADD_STAT(snoopDvmOp, statistics::units::Count::get(),
               "Number of SnpDvmOp snoop requests"),
      ADD_STAT(snoopQuery, statistics::units::Count::get(),
               "Number of SnpQuery snoop requests"),
      ADD_STAT(snoopSharedFwd, statistics::units::Count::get(),
               "Number of SnpSharedFwd snoop requests"),
      ADD_STAT(snoopCleanFwd, statistics::units::Count::get(),
               "Number of SnpCleanFwd snoop requests"),
      ADD_STAT(snoopOnceFwd, statistics::units::Count::get(),
               "Number of SnpOnceFwd snoop requests"),
      ADD_STAT(snoopNotSharedDirtyFwd, statistics::units::Count::get(),
               "Number of SnpNotSharedDirtyFwd snoop requests"),
      ADD_STAT(snoopPreferUnique, statistics::units::Count::get(),
               "Number of SnpPreferUnique snoop requests"),
      ADD_STAT(snoopPreferUniqueFwd, statistics::units::Count::get(),
               "Number of SnpPreferUniqueFwd snoop requests"),
      ADD_STAT(snoopUniqueFwd, statistics::units::Count::get(),
               "Number of SnpUniqueFwd snoop requests")
{}

void
SnoopHandler::SnoopStats::record(ARM::CHI::Phase *phase)
{
    switch (phase->snp_opcode) {
        case ARM::CHI::SNP_OPCODE_SNP_LCRD_RETURN:
            ++snoopLCrdReturn;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_SHARED:
            ++snoopShared;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_CLEAN:
            ++snoopClean;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_ONCE:
            ++snoopOnce;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_NOT_SHARED_DIRTY:
            ++snoopNotSharedDirty;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_UNIQUE_STASH:
            ++snoopUniqueStash;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_MAKE_INVALID_STASH:
            ++snoopMakeInvalidStash;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_UNIQUE:
            ++snoopUnique;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_CLEAN_SHARED:
            ++snoopCleanShared;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_CLEAN_INVALID:
            ++snoopCleanInvalid;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_MAKE_INVALID:
            ++snoopMakeInvalid;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_STASH_UNIQUE:
            ++snoopStashUnique;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_STASH_SHARED:
            ++snoopStashShared;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_DVM_OP:
            ++snoopDvmOp;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_QUERY:
            ++snoopQuery;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_SHARED_FWD:
            ++snoopSharedFwd;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_CLEAN_FWD:
            ++snoopCleanFwd;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_ONCE_FWD:
            ++snoopOnceFwd;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_NOT_SHARED_DIRTY_FWD:
            ++snoopNotSharedDirtyFwd;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_PREFER_UNIQUE:
            ++snoopPreferUnique;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_PREFER_UNIQUE_FWD:
            ++snoopPreferUniqueFwd;
            break;
        case ARM::CHI::SNP_OPCODE_SNP_UNIQUE_FWD:
            ++snoopUniqueFwd;
            break;
        case ARM::CHI::SNP_OPCODE_MASK:
            break;
    }
}

} // namespace tlm::chi

} // namespace gem5
