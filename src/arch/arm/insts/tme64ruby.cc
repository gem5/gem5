/*
 * Copyright (c) 2020-2021 ARM Limited
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

#include "arch/arm/faults.hh"
#include "arch/arm/htm.hh"
#include "arch/arm/insts/tme64.hh"
#include "arch/generic/memhelpers.hh"
#include "debug/ArmTme.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace gem5
{

using namespace ArmISA;

namespace ArmISAInst {

Fault
Tstart64::initiateAcc(ExecContext *xc,
                      trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    const uint64_t htm_depth = xc->getHtmTransactionalDepth();

    DPRINTF(ArmTme, "tme depth is %d\n", htm_depth);

    // Maximum TME nesting depth exceeded
    if (htm_depth > ArmISA::HTMCheckpoint::MAX_HTM_DEPTH) {
        const uint64_t htm_uid = xc->getHtmTransactionUid();
        fault = std::make_shared<GenericHtmFailureFault>(
            htm_uid, HtmFailureFaultCause::NEST);
    }

    if (fault == NoFault) {
        Request::Flags memAccessFlags =
            Request::STRICT_ORDER|Request::PHYSICAL|Request::HTM_START;

        // Nested transaction start/stops never leave the core.
        // These Requests are marked as NO_ACCESS to indicate that the request
        // should not be sent to the cache controller.
        if (htm_depth > 1) {
            memAccessFlags = memAccessFlags | Request::NO_ACCESS;
        }

        fault = xc->initiateMemMgmtCmd(memAccessFlags);
    }

    return fault;
}

Fault
Tstart64::completeAcc(PacketPtr pkt, ExecContext *xc,
                      trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    uint64_t Mem;
    uint64_t Dest64 = 0;
    ThreadContext *tc = xc->tcBase();
    const uint64_t htm_depth = xc->getHtmTransactionalDepth();

    getMemLE(pkt, Mem, traceData);

    // sanity check
    if (Mem != 0) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    // sanity check
    if (!xc->inHtmTransactionalState()) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    if (fault == NoFault) {
        Dest64 = 0x0; // tstart returns 0 on success

        // checkpointing occurs in the outer transaction only
        if (htm_depth == 1) {
            BaseHTMCheckpointPtr& cpt = xc->tcBase()->getHtmCheckpointPtr();

            HTMCheckpoint *armcpt =
                dynamic_cast<HTMCheckpoint*>(cpt.get());
            assert(armcpt != nullptr);

            armcpt->save(tc);
            armcpt->destinationRegister(dest);

            tc->getIsaPtr()->globalClearExclusive();
        }

        xc->setRegOperand(this, 0, Dest64 & mask(intWidth));


        uint64_t final_val = Dest64;
        if (traceData) { traceData->setData(intRegClass, final_val); }
    }

    return fault;
}

Fault
Ttest64::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    uint64_t Dest64 = 0;
    const uint64_t htm_depth = xc->getHtmTransactionalDepth();

    // sanity check
    if (htm_depth > ArmISA::HTMCheckpoint::MAX_HTM_DEPTH) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    Dest64 = htm_depth;


    // sanity check
    if (Dest64 == 0)
        assert(!xc->inHtmTransactionalState());
    else
        assert(xc->inHtmTransactionalState());

    if (fault == NoFault) {
        uint64_t final_val = Dest64;
        xc->setRegOperand(this, 0, Dest64 & mask(intWidth));
        if (traceData) { traceData->setData(intRegClass, final_val); }
    }

    return fault;
}

Fault
Tcancel64::initiateAcc(ExecContext *xc, trace::InstRecord *traceData) const
{
    Fault fault = NoFault;

    // sanity check
    if (!xc->inHtmTransactionalState()) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    Request::Flags memAccessFlags =
        Request::STRICT_ORDER|Request::PHYSICAL|Request::HTM_CANCEL;

    fault = xc->initiateMemMgmtCmd(memAccessFlags);

    return fault;
}

Fault
Tcancel64::completeAcc(PacketPtr pkt, ExecContext *xc,
                       trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    uint64_t Mem;

    getMemLE(pkt, Mem, traceData);

    // sanity check
    if (Mem != 0) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    if (fault == NoFault) {
        auto tme_checkpoint = static_cast<HTMCheckpoint*>(
            xc->tcBase()->getHtmCheckpointPtr().get());
        tme_checkpoint->cancelReason(imm);

        fault = std::make_shared<GenericHtmFailureFault>(
            xc->getHtmTransactionUid(),
            HtmFailureFaultCause::EXPLICIT);
    }

    return fault;
}

Fault
MicroTcommit64::initiateAcc(ExecContext *xc,
                            trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    const uint64_t htm_depth = xc->getHtmTransactionalDepth();

    // sanity check
    if (!xc->inHtmTransactionalState()) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    DPRINTF(ArmTme, "tme depth is %d\n", htm_depth);

    Request::Flags memAccessFlags =
        Request::STRICT_ORDER|Request::PHYSICAL|Request::HTM_COMMIT;

    // Nested transaction start/stops never leave the core.
    // These Requests are marked as NO_ACCESS to indicate that the request
    // should not be sent to the cache controller.
    if (htm_depth > 1) {
        memAccessFlags = memAccessFlags | Request::NO_ACCESS;
    }

    fault = xc->initiateMemMgmtCmd(memAccessFlags);

    return fault;
}

Fault
MicroTcommit64::completeAcc(PacketPtr pkt, ExecContext *xc,
                            trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    uint64_t Mem;
    ThreadContext *tc = xc->tcBase();
    const uint64_t htm_depth = xc->getHtmTransactionalDepth();

    getMemLE(pkt, Mem, traceData);

    // sanity check
    if (Mem != 0) {
        fault = std::make_shared<IllegalInstSetStateFault>();
    }

    if (fault == NoFault) {
        if (htm_depth == 1) {
            auto tme_checkpoint = static_cast<HTMCheckpoint*>(
                xc->tcBase()->getHtmCheckpointPtr().get());

            assert(tme_checkpoint);
            assert(tme_checkpoint->valid());

            tme_checkpoint->reset();
            tc->getIsaPtr()->globalClearExclusive();
        }
    }

    return fault;
}

} // namespace ArmISAInst
} // namespace gem5
