/*
 * Copyright (c) 2012-2013, 2015 ARM Limited
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

#include "arch/arm/stage2_mmu.hh"

#include "arch/arm/faults.hh"
#include "arch/arm/system.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlb.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"

using namespace ArmISA;

Stage2MMU::Stage2MMU(const Params *p)
    : SimObject(p), _stage1Tlb(p->tlb), _stage2Tlb(p->stage2_tlb),
      port(_stage1Tlb->getTableWalker(), p->sys),
      masterId(p->sys->getMasterId(_stage1Tlb->getTableWalker()))
{
    // we use the stage-one table walker as the parent of the port,
    // and to get our master id, this is done to keep things
    // symmetrical with other ISAs in terms of naming and stats
    stage1Tlb()->setMMU(this, masterId);
    stage2Tlb()->setMMU(this, masterId);
}

Fault
Stage2MMU::readDataUntimed(ThreadContext *tc, Addr oVAddr, Addr descAddr,
    uint8_t *data, int numBytes, Request::Flags flags, bool isFunctional)
{
    Fault fault;

    // translate to physical address using the second stage MMU
    auto req = std::make_shared<Request>();
    req->setVirt(descAddr, numBytes, flags | Request::PT_WALK, masterId, 0);
    if (isFunctional) {
        fault = stage2Tlb()->translateFunctional(req, tc, BaseTLB::Read);
    } else {
        fault = stage2Tlb()->translateAtomic(req, tc, BaseTLB::Read);
    }

    // Now do the access.
    if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
        Packet pkt = Packet(req, MemCmd::ReadReq);
        pkt.dataStatic(data);
        if (isFunctional) {
            port.sendFunctional(&pkt);
        } else {
            port.sendAtomic(&pkt);
        }
        assert(!pkt.isError());
    }

    // If there was a fault annotate it with the flag saying the foult occured
    // while doing a translation for a stage 1 page table walk.
    if (fault != NoFault) {
        ArmFault *armFault = reinterpret_cast<ArmFault *>(fault.get());
        armFault->annotate(ArmFault::S1PTW, true);
        armFault->annotate(ArmFault::OVA, oVAddr);
    }
    return fault;
}

void
Stage2MMU::readDataTimed(ThreadContext *tc, Addr descAddr,
                         Stage2Translation *translation, int numBytes,
                         Request::Flags flags)
{
    // translate to physical address using the second stage MMU
    translation->setVirt(
            descAddr, numBytes, flags | Request::PT_WALK, masterId);
    translation->translateTiming(tc);
}

Stage2MMU::Stage2Translation::Stage2Translation(Stage2MMU &_parent,
        uint8_t *_data, Event *_event, Addr _oVAddr)
    : data(_data), numBytes(0), event(_event), parent(_parent), oVAddr(_oVAddr),
    fault(NoFault)
{
    req = std::make_shared<Request>();
}

void
Stage2MMU::Stage2Translation::finish(const Fault &_fault,
                                     const RequestPtr &req,
                                     ThreadContext *tc, BaseTLB::Mode mode)
{
    fault = _fault;

    // If there was a fault annotate it with the flag saying the foult occured
    // while doing a translation for a stage 1 page table walk.
    if (fault != NoFault) {
        ArmFault *armFault = reinterpret_cast<ArmFault *>(fault.get());
        armFault->annotate(ArmFault::S1PTW, true);
        armFault->annotate(ArmFault::OVA, oVAddr);
    }

    if (_fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
        parent.getDMAPort().dmaAction(
            MemCmd::ReadReq, req->getPaddr(), numBytes, event, data,
            tc->getCpuPtr()->clockPeriod(), req->getFlags());
    } else {
        // We can't do the DMA access as there's been a problem, so tell the
        // event we're done
        event->process();
    }
}

ArmISA::Stage2MMU *
ArmStage2MMUParams::create()
{
    return new ArmISA::Stage2MMU(this);
}
