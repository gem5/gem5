/*
 * Copyright (c) 2010-2011 ARM Limited
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
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

#include "cpu/o3/dyn_inst.hh"

#include <algorithm>

#include "debug/DynInst.hh"
#include "debug/IQ.hh"
#include "debug/O3PipeView.hh"

namespace gem5
{

namespace o3
{

DynInst::DynInst(const StaticInstPtr &static_inst,
        const StaticInstPtr &_macroop, TheISA::PCState _pc,
        TheISA::PCState pred_pc, InstSeqNum seq_num, CPU *_cpu)
    : seqNum(seq_num), staticInst(static_inst), cpu(_cpu), pc(_pc),
      regs(staticInst->numSrcRegs(), staticInst->numDestRegs()),
      predPC(pred_pc), macroop(_macroop)
{
    this->regs.init();

    status.reset();

    instFlags.reset();
    instFlags[RecordResult] = true;
    instFlags[Predicate] = true;
    instFlags[MemAccPredicate] = true;

#ifndef NDEBUG
    ++cpu->instcount;

    if (cpu->instcount > 1500) {
#ifdef DEBUG
        cpu->dumpInsts();
        dumpSNList();
#endif
        assert(cpu->instcount <= 1500);
    }

    DPRINTF(DynInst,
        "DynInst: [sn:%lli] Instruction created. Instcount for %s = %i\n",
        seqNum, cpu->name(), cpu->instcount);
#endif

#ifdef DEBUG
    cpu->snList.insert(seqNum);
#endif

}

DynInst::DynInst(const StaticInstPtr &_staticInst,
        const StaticInstPtr &_macroop)
    : DynInst(_staticInst, _macroop, {}, {}, 0, nullptr)
{}

DynInst::~DynInst()
{
#if TRACING_ON
    if (debug::O3PipeView) {
        Tick fetch = this->fetchTick;
        // fetchTick can be -1 if the instruction fetched outside the trace
        // window.
        if (fetch != -1) {
            Tick val;
            // Print info needed by the pipeline activity viewer.
            DPRINTFR(O3PipeView, "O3PipeView:fetch:%llu:0x%08llx:%d:%llu:%s\n",
                     fetch,
                     this->instAddr(),
                     this->microPC(),
                     this->seqNum,
                     this->staticInst->disassemble(this->instAddr()));

            val = (this->decodeTick == -1) ? 0 : fetch + this->decodeTick;
            DPRINTFR(O3PipeView, "O3PipeView:decode:%llu\n", val);
            val = (this->renameTick == -1) ? 0 : fetch + this->renameTick;
            DPRINTFR(O3PipeView, "O3PipeView:rename:%llu\n", val);
            val = (this->dispatchTick == -1) ? 0 : fetch + this->dispatchTick;
            DPRINTFR(O3PipeView, "O3PipeView:dispatch:%llu\n", val);
            val = (this->issueTick == -1) ? 0 : fetch + this->issueTick;
            DPRINTFR(O3PipeView, "O3PipeView:issue:%llu\n", val);
            val = (this->completeTick == -1) ? 0 : fetch + this->completeTick;
            DPRINTFR(O3PipeView, "O3PipeView:complete:%llu\n", val);
            val = (this->commitTick == -1) ? 0 : fetch + this->commitTick;

            Tick valS = (this->storeTick == -1) ? 0 : fetch + this->storeTick;
            DPRINTFR(O3PipeView, "O3PipeView:retire:%llu:store:%llu\n",
                    val, valS);
        }
    }
#endif

    delete [] memData;
    delete traceData;
    fault = NoFault;

#ifndef NDEBUG
    --cpu->instcount;

    DPRINTF(DynInst,
        "DynInst: [sn:%lli] Instruction destroyed. Instcount for %s = %i\n",
        seqNum, cpu->name(), cpu->instcount);
#endif
#ifdef DEBUG
    cpu->snList.erase(seqNum);
#endif
};


#ifdef DEBUG
void
DynInst::dumpSNList()
{
    std::set<InstSeqNum>::iterator sn_it = cpu->snList.begin();

    int count = 0;
    while (sn_it != cpu->snList.end()) {
        cprintf("%i: [sn:%lli] not destroyed\n", count, (*sn_it));
        count++;
        sn_it++;
    }
}
#endif

void
DynInst::dump()
{
    cprintf("T%d : %#08d `", threadNumber, pc.instAddr());
    std::cout << staticInst->disassemble(pc.instAddr());
    cprintf("'\n");
}

void
DynInst::dump(std::string &outstring)
{
    std::ostringstream s;
    s << "T" << threadNumber << " : 0x" << pc.instAddr() << " "
      << staticInst->disassemble(pc.instAddr());

    outstring = s.str();
}

void
DynInst::markSrcRegReady()
{
    DPRINTF(IQ, "[sn:%lli] has %d ready out of %d sources. RTI %d)\n",
            seqNum, readyRegs+1, numSrcRegs(), readyToIssue());
    if (++readyRegs == numSrcRegs()) {
        setCanIssue();
    }
}

void
DynInst::markSrcRegReady(RegIndex src_idx)
{
    regs.readySrcIdx(src_idx, true);
    markSrcRegReady();
}


void
DynInst::setSquashed()
{
    status.set(Squashed);

    if (!isPinnedRegsRenamed() || isPinnedRegsSquashDone())
        return;

    // This inst has been renamed already so it may go through rename
    // again (e.g. if the squash is due to memory access order violation).
    // Reset the write counters for all pinned destination register to ensure
    // that they are in a consistent state for a possible re-rename. This also
    // ensures that dest regs will be pinned to the same phys register if
    // re-rename happens.
    for (int idx = 0; idx < numDestRegs(); idx++) {
        PhysRegIdPtr phys_dest_reg = regs.renamedDestIdx(idx);
        if (phys_dest_reg->isPinned()) {
            phys_dest_reg->incrNumPinnedWrites();
            if (isPinnedRegsWritten())
                phys_dest_reg->incrNumPinnedWritesToComplete();
        }
    }
    setPinnedRegsSquashDone();
}

Fault
DynInst::execute()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    this->fault = this->staticInst->execute(this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}

Fault
DynInst::initiateAcc()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    this->fault = this->staticInst->initiateAcc(this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}

Fault
DynInst::completeAcc(PacketPtr pkt)
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    if (this->cpu->checker) {
        if (this->isStoreConditional()) {
            this->reqToVerify->setExtraData(pkt->req->getExtraData());
        }
    }

    this->fault = this->staticInst->completeAcc(pkt, this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}

void
DynInst::trap(const Fault &fault)
{
    this->cpu->trap(fault, this->threadNumber, this->staticInst);
}

Fault
DynInst::initiateMemRead(Addr addr, unsigned size, Request::Flags flags,
                               const std::vector<bool> &byte_enable)
{
    assert(byte_enable.size() == size);
    return cpu->pushRequest(
        dynamic_cast<DynInstPtr::PtrType>(this),
        /* ld */ true, nullptr, size, addr, flags, nullptr, nullptr,
        byte_enable);
}

Fault
DynInst::initiateHtmCmd(Request::Flags flags)
{
    return cpu->pushRequest(
            dynamic_cast<DynInstPtr::PtrType>(this),
            /* ld */ true, nullptr, 8, 0x0ul, flags, nullptr, nullptr);
}

Fault
DynInst::writeMem(uint8_t *data, unsigned size, Addr addr,
                        Request::Flags flags, uint64_t *res,
                        const std::vector<bool> &byte_enable)
{
    assert(byte_enable.size() == size);
    return cpu->pushRequest(
        dynamic_cast<DynInstPtr::PtrType>(this),
        /* st */ false, data, size, addr, flags, res, nullptr,
        byte_enable);
}

Fault
DynInst::initiateMemAMO(Addr addr, unsigned size, Request::Flags flags,
                              AtomicOpFunctorPtr amo_op)
{
    // atomic memory instructions do not have data to be written to memory yet
    // since the atomic operations will be executed directly in cache/memory.
    // Therefore, its `data` field is nullptr.
    // Atomic memory requests need to carry their `amo_op` fields to cache/
    // memory
    return cpu->pushRequest(
            dynamic_cast<DynInstPtr::PtrType>(this),
            /* atomic */ false, nullptr, size, addr, flags, nullptr,
            std::move(amo_op), std::vector<bool>(size, true));
}

} // namespace o3
} // namespace gem5
