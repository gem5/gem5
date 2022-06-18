/*
 * Copyright (c) 2010-2011, 2021 ARM Limited
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

#include "base/intmath.hh"
#include "debug/DynInst.hh"
#include "debug/IQ.hh"
#include "debug/O3PipeView.hh"

namespace gem5
{

namespace o3
{

DynInst::DynInst(const Arrays &arrays, const StaticInstPtr &static_inst,
        const StaticInstPtr &_macroop, InstSeqNum seq_num, CPU *_cpu)
    : seqNum(seq_num), staticInst(static_inst), cpu(_cpu),
      _numSrcs(arrays.numSrcs), _numDests(arrays.numDests),
      _flatDestIdx(arrays.flatDestIdx), _destIdx(arrays.destIdx),
      _prevDestIdx(arrays.prevDestIdx), _srcIdx(arrays.srcIdx),
      _readySrcIdx(arrays.readySrcIdx), macroop(_macroop)
{
    std::fill(_readySrcIdx, _readySrcIdx + (numSrcs() + 7) / 8, 0);

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

DynInst::DynInst(const Arrays &arrays, const StaticInstPtr &static_inst,
        const StaticInstPtr &_macroop, const PCStateBase &_pc,
        const PCStateBase &pred_pc, InstSeqNum seq_num, CPU *_cpu)
    : DynInst(arrays, static_inst, _macroop, seq_num, _cpu)
{
    set(pc, _pc);
    set(predPC, pred_pc);
}

DynInst::DynInst(const Arrays &arrays, const StaticInstPtr &_staticInst,
        const StaticInstPtr &_macroop)
    : DynInst(arrays, _staticInst, _macroop, 0, nullptr)
{}

/*
 * This custom "new" operator uses the default "new" operator to allocate space
 * for a DynInst, but also pads out the number of bytes to make room for some
 * extra structures the DynInst needs. We save time and improve performance by
 * only going to the heap once to get space for all these structures.
 *
 * When a DynInst is allocated with new, the compiler will call this "new"
 * operator with "count" set to the number of bytes it needs to store the
 * DynInst. We ultimately call into the default new operator to get those
 * bytes, but before we do, we pad out "count" so that there will be extra
 * space for some structures the DynInst needs. We take into account both the
 * absolute size of these structures, and also what alignment they need.
 *
 * Once we've gotten a buffer large enough to hold the DynInst itself and these
 * extra structures, we construct the extra bits using placement new. This
 * constructs the structures in place in the space we created for them.
 *
 * Next, we return the buffer as the result of our operator. The compiler takes
 * that buffer and constructs the DynInst in the beginning of it using the
 * DynInst constructor.
 *
 * To avoid having to calculate where these extra structures are twice, once
 * when making room for them and initializing them, and then once again in the
 * DynInst constructor, we also pass in a structure called "arrays" which holds
 * pointers to them. The fields of "arrays" are initialized in this operator,
 * and are then consumed in the DynInst constructor.
 */
void *
DynInst::operator new(size_t count, Arrays &arrays)
{
    // Convenience variables for brevity.
    const auto num_dests = arrays.numDests;
    const auto num_srcs = arrays.numSrcs;

    // Figure out where everything will go.
    uintptr_t inst = 0;
    size_t inst_size = count;

    uintptr_t flat_dest_idx = roundUp(inst + inst_size, alignof(RegId));
    size_t flat_dest_idx_size = sizeof(*arrays.flatDestIdx) * num_dests;

    uintptr_t dest_idx =
        roundUp(flat_dest_idx + flat_dest_idx_size, alignof(PhysRegIdPtr));
    size_t dest_idx_size = sizeof(*arrays.destIdx) * num_dests;

    uintptr_t prev_dest_idx =
        roundUp(dest_idx + dest_idx_size, alignof(PhysRegIdPtr));
    size_t prev_dest_idx_size = sizeof(*arrays.prevDestIdx) * num_dests;

    uintptr_t src_idx =
        roundUp(prev_dest_idx + prev_dest_idx_size, alignof(PhysRegIdPtr));
    size_t src_idx_size = sizeof(*arrays.srcIdx) * num_srcs;

    uintptr_t ready_src_idx =
        roundUp(src_idx + src_idx_size, alignof(uint8_t));
    size_t ready_src_idx_size =
        sizeof(*arrays.readySrcIdx) * ((num_srcs + 7) / 8);

    // Figure out how much space we need in total.
    size_t total_size = ready_src_idx + ready_src_idx_size;

    // Actually allocate it.
    uint8_t *buf = (uint8_t *)::operator new(total_size);

    // Fill in "arrays" with pointers to all the arrays.
    arrays.flatDestIdx = (RegId *)(buf + flat_dest_idx);
    arrays.destIdx = (PhysRegIdPtr *)(buf + dest_idx);
    arrays.prevDestIdx = (PhysRegIdPtr *)(buf + prev_dest_idx);
    arrays.srcIdx = (PhysRegIdPtr *)(buf + src_idx);
    arrays.readySrcIdx = (uint8_t *)(buf + ready_src_idx);

    // Initialize all the extra components.
    new (arrays.flatDestIdx) RegId[num_dests];
    new (arrays.destIdx) PhysRegIdPtr[num_dests];
    new (arrays.prevDestIdx) PhysRegIdPtr[num_dests];
    new (arrays.srcIdx) PhysRegIdPtr[num_srcs];
    new (arrays.readySrcIdx) uint8_t[num_srcs];

    return buf;
}

DynInst::~DynInst()
{
    /*
     * The buffer this DynInst occupies also holds some of the structures it
     * points to. We need to call their destructors manually to make sure that
     * they're cleaned up appropriately, but we don't need to free their memory
     * explicitly since that's part of the DynInst's buffer and is already
     * going to be freed as part of deleting the DynInst.
     */
    for (int i = 0; i < _numDests; i++) {
        _flatDestIdx[i].~RegId();
        _destIdx[i].~PhysRegIdPtr();
        _prevDestIdx[i].~PhysRegIdPtr();
    }

    for (int i = 0; i < _numSrcs; i++)
        _srcIdx[i].~PhysRegIdPtr();

    for (int i = 0; i < ((_numSrcs + 7) / 8); i++)
        _readySrcIdx[i].~uint8_t();

#if TRACING_ON
    if (debug::O3PipeView) {
        Tick fetch = fetchTick;
        // fetchTick can be -1 if the instruction fetched outside the trace
        // window.
        if (fetch != -1) {
            Tick val;
            // Print info needed by the pipeline activity viewer.
            DPRINTFR(O3PipeView, "O3PipeView:fetch:%llu:0x%08llx:%d:%llu:%s\n",
                     fetch,
                     pcState().instAddr(),
                     pcState().microPC(),
                     seqNum,
                     staticInst->disassemble(pcState().instAddr()));

            val = (decodeTick == -1) ? 0 : fetch + decodeTick;
            DPRINTFR(O3PipeView, "O3PipeView:decode:%llu\n", val);
            val = (renameTick == -1) ? 0 : fetch + renameTick;
            DPRINTFR(O3PipeView, "O3PipeView:rename:%llu\n", val);
            val = (dispatchTick == -1) ? 0 : fetch + dispatchTick;
            DPRINTFR(O3PipeView, "O3PipeView:dispatch:%llu\n", val);
            val = (issueTick == -1) ? 0 : fetch + issueTick;
            DPRINTFR(O3PipeView, "O3PipeView:issue:%llu\n", val);
            val = (completeTick == -1) ? 0 : fetch + completeTick;
            DPRINTFR(O3PipeView, "O3PipeView:complete:%llu\n", val);
            val = (commitTick == -1) ? 0 : fetch + commitTick;

            Tick valS = (storeTick == -1) ? 0 : fetch + storeTick;
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
    cprintf("T%d : %#08d `", threadNumber, pc->instAddr());
    std::cout << staticInst->disassemble(pc->instAddr());
    cprintf("'\n");
}

void
DynInst::dump(std::string &outstring)
{
    std::ostringstream s;
    s << "T" << threadNumber << " : 0x" << pc->instAddr() << " "
      << staticInst->disassemble(pc->instAddr());

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
    readySrcIdx(src_idx, true);
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
        PhysRegIdPtr phys_dest_reg = renamedDestIdx(idx);
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
    bool no_squash_from_TC = thread->noSquashFromTC;
    thread->noSquashFromTC = true;

    fault = staticInst->execute(this, traceData);

    thread->noSquashFromTC = no_squash_from_TC;

    return fault;
}

Fault
DynInst::initiateAcc()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = thread->noSquashFromTC;
    thread->noSquashFromTC = true;

    fault = staticInst->initiateAcc(this, traceData);

    thread->noSquashFromTC = no_squash_from_TC;

    return fault;
}

Fault
DynInst::completeAcc(PacketPtr pkt)
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = thread->noSquashFromTC;
    thread->noSquashFromTC = true;

    if (cpu->checker) {
        if (isStoreConditional()) {
            reqToVerify->setExtraData(pkt->req->getExtraData());
        }
    }

    fault = staticInst->completeAcc(pkt, this, traceData);

    thread->noSquashFromTC = no_squash_from_TC;

    return fault;
}

void
DynInst::trap(const Fault &fault)
{
    cpu->trap(fault, threadNumber, staticInst);
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
DynInst::initiateMemMgmtCmd(Request::Flags flags)
{
    const unsigned int size = 8;
    return cpu->pushRequest(
            dynamic_cast<DynInstPtr::PtrType>(this),
            /* ld */ true, nullptr, size, 0x0ul, flags, nullptr, nullptr,
            std::vector<bool>(size, true));
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
