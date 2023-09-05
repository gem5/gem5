/*
 * Copyright (c) 2022-2023 The University of Edinburgh
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

#include "cpu/pred/ras.hh"

#include <iomanip>

#include "debug/RAS.hh"

namespace gem5
{

namespace branch_prediction
{


void
ReturnAddrStack::AddrStack::init(unsigned _numEntries)
{
    numEntries = _numEntries;
    addrStack.resize(numEntries);
    for (unsigned i = 0; i < numEntries; ++i) {
        addrStack[i] = nullptr;
    }
    reset();
}

void
ReturnAddrStack::AddrStack::reset()
{
    usedEntries = 0;
    tos = 0;
    cdTos = 0;
    firstWrap = 0;
}

const PCStateBase *
ReturnAddrStack::AddrStack::top()
{
    if (usedEntries > 0) {
        return addrStack[tos].get();
    }
    return nullptr;
}


void
ReturnAddrStack::AddrStack::push(const PCStateBase &return_addr)
{

    incrTos();
    // If TOS is now at the same position as CD-TOS it means
    // the entry was overwritten.
    // Increase the CD-TOS to indicate the last valid entry.
    if (tos == cdTos) {
        incrCdTos();
    }

    set(addrStack[tos], return_addr);

    if (usedEntries != numEntries) {
        ++usedEntries;
    }
}

bool
ReturnAddrStack::AddrStack::pop()
{
    if (usedEntries > 0) {
        --usedEntries;
    }

    decrTos();
    // The entry is invalid in case the TOS and CD-TOS
    // point to the same entry and the first wrap bit is unset.
    // Check the bit before decrementing the CD-TOS
    bool corrupted = (tos == cdTos) && !firstWrap;

    if (tos == cdTos){
        decrCdTos();
    }
    return corrupted;
}


void
ReturnAddrStack::AddrStack::restore(unsigned _tos, unsigned _cdTos,
                                    const PCStateBase *restored)
{
    if (usedEntries != numEntries) {
        ++usedEntries;
    }

    tos = _tos;
    cdTos = _cdTos;
    set(addrStack[tos], restored);
}

std::string
ReturnAddrStack::AddrStack::print(int n)
{
    std::stringstream ss;
    ss << "";
    for (int i = 0; i<n; i++) {
        int idx = int(tos)-i;
        if (idx < 0 || addrStack[idx] == nullptr) {
            break;
        }
        ss << std::dec << idx << ":0x" << std::setfill('0') << std::setw(8)
           << std::hex << addrStack[idx]->instAddr() << ";";
    }
    return ss.str();
}


// Return address stack class.
//

ReturnAddrStack::ReturnAddrStack(const Params &p)
    : SimObject(p),
      numEntries(p.numEntries),
      numThreads(p.numThreads),
      stats(this)
{
    DPRINTF(RAS, "Create RAS stacks.\n");

    for (unsigned i = 0; i < numThreads; ++i) {
        addrStacks.emplace_back(*this);
        addrStacks[i].init(numEntries);
    }
}

void
ReturnAddrStack::reset()
{
    DPRINTF(RAS, "RAS Reset.\n");
    for (auto& r : addrStacks)
        r.reset();
}

void
ReturnAddrStack::push(ThreadID tid, const PCStateBase &pc,
                        void * &ras_history)
{
    assert(ras_history==nullptr);
    stats.pushes++;

    RASHistory *history = new RASHistory;
    ras_history = static_cast<void*>(history);
    history->pushed = true;
    history->poped = false;

    addrStacks[tid].push(pc);

    DPRINTF(RAS, "%s: RAS[%i] <= %#x. Entries used: %i, tid:%i\n", __func__,
                    addrStacks[tid].tos, pc.instAddr(),
                    addrStacks[tid].usedEntries,tid);
    DPRINTF(RAS, "[%s]\n", addrStacks[tid].print(10));
}


const PCStateBase*
ReturnAddrStack::pop(ThreadID tid, void * &ras_history)
{
    assert(ras_history==nullptr);
    stats.pops++;
    stats.used++;

    RASHistory *history = new RASHistory;
    ras_history = static_cast<void*>(history);
    history->pushed = false;
    history->tos = addrStacks[tid].tos;
    history->cdTos = addrStacks[tid].cdTos;

    if (addrStacks[tid].empty()) {
        history->poped = false;
        history->ras_entry = nullptr;
        DPRINTF(RAS, "RAS empty. Don't pop entry.\n");
        return nullptr;
    }

    history->poped = true;
    set(history->ras_entry, addrStacks[tid].top());
    history->corrupted = addrStacks[tid].pop();

    DPRINTF(RAS, "%s: RAS[%i] => %#x. corrupted:%i, "
                "Entries used: %i, tid:%i\n", __func__, history->tos,
                history->ras_entry->instAddr(), history->corrupted,
                addrStacks[tid].usedEntries, tid);
    DPRINTF(RAS, "[%s]\n", addrStacks[tid].print(10));

    // TODO: return depending on config.
    if (history->corrupted) {
        stats.corrupt++;

        // If BTB fallback is enabled the RAS will return an invalid
        // address and the BPU will use the BTB as provider for the
        // target address.
        if (useBtbFallback) {
            return nullptr;
        }
    }
    return history->ras_entry.get();
}

void
ReturnAddrStack::squash(ThreadID tid, void * &ras_history)
{
    if (ras_history == nullptr) {
        // If ras_history is null no stack operation was performed for
        // this branch. Nothing to be done.
        return;
    }
    stats.squashes++;

    RASHistory *history = static_cast<RASHistory*>(ras_history);

    if (history->pushed) {
        stats.pops++;
        addrStacks[tid].pop();

        DPRINTF(RAS, "RAS::%s Incorrect push. Pop RAS[%i]. "
            "Entries used: %i, tid:%i\n", __func__,
            addrStacks[tid].tos,
            addrStacks[tid].usedEntries, tid);


    } else if (history->poped) {
        stats.pushes++;
        addrStacks[tid].restore(history->tos, history->cdTos,
                                history->ras_entry.get());
        DPRINTF(RAS, "RAS::%s Incorrect pop. Restore to: RAS[%i]:%#x. "
            "Entries used: %i, tid:%i\n", __func__,
            history->tos, history->ras_entry->instAddr(),
            addrStacks[tid].usedEntries, tid);
    }
    DPRINTF(RAS, "[%s]\n", addrStacks[tid].print(10));
    // DPRINTF(RAS, "RAS: Delete hist:%#x\n", history);
    delete history; ras_history = nullptr;
}

void
ReturnAddrStack::commit(ThreadID tid, bool misp,
                        const BranchType brType, void * &ras_history)
{
    // Skip branches that are not call or returns
    if (!(brType == BranchType::Return ||
          brType == BranchType::CallDirect ||
          brType == BranchType::CallIndirect)) {
        // If its not a call or return there should be no ras history.
        assert(ras_history == nullptr);
        return;
    }

    DPRINTF(RAS, "RAS::%s Commit Branch inst: %s, tid:%i\n",
                __func__, toString(brType),tid);


    if (ras_history == nullptr) {
        /**
         * The only case where we could have no history at this point is
         * for a conditional call that is not taken.
         *
         * Conditional calls
         *
         * Conditional calls have different scenarios:
         * 1. the call was predicted as non taken but was actually taken
         * 2. the call was predicted taken but was actually not taken.
         * 3. the call was taken but the target was incorrect.
         * 4. the call was correct.
         *
         * In case of mispredictions they will be handled during squashing
         * of the BPU. It will push and pop the RAS accordingly.
         **/
        return;
    }

    /* Handle all other commited returns and calls */
    RASHistory *history = static_cast<RASHistory*>(ras_history);

    if (history->poped) {
        stats.used++;
        if (misp) {
            stats.wrong++;
        } else {
            stats.correct++;
        }

        DPRINTF(RAS, "RAS::%s Commit Return PC %#x, correct:%i, tid:%i\n",
                __func__, !misp,
                history->ras_entry->instAddr(), tid);
    }
    delete history; ras_history = nullptr;
}



ReturnAddrStack::ReturnAddrStackStats::ReturnAddrStackStats(
    statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(pushes, statistics::units::Count::get(),
               "Number of times a PC was pushed onto the RAS"),
      ADD_STAT(pops, statistics::units::Count::get(),
               "Number of times a PC was poped from the RAS"),
      ADD_STAT(squashes, statistics::units::Count::get(),
               "Number of times the stack operation was squashed due to "
               "wrong speculation."),
      ADD_STAT(corrupt, statistics::units::Count::get(),
               "Number of times the RAS is corrupted during a pop."),
      ADD_STAT(used, statistics::units::Count::get(),
               "Number of times the RAS is the provider"),
      ADD_STAT(correct, statistics::units::Count::get(),
               "Number of times the RAS is the provider and the "
               "prediction is correct"),
      ADD_STAT(wrong, statistics::units::Count::get(),
               "Number of times the loop predictor is the provider and the "
               "prediction is wrong")
{
}

} // namespace branch_prediction
} // namespace gem5
