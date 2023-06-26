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

#include "cpu/o3/ftq.hh"

#include "arch/generic/pcstate.hh"
#include "base/logging.hh"
#include "cpu/o3/cpu.hh"
#include "debug/FTQ.hh"
#include "params/BaseO3CPU.hh"

namespace gem5
{

namespace o3
{


/** Fetch Target Methods -------------------------------- */
FetchTarget::FetchTarget(const PCStateBase &_start_pc, InstSeqNum _seqNum)
    : ftSeqNum(_seqNum),
      is_branch(false), taken(false),
      bpu_history(nullptr)
{
    set(startPC , _start_pc);
}


void
FetchTarget::finalize(const PCStateBase &exit_pc, InstSeqNum sn,
                      bool _is_branch, bool pred_taken,
                      const PCStateBase &pred_pc)
{
    set(endPC, exit_pc);
    set(predPC, pred_pc);
    taken = pred_taken;
    is_branch = _is_branch;
}


std::string
FetchTarget::print()
{
    std::stringstream ss;
    ss << "FT[" << ftSeqNum << "]: [0x" << std::hex
        << startPC->instAddr() << "->0x" << endPC->instAddr()
        << "|B:" << is_branch
        << "]";
    return ss.str();
}


/** Fetch Target Qeue Methods ----------------------------- */

FTQ::FTQ(CPU *_cpu, const BaseO3CPUParams &params)
    : cpu(_cpu),
      numThreads(params.numThreads),
      numEntries(params.numFTQEntries),
      stats(_cpu, this)
{
    resetState();
}


void
FTQ::resetState()
{
    for (ThreadID tid = 0; tid  < numThreads; tid++) {
        ftq[tid].clear();
        ftqStatus[tid] = Valid;
    }
}


std::string
FTQ::name() const
{
    return cpu->name() + ".ftq";
}


void
FTQ::regProbePoints()
{
    ppFTQInsert = new ProbePointArg<FetchTargetPtr>(cpu->getProbeManager(),
                                                        "FTQInsert");
    ppFTQRemove = new ProbePointArg<FetchTargetPtr>(cpu->getProbeManager(),
                                                        "FTQRemove");
}

unsigned
FTQ::numFreeEntries(ThreadID tid)
{
    return numEntries - ftq[tid].size();
}

unsigned
FTQ::size(ThreadID tid)
{
    return ftq[tid].size();
}

bool
FTQ::isFull(ThreadID tid)
{
    return ftq[tid].size() >= numEntries;
}

bool
FTQ::isEmpty() const
{
    for (ThreadID tid = numThreads; tid < MaxThreads; tid++) {
        if (!ftq[tid].empty()) return false;
    }
    return true;
}

bool
FTQ::isEmpty(ThreadID tid) const
{
    return ftq[tid].empty();
}


void
FTQ::invalidate(ThreadID tid)
{
    /** Only a full ftq can be invalid*/
    if (!ftq[tid].empty())
        ftqStatus[tid] = Invalid;
}

bool
FTQ::isValid(ThreadID tid)
{
    return ftqStatus[tid] != Invalid;
}


void
FTQ::lock(ThreadID tid)
{
    ftqStatus[tid] = Locked;
}

bool
FTQ::isLocked(ThreadID tid)
{
    return ftqStatus[tid] == Locked;
}


void
FTQ::forAllForward(ThreadID tid, std::function<void(FetchTargetPtr&)> f)
{
    for (auto it = ftq[tid].begin(); it != ftq[tid].end(); it++) {
        f(*it);
    }
}

void
FTQ::forAllBackward(ThreadID tid, std::function<void(FetchTargetPtr&)> f)
{
    for (auto it = ftq[tid].rbegin(); it != ftq[tid].rend(); it++) {
        f(*it);
    }
}



void
FTQ::insert(ThreadID tid, FetchTargetPtr fetchTarget)
{
    ftq[tid].push_back(fetchTarget);
    ppFTQInsert->notify(fetchTarget);
    stats.inserts++;
    stats.occupancy.sample(ftq[tid].size());

    DPRINTF(FTQ, "Insert %s in FTQ[T:%i]. size FTQ:%i\n",
                    fetchTarget->print(), tid, ftq[tid].size());
}


void
FTQ::squash(ThreadID tid)
{
    for (auto ft : ftq[tid]) {
        assert(ft->bpu_history == nullptr);
        ppFTQRemove->notify(ft);
    }
    ftq[tid].clear();
    ftqStatus[tid] = Valid;
    stats.squashes++;
}

void
FTQ::squashSanityCheck(ThreadID tid)
{
    for (auto ft : ftq[tid]) {
        assert(ft->bpu_history == nullptr);
    }
}


bool
FTQ::isHeadReady(ThreadID tid)
{
    return (ftqStatus[tid] != Invalid) && (ftq[tid].size() > 0);
}


FetchTargetPtr
FTQ::readHead(ThreadID tid)
{
    if (ftqStatus[tid] == Invalid) return nullptr;
    if (ftq[tid].empty()) return nullptr;

    return ftq[tid].front();
}


bool
FTQ::updateHead(ThreadID tid)
{
    if (ftq[tid].front()->bpu_history != nullptr) {
        DPRINTF(FTQ, "Pop FT:[fn%llu] failed. Still contains BP history.\n",
                    ftq[tid].front()->ftNum());
        ftqStatus[tid] = Invalid;
        return false;
    }

    bool ret_val = true;

    // TODO make this more efficient
    // Once the head of the FTQ gets updated and
    // the FTQ got blocked by a complex instruction resteere
    // we unblock by squashing
    if (ftqStatus[tid] == Locked) {
        DPRINTF(FTQ, "Pop FT:[fn%llu] unblocks FTQ. Require squash.\n",
                    ftq[tid].front()->ftNum());
        ftqStatus[tid] = Invalid;
        ret_val = false;
    }

    ppFTQRemove->notify(ftq[tid].front());
    ftq[tid].pop_front();
    stats.removals++;
    return ret_val;
}



void
FTQ::printFTQ(ThreadID tid) {
    int i = 0;
    for (auto ft : ftq[tid]) {
        DPRINTF(FTQ, "FTQ[tid:%i][%i]: %s.\n", tid, i, ft->print());
        i++;
    }
}



FTQ::FTQStats::FTQStats(o3::CPU *cpu, FTQ *ftq)
  : statistics::Group(cpu, "ftq"),
    ADD_STAT(inserts, statistics::units::Count::get(),
        "The number of FTQ insertions"),
    ADD_STAT(removals, statistics::units::Count::get(),
        "The number of FTQ removals. Not including squashes"),
    ADD_STAT(squashes, statistics::units::Count::get(),
        "The number of FTQ squashes"),
    ADD_STAT(locks, statistics::units::Count::get(),
        "The number of times the FTQ got locked."),
    ADD_STAT(occupancy, statistics::units::Count::get(),
        "Distribution of the FTQ occupation.")
{
    occupancy
        .init(/* base value */ 0,
              /* last value */ ftq->numEntries,
              /* bucket size */ 4)
        .flags(statistics::pdf);
}

} // namespace o3
} // namespace gem5
