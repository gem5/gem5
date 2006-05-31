/*
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
 *
 * Authors: Kevin Lim
 */

// Todo: Maybe have a special method for handling interrupts/traps.
//
// Traps:  Have IEW send a signal to commit saying that there's a trap to
// be handled.  Have commit send the PC back to the fetch stage, along
// with the current commit PC.  Fetch will directly access the IPR and save
// off all the proper stuff.  Commit can send out a squash, or something
// close to it.
// Do the same for hwrei().  However, requires that commit be specifically
// built to support that kind of stuff.  Probably not horrible to have
// commit support having the CPU tell it to squash the other stages and
// restart at a given address.  The IPR register does become an issue.
// Probably not a big deal if the IPR stuff isn't cycle accurate.  Can just
// have the original function handle writing to the IPR register.

#ifndef __CPU_O3_CPU_SIMPLE_COMMIT_HH__
#define __CPU_O3_CPU_SIMPLE_COMMIT_HH__

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "mem/memory_interface.hh"

template<class Impl>
class SimpleCommit
{
  public:
    // Typedefs from the Impl.
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::Params Params;
    typedef typename Impl::CPUPol CPUPol;

    typedef typename CPUPol::ROB ROB;

    typedef typename CPUPol::TimeStruct TimeStruct;
    typedef typename CPUPol::IEWStruct IEWStruct;
    typedef typename CPUPol::RenameStruct RenameStruct;

  public:
    // I don't believe commit can block, so it will only have two
    // statuses for now.
    // Actually if there's a cache access that needs to block (ie
    // uncachable load or just a mem access in commit) then the stage
    // may have to wait.
    enum Status {
        Running,
        Idle,
        ROBSquashing,
        DcacheMissStall,
        DcacheMissComplete
    };

  private:
    Status _status;

  public:
    SimpleCommit(Params &params);

    void regStats();

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    void setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr);

    void setROB(ROB *rob_ptr);

    void tick();

    void commit();

  private:

    void commitInsts();

    bool commitHead(DynInstPtr &head_inst, unsigned inst_num);

    void getInsts();

    void markCompletedInsts();

  public:
    uint64_t readCommitPC();

    void setSquashing() { _status = ROBSquashing; }

  private:
    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to write information heading to previous stages. */
    typename TimeBuffer<TimeStruct>::wire toIEW;

    /** Wire to read information from IEW (for ROB). */
    typename TimeBuffer<TimeStruct>::wire robInfoFromIEW;

    /** IEW instruction queue interface. */
    TimeBuffer<IEWStruct> *iewQueue;

    /** Wire to read information from IEW queue. */
    typename TimeBuffer<IEWStruct>::wire fromIEW;

    /** Rename instruction queue interface, for ROB. */
    TimeBuffer<RenameStruct> *renameQueue;

    /** Wire to read information from rename queue. */
    typename TimeBuffer<RenameStruct>::wire fromRename;

    /** ROB interface. */
    ROB *rob;

    /** Pointer to FullCPU. */
    FullCPU *cpu;

    /** Memory interface.  Used for d-cache accesses. */
    MemInterface *dcacheInterface;

  private:
    /** IEW to Commit delay, in ticks. */
    unsigned iewToCommitDelay;

    /** Rename to ROB delay, in ticks. */
    unsigned renameToROBDelay;

    /** Rename width, in instructions.  Used so ROB knows how many
     *  instructions to get from the rename instruction queue.
     */
    unsigned renameWidth;

    /** IEW width, in instructions.  Used so ROB knows how many
     *  instructions to get from the IEW instruction queue.
     */
    unsigned iewWidth;

    /** Commit width, in instructions. */
    unsigned commitWidth;

    Stats::Scalar<> commitCommittedInsts;
    Stats::Scalar<> commitSquashedInsts;
    Stats::Scalar<> commitSquashEvents;
    Stats::Scalar<> commitNonSpecStalls;
    Stats::Scalar<> commitCommittedBranches;
    Stats::Scalar<> commitCommittedLoads;
    Stats::Scalar<> commitCommittedMemRefs;
    Stats::Scalar<> branchMispredicts;

    Stats::Distribution<> n_committed_dist;
};

#endif // __CPU_O3_CPU_SIMPLE_COMMIT_HH__
