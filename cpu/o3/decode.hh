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
 */

#ifndef __CPU_BETA_CPU_SIMPLE_DECODE_HH__
#define __CPU_BETA_CPU_SIMPLE_DECODE_HH__

#include <queue>

#include "base/statistics.hh"
#include "base/timebuf.hh"

template<class Impl>
class SimpleDecode
{
  private:
    // Typedefs from the Impl.
    typedef typename Impl::ISA ISA;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::Params Params;
    typedef typename Impl::CPUPol CPUPol;

    // Typedefs from the CPU policy.
    typedef typename CPUPol::FetchStruct FetchStruct;
    typedef typename CPUPol::DecodeStruct DecodeStruct;
    typedef typename CPUPol::TimeStruct TimeStruct;

    // Typedefs from the ISA.
    typedef typename ISA::Addr Addr;

  public:
    // The only time decode will become blocked is if dispatch becomes
    // blocked, which means IQ or ROB is probably full.
    enum Status {
        Running,
        Idle,
        Squashing,
        Blocked,
        Unblocking
    };

  private:
    // May eventually need statuses on a per thread basis.
    Status _status;

  public:
    SimpleDecode(Params &params);

    void regStats();

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr);

    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    void tick();

    void decode();

  private:
    inline bool fetchInstsValid();

    void block();

    inline void unblock();

    void squash(DynInstPtr &inst);

  public:
    // Might want to make squash a friend function.
    void squash();

  private:
    // Interfaces to objects outside of decode.
    /** CPU interface. */
    FullCPU *cpu;

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get rename's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromRename;

    /** Wire to get iew's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire to write information heading to previous stages. */
    // Might not be the best name as not only fetch will read it.
    typename TimeBuffer<TimeStruct>::wire toFetch;

    /** Decode instruction queue. */
    TimeBuffer<DecodeStruct> *decodeQueue;

    /** Wire used to write any information heading to rename. */
    typename TimeBuffer<DecodeStruct>::wire toRename;

    /** Fetch instruction queue interface. */
    TimeBuffer<FetchStruct> *fetchQueue;

    /** Wire to get fetch's output from fetch queue. */
    typename TimeBuffer<FetchStruct>::wire fromFetch;

    /** Skid buffer between fetch and decode. */
    std::queue<FetchStruct> skidBuffer;

    //Consider making these unsigned to avoid any confusion.
    /** Rename to decode delay, in ticks. */
    unsigned renameToDecodeDelay;

    /** IEW to decode delay, in ticks. */
    unsigned iewToDecodeDelay;

    /** Commit to decode delay, in ticks. */
    unsigned commitToDecodeDelay;

    /** Fetch to decode delay, in ticks. */
    unsigned fetchToDecodeDelay;

    /** The width of decode, in instructions. */
    unsigned decodeWidth;

    /** The instruction that decode is currently on.  It needs to have
     *  persistent state so that when a stall occurs in the middle of a
     *  group of instructions, it can restart at the proper instruction.
     */
    unsigned numInst;

    Stats::Scalar<> decodeIdleCycles;
    Stats::Scalar<> decodeBlockedCycles;
    Stats::Scalar<> decodeUnblockCycles;
    Stats::Scalar<> decodeSquashCycles;
    Stats::Scalar<> decodeBranchMispred;
    Stats::Scalar<> decodeControlMispred;
    Stats::Scalar<> decodeDecodedInsts;
    Stats::Scalar<> decodeSquashedInsts;
};

#endif // __CPU_BETA_CPU_SIMPLE_DECODE_HH__
