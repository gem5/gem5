/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __CPU_OZONE_FRONT_END_HH__
#define __CPU_OZONE_FRONT_END_HH__

#include <deque>

#include "cpu/inst_seq.hh"
#include "cpu/o3/bpred_unit.hh"
#include "cpu/ozone/rename_table.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

class ExecContext;
class MemInterface;
template <class>
class OzoneThreadState;
class PageTable;
template <class>
class TimeBuffer;

template <class Impl>
class FrontEnd
{
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::BackEnd BackEnd;

    typedef typename Impl::FullCPU::OzoneXC OzoneXC;
    typedef typename Impl::FullCPU::CommStruct CommStruct;

    FrontEnd(Params *params);

    std::string name() const;

    void setCPU(FullCPU *cpu_ptr)
    { cpu = cpu_ptr; }

    void setBackEnd(BackEnd *back_end_ptr)
    { backEnd = back_end_ptr; }

    void setCommBuffer(TimeBuffer<CommStruct> *_comm);

    void setXC(ExecContext *xc_ptr);

    void setThreadState(OzoneThreadState<Impl> *thread_ptr)
    { thread = thread_ptr; }

    void regStats();

    void tick();
    Fault fetchCacheLine();
    void processInst(DynInstPtr &inst);
    void squash(const InstSeqNum &squash_num, const Addr &next_PC,
                const bool is_branch = false, const bool branch_taken = false);
    DynInstPtr getInst();

    void processCacheCompletion(Packet *pkt);

    void addFreeRegs(int num_freed);

    bool isEmpty() { return instBuffer.empty(); }

    void switchOut();

    void doSwitchOut();

    void takeOverFrom(ExecContext *old_xc = NULL);

    bool isSwitchedOut() { return switchedOut; }

    bool switchedOut;

  private:
    bool updateStatus();

    void checkBE();
    DynInstPtr getInstFromCacheline();
    void renameInst(DynInstPtr &inst);
    // Returns true if we need to stop the front end this cycle
    bool processBarriers(DynInstPtr &inst);

    void handleFault(Fault &fault);
  public:
    Fault getFault() { return fetchFault; }
  private:
    Fault fetchFault;

    // Align an address (typically a PC) to the start of an I-cache block.
    // We fold in the PISA 64- to 32-bit conversion here as well.
    Addr icacheBlockAlignPC(Addr addr)
    {
        addr = TheISA::realPCToFetchPC(addr);
        return (addr & ~(cacheBlkMask));
    }

    InstSeqNum getAndIncrementInstSeq()
    { return cpu->globalSeqNum++; }

  public:
    FullCPU *cpu;

    BackEnd *backEnd;

    ExecContext *xc;

    OzoneThreadState<Impl> *thread;

    enum Status {
        Running,
        Idle,
        IcacheMissStall,
        IcacheMissComplete,
        SerializeBlocked,
        SerializeComplete,
        RenameBlocked,
        QuiescePending,
        TrapPending,
        BEBlocked
    };

    Status status;

  private:
    TimeBuffer<CommStruct> *comm;
    typename TimeBuffer<CommStruct>::wire fromCommit;

    typedef typename Impl::BranchPred BranchPred;

    BranchPred branchPred;

    class IcachePort : public Port
    {
      protected:
        FrontEnd *fe;

      public:
        IcachePort(const std::string &_name, FrontEnd *_fe)
            : Port(_name), fe(_fe)
        { }

      protected:
        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        virtual void recvStatusChange(Status status);

        virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop)
        { resp.clear(); snoop.clear(); }

        virtual bool recvTiming(PacketPtr pkt);

        virtual void recvRetry();
    };

    IcachePort icachePort;

#if !FULL_SYSTEM
    PageTable *pTable;
#endif

    RequestPtr memReq;

    /** Mask to get a cache block's address. */
    Addr cacheBlkMask;

    unsigned cacheBlkSize;

    Addr cacheBlkPC;

    /** The cache line being fetched. */
    uint8_t *cacheData;

    bool fetchCacheLineNextCycle;

    bool cacheBlkValid;

  public:
    RenameTable<Impl> renameTable;

  private:
    Addr PC;
    Addr nextPC;

  public:
    void setPC(Addr val) { PC = val; }
    void setNextPC(Addr val) { nextPC = val; }

    void wakeFromQuiesce();

    void dumpInsts();

  private:
    typedef typename std::deque<DynInstPtr> InstBuff;
    typedef typename InstBuff::iterator InstBuffIt;

    InstBuff instBuffer;

    int instBufferSize;

    int maxInstBufferSize;

    int width;

    int freeRegs;

    int numPhysRegs;

    bool serializeNext;

    DynInstPtr barrierInst;

  public:
    bool interruptPending;
  private:
    // number of idle cycles
/*
    Stats::Average<> notIdleFraction;
    Stats::Formula idleFraction;
*/
    // @todo: Consider making these vectors and tracking on a per thread basis.
    /** Stat for total number of cycles stalled due to an icache miss. */
    Stats::Scalar<> icacheStallCycles;
    /** Stat for total number of fetched instructions. */
    Stats::Scalar<> fetchedInsts;
    Stats::Scalar<> fetchedBranches;
    /** Stat for total number of predicted branches. */
    Stats::Scalar<> predictedBranches;
    /** Stat for total number of cycles spent fetching. */
    Stats::Scalar<> fetchCycles;

    Stats::Scalar<> fetchIdleCycles;
    /** Stat for total number of cycles spent squashing. */
    Stats::Scalar<> fetchSquashCycles;
    /** Stat for total number of cycles spent blocked due to other stages in
     * the pipeline.
     */
    Stats::Scalar<> fetchBlockedCycles;
    /** Stat for total number of fetched cache lines. */
    Stats::Scalar<> fetchedCacheLines;

    Stats::Scalar<> fetchIcacheSquashes;
    /** Distribution of number of instructions fetched each cycle. */
    Stats::Distribution<> fetchNisnDist;
//    Stats::Vector<> qfull_iq_occupancy;
//    Stats::VectorDistribution<> qfull_iq_occ_dist_;
    Stats::Formula idleRate;
    Stats::Formula branchRate;
    Stats::Formula fetchRate;
    Stats::Scalar<> IFQCount;	// cumulative IFQ occupancy
    Stats::Formula IFQOccupancy;
    Stats::Formula IFQLatency;
    Stats::Scalar<> IFQFcount; // cumulative IFQ full count
    Stats::Formula IFQFullRate;

    Stats::Scalar<> dispatchCountStat;
    Stats::Scalar<> dispatchedSerializing;
    Stats::Scalar<> dispatchedTempSerializing;
    Stats::Scalar<> dispatchSerializeStallCycles;
    Stats::Formula dispatchRate;
    Stats::Formula regIntFull;
    Stats::Formula regFpFull;
};

#endif // __CPU_OZONE_FRONT_END_HH__
