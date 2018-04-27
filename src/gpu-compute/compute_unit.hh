/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: John Kalamatianos,
 *          Anthony Gutierrez
 */

#ifndef __COMPUTE_UNIT_HH__
#define __COMPUTE_UNIT_HH__

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "enums/PrefetchType.hh"
#include "gpu-compute/exec_stage.hh"
#include "gpu-compute/fetch_stage.hh"
#include "gpu-compute/global_memory_pipeline.hh"
#include "gpu-compute/local_memory_pipeline.hh"
#include "gpu-compute/qstruct.hh"
#include "gpu-compute/schedule_stage.hh"
#include "gpu-compute/scoreboard_check_stage.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"

static const int MAX_REGS_FOR_NON_VEC_MEM_INST = 1;
static const int MAX_WIDTH_FOR_MEM_INST = 32;

class NDRange;
class Shader;
class VectorRegisterFile;

struct ComputeUnitParams;

enum EXEC_POLICY
{
    OLDEST = 0,
    RR
};

// List of execution units
enum EXEC_UNIT
{
    SIMD0 = 0,
    SIMD1,
    SIMD2,
    SIMD3,
    GLBMEM_PIPE,
    LDSMEM_PIPE,
    NUM_UNITS
};

enum TLB_CACHE
{
    TLB_MISS_CACHE_MISS = 0,
    TLB_MISS_CACHE_HIT,
    TLB_HIT_CACHE_MISS,
    TLB_HIT_CACHE_HIT
};

class ComputeUnit : public MemObject
{
  public:
    FetchStage fetchStage;
    ScoreboardCheckStage scoreboardCheckStage;
    ScheduleStage scheduleStage;
    ExecStage execStage;
    GlobalMemPipeline globalMemoryPipe;
    LocalMemPipeline localMemoryPipe;

    // Buffers used to communicate between various pipeline stages

    // List of waves which are ready to be scheduled.
    // Each execution resource has a ready list. readyList is
    // used to communicate between scoreboardCheck stage and
    // schedule stage
    // TODO: make enum to index readyList
    std::vector<std::vector<Wavefront*>> readyList;

    // Stores the status of waves. A READY implies the
    // wave is ready to be scheduled this cycle and
    // is already present in the readyList. waveStatusList is
    // used to communicate between scoreboardCheck stage and
    // schedule stage
    // TODO: convert std::pair to a class to increase readability
    std::vector<std::vector<std::pair<Wavefront*, WAVE_STATUS>>> waveStatusList;

    // List of waves which will be dispatched to
    // each execution resource. A FILLED implies
    // dispatch list is non-empty and
    // execution unit has something to execute
    // this cycle. Currently, the dispatch list of
    // an execution resource can hold only one wave because
    // an execution resource can execute only one wave in a cycle.
    // dispatchList is used to communicate between schedule
    // and exec stage
    // TODO: convert std::pair to a class to increase readability
    std::vector<std::pair<Wavefront*, DISPATCH_STATUS>> dispatchList;

    int rrNextMemID; // used by RR WF exec policy to cycle through WF's
    int rrNextALUWp;
    typedef ComputeUnitParams Params;
    std::vector<std::vector<Wavefront*>> wfList;
    int cu_id;

    // array of vector register files, one per SIMD
    std::vector<VectorRegisterFile*> vrf;
    // Number of vector ALU units (SIMDs) in CU
    int numSIMDs;
    // number of pipe stages for bypassing data to next dependent single
    // precision vector instruction inside the vector ALU pipeline
    int spBypassPipeLength;
    // number of pipe stages for bypassing data to next dependent double
    // precision vector instruction inside the vector ALU pipeline
    int dpBypassPipeLength;
    // number of cycles per issue period
    int issuePeriod;

    // Number of global and local memory execution resources in CU
    int numGlbMemUnits;
    int numLocMemUnits;
    // tracks the last cycle a vector instruction was executed on a SIMD
    std::vector<uint64_t> lastExecCycle;

    // true if we allow a separate TLB per lane
    bool perLaneTLB;
    // if 0, TLB prefetching is off.
    int prefetchDepth;
    // if fixed-stride prefetching, this is the stride.
    int prefetchStride;

    std::vector<Addr> lastVaddrCU;
    std::vector<std::vector<Addr>> lastVaddrSimd;
    std::vector<std::vector<std::vector<Addr>>> lastVaddrWF;
    Enums::PrefetchType prefetchType;
    EXEC_POLICY exec_policy;

    bool xact_cas_mode;
    bool debugSegFault;
    bool functionalTLB;
    bool localMemBarrier;

    /*
     * for Counting page accesses
     *
     * cuExitCallback inherits from Callback. When you register a callback
     * function as an exit callback, it will get added to an exit callback
     * queue, such that on simulation exit, all callbacks in the callback
     * queue will have their process() function called.
     */
    bool countPages;

    Shader *shader;
    uint32_t barrier_id;
    // vector of Vector ALU (MACC) pipelines
    std::vector<WaitClass> aluPipe;
    // minimum issue period per SIMD unit (in cycles)
    std::vector<WaitClass> wfWait;

    // Resource control for Vector Register File->Global Memory pipe buses
    std::vector<WaitClass> vrfToGlobalMemPipeBus;
    // Resource control for Vector Register File->Local Memory pipe buses
    std::vector<WaitClass> vrfToLocalMemPipeBus;
    int nextGlbMemBus;
    int nextLocMemBus;
    // Resource control for global memory to VRF data/address bus
    WaitClass glbMemToVrfBus;
    // Resource control for local memory to VRF data/address bus
    WaitClass locMemToVrfBus;

    uint32_t vrfToCoalescerBusWidth; // VRF->Coalescer data bus width in bytes
    uint32_t coalescerToVrfBusWidth; // Coalescer->VRF data bus width in bytes
    uint32_t numCyclesPerStoreTransfer;  // number of cycles per vector store
    uint32_t numCyclesPerLoadTransfer;  // number of cycles per vector load

    Tick req_tick_latency;
    Tick resp_tick_latency;

    // number of vector registers being reserved for each SIMD unit
    std::vector<int> vectorRegsReserved;
    // number of vector registers per SIMD unit
    uint32_t numVecRegsPerSimd;
    // Support for scheduling VGPR status update events
    std::vector<std::pair<uint32_t, uint32_t> > regIdxVec;
    std::vector<uint64_t> timestampVec;
    std::vector<uint8_t>  statusVec;

    void
    registerEvent(uint32_t simdId,
                  uint32_t regIdx,
                  uint32_t operandSize,
                  uint64_t when,
                  uint8_t newStatus) {
        regIdxVec.push_back(std::make_pair(simdId, regIdx));
        timestampVec.push_back(when);
        statusVec.push_back(newStatus);
        if (operandSize > 4) {
            regIdxVec.push_back(std::make_pair(simdId,
                                               ((regIdx + 1) %
                                                numVecRegsPerSimd)));
            timestampVec.push_back(when);
            statusVec.push_back(newStatus);
        }
    }

    void updateEvents();

    // this hash map will keep track of page divergence
    // per memory instruction per wavefront. The hash map
    // is cleared in GPUDynInst::updateStats() in gpu_dyn_inst.cc.
    std::map<Addr, int> pagesTouched;

    ComputeUnit(const Params *p);
    ~ComputeUnit();
    int spBypassLength() { return spBypassPipeLength; };
    int dpBypassLength() { return dpBypassPipeLength; };
    int storeBusLength() { return numCyclesPerStoreTransfer; };
    int loadBusLength() { return numCyclesPerLoadTransfer; };
    int wfSize() const { return wavefrontSize; };

    void resizeRegFiles(int num_cregs, int num_sregs, int num_dregs);
    void exec();
    void initiateFetch(Wavefront *wavefront);
    void fetch(PacketPtr pkt, Wavefront *wavefront);
    void fillKernelState(Wavefront *w, NDRange *ndr);

    void startWavefront(Wavefront *w, int waveId, LdsChunk *ldsChunk,
                        NDRange *ndr);

    void StartWorkgroup(NDRange *ndr);
    int ReadyWorkgroup(NDRange *ndr);

    bool isVecAlu(int unitId) { return unitId >= SIMD0 && unitId <= SIMD3; }
    bool isGlbMem(int unitId) { return unitId == GLBMEM_PIPE; }
    bool isShrMem(int unitId) { return unitId == LDSMEM_PIPE; }
    int GlbMemUnitId() { return GLBMEM_PIPE; }
    int ShrMemUnitId() { return LDSMEM_PIPE; }
    int nextGlbRdBus() { return (++nextGlbMemBus) % numGlbMemUnits; }
    int nextLocRdBus() { return (++nextLocMemBus) % numLocMemUnits; }
    /* This function cycles through all the wavefronts in all the phases to see
     * if all of the wavefronts which should be associated with one barrier
     * (denoted with _barrier_id), are all at the same barrier in the program
     * (denoted by bcnt). When the number at the barrier matches bslots, then
     * return true.
     */
    int AllAtBarrier(uint32_t _barrier_id, uint32_t bcnt, uint32_t bslots);
    bool cedeSIMD(int simdId, int wfSlotId);

    template<typename c0, typename c1> void doSmReturn(GPUDynInstPtr gpuDynInst);
    virtual void init();
    void sendRequest(GPUDynInstPtr gpuDynInst, int index, PacketPtr pkt);
    void sendSyncRequest(GPUDynInstPtr gpuDynInst, int index, PacketPtr pkt);
    void injectGlobalMemFence(GPUDynInstPtr gpuDynInst,
                              bool kernelLaunch=true,
                              RequestPtr req=nullptr);
    void handleMemPacket(PacketPtr pkt, int memport_index);
    bool processTimingPacket(PacketPtr pkt);
    void processFetchReturn(PacketPtr pkt);
    void updatePageDivergenceDist(Addr addr);

    MasterID masterId() { return _masterId; }

    bool isDone() const;
    bool isSimdDone(uint32_t) const;

  protected:
    MasterID _masterId;

    LdsState &lds;

  public:
    Stats::Scalar vALUInsts;
    Stats::Formula vALUInstsPerWF;
    Stats::Scalar sALUInsts;
    Stats::Formula sALUInstsPerWF;
    Stats::Scalar instCyclesVALU;
    Stats::Scalar instCyclesSALU;
    Stats::Scalar threadCyclesVALU;
    Stats::Formula vALUUtilization;
    Stats::Scalar ldsNoFlatInsts;
    Stats::Formula ldsNoFlatInstsPerWF;
    Stats::Scalar flatVMemInsts;
    Stats::Formula flatVMemInstsPerWF;
    Stats::Scalar flatLDSInsts;
    Stats::Formula flatLDSInstsPerWF;
    Stats::Scalar vectorMemWrites;
    Stats::Formula vectorMemWritesPerWF;
    Stats::Scalar vectorMemReads;
    Stats::Formula vectorMemReadsPerWF;
    Stats::Scalar scalarMemWrites;
    Stats::Formula scalarMemWritesPerWF;
    Stats::Scalar scalarMemReads;
    Stats::Formula scalarMemReadsPerWF;

    void updateInstStats(GPUDynInstPtr gpuDynInst);

    // the following stats compute the avg. TLB accesslatency per
    // uncoalesced request (only for data)
    Stats::Scalar tlbRequests;
    Stats::Scalar tlbCycles;
    Stats::Formula tlbLatency;
    // hitsPerTLBLevel[x] are the hits in Level x TLB. x = 0 is the page table.
    Stats::Vector hitsPerTLBLevel;

    Stats::Scalar ldsBankAccesses;
    Stats::Distribution ldsBankConflictDist;

    // over all memory instructions executed over all wavefronts
    // how many touched 0-4 pages, 4-8, ..., 60-64 pages
    Stats::Distribution pageDivergenceDist;
    Stats::Scalar dynamicGMemInstrCnt;
    Stats::Scalar dynamicLMemInstrCnt;

    Stats::Scalar wgBlockedDueLdsAllocation;
    // Number of instructions executed, i.e. if 64 (or 32 or 7) lanes are active
    // when the instruction is committed, this number is still incremented by 1
    Stats::Scalar numInstrExecuted;
    // Number of cycles among successive instruction executions across all
    // wavefronts of the same CU
    Stats::Distribution execRateDist;
    // number of individual vector operations executed
    Stats::Scalar numVecOpsExecuted;
    // Total cycles that something is running on the GPU
    Stats::Scalar totalCycles;
    Stats::Formula vpc; // vector ops per cycle
    Stats::Formula ipc; // vector instructions per cycle
    Stats::Distribution controlFlowDivergenceDist;
    Stats::Distribution activeLanesPerGMemInstrDist;
    Stats::Distribution activeLanesPerLMemInstrDist;
    // number of vector ALU instructions received
    Stats::Formula numALUInstsExecuted;
    // number of times a WG can not start due to lack of free VGPRs in SIMDs
    Stats::Scalar numTimesWgBlockedDueVgprAlloc;
    Stats::Scalar numCASOps;
    Stats::Scalar numFailedCASOps;
    Stats::Scalar completedWfs;
    // flag per vector SIMD unit that is set when there is at least one
    // WV that has a vector ALU instruction as the oldest in its
    // Instruction Buffer: Defined in the Scoreboard stage, consumed
    // by the Execute stage.
    std::vector<bool> vectorAluInstAvail;
    // number of available (oldest) LDS instructions that could have
    // been issued to the LDS at a specific issue slot
    int shrMemInstAvail;
    // number of available Global memory instructions that could have
    // been issued to TCP at a specific issue slot
    int glbMemInstAvail;

    void
    regStats();

    LdsState &
    getLds() const
    {
        return lds;
    }

    int32_t
    getRefCounter(const uint32_t dispatchId, const uint32_t wgId) const;

    int cacheLineSize() const { return _cacheLineSize; }

    bool
    sendToLds(GPUDynInstPtr gpuDynInst) __attribute__((warn_unused_result));

    typedef std::unordered_map<Addr, std::pair<int, int>> pageDataStruct;
    pageDataStruct pageAccesses;

    class CUExitCallback : public Callback
    {
      private:
        ComputeUnit *computeUnit;

      public:
        virtual ~CUExitCallback() { }

        CUExitCallback(ComputeUnit *_cu)
        {
            computeUnit = _cu;
        }

        virtual void
        process();
    };

    CUExitCallback *cuExitCallback;

    /** Data access Port **/
    class DataPort : public MasterPort
    {
      public:
        DataPort(const std::string &_name, ComputeUnit *_cu, PortID _index)
            : MasterPort(_name, _cu), computeUnit(_cu),
              index(_index) { }

        bool snoopRangeSent;

        struct SenderState : public Packet::SenderState
        {
            GPUDynInstPtr _gpuDynInst;
            int port_index;
            Packet::SenderState *saved;

            SenderState(GPUDynInstPtr gpuDynInst, PortID _port_index,
                        Packet::SenderState *sender_state=nullptr)
                : _gpuDynInst(gpuDynInst),
                  port_index(_port_index),
                  saved(sender_state) { }
        };

        void processMemReqEvent(PacketPtr pkt);
        EventFunctionWrapper *createMemReqEvent(PacketPtr pkt);

        void processMemRespEvent(PacketPtr pkt);
        EventFunctionWrapper *createMemRespEvent(PacketPtr pkt);

        std::deque<std::pair<PacketPtr, GPUDynInstPtr>> retries;

      protected:
        ComputeUnit *computeUnit;
        int index;

        virtual bool recvTimingResp(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
        virtual void recvFunctional(PacketPtr pkt) { }
        virtual void recvRangeChange() { }
        virtual void recvReqRetry();

        virtual void
        getDeviceAddressRanges(AddrRangeList &resp, bool &snoop)
        {
            resp.clear();
            snoop = true;
        }

    };

    // Instruction cache access port
    class SQCPort : public MasterPort
    {
      public:
        SQCPort(const std::string &_name, ComputeUnit *_cu, PortID _index)
            : MasterPort(_name, _cu), computeUnit(_cu),
              index(_index) { }

        bool snoopRangeSent;

        struct SenderState : public Packet::SenderState
        {
            Wavefront *wavefront;
            Packet::SenderState *saved;

            SenderState(Wavefront *_wavefront, Packet::SenderState
                    *sender_state=nullptr)
                : wavefront(_wavefront), saved(sender_state) { }
        };

        std::deque<std::pair<PacketPtr, Wavefront*>> retries;

      protected:
        ComputeUnit *computeUnit;
        int index;

        virtual bool recvTimingResp(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
        virtual void recvFunctional(PacketPtr pkt) { }
        virtual void recvRangeChange() { }
        virtual void recvReqRetry();

        virtual void
        getDeviceAddressRanges(AddrRangeList &resp, bool &snoop)
        {
            resp.clear();
            snoop = true;
        }
     };

    /** Data TLB port **/
    class DTLBPort : public MasterPort
    {
      public:
        DTLBPort(const std::string &_name, ComputeUnit *_cu, PortID _index)
            : MasterPort(_name, _cu), computeUnit(_cu),
              index(_index), stalled(false)
        { }

        bool isStalled() { return stalled; }
        void stallPort() { stalled = true; }
        void unstallPort() { stalled = false; }

        /**
         * here we queue all the translation requests that were
         * not successfully sent.
         */
        std::deque<PacketPtr> retries;

        /** SenderState is information carried along with the packet
         * throughout the TLB hierarchy
         */
        struct SenderState: public Packet::SenderState
        {
            // the memInst that this is associated with
            GPUDynInstPtr _gpuDynInst;

            // the lane in the memInst this is associated with, so we send
            // the memory request down the right port
            int portIndex;

            // constructor used for packets involved in timing accesses
            SenderState(GPUDynInstPtr gpuDynInst, PortID port_index)
                : _gpuDynInst(gpuDynInst), portIndex(port_index) { }

        };

      protected:
        ComputeUnit *computeUnit;
        int index;
        bool stalled;

        virtual bool recvTimingResp(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
        virtual void recvFunctional(PacketPtr pkt) { }
        virtual void recvRangeChange() { }
        virtual void recvReqRetry();
    };

    class ITLBPort : public MasterPort
    {
      public:
        ITLBPort(const std::string &_name, ComputeUnit *_cu)
            : MasterPort(_name, _cu), computeUnit(_cu), stalled(false) { }


        bool isStalled() { return stalled; }
        void stallPort() { stalled = true; }
        void unstallPort() { stalled = false; }

        /**
         * here we queue all the translation requests that were
         * not successfully sent.
         */
        std::deque<PacketPtr> retries;

        /** SenderState is information carried along with the packet
         * throughout the TLB hierarchy
         */
        struct SenderState: public Packet::SenderState
        {
            // The wavefront associated with this request
            Wavefront *wavefront;

            SenderState(Wavefront *_wavefront) : wavefront(_wavefront) { }
        };

      protected:
        ComputeUnit *computeUnit;
        bool stalled;

        virtual bool recvTimingResp(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
        virtual void recvFunctional(PacketPtr pkt) { }
        virtual void recvRangeChange() { }
        virtual void recvReqRetry();
    };

    /**
     * the port intended to communicate between the CU and its LDS
     */
    class LDSPort : public MasterPort
    {
      public:
        LDSPort(const std::string &_name, ComputeUnit *_cu, PortID _id)
        : MasterPort(_name, _cu, _id), computeUnit(_cu)
        {
        }

        bool isStalled() const { return stalled; }
        void stallPort() { stalled = true; }
        void unstallPort() { stalled = false; }

        /**
         * here we queue all the requests that were
         * not successfully sent.
         */
        std::queue<PacketPtr> retries;

        /**
         *  SenderState is information carried along with the packet, esp. the
         *  GPUDynInstPtr
         */
        class SenderState: public Packet::SenderState
        {
          protected:
            // The actual read/write/atomic request that goes with this command
            GPUDynInstPtr _gpuDynInst = nullptr;

          public:
            SenderState(GPUDynInstPtr gpuDynInst):
              _gpuDynInst(gpuDynInst)
            {
            }

            GPUDynInstPtr
            getMemInst() const
            {
              return _gpuDynInst;
            }
        };

        virtual bool
        sendTimingReq(PacketPtr pkt);

      protected:

        bool stalled = false; ///< whether or not it is stalled

        ComputeUnit *computeUnit;

        virtual bool
        recvTimingResp(PacketPtr pkt);

        virtual Tick
        recvAtomic(PacketPtr pkt) { return 0; }

        virtual void
        recvFunctional(PacketPtr pkt)
        {
        }

        virtual void
        recvRangeChange()
        {
        }

        virtual void
        recvReqRetry();
    };

    /** The port to access the Local Data Store
     *  Can be connected to a LDS object
     */
    LDSPort *ldsPort = nullptr;

    LDSPort *
    getLdsPort() const
    {
        return ldsPort;
    }

    /** The memory port for SIMD data accesses.
     *  Can be connected to PhysMem for Ruby for timing simulations
     */
    std::vector<DataPort*> memPort;
    // port to the TLB hierarchy (i.e., the L1 TLB)
    std::vector<DTLBPort*> tlbPort;
    // port to the SQC (i.e. the I-cache)
    SQCPort *sqcPort;
    // port to the SQC TLB (there's a separate TLB for each I-cache)
    ITLBPort *sqcTLBPort;

    virtual BaseMasterPort&
    getMasterPort(const std::string &if_name, PortID idx)
    {
        if (if_name == "memory_port") {
            memPort[idx] = new DataPort(csprintf("%s-port%d", name(), idx),
                                        this, idx);
            return *memPort[idx];
        } else if (if_name == "translation_port") {
            tlbPort[idx] = new DTLBPort(csprintf("%s-port%d", name(), idx),
                                        this, idx);
            return *tlbPort[idx];
        } else if (if_name == "sqc_port") {
            sqcPort = new SQCPort(csprintf("%s-port%d", name(), idx),
                                  this, idx);
            return *sqcPort;
        } else if (if_name == "sqc_tlb_port") {
            sqcTLBPort = new ITLBPort(csprintf("%s-port", name()), this);
            return *sqcTLBPort;
        } else if (if_name == "ldsPort") {
            if (ldsPort) {
                fatal("an LDS port was already allocated");
            }
            ldsPort = new LDSPort(csprintf("%s-port", name()), this, idx);
            return *ldsPort;
        } else {
            panic("incorrect port name");
        }
    }

    // xact_cas_load()
    class waveIdentifier
    {
      public:
        waveIdentifier() { }
        waveIdentifier(int _simdId, int _wfSlotId)
          : simdId(_simdId), wfSlotId(_wfSlotId) { }

        int simdId;
        int wfSlotId;
    };

    class waveQueue
    {
      public:
        std::list<waveIdentifier> waveIDQueue;
    };
    std::map<unsigned, waveQueue> xactCasLoadMap;

    uint64_t getAndIncSeqNum() { return globalSeqNum++; }

  private:
    const int _cacheLineSize;
    uint64_t globalSeqNum;
    int wavefrontSize;
    GPUStaticInst *kernelLaunchInst;
};

#endif // __COMPUTE_UNIT_HH__
