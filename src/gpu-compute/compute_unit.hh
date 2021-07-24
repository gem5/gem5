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
 */

#ifndef __COMPUTE_UNIT_HH__
#define __COMPUTE_UNIT_HH__

#include <deque>
#include <map>
#include <unordered_set>
#include <vector>

#include "base/callback.hh"
#include "base/compiler.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "config/the_gpu_isa.hh"
#include "enums/PrefetchType.hh"
#include "gpu-compute/comm.hh"
#include "gpu-compute/exec_stage.hh"
#include "gpu-compute/fetch_stage.hh"
#include "gpu-compute/global_memory_pipeline.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "gpu-compute/local_memory_pipeline.hh"
#include "gpu-compute/register_manager.hh"
#include "gpu-compute/scalar_memory_pipeline.hh"
#include "gpu-compute/schedule_stage.hh"
#include "gpu-compute/scoreboard_check_stage.hh"
#include "mem/port.hh"
#include "mem/token_port.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class HSAQueueEntry;
class LdsChunk;
class ScalarRegisterFile;
class Shader;
class VectorRegisterFile;

struct ComputeUnitParams;

enum EXEC_POLICY
{
    OLDEST = 0,
    RR
};

enum TLB_CACHE
{
    TLB_MISS_CACHE_MISS = 0,
    TLB_MISS_CACHE_HIT,
    TLB_HIT_CACHE_MISS,
    TLB_HIT_CACHE_HIT
};

/**
 * WF barrier slots. This represents the barrier resource for
 * WF-level barriers (i.e., barriers to sync WFs within a WG).
 */
class WFBarrier
{
  public:
    WFBarrier() : _numAtBarrier(0), _maxBarrierCnt(0)
    {
    }

    static const int InvalidID = -1;

    int
    numAtBarrier() const
    {
        return _numAtBarrier;
    }

    /**
     * Number of WFs that have not yet reached the barrier.
     */
    int
    numYetToReachBarrier() const
    {
        return _maxBarrierCnt - _numAtBarrier;
    }

    int
    maxBarrierCnt() const
    {
        return _maxBarrierCnt;
    }

    /**
     * Set the maximum barrier count (i.e., the number of WFs that are
     * participating in the barrier).
     */
    void
    setMaxBarrierCnt(int max_barrier_cnt)
    {
        _maxBarrierCnt = max_barrier_cnt;
    }

    /**
     * Mark that a WF has reached the barrier.
     */
    void
    incNumAtBarrier()
    {
        assert(_numAtBarrier < _maxBarrierCnt);
        ++_numAtBarrier;
    }

    /**
     * Have all WFs participating in this barrier reached the barrier?
     * If so, then the barrier is satisfied and WFs may proceed past
     * the barrier.
     */
    bool
    allAtBarrier() const
    {
        return _numAtBarrier == _maxBarrierCnt;
    }

    /**
     * Decrement the number of WFs that are participating in this barrier.
     * This should be called when a WF exits.
     */
    void
    decMaxBarrierCnt()
    {
        assert(_maxBarrierCnt > 0);
        --_maxBarrierCnt;
    }

    /**
     * Release this barrier resource so it can be used by other WGs. This
     * is generally called when a WG has finished.
     */
    void
    release()
    {
        _numAtBarrier = 0;
        _maxBarrierCnt = 0;
    }

    /**
     * Reset the barrier. This is used to reset the barrier, usually when
     * a dynamic instance of a barrier has been satisfied.
     */
    void
    reset()
    {
        _numAtBarrier = 0;
    }

  private:
    /**
     * The number of WFs in the WG that have reached the barrier. Once
     * the number of WFs that reach a barrier matches the number of WFs
     * in the WG, the barrier is satisfied.
     */
    int _numAtBarrier;

    /**
     * The maximum number of WFs that can reach this barrier. This is
     * essentially the number of WFs in the WG, and a barrier is satisfied
     * when the number of WFs that reach the barrier equal this value. If
     * a WF exits early it must decrement this value so that it is no
     * longer considered for this barrier.
     */
    int _maxBarrierCnt;
};

class ComputeUnit : public ClockedObject
{
  public:


    // Execution resources
    //
    // The ordering of units is:
    // Vector ALUs
    // Scalar ALUs
    // GM Pipe
    // LM Pipe
    // Scalar Mem Pipe
    //
    // Note: the ordering of units is important and the code assumes the
    // above ordering. However, there may be more than one resource of
    // each type (e.g., 4 VALUs or 2 SALUs)

    int numVectorGlobalMemUnits;
    // Resource control for global memory to VRF data/address bus
    WaitClass glbMemToVrfBus;
    // Resource control for Vector Register File->Global Memory pipe buses
    WaitClass vrfToGlobalMemPipeBus;
    // Resource control for Vector Global Memory execution unit
    WaitClass vectorGlobalMemUnit;

    int numVectorSharedMemUnits;
    // Resource control for local memory to VRF data/address bus
    WaitClass locMemToVrfBus;
    // Resource control for Vector Register File->Local Memory pipe buses
    WaitClass vrfToLocalMemPipeBus;
    // Resource control for Vector Shared/Local Memory execution unit
    WaitClass vectorSharedMemUnit;

    int numScalarMemUnits;
    // Resource control for scalar memory to SRF data/address bus
    WaitClass scalarMemToSrfBus;
    // Resource control for Scalar Register File->Scalar Memory pipe buses
    WaitClass srfToScalarMemPipeBus;
    // Resource control for Scalar Memory execution unit
    WaitClass scalarMemUnit;

    // vector ALU execution resources
    int numVectorALUs;
    std::vector<WaitClass> vectorALUs;

    // scalar ALU execution resources
    int numScalarALUs;
    std::vector<WaitClass> scalarALUs;

    // Return total number of execution units on this CU
    int numExeUnits() const;
    // index into readyList of the first memory unit
    int firstMemUnit() const;
    // index into readyList of the last memory unit
    int lastMemUnit() const;
    // index into scalarALUs vector of SALU used by the wavefront
    int mapWaveToScalarAlu(Wavefront *w) const;
    // index into readyList of SALU used by wavefront
    int mapWaveToScalarAluGlobalIdx(Wavefront *w) const;
    // index into readyList of Global Memory unit used by wavefront
    int mapWaveToGlobalMem(Wavefront *w) const;
    // index into readyList of Local Memory unit used by wavefront
    int mapWaveToLocalMem(Wavefront *w) const;
    // index into readyList of Scalar Memory unit used by wavefront
    int mapWaveToScalarMem(Wavefront *w) const;

    int vrfToCoalescerBusWidth; // VRF->Coalescer data bus width in bytes
    int coalescerToVrfBusWidth; // Coalescer->VRF data bus width in bytes
    int numCyclesPerStoreTransfer;  // number of cycles per vector store
    int numCyclesPerLoadTransfer;  // number of cycles per vector load

    // track presence of dynamic instructions in the Schedule pipeline
    // stage. This is used to check the readiness of the oldest,
    // non-dispatched instruction of every WF in the Scoreboard stage.
    std::unordered_set<uint64_t> pipeMap;

    RegisterManager* registerManager;

    FetchStage fetchStage;
    ScoreboardCheckStage scoreboardCheckStage;
    ScheduleStage scheduleStage;
    ExecStage execStage;
    GlobalMemPipeline globalMemoryPipe;
    LocalMemPipeline localMemoryPipe;
    ScalarMemPipeline scalarMemoryPipe;

    EventFunctionWrapper tickEvent;

    typedef ComputeUnitParams Params;
    std::vector<std::vector<Wavefront*>> wfList;
    int cu_id;

    // array of vector register files, one per SIMD
    std::vector<VectorRegisterFile*> vrf;
    // array of scalar register files, one per SIMD
    std::vector<ScalarRegisterFile*> srf;

    // Width per VALU/SIMD unit: number of work items that can be executed
    // on the vector ALU simultaneously in a SIMD unit
    int simdWidth;
    // number of pipe stages for bypassing data to next dependent single
    // precision vector instruction inside the vector ALU pipeline
    int spBypassPipeLength;
    // number of pipe stages for bypassing data to next dependent double
    // precision vector instruction inside the vector ALU pipeline
    int dpBypassPipeLength;
    // number of pipe stages for scalar ALU
    int scalarPipeStages;
    // number of pipe stages for operand collection & distribution network
    int operandNetworkLength;
    // number of cycles per instruction issue period
    Cycles issuePeriod;

    // VRF to GM Bus latency
    Cycles vrf_gm_bus_latency;
    // SRF to Scalar Mem Bus latency
    Cycles srf_scm_bus_latency;
    // VRF to LM Bus latency
    Cycles vrf_lm_bus_latency;

    // tracks the last cycle a vector instruction was executed on a SIMD
    std::vector<uint64_t> lastExecCycle;

    // tracks the number of dyn inst executed per SIMD
    std::vector<uint64_t> instExecPerSimd;

    // true if we allow a separate TLB per lane
    bool perLaneTLB;
    // if 0, TLB prefetching is off.
    int prefetchDepth;
    // if fixed-stride prefetching, this is the stride.
    int prefetchStride;

    std::vector<Addr> lastVaddrCU;
    std::vector<std::vector<Addr>> lastVaddrSimd;
    std::vector<std::vector<std::vector<Addr>>> lastVaddrWF;
    enums::PrefetchType prefetchType;
    EXEC_POLICY exec_policy;

    bool debugSegFault;
    // Idle CU timeout in ticks
    Tick idleCUTimeout;
    int idleWfs;
    bool functionalTLB;
    bool localMemBarrier;

    /*
     * for Counting page accesses
     */
    bool countPages;

    Shader *shader;

    Tick req_tick_latency;
    Tick resp_tick_latency;

    /**
     * Number of WFs to schedule to each SIMD. This vector is populated
     * by hasDispResources(), and consumed by the subsequent call to
     * dispWorkgroup(), to schedule the specified number of WFs to the
     * SIMD units. Entry I provides the number of WFs to schedule to SIMD I.
     */
    std::vector<int> numWfsToSched;

    // number of currently reserved vector registers per SIMD unit
    std::vector<int> vectorRegsReserved;
    // number of currently reserved scalar registers per SIMD unit
    std::vector<int> scalarRegsReserved;
    // number of vector registers per SIMD unit
    int numVecRegsPerSimd;
    // number of available scalar registers per SIMD unit
    int numScalarRegsPerSimd;

    // this hash map will keep track of page divergence
    // per memory instruction per wavefront. The hash map
    // is cleared in GPUDynInst::updateStats() in gpu_dyn_inst.cc.
    std::map<Addr, int> pagesTouched;

    void insertInPipeMap(Wavefront *w);
    void deleteFromPipeMap(Wavefront *w);

    ComputeUnit(const Params &p);
    ~ComputeUnit();

    // Timing Functions
    int oprNetPipeLength() const { return operandNetworkLength; }
    int simdUnitWidth() const { return simdWidth; }
    int spBypassLength() const { return spBypassPipeLength; }
    int dpBypassLength() const { return dpBypassPipeLength; }
    int scalarPipeLength() const { return scalarPipeStages; }
    int storeBusLength() const { return numCyclesPerStoreTransfer; }
    int loadBusLength() const { return numCyclesPerLoadTransfer; }
    int wfSize() const { return wavefrontSize; }

    void exec();
    void initiateFetch(Wavefront *wavefront);
    void fetch(PacketPtr pkt, Wavefront *wavefront);
    void fillKernelState(Wavefront *w, HSAQueueEntry *task);

    void startWavefront(Wavefront *w, int waveId, LdsChunk *ldsChunk,
                        HSAQueueEntry *task, int bar_id,
                        bool fetchContext=false);

    void doInvalidate(RequestPtr req, int kernId);
    void doFlush(GPUDynInstPtr gpuDynInst);

    void dispWorkgroup(HSAQueueEntry *task, int num_wfs_in_wg);
    bool hasDispResources(HSAQueueEntry *task, int &num_wfs_in_wg);

    int cacheLineSize() const { return _cacheLineSize; }
    int getCacheLineBits() const { return cacheLineBits; }

    void resetRegisterPool();

  private:
    WFBarrier&
    barrierSlot(int bar_id)
    {
        assert(bar_id > WFBarrier::InvalidID);
        return wfBarrierSlots.at(bar_id);
    }

    int
    getFreeBarrierId()
    {
        assert(freeBarrierIds.size());
        auto free_bar_id = freeBarrierIds.begin();
        int bar_id = *free_bar_id;
        freeBarrierIds.erase(free_bar_id);
        return bar_id;
    }

  public:
    int numYetToReachBarrier(int bar_id);
    bool allAtBarrier(int bar_id);
    void incNumAtBarrier(int bar_id);
    int numAtBarrier(int bar_id);
    int maxBarrierCnt(int bar_id);
    void resetBarrier(int bar_id);
    void decMaxBarrierCnt(int bar_id);
    void releaseBarrier(int bar_id);
    void releaseWFsFromBarrier(int bar_id);
    int numBarrierSlots() const { return _numBarrierSlots; }

    template<typename c0, typename c1>
    void doSmReturn(GPUDynInstPtr gpuDynInst);

    virtual void init() override;
    void sendRequest(GPUDynInstPtr gpuDynInst, PortID index, PacketPtr pkt);
    void sendScalarRequest(GPUDynInstPtr gpuDynInst, PacketPtr pkt);
    void injectGlobalMemFence(GPUDynInstPtr gpuDynInst,
                              bool kernelMemSync,
                              RequestPtr req=nullptr);
    void handleMemPacket(PacketPtr pkt, int memport_index);
    bool processTimingPacket(PacketPtr pkt);
    void processFetchReturn(PacketPtr pkt);
    void updatePageDivergenceDist(Addr addr);

    RequestorID requestorId() { return _requestorId; }

    bool isDone() const;
    bool isVectorAluIdle(uint32_t simdId) const;

  protected:
    RequestorID _requestorId;

    LdsState &lds;

  public:
    LdsState &
    getLds() const
    {
        return lds;
    }

    int32_t
    getRefCounter(const uint32_t dispatchId, const uint32_t wgId) const;

    [[nodiscard]] bool sendToLds(GPUDynInstPtr gpuDynInst);

    typedef std::unordered_map<Addr, std::pair<int, int>> pageDataStruct;
    pageDataStruct pageAccesses;

    void exitCallback();

    class GMTokenPort : public TokenRequestPort
    {
      public:
        GMTokenPort(const std::string& name, SimObject *owner,
                    PortID id = InvalidPortID)
            : TokenRequestPort(name, owner, id)
        { }
        ~GMTokenPort() { }

      protected:
        bool recvTimingResp(PacketPtr) { return false; }
        void recvReqRetry() { }
    };

    // Manager for the number of tokens available to this compute unit to
    // send global memory request packets to the coalescer this is only used
    // between global memory pipe and TCP coalescer.
    TokenManager *memPortTokens;
    GMTokenPort gmTokenPort;

    /** Data access Port **/
    class DataPort : public RequestPort
    {
      public:
        DataPort(const std::string &_name, ComputeUnit *_cu, PortID id)
            : RequestPort(_name, _cu, id), computeUnit(_cu) { }

        bool snoopRangeSent;

        struct SenderState : public Packet::SenderState
        {
            GPUDynInstPtr _gpuDynInst;
            PortID port_index;
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

    // Scalar data cache access port
    class ScalarDataPort : public RequestPort
    {
      public:
        ScalarDataPort(const std::string &_name, ComputeUnit *_cu)
            : RequestPort(_name, _cu), computeUnit(_cu)
        {
        }

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

        struct SenderState : public Packet::SenderState
        {
            SenderState(GPUDynInstPtr gpuDynInst,
                        Packet::SenderState *sender_state=nullptr)
                : _gpuDynInst(gpuDynInst), saved(sender_state)
            {
            }

            GPUDynInstPtr _gpuDynInst;
            Packet::SenderState *saved;
        };

        class MemReqEvent : public Event
        {
          private:
            ScalarDataPort &scalarDataPort;
            PacketPtr pkt;

          public:
            MemReqEvent(ScalarDataPort &_scalar_data_port, PacketPtr _pkt)
                : Event(), scalarDataPort(_scalar_data_port), pkt(_pkt)
            {
              setFlags(Event::AutoDelete);
            }

            void process();
            const char *description() const;
        };

        std::deque<PacketPtr> retries;

      private:
        ComputeUnit *computeUnit;
    };

    // Instruction cache access port
    class SQCPort : public RequestPort
    {
      public:
        SQCPort(const std::string &_name, ComputeUnit *_cu)
            : RequestPort(_name, _cu), computeUnit(_cu) { }

        bool snoopRangeSent;

        struct SenderState : public Packet::SenderState
        {
            Wavefront *wavefront;
            Packet::SenderState *saved;
            // kernel id to be used in handling I-Cache invalidate response
            int kernId;

            SenderState(Wavefront *_wavefront, Packet::SenderState
                    *sender_state=nullptr, int _kernId=-1)
                : wavefront(_wavefront), saved(sender_state),
                kernId(_kernId){ }
        };

        std::deque<std::pair<PacketPtr, Wavefront*>> retries;

      protected:
        ComputeUnit *computeUnit;

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
    class DTLBPort : public RequestPort
    {
      public:
        DTLBPort(const std::string &_name, ComputeUnit *_cu, PortID id)
            : RequestPort(_name, _cu, id), computeUnit(_cu),
              stalled(false)
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
            PortID portIndex;

            // constructor used for packets involved in timing accesses
            SenderState(GPUDynInstPtr gpuDynInst, PortID port_index)
                : _gpuDynInst(gpuDynInst), portIndex(port_index) { }

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

    class ScalarDTLBPort : public RequestPort
    {
      public:
        ScalarDTLBPort(const std::string &_name, ComputeUnit *_cu)
            : RequestPort(_name, _cu), computeUnit(_cu), stalled(false)
        {
        }

        struct SenderState : public Packet::SenderState
        {
            SenderState(GPUDynInstPtr gpuDynInst) : _gpuDynInst(gpuDynInst) { }
            GPUDynInstPtr _gpuDynInst;
        };

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override { assert(false); }

        bool isStalled() const { return stalled; }
        void stallPort() { stalled = true; }
        void unstallPort() { stalled = false; }

        std::deque<PacketPtr> retries;

      private:
        ComputeUnit *computeUnit;
        bool stalled;
    };

    class ITLBPort : public RequestPort
    {
      public:
        ITLBPort(const std::string &_name, ComputeUnit *_cu)
            : RequestPort(_name, _cu), computeUnit(_cu), stalled(false) { }


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
    class LDSPort : public RequestPort
    {
      public:
        LDSPort(const std::string &_name, ComputeUnit *_cu)
        : RequestPort(_name, _cu), computeUnit(_cu)
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
    LDSPort ldsPort;

    TokenManager *
    getTokenManager()
    {
        return memPortTokens;
    }

    /** The memory port for SIMD data accesses.
     *  Can be connected to PhysMem for Ruby for timing simulations
     */
    std::vector<DataPort> memPort;
    // port to the TLB hierarchy (i.e., the L1 TLB)
    std::vector<DTLBPort> tlbPort;
    // port to the scalar data cache
    ScalarDataPort scalarDataPort;
    // port to the scalar data TLB
    ScalarDTLBPort scalarDTLBPort;
    // port to the SQC (i.e. the I-cache)
    SQCPort sqcPort;
    // port to the SQC TLB (there's a separate TLB for each I-cache)
    ITLBPort sqcTLBPort;

    Port &
    getPort(const std::string &if_name, PortID idx) override
    {
        if (if_name == "memory_port" && idx < memPort.size()) {
            return memPort[idx];
        } else if (if_name == "translation_port" && idx < tlbPort.size()) {
            return tlbPort[idx];
        } else if (if_name == "scalar_port") {
            return scalarDataPort;
        } else if (if_name == "scalar_tlb_port") {
            return scalarDTLBPort;
        } else if (if_name == "sqc_port") {
            return sqcPort;
        } else if (if_name == "sqc_tlb_port") {
            return sqcTLBPort;
        } else if (if_name == "ldsPort") {
            return ldsPort;
        } else if (if_name == "gmTokenPort") {
            return gmTokenPort;
        } else {
            return ClockedObject::getPort(if_name, idx);
        }
    }

    InstSeqNum getAndIncSeqNum() { return globalSeqNum++; }

  private:
    const int _cacheLineSize;
    const int _numBarrierSlots;
    int cacheLineBits;
    InstSeqNum globalSeqNum;
    int wavefrontSize;

    /**
     * TODO: Update these comments once the pipe stage interface has
     *       been fully refactored.
     *
     * Pipeline stage interfaces.
     *
     * Buffers used to communicate between various pipeline stages
     * List of waves which will be dispatched to
     * each execution resource. An EXREADY implies
     * dispatch list is non-empty and
     * execution unit has something to execute
     * this cycle. Currently, the dispatch list of
     * an execution resource can hold only one wave because
     * an execution resource can execute only one wave in a cycle.
     * dispatchList is used to communicate between schedule
     * and exec stage
     *
     * At a high level, the following intra-/inter-stage communication occurs:
     * SCB to SCH: readyList provides per exec resource list of waves that
     *             passed dependency and readiness checks. If selected by
     *             scheduler, attempt to add wave to schList conditional on
     *             RF support.
     * SCH: schList holds waves that are gathering operands or waiting
     *      for execution resource availability. Once ready, waves are
     *      placed on the dispatchList as candidates for execution. A wave
     *      may spend multiple cycles in SCH stage, on the schList due to
     *      RF access conflicts or execution resource contention.
     * SCH to EX: dispatchList holds waves that are ready to be executed.
     *            LM/FLAT arbitration may remove an LM wave and place it
     *            back on the schList. RF model may also force a wave back
     *            to the schList if using the detailed model.
     */
    ScoreboardCheckToSchedule scoreboardCheckToSchedule;
    ScheduleToExecute scheduleToExecute;

    /**
     * The barrier slots for this CU.
     */
    std::vector<WFBarrier> wfBarrierSlots;
    /**
     * A set used to easily retrieve a free barrier ID.
     */
    std::unordered_set<int> freeBarrierIds;

    // hold the time of the arrival of the first cache block related to
    // a particular GPUDynInst. This is used to calculate the difference
    // between the first and last chace block arrival times.
    std::unordered_map<GPUDynInstPtr, Tick> headTailMap;

  public:
    void updateInstStats(GPUDynInstPtr gpuDynInst);
    int activeWaves;

    struct ComputeUnitStats : public statistics::Group
    {
        ComputeUnitStats(statistics::Group *parent, int n_wf);

        statistics::Scalar vALUInsts;
        statistics::Formula vALUInstsPerWF;
        statistics::Scalar sALUInsts;
        statistics::Formula sALUInstsPerWF;
        statistics::Scalar instCyclesVALU;
        statistics::Scalar instCyclesSALU;
        statistics::Scalar threadCyclesVALU;
        statistics::Formula vALUUtilization;
        statistics::Scalar ldsNoFlatInsts;
        statistics::Formula ldsNoFlatInstsPerWF;
        statistics::Scalar flatVMemInsts;
        statistics::Formula flatVMemInstsPerWF;
        statistics::Scalar flatLDSInsts;
        statistics::Formula flatLDSInstsPerWF;
        statistics::Scalar vectorMemWrites;
        statistics::Formula vectorMemWritesPerWF;
        statistics::Scalar vectorMemReads;
        statistics::Formula vectorMemReadsPerWF;
        statistics::Scalar scalarMemWrites;
        statistics::Formula scalarMemWritesPerWF;
        statistics::Scalar scalarMemReads;
        statistics::Formula scalarMemReadsPerWF;

        statistics::Formula vectorMemReadsPerKiloInst;
        statistics::Formula vectorMemWritesPerKiloInst;
        statistics::Formula vectorMemInstsPerKiloInst;
        statistics::Formula scalarMemReadsPerKiloInst;
        statistics::Formula scalarMemWritesPerKiloInst;
        statistics::Formula scalarMemInstsPerKiloInst;

        // Cycles required to send register source (addr and data) from
        // register files to memory pipeline, per SIMD.
        statistics::Vector instCyclesVMemPerSimd;
        statistics::Vector instCyclesScMemPerSimd;
        statistics::Vector instCyclesLdsPerSimd;

        statistics::Scalar globalReads;
        statistics::Scalar globalWrites;
        statistics::Formula globalMemInsts;
        statistics::Scalar argReads;
        statistics::Scalar argWrites;
        statistics::Formula argMemInsts;
        statistics::Scalar spillReads;
        statistics::Scalar spillWrites;
        statistics::Formula spillMemInsts;
        statistics::Scalar groupReads;
        statistics::Scalar groupWrites;
        statistics::Formula groupMemInsts;
        statistics::Scalar privReads;
        statistics::Scalar privWrites;
        statistics::Formula privMemInsts;
        statistics::Scalar readonlyReads;
        statistics::Scalar readonlyWrites;
        statistics::Formula readonlyMemInsts;
        statistics::Scalar kernargReads;
        statistics::Scalar kernargWrites;
        statistics::Formula kernargMemInsts;

        statistics::Distribution waveLevelParallelism;

        // the following stats compute the avg. TLB accesslatency per
        // uncoalesced request (only for data)
        statistics::Scalar tlbRequests;
        statistics::Scalar tlbCycles;
        statistics::Formula tlbLatency;
        // hitsPerTLBLevel[x] are the hits in Level x TLB.
        // x = 0 is the page table.
        statistics::Vector hitsPerTLBLevel;

        statistics::Scalar ldsBankAccesses;
        statistics::Distribution ldsBankConflictDist;

        // over all memory instructions executed over all wavefronts
        // how many touched 0-4 pages, 4-8, ..., 60-64 pages
        statistics::Distribution pageDivergenceDist;
        // count of non-flat global memory vector instructions executed
        statistics::Scalar dynamicGMemInstrCnt;
        // count of flat global memory vector instructions executed
        statistics::Scalar dynamicFlatMemInstrCnt;
        statistics::Scalar dynamicLMemInstrCnt;

        statistics::Scalar wgBlockedDueBarrierAllocation;
        statistics::Scalar wgBlockedDueLdsAllocation;
        // Number of instructions executed, i.e. if 64 (or 32 or 7) lanes are
        // active when the instruction is committed, this number is still
        // incremented by 1
        statistics::Scalar numInstrExecuted;
        // Number of cycles among successive instruction executions across all
        // wavefronts of the same CU
        statistics::Distribution execRateDist;
        // number of individual vector operations executed
        statistics::Scalar numVecOpsExecuted;
        // number of individual f16 vector operations executed
        statistics::Scalar numVecOpsExecutedF16;
        // number of individual f32 vector operations executed
        statistics::Scalar numVecOpsExecutedF32;
        // number of individual f64 vector operations executed
        statistics::Scalar numVecOpsExecutedF64;
        // number of individual FMA 16,32,64 vector operations executed
        statistics::Scalar numVecOpsExecutedFMA16;
        statistics::Scalar numVecOpsExecutedFMA32;
        statistics::Scalar numVecOpsExecutedFMA64;
        // number of individual MAC 16,32,64 vector operations executed
        statistics::Scalar numVecOpsExecutedMAC16;
        statistics::Scalar numVecOpsExecutedMAC32;
        statistics::Scalar numVecOpsExecutedMAC64;
        // number of individual MAD 16,32,64 vector operations executed
        statistics::Scalar numVecOpsExecutedMAD16;
        statistics::Scalar numVecOpsExecutedMAD32;
        statistics::Scalar numVecOpsExecutedMAD64;
        // total number of two op FP vector operations executed
        statistics::Scalar numVecOpsExecutedTwoOpFP;
        // Total cycles that something is running on the GPU
        statistics::Scalar totalCycles;
        statistics::Formula vpc; // vector ops per cycle
        statistics::Formula vpc_f16; // vector ops per cycle
        statistics::Formula vpc_f32; // vector ops per cycle
        statistics::Formula vpc_f64; // vector ops per cycle
        statistics::Formula ipc; // vector instructions per cycle
        statistics::Distribution controlFlowDivergenceDist;
        statistics::Distribution activeLanesPerGMemInstrDist;
        statistics::Distribution activeLanesPerLMemInstrDist;
        // number of vector ALU instructions received
        statistics::Formula numALUInstsExecuted;
        // number of times a WG cannot start due to lack of free VGPRs in SIMDs
        statistics::Scalar numTimesWgBlockedDueVgprAlloc;
        // number of times a WG cannot start due to lack of free SGPRs in SIMDs
        statistics::Scalar numTimesWgBlockedDueSgprAlloc;
        statistics::Scalar numCASOps;
        statistics::Scalar numFailedCASOps;
        statistics::Scalar completedWfs;
        statistics::Scalar completedWGs;

        // distrubtion in latency difference between first and last cache block
        // arrival ticks
        statistics::Distribution headTailLatency;

        // Track the amount of interleaving between wavefronts on each SIMD.
        // This stat is sampled using instExecPerSimd to compute the number
        // of instructions that have been executed on a SIMD between a WF
        // executing two successive instructions.
        statistics::VectorDistribution instInterleave;
    } stats;
};

} // namespace gem5

#endif // __COMPUTE_UNIT_HH__
