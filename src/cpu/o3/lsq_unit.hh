/*
 * Copyright (c) 2012-2014,2017-2018 ARM Limited
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *          Korey Sewell
 */

#ifndef __CPU_O3_LSQ_UNIT_HH__
#define __CPU_O3_LSQ_UNIT_HH__

#include <algorithm>
#include <cstring>
#include <map>
#include <queue>

#include "arch/generic/debugfaults.hh"
#include "arch/generic/vec_reg.hh"
#include "arch/isa_traits.hh"
#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "cpu/timebuf.hh"
#include "debug/LSQUnit.hh"
#include "mem/packet.hh"
#include "mem/port.hh"

struct DerivO3CPUParams;
#include "base/circular_queue.hh"

/**
 * Class that implements the actual LQ and SQ for each specific
 * thread.  Both are circular queues; load entries are freed upon
 * committing, while store entries are freed once they writeback. The
 * LSQUnit tracks if there are memory ordering violations, and also
 * detects partial load to store forwarding cases (a store only has
 * part of a load's data) that requires the load to wait until the
 * store writes back. In the former case it holds onto the instruction
 * until the dependence unit looks at it, and in the latter it stalls
 * the LSQ until the store writes back. At that point the load is
 * replayed.
 */
template <class Impl>
class LSQUnit
{
  public:
    static constexpr auto MaxDataBytes = MaxVecRegLenInBytes;

    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::LSQ LSQ;
    typedef typename Impl::CPUPol::IssueStruct IssueStruct;

    using LSQSenderState = typename LSQ::LSQSenderState;
    using LSQRequest = typename Impl::CPUPol::LSQ::LSQRequest;
  private:
    class LSQEntry
    {
      private:
        /** The instruction. */
        DynInstPtr inst;
        /** The request. */
        LSQRequest* req;
        /** The size of the operation. */
        uint32_t _size;
        /** Valid entry. */
        bool _valid;
      public:
        /** Constructs an empty store queue entry. */
        LSQEntry()
            : inst(nullptr), req(nullptr), _size(0), _valid(false)
        {
        }

        ~LSQEntry()
        {
            inst = nullptr;
            if (req != nullptr) {
                req->freeLSQEntry();
                req = nullptr;
            }
        }

        void
        clear()
        {
            inst = nullptr;
            if (req != nullptr) {
                req->freeLSQEntry();
            }
            req = nullptr;
            _valid = false;
            _size = 0;
        }

        void
        set(const DynInstPtr& inst)
        {
            assert(!_valid);
            this->inst = inst;
            _valid = true;
            _size = 0;
        }
        LSQRequest* request() { return req; }
        void setRequest(LSQRequest* r) { req = r; }
        bool hasRequest() { return req != nullptr; }
        /** Member accessors. */
        /** @{ */
        bool valid() const { return _valid; }
        uint32_t& size() { return _size; }
        const uint32_t& size() const { return _size; }
        const DynInstPtr& instruction() const { return inst; }
        /** @} */
    };

    class SQEntry : public LSQEntry
    {
      private:
        /** The store data. */
        char _data[MaxDataBytes];
        /** Whether or not the store can writeback. */
        bool _canWB;
        /** Whether or not the store is committed. */
        bool _committed;
        /** Whether or not the store is completed. */
        bool _completed;
        /** Does this request write all zeros and thus doesn't
         * have any data attached to it. Used for cache block zero
         * style instructs (ARM DC ZVA; ALPHA WH64)
         */
        bool _isAllZeros;
      public:
        static constexpr size_t DataSize = sizeof(_data);
        /** Constructs an empty store queue entry. */
        SQEntry()
            : _canWB(false), _committed(false), _completed(false),
              _isAllZeros(false)
        {
            std::memset(_data, 0, DataSize);
        }

        ~SQEntry()
        {
        }

        void
        set(const DynInstPtr& inst)
        {
            LSQEntry::set(inst);
        }

        void
        clear()
        {
            LSQEntry::clear();
            _canWB = _completed = _committed = _isAllZeros = false;
        }
        /** Member accessors. */
        /** @{ */
        bool& canWB() { return _canWB; }
        const bool& canWB() const { return _canWB; }
        bool& completed() { return _completed; }
        const bool& completed() const { return _completed; }
        bool& committed() { return _committed; }
        const bool& committed() const { return _committed; }
        bool& isAllZeros() { return _isAllZeros; }
        const bool& isAllZeros() const { return _isAllZeros; }
        char* data() { return _data; }
        const char* data() const { return _data; }
        /** @} */
    };
    using LQEntry = LSQEntry;

  public:
    using LoadQueue = CircularQueue<LQEntry>;
    using StoreQueue = CircularQueue<SQEntry>;

  public:
    /** Constructs an LSQ unit. init() must be called prior to use. */
    LSQUnit(uint32_t lqEntries, uint32_t sqEntries);

    /** We cannot copy LSQUnit because it has stats for which copy
     * contructor is deleted explicitly. However, STL vector requires
     * a valid copy constructor for the base type at compile time.
     */
    LSQUnit(const LSQUnit &l) { panic("LSQUnit is not copy-able"); }

    /** Initializes the LSQ unit with the specified number of entries. */
    void init(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params,
            LSQ *lsq_ptr, unsigned id);

    /** Returns the name of the LSQ unit. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets the pointer to the dcache port. */
    void setDcachePort(MasterPort *dcache_port);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Inserts an instruction. */
    void insert(const DynInstPtr &inst);
    /** Inserts a load instruction. */
    void insertLoad(const DynInstPtr &load_inst);
    /** Inserts a store instruction. */
    void insertStore(const DynInstPtr &store_inst);

    /** Check for ordering violations in the LSQ. For a store squash if we
     * ever find a conflicting load. For a load, only squash if we
     * an external snoop invalidate has been seen for that load address
     * @param load_idx index to start checking at
     * @param inst the instruction to check
     */
    Fault checkViolations(typename LoadQueue::iterator& loadIt,
            const DynInstPtr& inst);

    /** Check if an incoming invalidate hits in the lsq on a load
     * that might have issued out of order wrt another load beacuse
     * of the intermediate invalidate.
     */
    void checkSnoop(PacketPtr pkt);

    /** Executes a load instruction. */
    Fault executeLoad(const DynInstPtr &inst);

    Fault executeLoad(int lq_idx) { panic("Not implemented"); return NoFault; }
    /** Executes a store instruction. */
    Fault executeStore(const DynInstPtr &inst);

    /** Commits the head load. */
    void commitLoad();
    /** Commits loads older than a specific sequence number. */
    void commitLoads(InstSeqNum &youngest_inst);

    /** Commits stores older than a specific sequence number. */
    void commitStores(InstSeqNum &youngest_inst);

    /** Writes back stores. */
    void writebackStores();

    /** Completes the data access that has been returned from the
     * memory system. */
    void completeDataAccess(PacketPtr pkt);

    /** Squashes all instructions younger than a specific sequence number. */
    void squash(const InstSeqNum &squashed_num);

    /** Returns if there is a memory ordering violation. Value is reset upon
     * call to getMemDepViolator().
     */
    bool violation() { return memDepViolator; }

    /** Returns the memory ordering violator. */
    DynInstPtr getMemDepViolator();

    /** Returns the number of free LQ entries. */
    unsigned numFreeLoadEntries();

    /** Returns the number of free SQ entries. */
    unsigned numFreeStoreEntries();

    /** Returns the number of loads in the LQ. */
    int numLoads() { return loads; }

    /** Returns the number of stores in the SQ. */
    int numStores() { return stores; }

    /** Returns if either the LQ or SQ is full. */
    bool isFull() { return lqFull() || sqFull(); }

    /** Returns if both the LQ and SQ are empty. */
    bool isEmpty() const { return lqEmpty() && sqEmpty(); }

    /** Returns if the LQ is full. */
    bool lqFull() { return loadQueue.full(); }

    /** Returns if the SQ is full. */
    bool sqFull() { return storeQueue.full(); }

    /** Returns if the LQ is empty. */
    bool lqEmpty() const { return loads == 0; }

    /** Returns if the SQ is empty. */
    bool sqEmpty() const { return stores == 0; }

    /** Returns the number of instructions in the LSQ. */
    unsigned getCount() { return loads + stores; }

    /** Returns if there are any stores to writeback. */
    bool hasStoresToWB() { return storesToWB; }

    /** Returns the number of stores to writeback. */
    int numStoresToWB() { return storesToWB; }

    /** Returns if the LSQ unit will writeback on this cycle. */
    bool
    willWB()
    {
        return storeWBIt.dereferenceable() &&
                        storeWBIt->valid() &&
                        storeWBIt->canWB() &&
                        !storeWBIt->completed() &&
                        !isStoreBlocked;
    }

    /** Handles doing the retry. */
    void recvRetry();

    unsigned int cacheLineSize();
  private:
    /** Reset the LSQ state */
    void resetState();

    /** Writes back the instruction, sending it to IEW. */
    void writeback(const DynInstPtr &inst, PacketPtr pkt);

    /** Try to finish a previously blocked write back attempt */
    void writebackBlockedStore();

    /** Completes the store at the specified index. */
    void completeStore(typename StoreQueue::iterator store_idx);

    /** Handles completing the send of a store to memory. */
    void storePostSend();

  public:
    /** Attempts to send a packet to the cache.
     * Check if there are ports available. Return true if
     * there are, false if there are not.
     */
    bool trySendPacket(bool isLoad, PacketPtr data_pkt);


    /** Debugging function to dump instructions in the LSQ. */
    void dumpInsts() const;

    /** Schedule event for the cpu. */
    void schedule(Event& ev, Tick when) { cpu->schedule(ev, when); }

    BaseTLB* dTLB() { return cpu->dtb; }

  private:
    /** Pointer to the CPU. */
    O3CPU *cpu;

    /** Pointer to the IEW stage. */
    IEW *iewStage;

    /** Pointer to the LSQ. */
    LSQ *lsq;

    /** Pointer to the dcache port.  Used only for sending. */
    MasterPort *dcachePort;

    /** Particularisation of the LSQSenderState to the LQ. */
    class LQSenderState : public LSQSenderState
    {
        using LSQSenderState::alive;
      public:
        LQSenderState(typename LoadQueue::iterator idx_)
            : LSQSenderState(idx_->request(), true), idx(idx_) { }

        /** The LQ index of the instruction. */
        typename LoadQueue::iterator idx;
        //virtual LSQRequest* request() { return idx->request(); }
        virtual void
        complete()
        {
            //if (alive())
            //  idx->request()->senderState(nullptr);
        }
    };

    /** Particularisation of the LSQSenderState to the SQ. */
    class SQSenderState : public LSQSenderState
    {
        using LSQSenderState::alive;
      public:
        SQSenderState(typename StoreQueue::iterator idx_)
            : LSQSenderState(idx_->request(), false), idx(idx_) { }
        /** The SQ index of the instruction. */
        typename StoreQueue::iterator idx;
        //virtual LSQRequest* request() { return idx->request(); }
        virtual void
        complete()
        {
            //if (alive())
            //   idx->request()->senderState(nullptr);
        }
    };

    /** Writeback event, specifically for when stores forward data to loads. */
    class WritebackEvent : public Event
    {
      public:
        /** Constructs a writeback event. */
        WritebackEvent(const DynInstPtr &_inst, PacketPtr pkt,
                LSQUnit *lsq_ptr);

        /** Processes the writeback event. */
        void process();

        /** Returns the description of this event. */
        const char *description() const;

      private:
        /** Instruction whose results are being written back. */
        DynInstPtr inst;

        /** The packet that would have been sent to memory. */
        PacketPtr pkt;

        /** The pointer to the LSQ unit that issued the store. */
        LSQUnit<Impl> *lsqPtr;
    };

  public:
    /**
     * Handles writing back and completing the load or store that has
     * returned from memory.
     *
     * @param pkt Response packet from the memory sub-system
     */
    bool recvTimingResp(PacketPtr pkt);

  private:
    /** The LSQUnit thread id. */
    ThreadID lsqID;
  public:
    /** The store queue. */
    CircularQueue<SQEntry> storeQueue;

    /** The load queue. */
    LoadQueue loadQueue;

  private:
    /** The number of places to shift addresses in the LSQ before checking
     * for dependency violations
     */
    unsigned depCheckShift;

    /** Should loads be checked for dependency issues */
    bool checkLoads;

    /** The number of load instructions in the LQ. */
    int loads;
    /** The number of store instructions in the SQ. */
    int stores;
    /** The number of store instructions in the SQ waiting to writeback. */
    int storesToWB;

    /** The index of the first instruction that may be ready to be
     * written back, and has not yet been written back.
     */
    typename StoreQueue::iterator storeWBIt;

    /** Address Mask for a cache block (e.g. ~(cache_block_size-1)) */
    Addr cacheBlockMask;

    /** Wire to read information from the issue stage time queue. */
    typename TimeBuffer<IssueStruct>::wire fromIssue;

    /** Whether or not the LSQ is stalled. */
    bool stalled;
    /** The store that causes the stall due to partial store to load
     * forwarding.
     */
    InstSeqNum stallingStoreIsn;
    /** The index of the above store. */
    int stallingLoadIdx;

    /** The packet that needs to be retried. */
    PacketPtr retryPkt;

    /** Whehter or not a store is blocked due to the memory system. */
    bool isStoreBlocked;

    /** Whether or not a store is in flight. */
    bool storeInFlight;

    /** The oldest load that caused a memory ordering violation. */
    DynInstPtr memDepViolator;

    /** Whether or not there is a packet that couldn't be sent because of
     * a lack of cache ports. */
    bool hasPendingRequest;

    /** The packet that is pending free cache ports. */
    LSQRequest* pendingRequest;

    /** Flag for memory model. */
    bool needsTSO;

    // Will also need how many read/write ports the Dcache has.  Or keep track
    // of that in stage that is one level up, and only call executeLoad/Store
    // the appropriate number of times.
    /** Total number of loads forwaded from LSQ stores. */
    Stats::Scalar lsqForwLoads;

    /** Total number of loads ignored due to invalid addresses. */
    Stats::Scalar invAddrLoads;

    /** Total number of squashed loads. */
    Stats::Scalar lsqSquashedLoads;

    /** Total number of responses from the memory system that are
     * ignored due to the instruction already being squashed. */
    Stats::Scalar lsqIgnoredResponses;

    /** Tota number of memory ordering violations. */
    Stats::Scalar lsqMemOrderViolation;

    /** Total number of squashed stores. */
    Stats::Scalar lsqSquashedStores;

    /** Total number of software prefetches ignored due to invalid addresses. */
    Stats::Scalar invAddrSwpfs;

    /** Ready loads blocked due to partial store-forwarding. */
    Stats::Scalar lsqBlockedLoads;

    /** Number of loads that were rescheduled. */
    Stats::Scalar lsqRescheduledLoads;

    /** Number of times the LSQ is blocked due to the cache. */
    Stats::Scalar lsqCacheBlocked;

  public:
    /** Executes the load at the given index. */
    Fault read(LSQRequest *req, int load_idx);

    /** Executes the store at the given index. */
    Fault write(LSQRequest *req, uint8_t *data, int store_idx);

    /** Returns the index of the head load instruction. */
    int getLoadHead() { return loadQueue.head(); }

    /** Returns the sequence number of the head load instruction. */
    InstSeqNum
    getLoadHeadSeqNum()
    {
        return loadQueue.front().valid()
            ? loadQueue.front().instruction()->seqNum
            : 0;
    }

    /** Returns the index of the head store instruction. */
    int getStoreHead() { return storeQueue.head(); }
    /** Returns the sequence number of the head store instruction. */
    InstSeqNum
    getStoreHeadSeqNum()
    {
        return storeQueue.front().valid()
            ? storeQueue.front().instruction()->seqNum
            : 0;
    }

    /** Returns whether or not the LSQ unit is stalled. */
    bool isStalled()  { return stalled; }
  public:
    typedef typename CircularQueue<LQEntry>::iterator LQIterator;
    typedef typename CircularQueue<SQEntry>::iterator SQIterator;
    typedef CircularQueue<LQEntry> LQueue;
    typedef CircularQueue<SQEntry> SQueue;
};

template <class Impl>
Fault
LSQUnit<Impl>::read(LSQRequest *req, int load_idx)
{
    LQEntry& load_req = loadQueue[load_idx];
    const DynInstPtr& load_inst = load_req.instruction();

    load_req.setRequest(req);
    assert(load_inst);

    assert(!load_inst->isExecuted());

    // Make sure this isn't a strictly ordered load
    // A bit of a hackish way to get strictly ordered accesses to work
    // only if they're at the head of the LSQ and are ready to commit
    // (at the head of the ROB too).

    if (req->mainRequest()->isStrictlyOrdered() &&
        (load_idx != loadQueue.head() || !load_inst->isAtCommit())) {
        // Tell IQ/mem dep unit that this instruction will need to be
        // rescheduled eventually
        iewStage->rescheduleMemInst(load_inst);
        load_inst->clearIssued();
        load_inst->effAddrValid(false);
        ++lsqRescheduledLoads;
        DPRINTF(LSQUnit, "Strictly ordered load [sn:%lli] PC %s\n",
                load_inst->seqNum, load_inst->pcState());

        // Must delete request now that it wasn't handed off to
        // memory.  This is quite ugly.  @todo: Figure out the proper
        // place to really handle request deletes.
        load_req.setRequest(nullptr);
        req->discard();
        return std::make_shared<GenericISA::M5PanicFault>(
            "Strictly ordered load [sn:%llx] PC %s\n",
            load_inst->seqNum, load_inst->pcState());
    }

    DPRINTF(LSQUnit, "Read called, load idx: %i, store idx: %i, "
            "storeHead: %i addr: %#x%s\n",
            load_idx - 1, load_inst->sqIt._idx, storeQueue.head() - 1,
            req->mainRequest()->getPaddr(), req->isSplit() ? " split" : "");

    if (req->mainRequest()->isLLSC()) {
        // Disable recording the result temporarily.  Writing to misc
        // regs normally updates the result, but this is not the
        // desired behavior when handling store conditionals.
        load_inst->recordResult(false);
        TheISA::handleLockedRead(load_inst.get(), req->mainRequest());
        load_inst->recordResult(true);
    }

    if (req->mainRequest()->isMmappedIpr()) {
        assert(!load_inst->memData);
        load_inst->memData = new uint8_t[MaxDataBytes];

        ThreadContext *thread = cpu->tcBase(lsqID);
        PacketPtr main_pkt = new Packet(req->mainRequest(), MemCmd::ReadReq);

        main_pkt->dataStatic(load_inst->memData);

        Cycles delay = req->handleIprRead(thread, main_pkt);

        WritebackEvent *wb = new WritebackEvent(load_inst, main_pkt, this);
        cpu->schedule(wb, cpu->clockEdge(delay));
        return NoFault;
    }

    // Check the SQ for any previous stores that might lead to forwarding
    auto store_it = load_inst->sqIt;
    assert (store_it >= storeWBIt);
    // End once we've reached the top of the LSQ
    while (store_it != storeWBIt) {
        // Move the index to one younger
        store_it--;
        assert(store_it->valid());
        assert(store_it->instruction()->seqNum < load_inst->seqNum);
        int store_size = store_it->size();

        // Cache maintenance instructions go down via the store
        // path but they carry no data and they shouldn't be
        // considered for forwarding
        if (store_size != 0 && !store_it->instruction()->strictlyOrdered() &&
            !(store_it->request()->mainRequest() &&
              store_it->request()->mainRequest()->isCacheMaintenance())) {
            assert(store_it->instruction()->effAddrValid());

            // Check if the store data is within the lower and upper bounds of
            // addresses that the request needs.
            auto req_s = req->mainRequest()->getVaddr();
            auto req_e = req_s + req->mainRequest()->getSize();
            auto st_s = store_it->instruction()->effAddr;
            auto st_e = st_s + store_size;

            bool store_has_lower_limit = req_s >= st_s;
            bool store_has_upper_limit = req_e <= st_e;
            bool lower_load_has_store_part = req_s < st_e;
            bool upper_load_has_store_part = req_e > st_s;

            // If the store entry is not atomic (atomic does not have valid
            // data), the store has all of the data needed, and
            // the load is not LLSC, then
            // we can forward data from the store to the load
            if (!store_it->instruction()->isAtomic() &&
                store_has_lower_limit && store_has_upper_limit &&
                !req->mainRequest()->isLLSC()) {

                // Get shift amount for offset into the store's data.
                int shift_amt = req->mainRequest()->getVaddr() -
                    store_it->instruction()->effAddr;

                // Allocate memory if this is the first time a load is issued.
                if (!load_inst->memData) {
                    load_inst->memData =
                        new uint8_t[req->mainRequest()->getSize()];
                }
                if (store_it->isAllZeros())
                    memset(load_inst->memData, 0,
                            req->mainRequest()->getSize());
                else
                    memcpy(load_inst->memData,
                        store_it->data() + shift_amt,
                        req->mainRequest()->getSize());

                DPRINTF(LSQUnit, "Forwarding from store idx %i to load to "
                        "addr %#x\n", store_it._idx,
                        req->mainRequest()->getVaddr());

                PacketPtr data_pkt = new Packet(req->mainRequest(),
                        MemCmd::ReadReq);
                data_pkt->dataStatic(load_inst->memData);

                if (req->isAnyOutstandingRequest()) {
                    assert(req->_numOutstandingPackets > 0);
                    // There are memory requests packets in flight already.
                    // This may happen if the store was not complete the
                    // first time this load got executed. Signal the senderSate
                    // that response packets should be discarded.
                    req->discardSenderState();
                }

                WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt,
                        this);

                // We'll say this has a 1 cycle load-store forwarding latency
                // for now.
                // @todo: Need to make this a parameter.
                cpu->schedule(wb, curTick());

                // Don't need to do anything special for split loads.
                ++lsqForwLoads;

                return NoFault;
            } else if (
                // This is the partial store-load forwarding case where a store
                // has only part of the load's data and the load isn't LLSC
                (!req->mainRequest()->isLLSC() &&
                 ((store_has_lower_limit && lower_load_has_store_part) ||
                  (store_has_upper_limit && upper_load_has_store_part) ||
                  (lower_load_has_store_part && upper_load_has_store_part))) ||
                // The load is LLSC, and the store has all or part of the
                // load's data
                (req->mainRequest()->isLLSC() &&
                 ((store_has_lower_limit || upper_load_has_store_part) &&
                  (store_has_upper_limit || lower_load_has_store_part))) ||
                // The store entry is atomic and has all or part of the load's
                // data
                (store_it->instruction()->isAtomic() &&
                 ((store_has_lower_limit || upper_load_has_store_part) &&
                  (store_has_upper_limit || lower_load_has_store_part)))) {

                // If it's already been written back, then don't worry about
                // stalling on it.
                if (store_it->completed()) {
                    panic("Should not check one of these");
                    continue;
                }

                // Must stall load and force it to retry, so long as it's the
                // oldest load that needs to do so.
                if (!stalled ||
                    (stalled &&
                     load_inst->seqNum <
                     loadQueue[stallingLoadIdx].instruction()->seqNum)) {
                    stalled = true;
                    stallingStoreIsn = store_it->instruction()->seqNum;
                    stallingLoadIdx = load_idx;
                }

                // Tell IQ/mem dep unit that this instruction will need to be
                // rescheduled eventually
                iewStage->rescheduleMemInst(load_inst);
                load_inst->clearIssued();
                load_inst->effAddrValid(false);
                ++lsqRescheduledLoads;

                // Do not generate a writeback event as this instruction is not
                // complete.
                DPRINTF(LSQUnit, "Load-store forwarding mis-match. "
                        "Store idx %i to load addr %#x\n",
                        store_it._idx, req->mainRequest()->getVaddr());

                // Must discard the request.
                req->discard();
                load_req.setRequest(nullptr);
                return NoFault;
            }
        }
    }

    // If there's no forwarding case, then go access memory
    DPRINTF(LSQUnit, "Doing memory access for inst [sn:%lli] PC %s\n",
            load_inst->seqNum, load_inst->pcState());

    // Allocate memory if this is the first time a load is issued.
    if (!load_inst->memData) {
        load_inst->memData = new uint8_t[req->mainRequest()->getSize()];
    }

    // For now, load throughput is constrained by the number of
    // load FUs only, and loads do not consume a cache port (only
    // stores do).
    // @todo We should account for cache port contention
    // and arbitrate between loads and stores.

    // if we the cache is not blocked, do cache access
    if (req->senderState() == nullptr) {
        LQSenderState *state = new LQSenderState(
                loadQueue.getIterator(load_idx));
        state->isLoad = true;
        state->inst = load_inst;
        state->isSplit = req->isSplit();
        req->senderState(state);
    }
    req->buildPackets();
    req->sendPacketToCache();
    if (!req->isSent())
        iewStage->blockMemInst(load_inst);

    return NoFault;
}

template <class Impl>
Fault
LSQUnit<Impl>::write(LSQRequest *req, uint8_t *data, int store_idx)
{
    assert(storeQueue[store_idx].valid());

    DPRINTF(LSQUnit, "Doing write to store idx %i, addr %#x | storeHead:%i "
            "[sn:%llu]\n",
            store_idx - 1, req->request()->getPaddr(), storeQueue.head() - 1,
            storeQueue[store_idx].instruction()->seqNum);

    storeQueue[store_idx].setRequest(req);
    unsigned size = req->_size;
    storeQueue[store_idx].size() = size;
    bool store_no_data =
        req->mainRequest()->getFlags() & Request::STORE_NO_DATA;
    storeQueue[store_idx].isAllZeros() = store_no_data;
    assert(size <= SQEntry::DataSize || store_no_data);

    // copy data into the storeQueue only if the store request has valid data
    if (!(req->request()->getFlags() & Request::CACHE_BLOCK_ZERO) &&
        !req->request()->isCacheMaintenance() &&
        !req->request()->isAtomic())
        memcpy(storeQueue[store_idx].data(), data, size);

    // This function only writes the data to the store queue, so no fault
    // can happen here.
    return NoFault;
}

#endif // __CPU_O3_LSQ_UNIT_HH__
