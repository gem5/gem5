/*
 * Copyright (c) 2012-2014,2017-2018,2020-2021 ARM Limited
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
 */

#ifndef __CPU_O3_LSQ_UNIT_HH__
#define __CPU_O3_LSQ_UNIT_HH__

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <queue>

#include "arch/generic/debugfaults.hh"
#include "arch/generic/vec_reg.hh"
#include "base/circular_queue.hh"
#include "cpu/base.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/lsq.hh"
#include "cpu/timebuf.hh"
#include "debug/HtmCpu.hh"
#include "debug/LSQUnit.hh"
#include "mem/packet.hh"
#include "mem/port.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class IEW;

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
class LSQUnit
{
  public:
    static constexpr auto MaxDataBytes = MaxVecRegLenInBytes;

    using LSQRequest = LSQ::LSQRequest;
  private:
    class LSQEntry
    {
      private:
        /** The instruction. */
        DynInstPtr _inst;
        /** The request. */
        LSQRequest* _request = nullptr;
        /** The size of the operation. */
        uint32_t _size = 0;
        /** Valid entry. */
        bool _valid = false;

      public:
        ~LSQEntry()
        {
            if (_request != nullptr) {
                _request->freeLSQEntry();
                _request = nullptr;
            }
        }

        void
        clear()
        {
            _inst = nullptr;
            if (_request != nullptr) {
                _request->freeLSQEntry();
            }
            _request = nullptr;
            _valid = false;
            _size = 0;
        }

        void
        set(const DynInstPtr& new_inst)
        {
            assert(!_valid);
            _inst = new_inst;
            _valid = true;
            _size = 0;
        }

        LSQRequest* request() { return _request; }
        void setRequest(LSQRequest* r) { _request = r; }
        bool hasRequest() { return _request != nullptr; }
        /** Member accessors. */
        /** @{ */
        bool valid() const { return _valid; }
        uint32_t& size() { return _size; }
        const uint32_t& size() const { return _size; }
        const DynInstPtr& instruction() const { return _inst; }
        /** @} */
    };

    class SQEntry : public LSQEntry
    {
      private:
        /** The store data. */
        char _data[MaxDataBytes];
        /** Whether or not the store can writeback. */
        bool _canWB = false;
        /** Whether or not the store is committed. */
        bool _committed = false;
        /** Whether or not the store is completed. */
        bool _completed = false;
        /** Does this request write all zeros and thus doesn't
         * have any data attached to it. Used for cache block zero
         * style instructs (ARM DC ZVA; ALPHA WH64)
         */
        bool _isAllZeros = false;

      public:
        static constexpr size_t DataSize = sizeof(_data);
        /** Constructs an empty store queue entry. */
        SQEntry()
        {
            std::memset(_data, 0, DataSize);
        }

        void set(const DynInstPtr& inst) { LSQEntry::set(inst); }

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

    /** Coverage of one address range with another */
    enum class AddrRangeCoverage
    {
        PartialAddrRangeCoverage, /* Two ranges partly overlap */
        FullAddrRangeCoverage, /* One range fully covers another */
        NoAddrRangeCoverage /* Two ranges are disjoint */
    };

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
    LSQUnit(const LSQUnit &l): stats(nullptr)
    {
        panic("LSQUnit is not copy-able");
    }

    /** Initializes the LSQ unit with the specified number of entries. */
    void init(CPU *cpu_ptr, IEW *iew_ptr, const BaseO3CPUParams &params,
            LSQ *lsq_ptr, unsigned id);

    /** Returns the name of the LSQ unit. */
    std::string name() const;

    /** Sets the pointer to the dcache port. */
    void setDcachePort(RequestPort *dcache_port);

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

    //[shadow]
    void updateLoadSafeState();

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
    int numLoads() { return loadQueue.size(); }

    /** Returns the number of stores in the SQ. */
    int numStores() { return storeQueue.size(); }

    // hardware transactional memory
    int numHtmStarts() const { return htmStarts; }
    int numHtmStops() const { return htmStops; }
    void resetHtmStartsStops() { htmStarts = htmStops = 0; }
    uint64_t getLatestHtmUid() const;
    void
    setLastRetiredHtmUid(uint64_t htm_uid)
    {
        assert(htm_uid >= lastRetiredHtmUid);
        lastRetiredHtmUid = htm_uid;
    }

    // Stale translation checks
    void startStaleTranslationFlush();
    bool checkStaleTranslations() const;

    /** Returns if either the LQ or SQ is full. */
    bool isFull() { return lqFull() || sqFull(); }

    /** Returns if both the LQ and SQ are empty. */
    bool isEmpty() const { return lqEmpty() && sqEmpty(); }

    /** Returns if the LQ is full. */
    bool lqFull() { return loadQueue.full(); }

    /** Returns if the SQ is full. */
    bool sqFull() { return storeQueue.full(); }

    /** Returns if the LQ is empty. */
    bool lqEmpty() const { return loadQueue.size() == 0; }

    /** Returns if the SQ is empty. */
    bool sqEmpty() const { return storeQueue.size() == 0; }

    /** Returns the number of instructions in the LSQ. */
    unsigned getCount() { return loadQueue.size() + storeQueue.size(); }

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
    void schedule(Event& ev, Tick when);

    BaseMMU *getMMUPtr();

  private:
    /** Pointer to the CPU. */
    CPU *cpu;

    /** Pointer to the IEW stage. */
    IEW *iewStage;

    /** Pointer to the LSQ. */
    LSQ *lsq;

    /** Pointer to the dcache port.  Used only for sending. */
    RequestPort *dcachePort;

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
        LSQUnit *lsqPtr;
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
    StoreQueue storeQueue;

    /** The load queue. */
    LoadQueue loadQueue;

  private:
    /** The number of places to shift addresses in the LSQ before checking
     * for dependency violations
     */
    unsigned depCheckShift;

    /** Should loads be checked for dependency issues */
    bool checkLoads;

    /** The number of store instructions in the SQ waiting to writeback. */
    int storesToWB;

    // hardware transactional memory
    // nesting depth
    int htmStarts;
    int htmStops;
    // sanity checks and debugging
    uint64_t lastRetiredHtmUid;

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
    ssize_t stallingLoadIdx;

    /** The packet that needs to be retried. */
    PacketPtr retryPkt;

    /** Whehter or not a store is blocked due to the memory system. */
    bool isStoreBlocked;

    /** Whether or not a store is in flight. */
    bool storeInFlight;

    //[shadow]
    bool laodInExec;

    /** The oldest load that caused a memory ordering violation. */
    DynInstPtr memDepViolator;

    /** Flag for memory model. */
    bool needsTSO;

  protected:
    // Will also need how many read/write ports the Dcache has.  Or keep track
    // of that in stage that is one level up, and only call executeLoad/Store
    // the appropriate number of times.
    struct LSQUnitStats : public statistics::Group
    {
        LSQUnitStats(statistics::Group *parent);

        /** Total number of loads forwaded from LSQ stores. */
        statistics::Scalar forwLoads;

        /** Total number of squashed loads. */
        statistics::Scalar squashedLoads;

        /** Total number of responses from the memory system that are
         * ignored due to the instruction already being squashed. */
        statistics::Scalar ignoredResponses;

        /** Tota number of memory ordering violations. */
        statistics::Scalar memOrderViolation;

        /** Total number of squashed stores. */
        statistics::Scalar squashedStores;

        /** Number of loads that were rescheduled. */
        statistics::Scalar rescheduledLoads;

        /** Number of times the LSQ is blocked due to the cache. */
        statistics::Scalar blockedByCache;

        /** Distribution of cycle latency between the first time a load
         * is issued and its completion */
        statistics::Distribution loadToUse;
    } stats;

  public:
    /** Executes the load at the given index. */
    Fault read(LSQRequest *request, ssize_t load_idx);

    /** Executes the store at the given index. */
    Fault write(LSQRequest *requst, uint8_t *data, ssize_t store_idx);

    /** Returns the index of the head load instruction. */
    int getLoadHead() { return loadQueue.head(); }

    /** Returns the sequence number of the head load instruction. */
    InstSeqNum getLoadHeadSeqNum();

    /** Returns the index of the head store instruction. */
    int getStoreHead() { return storeQueue.head(); }
    /** Returns the sequence number of the head store instruction. */
    InstSeqNum getStoreHeadSeqNum();

    /** Returns whether or not the LSQ unit is stalled. */
    bool isStalled()  { return stalled; }
  public:
    typedef typename CircularQueue<LQEntry>::iterator LQIterator;
    typedef typename CircularQueue<SQEntry>::iterator SQIterator;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_LSQ_UNIT_HH__
