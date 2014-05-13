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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_OZONE_LW_LSQ_HH__
#define __CPU_OZONE_LW_LSQ_HH__

#include <algorithm>
#include <list>
#include <map>
#include <queue>

#include "arch/types.hh"
#include "base/hashmap.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
//#include "mem/page_table.hh"
#include "sim/debug.hh"
#include "sim/fault_fwd.hh"
#include "sim/sim_object.hh"

class MemObject;

/**
 * Class that implements the actual LQ and SQ for each specific thread.
 * Both are circular queues; load entries are freed upon committing, while
 * store entries are freed once they writeback. The LSQUnit tracks if there
 * are memory ordering violations, and also detects partial load to store
 * forwarding cases (a store only has part of a load's data) that requires
 * the load to wait until the store writes back. In the former case it
 * holds onto the instruction until the dependence unit looks at it, and
 * in the latter it stalls the LSQ until the store writes back. At that
 * point the load is replayed.
 */
template <class Impl>
class OzoneLWLSQ {
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::OzoneCPU OzoneCPU;
    typedef typename Impl::BackEnd BackEnd;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::IssueStruct IssueStruct;

    typedef TheISA::IntReg IntReg;

    typedef typename std::map<InstSeqNum, DynInstPtr>::iterator LdMapIt;

  public:
    /** Constructs an LSQ unit. init() must be called prior to use. */
    OzoneLWLSQ();

    /** Initializes the LSQ unit with the specified number of entries. */
    void init(Params *params, unsigned maxLQEntries,
              unsigned maxSQEntries, unsigned id);

    /** Returns the name of the LSQ unit. */
    std::string name() const;

    void regStats();

    /** Sets the CPU pointer. */
    void setCPU(OzoneCPU *cpu_ptr);

    /** Sets the back-end stage pointer. */
    void setBE(BackEnd *be_ptr)
    { be = be_ptr; }

    Port *getDcachePort() { return &dcachePort; }

    /** Ticks the LSQ unit, which in this case only resets the number of
     * used cache ports.
     * @todo: Move the number of used ports up to the LSQ level so it can
     * be shared by all LSQ units.
     */
    void tick() { usedPorts = 0; }

    /** Inserts an instruction. */
    void insert(DynInstPtr &inst);
    /** Inserts a load instruction. */
    void insertLoad(DynInstPtr &load_inst);
    /** Inserts a store instruction. */
    void insertStore(DynInstPtr &store_inst);

    /** Executes a load instruction. */
    Fault executeLoad(DynInstPtr &inst);

    /** Executes a store instruction. */
    Fault executeStore(DynInstPtr &inst);

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

    // @todo: Include stats in the LSQ unit.
    //void regStats();

    /** Clears all the entries in the LQ. */
    void clearLQ();

    /** Clears all the entries in the SQ. */
    void clearSQ();

    /** Resizes the LQ to a given size. */
    void resizeLQ(unsigned size);

    /** Resizes the SQ to a given size. */
    void resizeSQ(unsigned size);

    /** Squashes all instructions younger than a specific sequence number. */
    void squash(const InstSeqNum &squashed_num);

    /** Returns if there is a memory ordering violation. Value is reset upon
     * call to getMemDepViolator().
     */
    bool violation() { return memDepViolator; }

    /** Returns the memory ordering violator. */
    DynInstPtr getMemDepViolator();

    /** Returns if a load became blocked due to the memory system.  It clears
     *  the bool's value upon this being called.
     */
    bool loadBlocked()
    { return isLoadBlocked; }

    void clearLoadBlocked()
    { isLoadBlocked = false; }

    bool isLoadBlockedHandled()
    { return loadBlockedHandled; }

    void setLoadBlockedHandled()
    { loadBlockedHandled = true; }

    /** Returns the number of free entries (min of free LQ and SQ entries). */
    unsigned numFreeEntries();

    /** Returns the number of loads ready to execute. */
    int numLoadsReady();

    /** Returns the number of loads in the LQ. */
    int numLoads() { return loads; }

    /** Returns the number of stores in the SQ. */
    int numStores() { return stores + storesInFlight; }

    /** Returns if either the LQ or SQ is full. */
    bool isFull() { return lqFull() || sqFull(); }

    /** Returns if the LQ is full. */
    bool lqFull() { return loads >= (LQEntries - 1); }

    /** Returns if the SQ is full. */
    bool sqFull() { return (stores + storesInFlight) >= (SQEntries - 1); }

    /** Debugging function to dump instructions in the LSQ. */
    void dumpInsts();

    /** Returns the number of instructions in the LSQ. */
    unsigned getCount() { return loads + stores; }

    /** Returns if there are any stores to writeback. */
    bool hasStoresToWB() { return storesToWB; }

    /** Returns the number of stores to writeback. */
    int numStoresToWB() { return storesToWB; }

    /** Returns if the LSQ unit will writeback on this cycle. */
    bool willWB() { return storeQueue.back().canWB &&
                        !storeQueue.back().completed &&
                        !isStoreBlocked; }

    void switchOut();

    void takeOverFrom(ThreadContext *old_tc = NULL);

    bool isSwitchedOut() { return switchedOut; }

    bool switchedOut;

  private:
    /** Writes back the instruction, sending it to IEW. */
    void writeback(DynInstPtr &inst, PacketPtr pkt);

    /** Handles completing the send of a store to memory. */
    void storePostSend(PacketPtr pkt, DynInstPtr &inst);

    /** Completes the store at the specified index. */
    void completeStore(DynInstPtr &inst);

    void removeStore(int store_idx);

    /** Handles doing the retry. */
    void recvRetry();

  private:
    /** Pointer to the CPU. */
    OzoneCPU *cpu;

    /** Pointer to the back-end stage. */
    BackEnd *be;

    class DcachePort : public MasterPort
    {
      protected:
        OzoneLWLSQ *lsq;

      public:
        DcachePort(OzoneLWLSQ *_lsq)
            : lsq(_lsq)
        { }

      protected:
        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        /**
         * Is a snooper due to LSQ maintenance
         */
        virtual bool isSnooping() const { return true; }

        virtual bool recvTiming(PacketPtr pkt);

        virtual void recvRetry();
    };

    /** D-cache port. */
    DcachePort dcachePort;

  public:
    struct SQEntry {
        /** Constructs an empty store queue entry. */
        SQEntry()
            : inst(NULL), req(NULL), size(0), data(0),
              canWB(0), committed(0), completed(0), lqIt(NULL)
        { }

        /** Constructs a store queue entry for a given instruction. */
        SQEntry(DynInstPtr &_inst)
            : inst(_inst), req(NULL), size(0), data(0),
              canWB(0), committed(0), completed(0), lqIt(NULL)
        { }

        /** The store instruction. */
        DynInstPtr inst;
        /** The memory request for the store. */
        RequestPtr req;
        /** The size of the store. */
        int size;
        /** The store data. */
        IntReg data;
        /** Whether or not the store can writeback. */
        bool canWB;
        /** Whether or not the store is committed. */
        bool committed;
        /** Whether or not the store is completed. */
        bool completed;

        typename std::list<DynInstPtr>::iterator lqIt;
    };

    /** Derived class to hold any sender state the LSQ needs. */
    class LSQSenderState : public Packet::SenderState
    {
      public:
        /** Default constructor. */
        LSQSenderState()
            : noWB(false)
        { }

        /** Instruction who initiated the access to memory. */
        DynInstPtr inst;
        /** Whether or not it is a load. */
        bool isLoad;
        /** The LQ/SQ index of the instruction. */
        int idx;
        /** Whether or not the instruction will need to writeback. */
        bool noWB;
    };

    /** Writeback event, specifically for when stores forward data to loads. */
    class WritebackEvent : public Event {
      public:
        /** Constructs a writeback event. */
        WritebackEvent(DynInstPtr &_inst, PacketPtr pkt, OzoneLWLSQ *lsq_ptr);

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
        OzoneLWLSQ<Impl> *lsqPtr;
    };

    enum Status {
        Running,
        Idle,
        DcacheMissStall,
        DcacheMissSwitch
    };

  private:
    /** The OzoneLWLSQ thread id. */
    unsigned lsqID;

    /** The status of the LSQ unit. */
    Status _status;

    /** The store queue. */
    std::list<SQEntry> storeQueue;
    /** The load queue. */
    std::list<DynInstPtr> loadQueue;

    typedef typename std::list<SQEntry>::iterator SQIt;
    typedef typename std::list<DynInstPtr>::iterator LQIt;


    struct HashFn {
    size_t operator() (const int a) const
    {
        unsigned hash = (((a >> 14) ^ ((a >> 2) & 0xffff))) & 0x7FFFFFFF;

        return hash;
    }
    };

    m5::hash_map<int, SQIt, HashFn> SQItHash;
    std::queue<int> SQIndices;
    m5::hash_map<int, LQIt, HashFn> LQItHash;
    std::queue<int> LQIndices;

    typedef typename m5::hash_map<int, LQIt, HashFn>::iterator LQHashIt;
    typedef typename m5::hash_map<int, SQIt, HashFn>::iterator SQHashIt;
    // Consider making these 16 bits
    /** The number of LQ entries. */
    unsigned LQEntries;
    /** The number of SQ entries. */
    unsigned SQEntries;

    /** The number of load instructions in the LQ. */
    int loads;
    /** The number of store instructions in the SQ (excludes those waiting to
     * writeback).
     */
    int stores;

    int storesToWB;

  public:
    int storesInFlight;

  private:
    /// @todo Consider moving to a more advanced model with write vs read ports
    /** The number of cache ports available each cycle. */
    int cachePorts;

    /** The number of used cache ports in this cycle. */
    int usedPorts;

    //list<InstSeqNum> mshrSeqNums;

    /** Tota number of memory ordering violations. */
    Stats::Scalar lsqMemOrderViolation;

     //Stats::Scalar dcacheStallCycles;
    Counter lastDcacheStall;

    // Make these per thread?
    /** Whether or not the LSQ is stalled. */
    bool stalled;
    /** The store that causes the stall due to partial store to load
     * forwarding.
     */
    InstSeqNum stallingStoreIsn;
    /** The index of the above store. */
    LQIt stallingLoad;

    /** The packet that needs to be retried. */
    PacketPtr retryPkt;

    /** Whehter or not a store is blocked due to the memory system. */
    bool isStoreBlocked;

    /** Whether or not a load is blocked due to the memory system.  It is
     *  cleared when this value is checked via loadBlocked().
     */
    bool isLoadBlocked;

    bool loadBlockedHandled;

    InstSeqNum blockedLoadSeqNum;

    /** The oldest faulting load instruction. */
    DynInstPtr loadFaultInst;
    /** The oldest faulting store instruction. */
    DynInstPtr storeFaultInst;

    /** The oldest load that caused a memory ordering violation. */
    DynInstPtr memDepViolator;

    // Will also need how many read/write ports the Dcache has.  Or keep track
    // of that in stage that is one level up, and only call executeLoad/Store
    // the appropriate number of times.

  public:
    /** Executes the load at the given index. */
    template <class T>
    Fault read(RequestPtr req, T &data, int load_idx);

    /** Executes the store at the given index. */
    template <class T>
    Fault write(RequestPtr req, T &data, int store_idx);

    /** Returns the sequence number of the head load instruction. */
    InstSeqNum getLoadHeadSeqNum()
    {
        if (!loadQueue.empty()) {
            return loadQueue.back()->seqNum;
        } else {
            return 0;
        }

    }

    /** Returns the sequence number of the head store instruction. */
    InstSeqNum getStoreHeadSeqNum()
    {
        if (!storeQueue.empty()) {
            return storeQueue.back().inst->seqNum;
        } else {
            return 0;
        }

    }

    /** Returns whether or not the LSQ unit is stalled. */
    bool isStalled()  { return stalled; }
};

template <class Impl>
template <class T>
Fault
OzoneLWLSQ<Impl>::read(RequestPtr req, T &data, int load_idx)
{
    //Depending on issue2execute delay a squashed load could
    //execute if it is found to be squashed in the same
    //cycle it is scheduled to execute
    typename m5::hash_map<int, LQIt, HashFn>::iterator
        lq_hash_it = LQItHash.find(load_idx);
    assert(lq_hash_it != LQItHash.end());
    DynInstPtr inst = (*(*lq_hash_it).second);

    // Make sure this isn't an uncacheable access
    // A bit of a hackish way to get uncached accesses to work only if they're
    // at the head of the LSQ and are ready to commit (at the head of the ROB
    // too).
    // @todo: Fix uncached accesses.
    if (req->isUncacheable() &&
        (inst != loadQueue.back() || !inst->isAtCommit())) {
        DPRINTF(OzoneLSQ, "[sn:%lli] Uncached load and not head of "
                "commit/LSQ!\n",
                inst->seqNum);
        be->rescheduleMemInst(inst);
        return TheISA::genMachineCheckFault();
    }

    // Check the SQ for any previous stores that might lead to forwarding
    SQIt sq_it = storeQueue.begin();
    int store_size = 0;

    DPRINTF(OzoneLSQ, "Read called, load idx: %i addr: %#x\n",
            load_idx, req->getPaddr());

    while (sq_it != storeQueue.end() && (*sq_it).inst->seqNum > inst->seqNum)
        ++sq_it;

    while (1) {
        // End once we've reached the top of the LSQ
        if (sq_it == storeQueue.end()) {
            break;
        }

        assert((*sq_it).inst);

        store_size = (*sq_it).size;

        if (store_size == 0 || (*sq_it).committed) {
            sq_it++;
            continue;
        }

        // Check if the store data is within the lower and upper bounds of
        // addresses that the request needs.
        bool store_has_lower_limit =
            req->getVaddr() >= (*sq_it).inst->effAddr;
        bool store_has_upper_limit =
            (req->getVaddr() + req->getSize()) <= ((*sq_it).inst->effAddr +
                                                   store_size);
        bool lower_load_has_store_part =
            req->getVaddr() < ((*sq_it).inst->effAddr +
                               store_size);
        bool upper_load_has_store_part =
            (req->getVaddr() + req->getSize()) > (*sq_it).inst->effAddr;

        // If the store's data has all of the data needed, we can forward.
        if (store_has_lower_limit && store_has_upper_limit) {
            int shift_amt = req->getVaddr() & (store_size - 1);
            // Assumes byte addressing
            shift_amt = shift_amt << 3;

            // Cast this to type T?
            data = (*sq_it).data >> shift_amt;

            assert(!inst->memData);
            inst->memData = new uint8_t[64];

            memcpy(inst->memData, &data, req->getSize());

            DPRINTF(OzoneLSQ, "Forwarding from store [sn:%lli] to load to "
                    "[sn:%lli] addr %#x, data %#x\n",
                    (*sq_it).inst->seqNum, inst->seqNum, req->getVaddr(),
                    *(inst->memData));

            PacketPtr data_pkt = new Packet(req, Packet::ReadReq);
            data_pkt->dataStatic(inst->memData);

            WritebackEvent *wb = new WritebackEvent(inst, data_pkt, this);

            // We'll say this has a 1 cycle load-store forwarding latency
            // for now.
            // @todo: Need to make this a parameter.
            wb->schedule(curTick());

            // Should keep track of stat for forwarded data
            return NoFault;
        } else if ((store_has_lower_limit && lower_load_has_store_part) ||
                   (store_has_upper_limit && upper_load_has_store_part) ||
                   (lower_load_has_store_part && upper_load_has_store_part)) {
            // This is the partial store-load forwarding case where a store
            // has only part of the load's data.

            // If it's already been written back, then don't worry about
            // stalling on it.
            if ((*sq_it).completed) {
                sq_it++;
                break;
            }

            // Must stall load and force it to retry, so long as it's the oldest
            // load that needs to do so.
            if (!stalled ||
                (stalled &&
                 inst->seqNum <
                 (*stallingLoad)->seqNum)) {
                stalled = true;
                stallingStoreIsn = (*sq_it).inst->seqNum;
                stallingLoad = (*lq_hash_it).second;
            }

            // Tell IQ/mem dep unit that this instruction will need to be
            // rescheduled eventually
            be->rescheduleMemInst(inst);

            DPRINTF(OzoneLSQ, "Load-store forwarding mis-match. "
                    "Store [sn:%lli] to load addr %#x\n",
                    (*sq_it).inst->seqNum, req->getVaddr());

            return NoFault;
        }
        sq_it++;
    }

    // If there's no forwarding case, then go access memory
    DPRINTF(OzoneLSQ, "Doing functional access for inst PC %#x\n",
            inst->readPC());

    assert(!inst->memData);
    inst->memData = new uint8_t[64];

    ++usedPorts;

    DPRINTF(OzoneLSQ, "Doing timing access for inst PC %#x\n",
            inst->readPC());

    PacketPtr data_pkt = Packet::createRead(req);
    data_pkt->dataStatic(inst->memData);

    LSQSenderState *state = new LSQSenderState;
    state->isLoad = true;
    state->idx = load_idx;
    state->inst = inst;
    data_pkt->senderState = state;

    // if we have a cache, do cache access too
    if (!dcachePort.sendTiming(data_pkt)) {
        // There's an older load that's already going to squash.
        if (isLoadBlocked && blockedLoadSeqNum < inst->seqNum)
            return NoFault;

        // Record that the load was blocked due to memory.  This
        // load will squash all instructions after it, be
        // refetched, and re-executed.
        isLoadBlocked = true;
        loadBlockedHandled = false;
        blockedLoadSeqNum = inst->seqNum;
        // No fault occurred, even though the interface is blocked.
        return NoFault;
    }

    if (req->isLLSC()) {
        cpu->lockFlag = true;
    }

    return NoFault;
}

template <class Impl>
template <class T>
Fault
OzoneLWLSQ<Impl>::write(RequestPtr req, T &data, int store_idx)
{
    SQHashIt sq_hash_it = SQItHash.find(store_idx);
    assert(sq_hash_it != SQItHash.end());

    SQIt sq_it = (*sq_hash_it).second;
    assert((*sq_it).inst);

    DPRINTF(OzoneLSQ, "Doing write to store idx %i, addr %#x data %#x"
            " | [sn:%lli]\n",
            store_idx, req->getPaddr(), data, (*sq_it).inst->seqNum);

    (*sq_it).req = req;
    (*sq_it).size = sizeof(T);
    (*sq_it).data = data;
/*
    assert(!req->data);
    req->data = new uint8_t[64];
    memcpy(req->data, (uint8_t *)&(*sq_it).data, req->size);
*/

    // This function only writes the data to the store queue, so no fault
    // can happen here.
    return NoFault;
}

#endif // __CPU_OZONE_LW_LSQ_HH__
