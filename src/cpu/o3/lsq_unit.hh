/*
 * Copyright (c) 2012-2013 ARM Limited
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
#include "arch/isa_traits.hh"
#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "base/hashmap.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "cpu/timebuf.hh"
#include "debug/LSQUnit.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/fault_fwd.hh"

struct DerivO3CPUParams;

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
class LSQUnit {
  public:
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::LSQ LSQ;
    typedef typename Impl::CPUPol::IssueStruct IssueStruct;

  public:
    /** Constructs an LSQ unit. init() must be called prior to use. */
    LSQUnit();

    /** Initializes the LSQ unit with the specified number of entries. */
    void init(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params,
            LSQ *lsq_ptr, unsigned maxLQEntries, unsigned maxSQEntries,
            unsigned id);

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

    /** Check for ordering violations in the LSQ. For a store squash if we
     * ever find a conflicting load. For a load, only squash if we
     * an external snoop invalidate has been seen for that load address
     * @param load_idx index to start checking at
     * @param inst the instruction to check
     */
    Fault checkViolations(int load_idx, DynInstPtr &inst);

    /** Check if an incoming invalidate hits in the lsq on a load
     * that might have issued out of order wrt another load beacuse
     * of the intermediate invalidate.
     */
    void checkSnoop(PacketPtr pkt);

    /** Executes a load instruction. */
    Fault executeLoad(DynInstPtr &inst);

    Fault executeLoad(int lq_idx) { panic("Not implemented"); return NoFault; }
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

    /** Returns if a load became blocked due to the memory system. */
    bool loadBlocked()
    { return isLoadBlocked; }

    /** Clears the signal that a load became blocked. */
    void clearLoadBlocked()
    { isLoadBlocked = false; }

    /** Returns if the blocked load was handled. */
    bool isLoadBlockedHandled()
    { return loadBlockedHandled; }

    /** Records the blocked load as being handled. */
    void setLoadBlockedHandled()
    { loadBlockedHandled = true; }

    /** Returns the number of free entries (min of free LQ and SQ entries). */
    unsigned numFreeEntries();

    /** Returns the number of loads in the LQ. */
    int numLoads() { return loads; }

    /** Returns the number of stores in the SQ. */
    int numStores() { return stores; }

    /** Returns if either the LQ or SQ is full. */
    bool isFull() { return lqFull() || sqFull(); }

    /** Returns if both the LQ and SQ are empty. */
    bool isEmpty() const { return lqEmpty() && sqEmpty(); }

    /** Returns if the LQ is full. */
    bool lqFull() { return loads >= (LQEntries - 1); }

    /** Returns if the SQ is full. */
    bool sqFull() { return stores >= (SQEntries - 1); }

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
    bool willWB() { return storeQueue[storeWBIdx].canWB &&
                        !storeQueue[storeWBIdx].completed &&
                        !isStoreBlocked; }

    /** Handles doing the retry. */
    void recvRetry();

  private:
    /** Reset the LSQ state */
    void resetState();

    /** Writes back the instruction, sending it to IEW. */
    void writeback(DynInstPtr &inst, PacketPtr pkt);

    /** Writes back a store that couldn't be completed the previous cycle. */
    void writebackPendingStore();

    /** Handles completing the send of a store to memory. */
    void storePostSend(PacketPtr pkt);

    /** Completes the store at the specified index. */
    void completeStore(int store_idx);

    /** Attempts to send a store to the cache. */
    bool sendStore(PacketPtr data_pkt);

    /** Increments the given store index (circular queue). */
    inline void incrStIdx(int &store_idx) const;
    /** Decrements the given store index (circular queue). */
    inline void decrStIdx(int &store_idx) const;
    /** Increments the given load index (circular queue). */
    inline void incrLdIdx(int &load_idx) const;
    /** Decrements the given load index (circular queue). */
    inline void decrLdIdx(int &load_idx) const;

  public:
    /** Debugging function to dump instructions in the LSQ. */
    void dumpInsts() const;

  private:
    /** Pointer to the CPU. */
    O3CPU *cpu;

    /** Pointer to the IEW stage. */
    IEW *iewStage;

    /** Pointer to the LSQ. */
    LSQ *lsq;

    /** Pointer to the dcache port.  Used only for sending. */
    MasterPort *dcachePort;

    /** Derived class to hold any sender state the LSQ needs. */
    class LSQSenderState : public Packet::SenderState
    {
      public:
        /** Default constructor. */
        LSQSenderState()
            : mainPkt(NULL), pendingPacket(NULL), outstanding(1),
              noWB(false), isSplit(false), pktToSend(false)
          { }

        /** Instruction who initiated the access to memory. */
        DynInstPtr inst;
        /** The main packet from a split load, used during writeback. */
        PacketPtr mainPkt;
        /** A second packet from a split store that needs sending. */
        PacketPtr pendingPacket;
        /** The LQ/SQ index of the instruction. */
        uint8_t idx;
        /** Number of outstanding packets to complete. */
        uint8_t outstanding;
        /** Whether or not it is a load. */
        bool isLoad;
        /** Whether or not the instruction will need to writeback. */
        bool noWB;
        /** Whether or not this access is split in two. */
        bool isSplit;
        /** Whether or not there is a packet that needs sending. */
        bool pktToSend;

        /** Completes a packet and returns whether the access is finished. */
        inline bool complete() { return --outstanding == 0; }
    };

    /** Writeback event, specifically for when stores forward data to loads. */
    class WritebackEvent : public Event {
      public:
        /** Constructs a writeback event. */
        WritebackEvent(DynInstPtr &_inst, PacketPtr pkt, LSQUnit *lsq_ptr);

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
    struct SQEntry {
        /** Constructs an empty store queue entry. */
        SQEntry()
            : inst(NULL), req(NULL), size(0),
              canWB(0), committed(0), completed(0)
        {
            std::memset(data, 0, sizeof(data));
        }

        ~SQEntry()
        {
            inst = NULL;
        }

        /** Constructs a store queue entry for a given instruction. */
        SQEntry(DynInstPtr &_inst)
            : inst(_inst), req(NULL), sreqLow(NULL), sreqHigh(NULL), size(0),
              isSplit(0), canWB(0), committed(0), completed(0), isAllZeros(0)
        {
            std::memset(data, 0, sizeof(data));
        }
        /** The store data. */
        char data[16];
        /** The store instruction. */
        DynInstPtr inst;
        /** The request for the store. */
        RequestPtr req;
        /** The split requests for the store. */
        RequestPtr sreqLow;
        RequestPtr sreqHigh;
        /** The size of the store. */
        uint8_t size;
        /** Whether or not the store is split into two requests. */
        bool isSplit;
        /** Whether or not the store can writeback. */
        bool canWB;
        /** Whether or not the store is committed. */
        bool committed;
        /** Whether or not the store is completed. */
        bool completed;
        /** Does this request write all zeros and thus doesn't
         * have any data attached to it. Used for cache block zero
         * style instructs (ARM DC ZVA; ALPHA WH64)
         */
        bool isAllZeros;
    };

  private:
    /** The LSQUnit thread id. */
    ThreadID lsqID;

    /** The store queue. */
    std::vector<SQEntry> storeQueue;

    /** The load queue. */
    std::vector<DynInstPtr> loadQueue;

    /** The number of LQ entries, plus a sentinel entry (circular queue).
     *  @todo: Consider having var that records the true number of LQ entries.
     */
    unsigned LQEntries;
    /** The number of SQ entries, plus a sentinel entry (circular queue).
     *  @todo: Consider having var that records the true number of SQ entries.
     */
    unsigned SQEntries;

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

    /** The index of the head instruction in the LQ. */
    int loadHead;
    /** The index of the tail instruction in the LQ. */
    int loadTail;

    /** The index of the head instruction in the SQ. */
    int storeHead;
    /** The index of the first instruction that may be ready to be
     * written back, and has not yet been written back.
     */
    int storeWBIdx;
    /** The index of the tail instruction in the SQ. */
    int storeTail;

    /// @todo Consider moving to a more advanced model with write vs read ports
    /** The number of cache ports available each cycle. */
    int cachePorts;

    /** The number of used cache ports in this cycle. */
    int usedPorts;

    //list<InstSeqNum> mshrSeqNums;

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

    /** Whether or not a load is blocked due to the memory system. */
    bool isLoadBlocked;

    /** Has the blocked load been handled. */
    bool loadBlockedHandled;

    /** Whether or not a store is in flight. */
    bool storeInFlight;

    /** The sequence number of the blocked load. */
    InstSeqNum blockedLoadSeqNum;

    /** The oldest load that caused a memory ordering violation. */
    DynInstPtr memDepViolator;

    /** Whether or not there is a packet that couldn't be sent because of
     * a lack of cache ports. */
    bool hasPendingPkt;

    /** The packet that is pending free cache ports. */
    PacketPtr pendingPkt;

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
    Fault read(Request *req, Request *sreqLow, Request *sreqHigh,
               uint8_t *data, int load_idx);

    /** Executes the store at the given index. */
    Fault write(Request *req, Request *sreqLow, Request *sreqHigh,
                uint8_t *data, int store_idx);

    /** Returns the index of the head load instruction. */
    int getLoadHead() { return loadHead; }
    /** Returns the sequence number of the head load instruction. */
    InstSeqNum getLoadHeadSeqNum()
    {
        if (loadQueue[loadHead]) {
            return loadQueue[loadHead]->seqNum;
        } else {
            return 0;
        }

    }

    /** Returns the index of the head store instruction. */
    int getStoreHead() { return storeHead; }
    /** Returns the sequence number of the head store instruction. */
    InstSeqNum getStoreHeadSeqNum()
    {
        if (storeQueue[storeHead].inst) {
            return storeQueue[storeHead].inst->seqNum;
        } else {
            return 0;
        }

    }

    /** Returns whether or not the LSQ unit is stalled. */
    bool isStalled()  { return stalled; }
};

template <class Impl>
Fault
LSQUnit<Impl>::read(Request *req, Request *sreqLow, Request *sreqHigh,
                    uint8_t *data, int load_idx)
{
    DynInstPtr load_inst = loadQueue[load_idx];

    assert(load_inst);

    assert(!load_inst->isExecuted());

    // Make sure this isn't an uncacheable access
    // A bit of a hackish way to get uncached accesses to work only if they're
    // at the head of the LSQ and are ready to commit (at the head of the ROB
    // too).
    if (req->isUncacheable() &&
        (load_idx != loadHead || !load_inst->isAtCommit())) {
        iewStage->rescheduleMemInst(load_inst);
        ++lsqRescheduledLoads;
        DPRINTF(LSQUnit, "Uncachable load [sn:%lli] PC %s\n",
                load_inst->seqNum, load_inst->pcState());

        // Must delete request now that it wasn't handed off to
        // memory.  This is quite ugly.  @todo: Figure out the proper
        // place to really handle request deletes.
        delete req;
        if (TheISA::HasUnalignedMemAcc && sreqLow) {
            delete sreqLow;
            delete sreqHigh;
        }
        return new GenericISA::M5PanicFault(
                "Uncachable load [sn:%llx] PC %s\n",
                load_inst->seqNum, load_inst->pcState());
    }

    // Check the SQ for any previous stores that might lead to forwarding
    int store_idx = load_inst->sqIdx;

    int store_size = 0;

    DPRINTF(LSQUnit, "Read called, load idx: %i, store idx: %i, "
            "storeHead: %i addr: %#x%s\n",
            load_idx, store_idx, storeHead, req->getPaddr(),
            sreqLow ? " split" : "");

    if (req->isLLSC()) {
        assert(!sreqLow);
        // Disable recording the result temporarily.  Writing to misc
        // regs normally updates the result, but this is not the
        // desired behavior when handling store conditionals.
        load_inst->recordResult(false);
        TheISA::handleLockedRead(load_inst.get(), req);
        load_inst->recordResult(true);
    }

    if (req->isMmappedIpr()) {
        assert(!load_inst->memData);
        load_inst->memData = new uint8_t[64];

        ThreadContext *thread = cpu->tcBase(lsqID);
        Cycles delay(0);
        PacketPtr data_pkt = new Packet(req, MemCmd::ReadReq);

        if (!TheISA::HasUnalignedMemAcc || !sreqLow) {
            data_pkt->dataStatic(load_inst->memData);
            delay = TheISA::handleIprRead(thread, data_pkt);
        } else {
            assert(sreqLow->isMmappedIpr() && sreqHigh->isMmappedIpr());
            PacketPtr fst_data_pkt = new Packet(sreqLow, MemCmd::ReadReq);
            PacketPtr snd_data_pkt = new Packet(sreqHigh, MemCmd::ReadReq);

            fst_data_pkt->dataStatic(load_inst->memData);
            snd_data_pkt->dataStatic(load_inst->memData + sreqLow->getSize());

            delay = TheISA::handleIprRead(thread, fst_data_pkt);
            Cycles delay2 = TheISA::handleIprRead(thread, snd_data_pkt);
            if (delay2 > delay)
                delay = delay2;

            delete sreqLow;
            delete sreqHigh;
            delete fst_data_pkt;
            delete snd_data_pkt;
        }
        WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt, this);
        cpu->schedule(wb, cpu->clockEdge(delay));
        return NoFault;
    }

    while (store_idx != -1) {
        // End once we've reached the top of the LSQ
        if (store_idx == storeWBIdx) {
            break;
        }

        // Move the index to one younger
        if (--store_idx < 0)
            store_idx += SQEntries;

        assert(storeQueue[store_idx].inst);

        store_size = storeQueue[store_idx].size;

        if (store_size == 0)
            continue;
        else if (storeQueue[store_idx].inst->uncacheable())
            continue;

        assert(storeQueue[store_idx].inst->effAddrValid());

        // Check if the store data is within the lower and upper bounds of
        // addresses that the request needs.
        bool store_has_lower_limit =
            req->getVaddr() >= storeQueue[store_idx].inst->effAddr;
        bool store_has_upper_limit =
            (req->getVaddr() + req->getSize()) <=
            (storeQueue[store_idx].inst->effAddr + store_size);
        bool lower_load_has_store_part =
            req->getVaddr() < (storeQueue[store_idx].inst->effAddr +
                           store_size);
        bool upper_load_has_store_part =
            (req->getVaddr() + req->getSize()) >
            storeQueue[store_idx].inst->effAddr;

        // If the store's data has all of the data needed, we can forward.
        if ((store_has_lower_limit && store_has_upper_limit)) {
            // Get shift amount for offset into the store's data.
            int shift_amt = req->getVaddr() - storeQueue[store_idx].inst->effAddr;

            if (storeQueue[store_idx].isAllZeros)
                memset(data, 0, req->getSize());
            else
                memcpy(data, storeQueue[store_idx].data + shift_amt,
                   req->getSize());

            assert(!load_inst->memData);
            load_inst->memData = new uint8_t[req->getSize()];
            if (storeQueue[store_idx].isAllZeros)
                memset(load_inst->memData, 0, req->getSize());
            else
                memcpy(load_inst->memData,
                    storeQueue[store_idx].data + shift_amt, req->getSize());

            DPRINTF(LSQUnit, "Forwarding from store idx %i to load to "
                    "addr %#x, data %#x\n",
                    store_idx, req->getVaddr(), data);

            PacketPtr data_pkt = new Packet(req, MemCmd::ReadReq);
            data_pkt->dataStatic(load_inst->memData);

            WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt, this);

            // We'll say this has a 1 cycle load-store forwarding latency
            // for now.
            // @todo: Need to make this a parameter.
            cpu->schedule(wb, curTick());

            // Don't need to do anything special for split loads.
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete sreqLow;
                delete sreqHigh;
            }

            ++lsqForwLoads;
            return NoFault;
        } else if ((store_has_lower_limit && lower_load_has_store_part) ||
                   (store_has_upper_limit && upper_load_has_store_part) ||
                   (lower_load_has_store_part && upper_load_has_store_part)) {
            // This is the partial store-load forwarding case where a store
            // has only part of the load's data.

            // If it's already been written back, then don't worry about
            // stalling on it.
            if (storeQueue[store_idx].completed) {
                panic("Should not check one of these");
                continue;
            }

            // Must stall load and force it to retry, so long as it's the oldest
            // load that needs to do so.
            if (!stalled ||
                (stalled &&
                 load_inst->seqNum <
                 loadQueue[stallingLoadIdx]->seqNum)) {
                stalled = true;
                stallingStoreIsn = storeQueue[store_idx].inst->seqNum;
                stallingLoadIdx = load_idx;
            }

            // Tell IQ/mem dep unit that this instruction will need to be
            // rescheduled eventually
            iewStage->rescheduleMemInst(load_inst);
            iewStage->decrWb(load_inst->seqNum);
            load_inst->clearIssued();
            ++lsqRescheduledLoads;

            // Do not generate a writeback event as this instruction is not
            // complete.
            DPRINTF(LSQUnit, "Load-store forwarding mis-match. "
                    "Store idx %i to load addr %#x\n",
                    store_idx, req->getVaddr());

            // Must delete request now that it wasn't handed off to
            // memory.  This is quite ugly.  @todo: Figure out the
            // proper place to really handle request deletes.
            delete req;
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete sreqLow;
                delete sreqHigh;
            }

            return NoFault;
        }
    }

    // If there's no forwarding case, then go access memory
    DPRINTF(LSQUnit, "Doing memory access for inst [sn:%lli] PC %s\n",
            load_inst->seqNum, load_inst->pcState());

    assert(!load_inst->memData);
    load_inst->memData = new uint8_t[req->getSize()];

    ++usedPorts;

    // if we the cache is not blocked, do cache access
    bool completedFirst = false;
    if (!lsq->cacheBlocked()) {
        MemCmd command =
            req->isLLSC() ? MemCmd::LoadLockedReq : MemCmd::ReadReq;
        PacketPtr data_pkt = new Packet(req, command);
        PacketPtr fst_data_pkt = NULL;
        PacketPtr snd_data_pkt = NULL;

        data_pkt->dataStatic(load_inst->memData);

        LSQSenderState *state = new LSQSenderState;
        state->isLoad = true;
        state->idx = load_idx;
        state->inst = load_inst;
        data_pkt->senderState = state;

        if (!TheISA::HasUnalignedMemAcc || !sreqLow) {

            // Point the first packet at the main data packet.
            fst_data_pkt = data_pkt;
        } else {

            // Create the split packets.
            fst_data_pkt = new Packet(sreqLow, command);
            snd_data_pkt = new Packet(sreqHigh, command);

            fst_data_pkt->dataStatic(load_inst->memData);
            snd_data_pkt->dataStatic(load_inst->memData + sreqLow->getSize());

            fst_data_pkt->senderState = state;
            snd_data_pkt->senderState = state;

            state->isSplit = true;
            state->outstanding = 2;
            state->mainPkt = data_pkt;
        }

        if (!dcachePort->sendTimingReq(fst_data_pkt)) {
            // Delete state and data packet because a load retry
            // initiates a pipeline restart; it does not retry.
            delete state;
            delete data_pkt->req;
            delete data_pkt;
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete fst_data_pkt->req;
                delete fst_data_pkt;
                delete snd_data_pkt->req;
                delete snd_data_pkt;
                sreqLow = NULL;
                sreqHigh = NULL;
            }

            req = NULL;

            // If the access didn't succeed, tell the LSQ by setting
            // the retry thread id.
            lsq->setRetryTid(lsqID);
        } else if (TheISA::HasUnalignedMemAcc && sreqLow) {
            completedFirst = true;

            // The first packet was sent without problems, so send this one
            // too. If there is a problem with this packet then the whole
            // load will be squashed, so indicate this to the state object.
            // The first packet will return in completeDataAccess and be
            // handled there.
            ++usedPorts;
            if (!dcachePort->sendTimingReq(snd_data_pkt)) {

                // The main packet will be deleted in completeDataAccess.
                delete snd_data_pkt->req;
                delete snd_data_pkt;

                state->complete();

                req = NULL;
                sreqHigh = NULL;

                lsq->setRetryTid(lsqID);
            }
        }
    }

    // If the cache was blocked, or has become blocked due to the access,
    // handle it.
    if (lsq->cacheBlocked()) {
        if (req)
            delete req;
        if (TheISA::HasUnalignedMemAcc && sreqLow && !completedFirst) {
            delete sreqLow;
            delete sreqHigh;
        }

        ++lsqCacheBlocked;

        // If the first part of a split access succeeds, then let the LSQ
        // handle the decrWb when completeDataAccess is called upon return
        // of the requested first part of data
        if (!completedFirst)
            iewStage->decrWb(load_inst->seqNum);

        // There's an older load that's already going to squash.
        if (isLoadBlocked && blockedLoadSeqNum < load_inst->seqNum)
            return NoFault;

        // Record that the load was blocked due to memory.  This
        // load will squash all instructions after it, be
        // refetched, and re-executed.
        isLoadBlocked = true;
        loadBlockedHandled = false;
        blockedLoadSeqNum = load_inst->seqNum;
        // No fault occurred, even though the interface is blocked.
        return NoFault;
    }

    return NoFault;
}

template <class Impl>
Fault
LSQUnit<Impl>::write(Request *req, Request *sreqLow, Request *sreqHigh,
                     uint8_t *data, int store_idx)
{
    assert(storeQueue[store_idx].inst);

    DPRINTF(LSQUnit, "Doing write to store idx %i, addr %#x data %#x"
            " | storeHead:%i [sn:%i]\n",
            store_idx, req->getPaddr(), data, storeHead,
            storeQueue[store_idx].inst->seqNum);

    storeQueue[store_idx].req = req;
    storeQueue[store_idx].sreqLow = sreqLow;
    storeQueue[store_idx].sreqHigh = sreqHigh;
    unsigned size = req->getSize();
    storeQueue[store_idx].size = size;
    storeQueue[store_idx].isAllZeros = req->getFlags() & Request::CACHE_BLOCK_ZERO;
    assert(size <= sizeof(storeQueue[store_idx].data) ||
            (req->getFlags() & Request::CACHE_BLOCK_ZERO));

    // Split stores can only occur in ISAs with unaligned memory accesses.  If
    // a store request has been split, sreqLow and sreqHigh will be non-null.
    if (TheISA::HasUnalignedMemAcc && sreqLow) {
        storeQueue[store_idx].isSplit = true;
    }

    if (!(req->getFlags() & Request::CACHE_BLOCK_ZERO))
        memcpy(storeQueue[store_idx].data, data, size);

    // This function only writes the data to the store queue, so no fault
    // can happen here.
    return NoFault;
}

#endif // __CPU_O3_LSQ_UNIT_HH__
