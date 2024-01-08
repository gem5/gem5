/*
 * Copyright (c) 2011-2012, 2014, 2018-2019, 2021 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 */

#ifndef __CPU_O3_LSQ_HH__
#define __CPU_O3_LSQ_HH__

#include <cassert>
#include <cstdint>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "arch/generic/mmu.hh"
#include "arch/generic/tlb.hh"
#include "base/flags.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/utils.hh"
#include "enums/SMTQueuePolicy.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class CPU;
class IEW;
class LSQUnit;

class LSQ
{
  public:
    class LSQRequest;

    /**
     * DcachePort class for the load/store queue.
     */
    class DcachePort : public RequestPort
    {
      protected:

        /** Pointer to LSQ. */
        LSQ *lsq;
        CPU *cpu;

      public:
        /** Default constructor. */
        DcachePort(LSQ *_lsq, CPU *_cpu);

      protected:

        /** Timing version of receive.  Handles writing back and
         * completing the load or store that has returned from
         * memory. */
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void recvTimingSnoopReq(PacketPtr pkt);

        virtual void
        recvFunctionalSnoop(PacketPtr pkt)
        {
            // @todo: Is there a need for potential invalidation here?
        }

        /** Handles doing a retry of the previous send. */
        virtual void recvReqRetry();

        /**
         * As this CPU requires snooping to maintain the load store queue
         * change the behaviour from the base CPU port.
         *
         * @return true since we have to snoop
         */
        virtual bool isSnooping() const { return true; }
    };

    /** Memory operation metadata.
     * This class holds the information about a memory operation. It lives
     * from initiateAcc to resource deallocation at commit or squash.
     * LSQRequest objects are owned by the LQ/SQ Entry in the LSQUnit that
     * holds the operation. In addition, the LSQRequest is a TranslationState,
     * therefore, upon squash, there must be a defined ownership transferal
     * in case the LSQ resources are deallocated before the TLB is done using
     * the TranslationState.
     * If that happens, the LSQRequest will be self-owned, and responsible to
     * detect that its services are no longer required and self-destruct.
     *
     * Lifetime of a LSQRequest:
     *                 +--------------------+
     *                 |LSQ creates and owns|
     *                 +--------------------+
     *                           |
     *                 +--------------------+
     *                 | Initate translation|
     *                 +--------------------+
     *                           |
     *                        ___^___
     *                    ___/       \___
     *             ______/   Squashed?   \
     *            |      \___         ___/
     *            |          \___ ___/
     *            |              v
     *            |              |
     *            |    +--------------------+
     *            |    |  Translation done  |
     *            |    +--------------------+
     *            |              |
     *            |    +--------------------+
     *            |    |     Send packet    |<------+
     *            |    +--------------------+       |
     *            |              |                  |
     *            |           ___^___               |
     *            |       ___/       \___           |
     *            |  ____/   Squashed?   \          |
     *            | |    \___         ___/          |
     *            | |        \___ ___/              |
     *            | |            v                  |
     *            | |            |                  |
     *            | |         ___^___               |
     *            | |     ___/       \___           |
     *            | |    /     Done?     \__________|
     *            | |    \___         ___/
     *            | |        \___ ___/
     *            | |            v
     *            | |            |
     *            | |  +--------------------+
     *            | |  |    Manage stuff    |
     *            | |  |   Free resources   |
     *            | |  +--------------------+
     *            | |
     *            | |  +--------------------+
     *            | |  |    self owned      |
     *            | +->|  on recvTimingResp |
     *            |    |   free resources   |
     *            |    +--------------------+
     *            |
     *            |   +----------------------+
     *            |   |  self owned (Trans)  |
     *            +-->| on TranslationFinish |
     *                |    free resources    |
     *                +----------------------+
     *
     *
     */
    class LSQRequest : public BaseMMU::Translation, public Packet::SenderState
    {
      protected:
        typedef uint32_t FlagsStorage;
        typedef Flags<FlagsStorage> FlagsType;

        enum Flag : FlagsStorage
        {
            IsLoad              = 0x00000001,
            /** True if this request needs to writeBack to register.
              * Will be set in case of load or a store/atomic
              * that writes registers (SC)
              */
            WriteBackToRegister = 0x00000002,
            Delayed             = 0x00000004,
            IsSplit             = 0x00000008,
            /** True if any translation has been sent to TLB. */
            TranslationStarted  = 0x00000010,
            /** True if there are un-replied outbound translations.. */
            TranslationFinished = 0x00000020,
            Sent                = 0x00000040,
            Retry               = 0x00000080,
            Complete            = 0x00000100,
            /** Ownership tracking flags. */
            /** Translation squashed. */
            TranslationSquashed = 0x00000200,
            /** Request discarded */
            Discarded           = 0x00000400,
            /** LSQ resources freed. */
            LSQEntryFreed       = 0x00000800,
            /** Store written back. */
            WritebackScheduled  = 0x00001000,
            WritebackDone       = 0x00002000,
            /** True if this is an atomic request */
            IsAtomic            = 0x00004000
        };
        FlagsType flags;

        enum class State
        {
            NotIssued,
            Translation,
            Request,
            Fault,
            PartialFault,
        };
        State _state;
        void setState(const State& newState) { _state = newState; }

        uint32_t numTranslatedFragments;
        uint32_t numInTranslationFragments;


        void markDelayed() override { flags.set(Flag::Delayed); }
        bool isDelayed() { return flags.isSet(Flag::Delayed); }

      public:
        LSQUnit& _port;
        const DynInstPtr _inst;
        uint32_t _taskId;
        PacketDataPtr _data;
        std::vector<PacketPtr> _packets;
        std::vector<RequestPtr> _reqs;
        std::vector<Fault> _fault;
        uint64_t* _res;
        const Addr _addr;
        const uint32_t _size;
        const Request::Flags _flags;
        std::vector<bool> _byteEnable;
        uint32_t _numOutstandingPackets;
        AtomicOpFunctorPtr _amo_op;
        bool _hasStaleTranslation;

      protected:
        LSQUnit* lsqUnit() { return &_port; }
        LSQRequest(LSQUnit* port, const DynInstPtr& inst, bool isLoad);
        LSQRequest(LSQUnit* port, const DynInstPtr& inst, bool isLoad,
                const Addr& addr, const uint32_t& size,
                const Request::Flags& flags_, PacketDataPtr data=nullptr,
                uint64_t* res=nullptr, AtomicOpFunctorPtr amo_op=nullptr,
                bool stale_translation=false);

        bool
        isLoad() const
        {
            return flags.isSet(Flag::IsLoad);
        }

        bool
        isAtomic() const
        {
            return flags.isSet(Flag::IsAtomic);
        }

        /** Install the request in the LQ/SQ. */
        void install();

        bool squashed() const override;


        /** Release the LSQRequest.
         * Notify the sender state that the request it points to is not valid
         * anymore. Understand if the request is orphan (self-managed) and if
         * so, mark it as freed, else destroy it, as this means
         * the end of its life cycle.
         * An LSQRequest is orphan when its resources are released
         * but there is any in-flight translation request to the TLB or access
         * request to the memory.
         */
        void
        release(Flag reason)
        {
            assert(reason == Flag::LSQEntryFreed || reason == Flag::Discarded);
            if (!isAnyOutstandingRequest()) {
                delete this;
            } else {
                flags.set(reason);
            }
        }

        /** Helper function used to add a (sub)request, given its address
         * `addr`, size `size` and byte-enable mask `byteEnable`.
         *
         * The request is only added if there is at least one active
         * element in the mask.
         */
        void addReq(Addr addr, unsigned size,
                const std::vector<bool>& byte_enable);

        /** Destructor.
         * The LSQRequest owns the request. If the packet has already been
         * sent, the sender state will be deleted upon receiving the reply.
         */
        virtual ~LSQRequest();

      public:
        /** Convenience getters/setters. */
        /** @{ */
        /** Set up Context numbers. */
        void
        setContext(const ContextID& context_id)
        {
            req()->setContext(context_id);
        }

        const DynInstPtr& instruction() { return _inst; }

        bool hasStaleTranslation() const { return _hasStaleTranslation; }

        virtual void markAsStaleTranslation() = 0;

        /** Set up virtual request.
         * For a previously allocated Request objects.
         */
        void
        setVirt(Addr vaddr, unsigned size, Request::Flags flags_,
                RequestorID requestor_id, Addr pc)
        {
            req()->setVirt(vaddr, size, flags_, requestor_id, pc);
        }

        ContextID contextId() const;

        void
        taskId(const uint32_t& v)
        {
            _taskId = v;
            for (auto& r: _reqs)
                r->taskId(v);
        }

        uint32_t taskId() const { return _taskId; }

        RequestPtr req(int idx = 0) { return _reqs.at(idx); }
        const RequestPtr req(int idx = 0) const { return _reqs.at(idx); }

        Addr getVaddr(int idx = 0) const { return req(idx)->getVaddr(); }
        virtual void initiateTranslation() = 0;

        PacketPtr packet(int idx = 0) { return _packets.at(idx); }

        virtual PacketPtr
        mainPacket()
        {
            assert (_packets.size() == 1);
            return packet();
        }

        virtual RequestPtr
        mainReq()
        {
            assert (_reqs.size() == 1);
            return req();
        }

        /**
         * Test if there is any in-flight translation or mem access request
         */
        bool
        isAnyOutstandingRequest()
        {
            return numInTranslationFragments > 0 ||
                _numOutstandingPackets > 0 ||
                (flags.isSet(Flag::WritebackScheduled) &&
                 !flags.isSet(Flag::WritebackDone));
        }

        /**
         * Test if the LSQRequest has been released, i.e. self-owned.
         * An LSQRequest manages itself when the resources on the LSQ are freed
         * but the translation is still going on and the LSQEntry was freed.
         */
        bool
        isReleased()
        {
            return flags.isSet(Flag::LSQEntryFreed) ||
                flags.isSet(Flag::Discarded);
        }

        bool
        isSplit() const
        {
            return flags.isSet(Flag::IsSplit);
        }

        bool
        needWBToRegister() const
        {
            return flags.isSet(Flag::WriteBackToRegister);
        }
        /** @} */
        virtual bool recvTimingResp(PacketPtr pkt) = 0;
        virtual void sendPacketToCache() = 0;
        virtual void buildPackets() = 0;

        /**
         * Memory mapped IPR accesses
         */
        virtual Cycles handleLocalAccess(
                gem5::ThreadContext *thread, PacketPtr pkt) = 0;

        /**
         * Test if the request accesses a particular cache line.
         */
        virtual bool isCacheBlockHit(Addr blockAddr, Addr cacheBlockMask) = 0;

        /** Update the status to reflect that a packet was sent. */
        void
        packetSent()
        {
            flags.set(Flag::Sent);
        }
        /** Update the status to reflect that a packet was not sent.
         * When a packet fails to be sent, we mark the request as needing a
         * retry. Note that Retry flag is sticky.
         */
        void
        packetNotSent()
        {
            flags.set(Flag::Retry);
            flags.clear(Flag::Sent);
        }

        void sendFragmentToTranslation(int i);
        bool
        isComplete()
        {
            return flags.isSet(Flag::Complete);
        }

        bool
        isInTranslation()
        {
            return _state == State::Translation;
        }

        bool
        isTranslationComplete()
        {
            return flags.isSet(Flag::TranslationStarted) &&
                   !isInTranslation();
        }

        bool
        isTranslationBlocked()
        {
            return _state == State::Translation &&
                flags.isSet(Flag::TranslationStarted) &&
                !flags.isSet(Flag::TranslationFinished);
        }

        bool
        isSent()
        {
            return flags.isSet(Flag::Sent);
        }

        bool
        isPartialFault()
        {
            return _state == State::PartialFault;
        }

        bool
        isMemAccessRequired()
        {
            return (_state == State::Request ||
                    (isPartialFault() && isLoad()));
        }

        void
        setStateToFault()
        {
            setState(State::Fault);
        }

        /**
         * The LSQ entry is cleared
         */
        void
        freeLSQEntry()
        {
            release(Flag::LSQEntryFreed);
        }

        /**
         * The request is discarded (e.g. partial store-load forwarding)
         */
        void
        discard()
        {
            release(Flag::Discarded);
        }

        void
        packetReplied()
        {
            assert(_numOutstandingPackets > 0);
            _numOutstandingPackets--;
            if (_numOutstandingPackets == 0 && isReleased())
                delete this;
        }

        void
        writebackScheduled()
        {
            assert(!flags.isSet(Flag::WritebackScheduled));
            flags.set(Flag::WritebackScheduled);
        }

        void
        writebackDone()
        {
            flags.set(Flag::WritebackDone);
            /* If the lsq resources are already free */
            if (isReleased()) {
                delete this;
            }
        }

        void
        squashTranslation()
        {
            assert(numInTranslationFragments == 0);
            flags.set(Flag::TranslationSquashed);
            /* If we are on our own, self-destruct. */
            if (isReleased()) {
                delete this;
            }
        }

        void
        complete()
        {
            flags.set(Flag::Complete);
        }

        virtual std::string name() const { return "LSQRequest"; }
    };

    class SingleDataRequest : public LSQRequest
    {
      public:
        SingleDataRequest(LSQUnit* port, const DynInstPtr& inst,
                bool isLoad, const Addr& addr, const uint32_t& size,
                const Request::Flags& flags_, PacketDataPtr data=nullptr,
                uint64_t* res=nullptr, AtomicOpFunctorPtr amo_op=nullptr) :
            LSQRequest(port, inst, isLoad, addr, size, flags_, data, res,
                       std::move(amo_op)) {}

        virtual ~SingleDataRequest() {}
        virtual void markAsStaleTranslation();
        virtual void initiateTranslation();
        virtual void finish(const Fault &fault, const RequestPtr &req,
                gem5::ThreadContext* tc, BaseMMU::Mode mode);
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void sendPacketToCache();
        virtual void buildPackets();
        virtual Cycles handleLocalAccess(
                gem5::ThreadContext *thread, PacketPtr pkt);
        virtual bool isCacheBlockHit(Addr blockAddr, Addr cacheBlockMask);
        virtual std::string name() const { return "SingleDataRequest"; }
    };

    // This class extends SingleDataRequest for the purpose
    // of allowing special requests (eg Hardware transactional memory, TLB
    // shootdowns) to bypass irrelevant system elements like translation &
    // squashing.
    class UnsquashableDirectRequest : public SingleDataRequest
    {
      public:
        UnsquashableDirectRequest(LSQUnit* port, const DynInstPtr& inst,
                const Request::Flags& flags_);
        inline virtual ~UnsquashableDirectRequest() {}
        virtual void initiateTranslation();
        virtual void markAsStaleTranslation();
        virtual void finish(const Fault &fault, const RequestPtr &req,
                gem5::ThreadContext* tc, BaseMMU::Mode mode);
        virtual std::string
        name() const
        {
            return "UnsquashableDirectRequest";
        }
    };

    class SplitDataRequest : public LSQRequest
    {
      protected:
        uint32_t numFragments;
        uint32_t numReceivedPackets;
        RequestPtr _mainReq;
        PacketPtr _mainPacket;

      public:
        SplitDataRequest(LSQUnit* port, const DynInstPtr& inst,
                bool isLoad, const Addr& addr, const uint32_t& size,
                const Request::Flags & flags_, PacketDataPtr data=nullptr,
                uint64_t* res=nullptr) :
            LSQRequest(port, inst, isLoad, addr, size, flags_, data, res,
                       nullptr),
            numFragments(0),
            numReceivedPackets(0),
            _mainReq(nullptr),
            _mainPacket(nullptr)
        {
            flags.set(Flag::IsSplit);
        }
        virtual ~SplitDataRequest()
        {
            if (_mainReq) {
                _mainReq = nullptr;
            }
            if (_mainPacket) {
                delete _mainPacket;
                _mainPacket = nullptr;
            }
        }
        virtual void markAsStaleTranslation();
        virtual void finish(const Fault &fault, const RequestPtr &req,
                gem5::ThreadContext* tc, BaseMMU::Mode mode);
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void initiateTranslation();
        virtual void sendPacketToCache();
        virtual void buildPackets();

        virtual Cycles handleLocalAccess(
                gem5::ThreadContext *thread, PacketPtr pkt);
        virtual bool isCacheBlockHit(Addr blockAddr, Addr cacheBlockMask);

        virtual RequestPtr mainReq();
        virtual PacketPtr mainPacket();
        virtual std::string name() const { return "SplitDataRequest"; }
    };

    /** Constructs an LSQ with the given parameters. */
    LSQ(CPU *cpu_ptr, IEW *iew_ptr, const BaseO3CPUParams &params);

    /** Returns the name of the LSQ. */
    std::string name() const;

    /** Sets the pointer to the list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;
    /** Has the LSQ drained? */
    bool isDrained() const;
    /** Takes over execution from another CPU's thread. */
    void takeOverFrom();

    /** Number of entries needed for the given amount of threads.*/
    int entryAmount(ThreadID num_threads);

    /** Ticks the LSQ. */
    void tick();

    /** Inserts a load into the LSQ. */
    void insertLoad(const DynInstPtr &load_inst);
    /** Inserts a store into the LSQ. */
    void insertStore(const DynInstPtr &store_inst);

    /** Executes a load. */
    Fault executeLoad(const DynInstPtr &inst);

    /** Executes a store. */
    Fault executeStore(const DynInstPtr &inst);

    /**
     * Commits loads up until the given sequence number for a specific thread.
     */
    void commitLoads(InstSeqNum &youngest_inst, ThreadID tid);

    /**
     * Commits stores up until the given sequence number for a specific thread.
     */
    void commitStores(InstSeqNum &youngest_inst, ThreadID tid);

    /**
     * Attempts to write back stores until all cache ports are used or the
     * interface becomes blocked.
     */
    void writebackStores();
    /** Same as above, but only for one thread. */
    void writebackStores(ThreadID tid);

    //[shadow]
    void updateLoadSafeState();
    /** Same as above, but only for one thread. */
    void updateLoadSafeState(ThreadID tid);

    /**
     * Squash instructions from a thread until the specified sequence number.
     */
    void squash(const InstSeqNum &squashed_num, ThreadID tid);

    /** Returns whether or not there was a memory ordering violation. */
    bool violation();

    /**
     * Returns whether or not there was a memory ordering violation for a
     * specific thread.
     */
    bool violation(ThreadID tid);

    /** Gets the instruction that caused the memory ordering violation. */
    DynInstPtr getMemDepViolator(ThreadID tid);

    /** Returns the head index of the load queue for a specific thread. */
    int getLoadHead(ThreadID tid);

    /** Returns the sequence number of the head of the load queue. */
    InstSeqNum getLoadHeadSeqNum(ThreadID tid);

    /** Returns the head index of the store queue. */
    int getStoreHead(ThreadID tid);

    /** Returns the sequence number of the head of the store queue. */
    InstSeqNum getStoreHeadSeqNum(ThreadID tid);

    /** Returns the number of instructions in all of the queues. */
    int getCount();
    /** Returns the number of instructions in the queues of one thread. */
    int getCount(ThreadID tid);

    /** Returns the total number of loads in the load queue. */
    int numLoads();
    /** Returns the total number of loads for a single thread. */
    int numLoads(ThreadID tid);

    /** Returns the total number of stores in the store queue. */
    int numStores();
    /** Returns the total number of stores for a single thread. */
    int numStores(ThreadID tid);


    // hardware transactional memory

    int numHtmStarts(ThreadID tid) const;
    int numHtmStops(ThreadID tid) const;
    void resetHtmStartsStops(ThreadID tid);
    uint64_t getLatestHtmUid(ThreadID tid) const;
    void setLastRetiredHtmUid(ThreadID tid, uint64_t htmUid);

    /** Returns the number of free load entries. */
    unsigned numFreeLoadEntries();

    /** Returns the number of free store entries. */
    unsigned numFreeStoreEntries();

    /** Returns the number of free entries for a specific thread. */
    unsigned numFreeEntries(ThreadID tid);

    /** Returns the number of free entries in the LQ for a specific thread. */
    unsigned numFreeLoadEntries(ThreadID tid);

    /** Returns the number of free entries in the SQ for a specific thread. */
    unsigned numFreeStoreEntries(ThreadID tid);

    /** Returns if the LSQ is full (either LQ or SQ is full). */
    bool isFull();
    /**
     * Returns if the LSQ is full for a specific thread (either LQ or SQ is
     * full).
     */
    bool isFull(ThreadID tid);

    /** Returns if the LSQ is empty (both LQ and SQ are empty). */
    bool isEmpty() const;
    /** Returns if all of the LQs are empty. */
    bool lqEmpty() const;
    /** Returns if all of the SQs are empty. */
    bool sqEmpty() const;

    /** Returns if any of the LQs are full. */
    bool lqFull();
    /** Returns if the LQ of a given thread is full. */
    bool lqFull(ThreadID tid);

    /** Returns if any of the SQs are full. */
    bool sqFull();
    /** Returns if the SQ of a given thread is full. */
    bool sqFull(ThreadID tid);

    /**
     * Returns if the LSQ is stalled due to a memory operation that must be
     * replayed.
     */
    bool isStalled();
    /**
     * Returns if the LSQ of a specific thread is stalled due to a memory
     * operation that must be replayed.
     */
    bool isStalled(ThreadID tid);

    /** Returns whether or not there are any stores to write back to memory. */
    bool hasStoresToWB();

    /** Returns whether or not a specific thread has any stores to write back
     * to memory.
     */
    bool hasStoresToWB(ThreadID tid);

    /** Returns the number of stores a specific thread has to write back. */
    int numStoresToWB(ThreadID tid);

    /** Returns if the LSQ will write back to memory this cycle. */
    bool willWB();
    /** Returns if the LSQ of a specific thread will write back to memory this
     * cycle.
     */
    bool willWB(ThreadID tid);

    /** Debugging function to print out all instructions. */
    void dumpInsts() const;
    /** Debugging function to print out instructions from a specific thread. */
    void dumpInsts(ThreadID tid) const;

    /** Executes a read operation, using the load specified at the load
     * index.
     */
    Fault read(LSQRequest* request, ssize_t load_idx);

    /** Executes a store operation, using the store specified at the store
     * index.
     */
    Fault write(LSQRequest* request, uint8_t *data, ssize_t store_idx);

    /** Checks if queues have any marked operations left,
     * and sends the appropriate Sync Completion message if not.
     */
    void checkStaleTranslations();

    /**
     * Retry the previous send that failed.
     */
    void recvReqRetry();

    void completeDataAccess(PacketPtr pkt);
    /**
     * Handles writing back and completing the load or store that has
     * returned from memory.
     *
     * @param pkt Response packet from the memory sub-system
     */
    bool recvTimingResp(PacketPtr pkt);

    void recvTimingSnoopReq(PacketPtr pkt);

    Fault pushRequest(const DynInstPtr& inst, bool isLoad, uint8_t *data,
                      unsigned int size, Addr addr, Request::Flags flags,
                      uint64_t *res, AtomicOpFunctorPtr amo_op,
                      const std::vector<bool>& byte_enable);

    /** The CPU pointer. */
    CPU *cpu;

    /** The IEW stage pointer. */
    IEW *iewStage;

    /** Is D-cache blocked? */
    bool cacheBlocked() const;
    /** Set D-cache blocked status */
    void cacheBlocked(bool v);
    /** Is any store port available to use? */
    bool cachePortAvailable(bool is_load) const;
    /** Another store port is in use */
    void cachePortBusy(bool is_load);

    RequestPort &getDataPort() { return dcachePort; }

  protected:
    /** D-cache is blocked */
    bool _cacheBlocked;
    /** The number of cache ports available each cycle (stores only). */
    int cacheStorePorts;
    /** The number of used cache ports in this cycle by stores. */
    int usedStorePorts;
    /** The number of cache ports available each cycle (loads only). */
    int cacheLoadPorts;
    /** The number of used cache ports in this cycle by loads. */
    int usedLoadPorts;

    /** If the LSQ is currently waiting for stale translations */
    bool waitingForStaleTranslation;
    /** The ID if the transaction that made translations stale */
    Addr staleTranslationWaitTxnId;

    /** The LSQ policy for SMT mode. */
    SMTQueuePolicy lsqPolicy;

    /** Auxiliary function to calculate per-thread max LSQ allocation limit.
     * Depending on a policy, number of entries and possibly number of threads
     * and threshold, this function calculates how many resources each thread
     * can occupy at most.
     */
    static uint32_t
    maxLSQAllocation(SMTQueuePolicy pol, uint32_t entries,
            uint32_t numThreads, uint32_t SMTThreshold)
    {
        if (pol == SMTQueuePolicy::Dynamic) {
            return entries;
        } else if (pol == SMTQueuePolicy::Partitioned) {
            //@todo:make work if part_amt doesnt divide evenly.
            return entries / numThreads;
        } else if (pol == SMTQueuePolicy::Threshold) {
            //Divide up by threshold amount
            //@todo: Should threads check the max and the total
            //amount of the LSQ
            return SMTThreshold;
        }
        return 0;
    }

    /** List of Active Threads in System. */
    std::list<ThreadID> *activeThreads;

    /** Total Size of LQ Entries. */
    unsigned LQEntries;
    /** Total Size of SQ Entries. */
    unsigned SQEntries;

    /** Max LQ Size - Used to Enforce Sharing Policies. */
    unsigned maxLQEntries;

    /** Max SQ Size - Used to Enforce Sharing Policies. */
    unsigned maxSQEntries;

    /** Data port. */
    DcachePort dcachePort;

    /** The LSQ units for individual threads. */
    std::vector<LSQUnit> thread;

    /** Number of Threads. */
    ThreadID numThreads;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_LSQ_HH__
