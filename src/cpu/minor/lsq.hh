/*
 * Copyright (c) 2013-2014, 2018 ARM Limited
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

/**
 * @file
 *
 *  A load/store queue that allows outstanding reads and writes.
 *
 */

#ifndef __CPU_MINOR_NEW_LSQ_HH__
#define __CPU_MINOR_NEW_LSQ_HH__

#include <string>
#include <vector>

#include "base/named.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/pipe_data.hh"
#include "cpu/minor/trace.hh"
#include "mem/packet.hh"

namespace gem5
{

namespace minor
{

/* Forward declaration */
class Execute;

class LSQ : public Named
{
  protected:
    /** My owner(s) */
    MinorCPU &cpu;
    Execute &execute;

  protected:
    /** State of memory access for head access. */
    enum MemoryState
    {
        MemoryRunning,   /* Default. Step dcache queues when possible. */
        MemoryNeedsRetry /* Request rejected, will be asked to retry */
    };

    /** Print MemoryState values as shown in the enum definition */
    friend std::ostream &operator<<(std::ostream &os, MemoryState state);

    /** Coverage of one address range with another */
    enum AddrRangeCoverage
    {
        PartialAddrRangeCoverage, /* Two ranges partly overlap */
        FullAddrRangeCoverage,    /* One range fully covers another */
        NoAddrRangeCoverage       /* Two ranges are disjoint */
    };

    /** Exposable data port */
    class DcachePort : public MinorCPU::MinorCPUPort
    {
      protected:
        /** My owner */
        LSQ &lsq;

      public:
        DcachePort(std::string name, LSQ &lsq_, MinorCPU &cpu)
            : MinorCPU::MinorCPUPort(name, cpu), lsq(lsq_)
        {}

      protected:
        bool
        recvTimingResp(PacketPtr pkt) override
        {
            return lsq.recvTimingResp(pkt);
        }

        void
        recvReqRetry() override
        {
            lsq.recvReqRetry();
        }

        bool
        isSnooping() const override
        {
            return true;
        }

        void
        recvTimingSnoopReq(PacketPtr pkt) override
        {
            return lsq.recvTimingSnoopReq(pkt);
        }

        void
        recvFunctionalSnoop(PacketPtr pkt) override
        {}
    };

    DcachePort dcachePort;

  public:
    /** Derived SenderState to carry data access info. through address
     *  translation, the queues in this port and back from the memory
     *  system. */
    class LSQRequest :
        public BaseMMU::Translation, /* For TLB lookups */
        public Packet::SenderState   /* For packing into a Packet */
    {
      public:
        /** Owning port */
        LSQ &port;

        /** Instruction which made this request */
        MinorDynInstPtr inst;

        /** Load/store indication used for building packet.  This isn't
         *  carried by Request so we need to keep it here */
        bool isLoad;

        /** Dynamically allocated and populated data carried for
         *  building write packets */
        PacketDataPtr data;

        /* Requests carry packets on their way to the memory system.
         *  When a Packet returns from the memory system, its
         *  request needs to have its packet updated as this
         *  may have changed in flight */
        PacketPtr packet;

        /** The underlying request of this LSQRequest */
        RequestPtr request;

        /** Res from pushRequest */
        uint64_t *res;

        /** Was skipped.  Set to indicate any reason (faulted, bad
         *  stream sequence number, in a fault shadow) that this
         *  request did not perform a memory transfer */
        bool skipped;

        /** This in an access other than a normal cacheable load
         *  that's visited the memory system */
        bool issuedToMemory;

        /** Address translation is delayed due to table walk */
        bool isTranslationDelayed;

        enum LSQRequestState
        {
            NotIssued,          /* Newly created */
            InTranslation,      /* TLB accessed, no reply yet */
            Translated,         /* Finished address translation */
            Failed,             /* The starting start of FailedDataRequests */
            RequestIssuing,     /* Load/store issued to memory in the requests
                    queue */
            StoreToStoreBuffer, /* Store in transfers on its way to the
                store buffer */
            RequestNeedsRetry,  /* Retry needed for load */
            StoreInStoreBuffer, /* Store in the store buffer, before issuing
                a memory transfer */
            StoreBufferIssuing, /* Store in store buffer and has been
                issued */
            StoreBufferNeedsRetry, /* Retry needed for store */
            /* All completed states.  Includes
                completed loads, TLB faults and skipped requests whose
                seqNum's no longer match */
            Complete
        };

        LSQRequestState state;

      protected:
        /** BaseMMU::Translation interface */
        void
        markDelayed()
        {
            isTranslationDelayed = true;
        }

        /** Instructions may want to suppress translation faults (e.g.
         *  non-faulting vector loads).*/
        void tryToSuppressFault();

        void disableMemAccess();
        void completeDisabledMemAccess();

      public:
        LSQRequest(LSQ &port_, MinorDynInstPtr inst_, bool isLoad_,
                   PacketDataPtr data_ = NULL, uint64_t *res_ = NULL);

        virtual ~LSQRequest();

      public:
        /** Make a packet to use with the memory transaction */
        void makePacket();

        /** Was no memory access attempted for this request? */
        bool
        skippedMemAccess()
        {
            return skipped;
        }

        /** Set this request as having been skipped before a memory
         *  transfer was attempt */
        void
        setSkipped()
        {
            skipped = true;
        }

        /** Does address range req1 (req1_addr to req1_addr + req1_size - 1)
         *  fully cover, partially cover or not cover at all the range req2 */
        static AddrRangeCoverage containsAddrRangeOf(Addr req1_addr,
                                                     unsigned int req1_size,
                                                     Addr req2_addr,
                                                     unsigned int req2_size);

        /** Does this request's address range fully cover the range
         *  of other_request? */
        AddrRangeCoverage containsAddrRangeOf(LSQRequest *other_request);

        /** Start the address translation process for this request.  This
         *  will issue a translation request to the TLB. */
        virtual void startAddrTranslation() = 0;

        /** Get the next packet to issue for this request.  For split
         *  transfers, it will be necessary to step through the available
         *  packets by calling do { getHeadPacket ; stepToNextPacket } while
         *  (!sentAllPackets) and by retiring response using retireResponse */
        virtual PacketPtr getHeadPacket() = 0;

        /** Step to the next packet for the next call to getHeadPacket */
        virtual void stepToNextPacket() = 0;

        /** Have all packets been sent? */
        virtual bool sentAllPackets() = 0;

        /** True if this request has any issued packets in the memory
         *  system and so can't be interrupted until it gets responses */
        virtual bool hasPacketsInMemSystem() = 0;

        /** Retire a response packet into the LSQRequest packet possibly
         *  completing this transfer */
        virtual void retireResponse(PacketPtr packet_) = 0;

        /** Is this a request a barrier? */
        virtual bool isBarrier();

        /** This request, once processed by the requests/transfers
         *  queues, will need to go to the store buffer */
        bool needsToBeSentToStoreBuffer();

        /** Set state and output trace output */
        void setState(LSQRequestState new_state);

        /** Has this request been completed.  This includes *all* reasons
         *  for completion: successful transfers, faults, skipped because
         *  of preceding faults */
        bool isComplete() const;

        /** MinorTrace report interface */
        void reportData(std::ostream &os) const;
    };

    typedef LSQRequest *LSQRequestPtr;

    friend std::ostream &operator<<(std::ostream &os, AddrRangeCoverage state);

    friend std::ostream &operator<<(std::ostream &os,
                                    LSQRequest::LSQRequestState state);

  protected:
    /** Special request types that don't actually issue memory requests */
    class SpecialDataRequest : public LSQRequest
    {
      protected:
        /** TLB interace */
        void
        finish(const Fault &fault_, const RequestPtr &request_,
               ThreadContext *tc, BaseMMU::Mode mode)
        {}

      public:
        /** Send single translation request */
        void
        startAddrTranslation()
        {}

        /** Get the head packet as counted by numIssuedFragments */
        PacketPtr
        getHeadPacket()
        {
            fatal("No packets in a SpecialDataRequest");
        }

        /** Step on numIssuedFragments */
        void
        stepToNextPacket()
        {}

        /** Has no packets to send */
        bool
        sentAllPackets()
        {
            return true;
        }

        /** Never sends any requests */
        bool
        hasPacketsInMemSystem()
        {
            return false;
        }

        /** Keep the given packet as the response packet
         *  LSQRequest::packet */
        void
        retireResponse(PacketPtr packet_)
        {}

      public:
        SpecialDataRequest(LSQ &port_, MinorDynInstPtr inst_)
            : /* Say this is a load, not actually relevant */
              LSQRequest(port_, inst_, true, NULL, 0)
        {}
    };

    /** FailedDataRequest represents requests from instructions that
     *  failed their predicates but need to ride the requests/transfers
     *  queues to maintain trace ordering */
    class FailedDataRequest : public SpecialDataRequest
    {
      public:
        FailedDataRequest(LSQ &port_, MinorDynInstPtr inst_)
            : SpecialDataRequest(port_, inst_)
        {
            state = Failed;
        }
    };

    /** Request for doing barrier accounting in the store buffer.  Not
     *  for use outside that unit */
    class BarrierDataRequest : public SpecialDataRequest
    {
      public:
        bool
        isBarrier()
        {
            return true;
        }

      public:
        BarrierDataRequest(LSQ &port_, MinorDynInstPtr inst_)
            : SpecialDataRequest(port_, inst_)
        {
            state = Complete;
        }
    };

    /** SingleDataRequest is used for requests that don't fragment */
    class SingleDataRequest : public LSQRequest
    {
      protected:
        /** TLB interace */
        void finish(const Fault &fault_, const RequestPtr &request_,
                    ThreadContext *tc, BaseMMU::Mode mode);

        /** Has my only packet been sent to the memory system but has not
         *  yet been responded to */
        bool packetInFlight;

        /** Has the packet been at least sent to the memory system? */
        bool packetSent;

      public:
        /** Send single translation request */
        void startAddrTranslation();

        /** Get the head packet as counted by numIssuedFragments */
        PacketPtr
        getHeadPacket()
        {
            return packet;
        }

        /** Remember that the packet has been sent */
        void
        stepToNextPacket()
        {
            packetInFlight = true;
            packetSent = true;
        }

        /** Has packet been sent */
        bool
        hasPacketsInMemSystem()
        {
            return packetInFlight;
        }

        /** packetInFlight can become false again, so need to check
         *  packetSent */
        bool
        sentAllPackets()
        {
            return packetSent;
        }

        /** Keep the given packet as the response packet
         *  LSQRequest::packet */
        void retireResponse(PacketPtr packet_);

      public:
        SingleDataRequest(LSQ &port_, MinorDynInstPtr inst_, bool isLoad_,
                          PacketDataPtr data_ = NULL, uint64_t *res_ = NULL)
            : LSQRequest(port_, inst_, isLoad_, data_, res_),
              packetInFlight(false),
              packetSent(false)
        {}
    };

    class SplitDataRequest : public LSQRequest
    {
      protected:
        /** Event to step between translations */
        EventFunctionWrapper translationEvent;

      protected:
        /** Number of fragments this request is split into */
        unsigned int numFragments;

        /** Number of fragments in the address translation mechanism */
        unsigned int numInTranslationFragments;

        /** Number of fragments that have completed address translation,
         *  (numTranslatedFragments + numInTranslationFragments) <=
         *  numFragments.  When numTranslatedFramgents == numFragments,
         *  translation is complete */
        unsigned int numTranslatedFragments;

        /** Number of fragments already issued (<= numFragments) */
        unsigned int numIssuedFragments;

        /** Number of fragments retired back to this request */
        unsigned int numRetiredFragments;

        /** Fragment Requests corresponding to the address ranges of
         *  each fragment */
        std::vector<RequestPtr> fragmentRequests;

        /** Packets matching fragmentRequests to issue fragments to memory */
        std::vector<Packet *> fragmentPackets;

      protected:
        /** TLB response interface */
        void finish(const Fault &fault_, const RequestPtr &request_,
                    ThreadContext *tc, BaseMMU::Mode mode);

      public:
        SplitDataRequest(LSQ &port_, MinorDynInstPtr inst_, bool isLoad_,
                         PacketDataPtr data_ = NULL, uint64_t *res_ = NULL);

        ~SplitDataRequest();

      public:
        /** Make all the Requests for this transfer's fragments so that those
         *  requests can be sent for address translation */
        void makeFragmentRequests();

        /** Make the packets to go with the requests so they can be sent to
         *  the memory system */
        void makeFragmentPackets();

        /** Start a loop of do { sendNextFragmentToTranslation ;
         *  translateTiming ; finish } while (numTranslatedFragments !=
         *  numFragments) to complete all this requests' fragments' address
         *  translations */
        void startAddrTranslation();

        /** Get the head packet as counted by numIssuedFragments */
        PacketPtr getHeadPacket();

        /** Step on numIssuedFragments */
        void stepToNextPacket();

        bool
        hasPacketsInMemSystem()
        {
            return numIssuedFragments != numRetiredFragments;
        }

        /** Have we stepped past the end of fragmentPackets? */
        bool
        sentAllPackets()
        {
            return numIssuedFragments == numTranslatedFragments;
        }

        /** For loads, paste the response data into the main
         *  response packet */
        void retireResponse(PacketPtr packet_);

        /** Part of the address translation loop, see startAddTranslation */
        void sendNextFragmentToTranslation();
    };

    /** Store buffer.  This contains stores which have been committed
     *  but whose memory transfers have not yet been issued. Load data
     *  can be forwarded out of the store buffer */
    class StoreBuffer : public Named
    {
      public:
        /** My owner */
        LSQ &lsq;

        /** Number of slots, this is a bound on the size of slots */
        const unsigned int numSlots;

        /** Maximum number of stores that can be issued per cycle */
        const unsigned int storeLimitPerCycle;

      public:
        /** Queue of store requests on their way to memory */
        std::deque<LSQRequestPtr> slots;

        /** Number of occupied slots which have not yet issued a
         *  memory access */
        unsigned int numUnissuedAccesses;

      public:
        StoreBuffer(std::string name_, LSQ &lsq_,
                    unsigned int store_buffer_size,
                    unsigned int store_limit_per_cycle);

      public:
        /** Can a new request be inserted into the queue? */
        bool canInsert() const;

        /** Delete the given request and free the slot it occupied */
        void deleteRequest(LSQRequestPtr request);

        /** Insert a request at the back of the queue */
        void insert(LSQRequestPtr request);

        /** Look for a store which satisfies the given load.  Returns an
         *  indication whether the forwarding request can be wholly,
         *  partly or not all all satisfied.  If the request can be
         *  wholly satisfied, the store buffer slot number which can be used
         *  is returned in found_slot */
        AddrRangeCoverage canForwardDataToLoad(LSQRequestPtr request,
                                               unsigned int &found_slot);

        /** Fill the given packet with appropriate date from slot
         *  slot_number */
        void forwardStoreData(LSQRequestPtr load, unsigned int slot_number);

        /** Number of stores in the store buffer which have not been
         *  completely issued to the memory system */
        unsigned int
        numUnissuedStores()
        {
            return numUnissuedAccesses;
        }

        /** Count a store being issued to memory by decrementing
         *  numUnissuedAccesses.  Does not count barrier requests as they
         *  will be handles as barriers are cleared from the buffer */
        void countIssuedStore(LSQRequestPtr request);

        /** Drained if there is absolutely nothing left in the buffer */
        bool
        isDrained() const
        {
            return slots.empty();
        }

        /** Try to issue more stores to memory */
        void step();

        /** Report queue contents for MinorTrace */
        void minorTrace() const;
    };

  protected:
    /** Most recent execSeqNum of a memory barrier instruction or
     *  0 if there are no in-flight barriers.  Useful as a
     *  dependency for early-issued memory operations */
    std::vector<InstSeqNum> lastMemBarrier;

  public:
    /** Retry state of last issued memory transfer */
    MemoryState state;

    /** Maximum number of in-flight accesses issued to the memory system */
    const unsigned int inMemorySystemLimit;

    /** Memory system access width (and snap) in bytes */
    const Addr lineWidth;

  public:
    /** The LSQ consists of three queues: requests, transfers and the
     *  store buffer storeBuffer. */

    typedef Queue<LSQRequestPtr, ReportTraitsPtrAdaptor<LSQRequestPtr>,
                  NoBubbleTraits<LSQRequestPtr> >
        LSQQueue;

    /** requests contains LSQRequests which have been issued to the TLB by
     *  calling ExecContext::readMem/writeMem (which in turn calls
     *  LSQ::pushRequest and LSQRequest::startAddrTranslation).  Once they
     *  have a physical address, requests at the head of requests can be
     *  issued to the memory system.  At this stage, it cannot be clear that
     *  memory accesses *must* happen (that there are no preceding faults or
     *  changes of flow of control) and so only cacheable reads are issued
     *  to memory.
     *  Cacheable stores are not issued at all (and just pass through
     *  'transfers' in order) and all other transfers are stalled in requests
     *  until their corresponding instructions are at the head of the
     *  inMemInsts instruction queue and have the right streamSeqNum. */
    LSQQueue requests;

    /** Once issued to memory (or, for stores, just had their
     *  state changed to StoreToStoreBuffer) LSQRequests pass through
     *  transfers waiting for memory responses.  At the head of transfers,
     *  Execute::commitInst can pick up the memory response for a request
     *  using LSQ::findResponse.  Responses to be committed can then
     *  have ExecContext::completeAcc on them.  Stores can then be pushed
     *  into the store buffer.  All other transfers will then be complete. */
    LSQQueue transfers;

    /* The store buffer contains committed cacheable stores on
     * their way to memory decoupled from subsequence instruction execution.
     * Before trying to issue a cacheable read from 'requests' to memory,
     * the store buffer is checked to see if a previous store contains the
     * needed data (StoreBuffer::canForwardDataToLoad) which can be
     * forwarded in lieu of a memory access.  If there are outstanding
     * stores in the transfers queue, they must be promoted to the store
     * buffer (and so be commited) before they can be correctly checked
     * for forwarding. */
    StoreBuffer storeBuffer;

  protected:
    /** Count of the number of mem. accesses which have left the
     *  requests queue and are in the 'wild' in the memory system and who
     *  *must not* be interrupted as they are not normal cacheable
     *  accesses.  This is a count of the number of in-flight requests
     *  with issuedToMemory set who have visited tryToSendRequest at least
     *  once */
    unsigned int numAccessesInMemorySystem;

    /** Number of requests in the DTLB in the requests queue */
    unsigned int numAccessesInDTLB;

    /** The number of stores in the transfers queue.  Useful when
     *  testing if the store buffer contains all the forwardable stores */
    unsigned int numStoresInTransfers;

    /** The number of accesses which have been issued to the memory
     *  system but have not been committed/discarded *excluding*
     *  cacheable normal loads which don't need to be tracked */
    unsigned int numAccessesIssuedToMemory;

    /** The request (from either requests or the store buffer) which is
     *  currently waiting have its memory access retried */
    LSQRequestPtr retryRequest;

    /** Address Mask for a cache block (e.g. ~(cache_block_size-1)) */
    Addr cacheBlockMask;

  protected:
    /** Try and issue a memory access for a translated request at the
     *  head of the requests queue.  Also tries to move the request
     *  between queues */
    void tryToSendToTransfers(LSQRequestPtr request);

    /** Try to send (or resend) a memory request's next/only packet to
     *  the memory system.  Returns true if the request was successfully
     *  sent to memory (and was also the last packet in a transfer) */
    bool tryToSend(LSQRequestPtr request);

    /** Clear a barrier (if it's the last one marked up in lastMemBarrier) */
    void clearMemBarrier(MinorDynInstPtr inst);

    /** Move a request between queues */
    void moveFromRequestsToTransfers(LSQRequestPtr request);

    /** Can a request be sent to the memory system */
    bool canSendToMemorySystem();

    /** Snoop other threads monitors on memory system accesses */
    void threadSnoop(LSQRequestPtr request);

  public:
    LSQ(std::string name_, std::string dcache_port_name_, MinorCPU &cpu_,
        Execute &execute_, unsigned int max_accesses_in_memory_system,
        unsigned int line_width, unsigned int requests_queue_size,
        unsigned int transfers_queue_size, unsigned int store_buffer_size,
        unsigned int store_buffer_cycle_store_limit);

    virtual ~LSQ();

  public:
    /** Step checks the queues to see if their are issuable transfers
     *  which were not otherwise picked up by tests at the end of other
     *  events.
     *
     *  Steppable actions include deferred actions which couldn't be
     *  cascaded on the end of a memory response/TLB response event
     *  because of resource congestion. */
    void step();

    /** Is their space in the request queue to be able to push a request by
     *  issuing an isMemRef instruction */
    bool
    canRequest()
    {
        return requests.unreservedRemainingSpace() != 0;
    }

    /** Returns a response if it's at the head of the transfers queue and
     *  it's either complete or can be sent on to the store buffer.  After
     *  calling, the request still remains on the transfer queue until
     *  popResponse is called */
    LSQRequestPtr findResponse(MinorDynInstPtr inst);

    /** Sanity check and pop the head response */
    void popResponse(LSQRequestPtr response);

    /** Must check this before trying to insert into the store buffer */
    bool
    canPushIntoStoreBuffer() const
    {
        return storeBuffer.canInsert();
    }

    /** A store has been committed, please move it to the store buffer */
    void sendStoreToStoreBuffer(LSQRequestPtr request);

    /** Are there any accesses other than normal cached loads in the
     *  memory system or having received responses which need to be
     *  handled for their instruction's to be completed */
    bool
    accessesInFlight() const
    {
        return numAccessesIssuedToMemory != 0;
    }

    /** A memory barrier instruction has been issued, remember its
     *  execSeqNum that we can avoid issuing memory ops until it is
     *  committed */
    void issuedMemBarrierInst(MinorDynInstPtr inst);

    /** Get the execSeqNum of the last issued memory barrier */
    InstSeqNum
    getLastMemBarrier(ThreadID thread_id) const
    {
        return lastMemBarrier[thread_id];
    }

    /** Is there nothing left in the LSQ */
    bool isDrained();

    /** May need to be ticked next cycle as one of the queues contains
     *  an actionable transfers or address translation */
    bool needsToTick();

    /** Complete a barrier instruction.  Where committed, makes a
     *  BarrierDataRequest and pushed it into the store buffer */
    void completeMemBarrierInst(MinorDynInstPtr inst, bool committed);

    /** Single interface for readMem/writeMem/amoMem to issue requests into
     *  the LSQ */
    Fault
    pushRequest(MinorDynInstPtr inst, bool isLoad, uint8_t *data,
                unsigned int size, Addr addr, Request::Flags flags,
                uint64_t *res, AtomicOpFunctorPtr amo_op,
                const std::vector<bool> &byte_enable = std::vector<bool>());

    /** Push a predicate failed-representing request into the queues just
     *  to maintain commit order */
    void pushFailedRequest(MinorDynInstPtr inst);

    /** Memory interface */
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    void recvTimingSnoopReq(PacketPtr pkt);

    /** Return the raw-bindable port */
    MinorCPU::MinorCPUPort &
    getDcachePort()
    {
        return dcachePort;
    }

    void minorTrace() const;
};

/** Make a suitable packet for the given request.  If the request is a store,
 *  data will be the payload data.  If sender_state is NULL, it won't be
 *  pushed into the packet as senderState */
PacketPtr makePacketForRequest(const RequestPtr &request, bool isLoad,
                               Packet::SenderState *sender_state = NULL,
                               PacketDataPtr data = NULL);

} // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_NEW_LSQ_HH__ */
