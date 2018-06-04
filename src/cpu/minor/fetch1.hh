/*
 * Copyright (c) 2013-2014 ARM Limited
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
 *
 * Authors: Andrew Bardsley
 */

/**
 * @file
 *
 *  Fetch1 is responsible for fetching "lines" from memory and passing
 *  them to Fetch2
 */

#ifndef __CPU_MINOR_FETCH1_HH__
#define __CPU_MINOR_FETCH1_HH__

#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/pipe_data.hh"
#include "cpu/base.hh"
#include "mem/packet.hh"

namespace Minor
{

/** A stage responsible for fetching "lines" from memory and passing
 *  them to Fetch2 */
class Fetch1 : public Named
{
  protected:
    /** Exposable fetch port */
    class IcachePort : public MinorCPU::MinorCPUPort
    {
      protected:
        /** My owner */
        Fetch1 &fetch;

      public:
        IcachePort(std::string name, Fetch1 &fetch_, MinorCPU &cpu) :
            MinorCPU::MinorCPUPort(name, cpu), fetch(fetch_)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt)
        { return fetch.recvTimingResp(pkt); }

        void recvReqRetry() { fetch.recvReqRetry(); }
    };

    /** Memory access queuing.
     *
     *  A request can be submitted by pushing it onto the requests queue after
     *  issuing an ITLB lookup (state becomes InTranslation) with a
     *  FetchSenderState senderState containing the current lineSeqNum and
     *  stream/predictionSeqNum.
     *
     *  Translated packets (state becomes Translation) are then passed to the
     *  memory system and the transfers queue (state becomes RequestIssuing).
     *  Retries are handled by leaving the packet on the requests queue and
     *  changing the state to IcacheNeedsRetry).
     *
     *  Responses from the memory system alter the request object (state
     *  become Complete).  Responses can be picked up from the head of the
     *  transfers queue to pass on to Fetch2. */

    /** Structure to hold SenderState info through
     *  translation and memory accesses. */
    class FetchRequest :
        public BaseTLB::Translation, /* For TLB lookups */
        public Packet::SenderState /* For packing into a Packet */
    {
      protected:
        /** Owning fetch unit */
        Fetch1 &fetch;

      public:
        /** Progress of this request through address translation and
         *  memory */
        enum FetchRequestState
        {
            NotIssued, /* Just been made */
            InTranslation, /* Issued to ITLB, must wait for reqply */
            Translated, /* Translation complete */
            RequestIssuing, /* Issued to memory, must wait for response */
            Complete /* Complete.  Either a fault, or a fetched line */
        };

        FetchRequestState state;

        /** Identity of the line that this request will generate */
        InstId id;

        /** FetchRequests carry packets while they're in the requests and
         * transfers responses queues.  When a Packet returns from the memory
         * system, its request needs to have its packet updated as this may
         * have changed in flight */
        PacketPtr packet;

        /** The underlying request that this fetch represents */
        RequestPtr request;

        /** PC to fixup with line address */
        TheISA::PCState pc;

        /** Fill in a fault if one happens during fetch, check this by
         *  picking apart the response packet */
        Fault fault;

        /** Make a packet to use with the memory transaction */
        void makePacket();

        /** Report interface */
        void reportData(std::ostream &os) const;

        /** Is this line out of date with the current stream/prediction
         *  sequence and can it be discarded without orphaning in flight
         *  TLB lookups/memory accesses? */
        bool isDiscardable() const;

        /** Is this a complete read line or fault */
        bool isComplete() const { return state == Complete; }

      protected:
        /** BaseTLB::Translation interface */

        /** Interface for ITLB responses.  We can handle delay, so don't
         *  do anything */
        void markDelayed() { }

        /** Interface for ITLB responses.  Populates self and then passes
         *  the request on to the ports' handleTLBResponse member
         *  function */
        void finish(const Fault &fault_, const RequestPtr &request_,
                    ThreadContext *tc, BaseTLB::Mode mode);

      public:
        FetchRequest(Fetch1 &fetch_, InstId id_, TheISA::PCState pc_) :
            SenderState(),
            fetch(fetch_),
            state(NotIssued),
            id(id_),
            packet(NULL),
            request(),
            pc(pc_),
            fault(NoFault)
        {
            request = std::make_shared<Request>();
        }

        ~FetchRequest();
    };

    typedef FetchRequest *FetchRequestPtr;

  protected:
    /** Construction-assigned data members */

    /** Pointer back to the containing CPU */
    MinorCPU &cpu;

    /** Input port carrying branch requests from Execute */
    Latch<BranchData>::Output inp;
    /** Output port carrying read lines to Fetch2 */
    Latch<ForwardLineData>::Input out;
    /** Input port carrying branch predictions from Fetch2 */
    Latch<BranchData>::Output prediction;

    /** Interface to reserve space in the next stage */
    std::vector<InputBuffer<ForwardLineData>> &nextStageReserve;

    /** IcachePort to pass to the CPU.  Fetch1 is the only module that uses
     *  it. */
    IcachePort icachePort;

    /** Line snap size in bytes.  All fetches clip to make their ends not
     *  extend beyond this limit.  Setting this to the machine L1 cache line
     *  length will result in fetches never crossing line boundaries. */
    unsigned int lineSnap;

    /** Maximum fetch width in bytes.  Setting this (and lineSnap) to the
     *  machine L1 cache line length will result in fetches of whole cache
     *  lines.  Setting this to sizeof(MachInst) will result it fetches of
     *  single instructions (except near the end of lineSnap lines) */
    unsigned int maxLineWidth;

    /** Maximum number of fetches allowed in flight (in queues or memory) */
    unsigned int fetchLimit;

  protected:
    /** Cycle-by-cycle state */

    /** State of memory access for head instruction fetch */
    enum FetchState
    {
        FetchHalted, /* Not fetching, waiting to be woken by transition
            to FetchWaitingForPC.  The PC is not valid in this state */
        FetchWaitingForPC, /* Not fetching, waiting for stream change.
            This doesn't stop issued fetches from being returned and
            processed or for branches to change the state to Running. */
        FetchRunning /* Try to fetch, when possible */
    };

    /** Stage cycle-by-cycle state */

    struct Fetch1ThreadInfo {

        /** Consturctor to initialize all fields. */
        Fetch1ThreadInfo() :
            state(FetchWaitingForPC),
            pc(TheISA::PCState(0)),
            streamSeqNum(InstId::firstStreamSeqNum),
            predictionSeqNum(InstId::firstPredictionSeqNum),
            blocked(false),
            wakeupGuard(false)
        { }

        Fetch1ThreadInfo(const Fetch1ThreadInfo& other) :
            state(other.state),
            pc(other.pc),
            streamSeqNum(other.streamSeqNum),
            predictionSeqNum(other.predictionSeqNum),
            blocked(other.blocked)
        { }

        FetchState state;

        /** Fetch PC value. This is updated by branches from Execute, branch
         *  prediction targets from Fetch2 and by incrementing it as we fetch
         *  lines subsequent to those two sources. */
        TheISA::PCState pc;

        /** Stream sequence number.  This changes on request from Execute and is
         *  used to tag instructions by the fetch stream to which they belong.
         *  Execute originates new prediction sequence numbers. */
        InstSeqNum streamSeqNum;

        /** Prediction sequence number.  This changes when requests from Execute
         *  or Fetch2 ask for a change of fetch address and is used to tag lines
         *  by the prediction to which they belong.  Fetch2 originates
         *  prediction sequence numbers. */
        InstSeqNum predictionSeqNum;

        /** Blocked indication for report */
        bool blocked;

        /** Signal to guard against sleeping first cycle of wakeup */
        bool wakeupGuard;
    };

    std::vector<Fetch1ThreadInfo> fetchInfo;
    ThreadID threadPriority;

    /** State of memory access for head instruction fetch */
    enum IcacheState
    {
        IcacheRunning, /* Default. Step icache queues when possible */
        IcacheNeedsRetry /* Request rejected, will be asked to retry */
    };

    typedef Queue<FetchRequestPtr,
        ReportTraitsPtrAdaptor<FetchRequestPtr>,
        NoBubbleTraits<FetchRequestPtr> >
        FetchQueue;

    /** Queue of address translated requests from Fetch1 */
    FetchQueue requests;

    /** Queue of in-memory system requests and responses */
    FetchQueue transfers;

    /** Retry state of icache_port */
    IcacheState icacheState;

    /** Sequence number for line fetch used for ordering lines to flush */
    InstSeqNum lineSeqNum;

    /** Count of the number fetches which have left the transfers queue
     *  and are in the 'wild' in the memory system.  Try not to rely on
     *  this value, it's better to code without knowledge of the number
     *  of outstanding accesses */
    unsigned int numFetchesInMemorySystem;
    /** Number of requests inside the ITLB rather than in the queues.
     *  All requests so located *must* have reserved space in the
     *  transfers queue */
    unsigned int numFetchesInITLB;

  protected:
    friend std::ostream &operator <<(std::ostream &os,
        Fetch1::FetchState state);

    /** Start fetching from a new address. */
    void changeStream(const BranchData &branch);

    /** Update streamSeqNum and predictionSeqNum from the given branch (and
     *  assume these have changed and discard (on delivery) all lines in
     *  flight) */
    void updateExpectedSeqNums(const BranchData &branch);

    /** Convert a response to a ForwardLineData */
    void processResponse(FetchRequestPtr response,
        ForwardLineData &line);

    friend std::ostream &operator <<(std::ostream &os,
        IcacheState state);


    /** Use the current threading policy to determine the next thread to
     *  fetch from. */
    ThreadID getScheduledThread();

    /** Insert a line fetch into the requests.  This can be a partial
     *  line request where the given address has a non-0 offset into a
     *  line. */
    void fetchLine(ThreadID tid);

    /** Try and issue a fetch for a translated request at the
     *  head of the requests queue.  Also tries to move the request
     *  between queues */
    void tryToSendToTransfers(FetchRequestPtr request);

    /** Try to send (or resend) a memory request's next/only packet to
     *  the memory system.  Returns true if the fetch was successfully
     *  sent to memory */
    bool tryToSend(FetchRequestPtr request);

    /** Move a request between queues */
    void moveFromRequestsToTransfers(FetchRequestPtr request);

    /** Step requests along between requests and transfers queues */
    void stepQueues();

    /** Pop a request from the given queue and correctly deallocate and
     *  discard it. */
    void popAndDiscard(FetchQueue &queue);

    /** Handle pushing a TLB response onto the right queue */
    void handleTLBResponse(FetchRequestPtr response);

    /** Returns the total number of queue occupancy, in-ITLB and
     *  in-memory system fetches */
    unsigned int numInFlightFetches();

    /** Print the appropriate MinorLine line for a fetch response */
    void minorTraceResponseLine(const std::string &name,
        FetchRequestPtr response) const;

    /** Memory interface */
    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();

  public:
    Fetch1(const std::string &name_,
        MinorCPU &cpu_,
        MinorCPUParams &params,
        Latch<BranchData>::Output inp_,
        Latch<ForwardLineData>::Input out_,
        Latch<BranchData>::Output prediction_,
        std::vector<InputBuffer<ForwardLineData>> &next_stage_input_buffer);

  public:
    /** Returns the IcachePort owned by this Fetch1 */
    MinorCPU::MinorCPUPort &getIcachePort() { return icachePort; }

    /** Pass on input/buffer data to the output if you can */
    void evaluate();

    /** Initiate fetch1 fetching */
    void wakeupFetch(ThreadID tid);

    void minorTrace() const;

    /** Is this stage drained?  For Fetch1, draining is initiated by
     *  Execute signalling a branch with the reason HaltFetch */
    bool isDrained();
};

}

#endif /* __CPU_MINOR_FETCH1_HH__ */
