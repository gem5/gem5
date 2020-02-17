/*
 * Copyright (c) 2012-2013, 2016-2019 ARM Limited
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

#ifndef __CPU_TRAFFIC_GEN_BASE_HH__
#define __CPU_TRAFFIC_GEN_BASE_HH__

#include <memory>
#include <tuple>
#include <unordered_map>

#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "mem/qport.hh"
#include "sim/clocked_object.hh"

class BaseGen;
class StreamGen;
class System;
struct BaseTrafficGenParams;

/**
 * The traffic generator is a master module that generates stimuli for
 * the memory system, based on a collection of simple generator
 * behaviours that are either probabilistic or based on traces. It can
 * be used stand alone for creating test cases for interconnect and
 * memory controllers, or function as a black box replacement for
 * system components that are not yet modelled in detail, e.g. a video
 * engine or baseband subsystem.
 */
class BaseTrafficGen : public ClockedObject
{
    friend class BaseGen;

  protected: // Params
    /**
     * The system used to determine which mode we are currently operating
     * in.
     */
    System *const system;

    /**
     * Determine whether to add elasticity in the request injection,
     * thus responding to backpressure by slowing things down.
     */
    const bool elasticReq;

    /**
     * Time to tolerate waiting for retries (not making progress),
     * until we declare things broken.
     */
    const Tick progressCheck;

  private:
    /**
     * Receive a retry from the neighbouring port and attempt to
     * resend the waiting packet.
     */
    void recvReqRetry();

    void retryReq();

    bool recvTimingResp(PacketPtr pkt);

    /** Transition to the next generator */
    void transition();

    /**
     * Schedule the update event based on nextPacketTick and
     * nextTransitionTick.
     */
    void scheduleUpdate();

    /**
     * Method to inform the user we have made no progress.
     */
    void noProgress();

    /**
     * Event to keep track of our progress, or lack thereof.
     */
    EventFunctionWrapper noProgressEvent;

    /** Time of next transition */
    Tick nextTransitionTick;

    /** Time of the next packet. */
    Tick nextPacketTick;

    const int maxOutstandingReqs;


    /** Master port specialisation for the traffic generator */
    class TrafficGenPort : public MasterPort
    {
      public:

        TrafficGenPort(const std::string& name, BaseTrafficGen& traffic_gen)
            : MasterPort(name, &traffic_gen), trafficGen(traffic_gen)
        { }

      protected:

        void recvReqRetry() { trafficGen.recvReqRetry(); }

        bool recvTimingResp(PacketPtr pkt)
        { return trafficGen.recvTimingResp(pkt); }

        void recvTimingSnoopReq(PacketPtr pkt) { }

        void recvFunctionalSnoop(PacketPtr pkt) { }

        Tick recvAtomicSnoop(PacketPtr pkt) { return 0; }

      private:

        BaseTrafficGen& trafficGen;

    };

    /**
     * Schedules event for next update and generates a new packet or
     * requests a new generatoir depending on the current time.
     */
    void update();

    /** The instance of master port used by the traffic generator. */
    TrafficGenPort port;

    /** Packet waiting to be sent. */
    PacketPtr retryPkt;

    /** Tick when the stalled packet was meant to be sent. */
    Tick retryPktTick;

    /** Set when we blocked waiting for outstanding reqs */
    bool blockedWaitingResp;

    /**
     * Puts this packet in the waitingResp list and returns true if
     * we are above the maximum number of oustanding requests.
     */
    bool allocateWaitingRespSlot(PacketPtr pkt)
    {
        assert(waitingResp.find(pkt->req) == waitingResp.end());
        assert(pkt->needsResponse());

        waitingResp[pkt->req] = curTick();

        return (maxOutstandingReqs > 0) &&
               (waitingResp.size() > maxOutstandingReqs);
    }

    /** Event for scheduling updates */
    EventFunctionWrapper updateEvent;

  protected: // Stats
    /** Reqs waiting for response **/
    std::unordered_map<RequestPtr,Tick> waitingResp;

    struct StatGroup : public Stats::Group {
        StatGroup(Stats::Group *parent);

        /** Count the number of dropped requests. */
        Stats::Scalar numSuppressed;

        /** Count the number of generated packets. */
        Stats::Scalar numPackets;

        /** Count the number of retries. */
        Stats::Scalar numRetries;

        /** Count the time incurred from back-pressure. */
        Stats::Scalar retryTicks;

        /** Count the number of bytes read. */
        Stats::Scalar bytesRead;

        /** Count the number of bytes written. */
        Stats::Scalar bytesWritten;

        /** Total num of ticks read reqs took to complete  */
        Stats::Scalar totalReadLatency;

        /** Total num of ticks write reqs took to complete  */
        Stats::Scalar totalWriteLatency;

        /** Count the number reads. */
        Stats::Scalar totalReads;

        /** Count the number writes. */
        Stats::Scalar totalWrites;

        /** Avg num of ticks each read req took to complete  */
        Stats::Formula avgReadLatency;

        /** Avg num of ticks each write reqs took to complete  */
        Stats::Formula avgWriteLatency;

        /** Read bandwidth in bytes/s  */
        Stats::Formula readBW;

        /** Write bandwidth in bytes/s  */
        Stats::Formula writeBW;
    } stats;

  public:
    BaseTrafficGen(const BaseTrafficGenParams* p);

    ~BaseTrafficGen();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void init() override;

    DrainState drain() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // Generator factory methods
    std::shared_ptr<BaseGen> createIdle(Tick duration);
    std::shared_ptr<BaseGen> createExit(Tick duration);

    std::shared_ptr<BaseGen> createLinear(
        Tick duration,
        Addr start_addr, Addr end_addr, Addr blocksize,
        Tick min_period, Tick max_period,
        uint8_t read_percent, Addr data_limit);

    std::shared_ptr<BaseGen> createRandom(
        Tick duration,
        Addr start_addr, Addr end_addr, Addr blocksize,
        Tick min_period, Tick max_period,
        uint8_t read_percent, Addr data_limit);

    std::shared_ptr<BaseGen> createDram(
        Tick duration,
        Addr start_addr, Addr end_addr, Addr blocksize,
        Tick min_period, Tick max_period,
        uint8_t read_percent, Addr data_limit,
        unsigned int num_seq_pkts, unsigned int page_size,
        unsigned int nbr_of_banks_DRAM, unsigned int nbr_of_banks_util,
        Enums::AddrMap addr_mapping,
        unsigned int nbr_of_ranks);

    std::shared_ptr<BaseGen> createDramRot(
        Tick duration,
        Addr start_addr, Addr end_addr, Addr blocksize,
        Tick min_period, Tick max_period,
        uint8_t read_percent, Addr data_limit,
        unsigned int num_seq_pkts, unsigned int page_size,
        unsigned int nbr_of_banks_DRAM, unsigned int nbr_of_banks_util,
        Enums::AddrMap addr_mapping,
        unsigned int nbr_of_ranks,
        unsigned int max_seq_count_per_rank);

    std::shared_ptr<BaseGen> createTrace(
        Tick duration,
        const std::string& trace_file, Addr addr_offset);

  protected:
    void start();

    virtual std::shared_ptr<BaseGen> nextGenerator() = 0;

    /**
     * MasterID used in generated requests.
     */
    const MasterID masterID;

    /** Currently active generator */
    std::shared_ptr<BaseGen> activeGenerator;

    /** Stream/SubStreamID Generator */
    std::unique_ptr<StreamGen> streamGenerator;
};

#endif //__CPU_TRAFFIC_GEN_BASE_HH__
