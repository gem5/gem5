/*
 * Copyright (c) 2012-2013, 2016 ARM Limited
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
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Sascha Bischoff
 */
#ifndef __CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__
#define __CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__

#include <unordered_map>

#include "base/statistics.hh"
#include "cpu/testers/traffic_gen/generators.hh"
#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/TrafficGen.hh"

/**
 * The traffic generator is a master module that generates stimuli for
 * the memory system, based on a collection of simple generator
 * behaviours that are either probabilistic or based on traces. It can
 * be used stand alone for creating test cases for interconnect and
 * memory controllers, or function as a black box replacement for
 * system components that are not yet modelled in detail, e.g. a video
 * engine or baseband subsystem.
 */
class TrafficGen : public MemObject
{

  private:

    /**
     * Determine next state and perform the transition.
     */
    void transition();

    /**
     * Enter a new state.
     *
     * @param newState identifier of state to enter
     */
    void enterState(uint32_t newState);

    /**
     * Parse the config file and build the state map and
     * transition matrix.
     */
    void parseConfig();

    /**
     * Schedules event for next update and executes an update on the
     * state graph, either performing a state transition or executing
     * the current state, depending on the current time.
     */
    void update();

    /**
     * Receive a retry from the neighbouring port and attempt to
     * resend the waiting packet.
     */
    void recvReqRetry();

    /**
     * Method to inform the user we have made no progress.
     */
    void noProgress();

    /** Struct to represent a probabilistic transition during parsing. */
    struct Transition {
        uint32_t from;
        uint32_t to;
        double p;
    };

    /**
     * The system used to determine which mode we are currently operating
     * in.
     */
    System* system;

    /**
     * MasterID used in generated requests.
     */
    MasterID masterID;

    /**
     * The config file to parse.
     */
    const std::string configFile;

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

    /**
     * Event to keep track of our progress, or lack thereof.
     */
    EventWrapper<TrafficGen, &TrafficGen::noProgress> noProgressEvent;

    /** Time of next transition */
    Tick nextTransitionTick;

    /** Time of the next packet. */
    Tick nextPacketTick;

    /** State transition matrix */
    std::vector<std::vector<double> > transitionMatrix;

    /** Index of the current state */
    uint32_t currState;

    /** Map of generator states */
    std::unordered_map<uint32_t, BaseGen*> states;

    /** Master port specialisation for the traffic generator */
    class TrafficGenPort : public MasterPort
    {
      public:

        TrafficGenPort(const std::string& name, TrafficGen& traffic_gen)
            : MasterPort(name, &traffic_gen), trafficGen(traffic_gen)
        { }

      protected:

        void recvReqRetry() { trafficGen.recvReqRetry(); }

        bool recvTimingResp(PacketPtr pkt);

        void recvTimingSnoopReq(PacketPtr pkt) { }

        void recvFunctionalSnoop(PacketPtr pkt) { }

        Tick recvAtomicSnoop(PacketPtr pkt) { return 0; }

      private:

        TrafficGen& trafficGen;

    };

    /** The instance of master port used by the traffic generator. */
    TrafficGenPort port;

    /** Packet waiting to be sent. */
    PacketPtr retryPkt;

    /** Tick when the stalled packet was meant to be sent. */
    Tick retryPktTick;

    /** Event for scheduling updates */
    EventWrapper<TrafficGen, &TrafficGen::update> updateEvent;

    uint64_t numSuppressed;

    /** Count the number of generated packets. */
    Stats::Scalar numPackets;

    /** Count the number of retries. */
    Stats::Scalar numRetries;

    /** Count the time incurred from back-pressure. */
    Stats::Scalar retryTicks;

  public:

    TrafficGen(const TrafficGenParams* p);

    ~TrafficGen() {}

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    void init() override;

    void initState() override;

    DrainState drain() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** Register statistics */
    void regStats() override;

};

#endif //__CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__
