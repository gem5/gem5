/*
 * Copyright (c) 2012 ARM Limited
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

#include "base/hashmap.hh"
#include "cpu/testers/traffic_gen/generators.hh"
#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/TrafficGen.hh"

/**
 * The traffic generator is a master module that generates stimuli for
 * the memory system, based on a collection of simple behaviours that
 * are either probabilistic or based on traces. It can be used stand
 * alone for creating test cases for interconnect and memory
 * controllers, or function as a black box replacement for system
 * components that are not yet modelled in detail, e.g. a video engine
 * or baseband subsystem.
 */
class TrafficGen : public MemObject
{

  private:

    /**
     * The system used to determine which mode we are currently operating
     * in.
     */
    System* system;

    /**
     * MasterID used in generated requests.
     */
    MasterID masterID;

  protected:

    /**
     * The state graph is responsible for instantiating and keeping
     * track of the various generator states and also perform the
     * transitions and call the appropriate functions when entering,
     * executing and exiting a state.
     */
    class StateGraph
    {

      public:

        /**
         * Create a state graph from an input file.
         *
         * @param _owner used solely for the name
         * @param _port port used to send requests
         * @param file_name configuration description to read in
         * @param master_id the unique id used for all requests
         */
        StateGraph(TrafficGen& _owner, QueuedMasterPort& _port,
                   const std::string& file_name, MasterID master_id)
            : nextTransitionTick(0), owner(_owner), port(_port)
        {
            parseConfig(file_name, master_id);
        }

        /**
         * Get the name, used for DPRINTFs.
         *
         * @return the owner's name
         */
        std::string name() const { return owner.name(); }

        /**
         * Either perform a state transition or execute the current
         * state, depending on the current time.
         */
        void update();

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
         * Get the tick of the next event, either an execution or a
         * transition.
         *
         * @return tick of the next state graph event
         */
        Tick nextEventTick()
        {
            return std::min(states[currState]->nextExecuteTick(),
                            nextTransitionTick);

        }

        /** Time of next transition */
        Tick nextTransitionTick;

      private:

        /**
         * Parse the config file and build the state map and
         * transition matrix.
         *
         * @param file_name Config file name to parse
         * @param master_id MasterID to use for generated requests
         */
        void parseConfig(const std::string& file_name, MasterID master_id);

        /** Struct to represent a probabilistic transition during parsing. */
        struct Transition {
            uint32_t from;
            uint32_t to;
            double p;
        };

        /** Pointer to owner of request handler */
        TrafficGen& owner;

        /** Pointer to request handler */
        QueuedMasterPort& port;

        /** State transition matrix */
        std::vector<std::vector<double> > transitionMatrix;

      public:

        /** Index of the current state */
        uint32_t currState;

        /** Map of states */
        m5::hash_map<uint32_t, BaseGen*> states;
    };


    /** Queued handler */
    class TrafficGenPort : public QueuedMasterPort
    {
      public:

        TrafficGenPort(const std::string& name, TrafficGen& _owner)
            : QueuedMasterPort(name, &_owner, queue), queue(_owner, *this)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt);

      private:

        MasterPacketQueue queue;

    };

    TrafficGenPort port;

    /** Request generator state graph */
    StateGraph stateGraph;

    /**
     * Schedules event for next update and executes an update on the
     * state graph.
     */
    void updateStateGraph();

    /** Event for updating the state graph */
    EventWrapper<TrafficGen,
                 &TrafficGen::updateStateGraph> updateStateGraphEvent;


  public:

    TrafficGen(const TrafficGenParams* p);

    ~TrafficGen() {}

    virtual BaseMasterPort& getMasterPort(const std::string &if_name,
                                          PortID idx = InvalidPortID);

    void init();

    void initState();

    unsigned int drain(DrainManager *dm);

    void serialize(std::ostream &os);

    void unserialize(Checkpoint* cp, const std::string& section);

};

#endif //__CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__
