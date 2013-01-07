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
#ifndef __MEM_TRAFFIC_GEN_HH__
#define __MEM_TRAFFIC_GEN_HH__

#include "base/hashmap.hh"
#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/TrafficGen.hh"
#include "proto/protoio.hh"

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

        /** Base class for all generator states */
        class BaseGen
        {

          protected:

            /** Port used to send requests */
            QueuedMasterPort& port;

            /** The MasterID used for generating requests */
            const MasterID masterID;

            /**
             * Create a new request and associated packet and schedule
             * it to be sent in the current tick.
             *
             * @param addr Physical address to use
             * @param size Size of the request
             * @param cmd Memory command to send
             */
            void send(Addr addr, unsigned size, const MemCmd& cmd);

          public:

            /** Time to spend in this state */
            const Tick duration;

            /**
             * Create a base generator.
             *
             * @param _port port used to send requests
             * @param master_id MasterID set on each request
             * @param _duration duration of this state before transitioning
             */
            BaseGen(QueuedMasterPort& _port, MasterID master_id,
                    Tick _duration);

            virtual ~BaseGen() { }

            /**
             * Get the name, useful for DPRINTFs.
             *
             * @return the port name
             */
            std::string name() const { return port.name(); }

            /**
             * Enter this generator state.
             */
            virtual void enter() = 0;

            /**
             * Execute this generator state.
             */
            virtual void execute() = 0;

            /**
             * Exit this generator state. By default do nothing.
             */
            virtual void exit() { };

            /**
             * Determine the next execute tick. MaxTick means that
             * there will not be any further event in the current
             * activation cycle of the state.
             *
             * @return next tick when the state should be executed
             */
            virtual Tick nextExecuteTick() = 0;

        };

        /**
         * The idle generator does nothing.
         */
        class IdleGen : public BaseGen
        {

          public:

            IdleGen(QueuedMasterPort& _port, MasterID master_id,
                    Tick _duration)
                : BaseGen(_port, master_id, _duration)
            { }

            void enter() { }

            void execute() { }

            Tick nextExecuteTick() { return MaxTick; }
        };

        /**
         * The linear generator generates sequential requests from a
         * start to an end address, with a fixed block size. A
         * fraction of the requests are reads, as determined by the
         * read percent. There is an optional data limit for when to
         * stop generating new requests.
         */
        class LinearGen : public BaseGen
        {

          public:

            /**
             * Create a linear address sequence generator. Set
             * min_period == max_period for a fixed inter-transaction
             * time.
             *
             * @param _port port used to send requests
             * @param master_id MasterID set on each request
             * @param _duration duration of this state before transitioning
             * @param start_addr Start address
             * @param end_addr End address
             * @param _blocksize Size used for transactions injected
             * @param min_period Lower limit of random inter-transaction time
             * @param max_period Upper limit of random inter-transaction time
             * @param read_percent Percent of transactions that are reads
             * @param data_limit Upper limit on how much data to read/write
             */
            LinearGen(QueuedMasterPort& _port, MasterID master_id,
                      Tick _duration, Addr start_addr, Addr end_addr,
                      Addr _blocksize, Tick min_period, Tick max_period,
                      uint8_t read_percent, Addr data_limit)
                : BaseGen(_port, master_id, _duration),
                  startAddr(start_addr), endAddr(end_addr),
                  blocksize(_blocksize), minPeriod(min_period),
                  maxPeriod(max_period), readPercent(read_percent),
                  dataLimit(data_limit)
            { }

            void enter();

            void execute();

            Tick nextExecuteTick();

          private:

            /** Start of address range */
            const Addr startAddr;

            /** End of address range */
            const Addr endAddr;

            /** Blocksize and address increment */
            const Addr blocksize;

            /** Request generation period */
            const Tick minPeriod;
            const Tick maxPeriod;

            /**
             * Percent of generated transactions that should be reads
             */
            const uint8_t readPercent;

            /** Maximum amount of data to manipulate */
            const Addr dataLimit;

            /** Address of next request */
            Addr nextAddr;

            /**
             * Counter to determine the amount of data
             * manipulated. Used to determine if we should continue
             * generating requests.
             */
            Addr dataManipulated;
        };

        /**
         * The random generator is similar to the linear one, but does
         * not generate sequential addresses. Instead it randomly
         * picks an address in the range, aligned to the block size.
         */
        class RandomGen : public BaseGen
        {

          public:

            /**
             * Create a random address sequence generator. Set
             * min_period == max_period for a fixed inter-transaction
             * time.
             *
             * @param _port port used to send requests
             * @param master_id MasterID set on each request
             * @param _duration duration of this state before transitioning
             * @param start_addr Start address
             * @param end_addr End address
             * @param _blocksize Size used for transactions injected
             * @param min_period Lower limit of random inter-transaction time
             * @param max_period Upper limit of random inter-transaction time
             * @param read_percent Percent of transactions that are reads
             * @param data_limit Upper limit on how much data to read/write
             */
            RandomGen(QueuedMasterPort& _port, MasterID master_id,
                      Tick _duration, Addr start_addr, Addr end_addr,
                      Addr _blocksize, Tick min_period, Tick max_period,
                      uint8_t read_percent, Addr data_limit)
                : BaseGen(_port, master_id, _duration),
                  startAddr(start_addr), endAddr(end_addr),
                  blocksize(_blocksize), minPeriod(min_period),
                  maxPeriod(max_period), readPercent(read_percent),
                  dataLimit(data_limit)
            { }

            void enter();

            void execute();

            Tick nextExecuteTick();

          private:

            /** Start of address range */
            const Addr startAddr;

            /** End of address range */
            const Addr endAddr;

            /** Block size */
            const Addr blocksize;

            /** Request generation period */
            const Tick minPeriod;
            const Tick maxPeriod;

            /**
             * Percent of generated transactions that should be reads
             */
            const uint8_t readPercent;

            /** Maximum amount of data to manipulate */
            const Addr dataLimit;

            /**
             * Counter to determine the amount of data
             * manipulated. Used to determine if we should continue
             * generating requests.
             */
            Addr dataManipulated;
        };

        /**
         * The trace replay generator reads a trace file and plays
         * back the transactions. The trace is offset with respect to
         * the time when the state was entered.
         */
        class TraceGen : public BaseGen
        {

          private:

            /**
             * This struct stores a line in the trace file.
             */
            struct TraceElement {

                /** Specifies if the request is to be a read or a write */
                MemCmd cmd;

                /** The address for the request */
                Addr addr;

                /** The size of the access for the request */
                Addr blocksize;

                /** The time at which the request should be sent */
                Tick tick;

                /**
                 * Check validity of this element.
                 *
                 * @return if this element is valid
                 */
                bool isValid() const {
                    return cmd != MemCmd::InvalidCmd;
                }

                /**
                 * Make this element invalid.
                 */
                void clear() {
                    cmd = MemCmd::InvalidCmd;
                }
            };

            /**
             * The InputStream encapsulates a trace file and the
             * internal buffers and populates TraceElements based on
             * the input.
             */
            class InputStream
            {

              private:

                /// Input file stream for the protobuf trace
                ProtoInputStream trace;

              public:

                /**
                 * Create a trace input stream for a given file name.
                 *
                 * @param filename Path to the file to read from
                 */
                InputStream(const std::string& filename);

                /**
                 * Reset the stream such that it can be played once
                 * again.
                 */
                void reset();

                /**
                 * Attempt to read a trace element from the stream,
                 * and also notify the caller if the end of the file
                 * was reached.
                 *
                 * @param element Trace element to populate
                 * @return True if an element could be read successfully
                 */
                bool read(TraceElement& element);
            };

          public:

           /**
             * Create a trace generator.
             *
             * @param _port port used to send requests
             * @param master_id MasterID set on each request
             * @param _duration duration of this state before transitioning
             * @param trace_file File to read the transactions from
             * @param addr_offset Positive offset to add to trace address
             */
            TraceGen(QueuedMasterPort& _port, MasterID master_id,
                     Tick _duration, const std::string& trace_file,
                     Addr addr_offset)
                : BaseGen(_port, master_id, _duration),
                  trace(trace_file),
                  addrOffset(addr_offset),
                  traceComplete(false)
            {
            }

            void enter();

            void execute();

            void exit();

            /**
             * Read a line of the trace file. Returns the raw tick
             * when the next request should be generated. If the end
             * of the file has been reached, it returns MaxTick to
             * indicate that there will be no more requests.
             */
            Tick nextExecuteTick();

          private:

            /** Input stream used for reading the input trace file */
            InputStream trace;

            /** Store the current and next element in the trace */
            TraceElement currElement;
            TraceElement nextElement;

            /**
             * Stores the time when the state was entered. This is to add an
             * offset to the times stored in the trace file.
             */
            Tick tickOffset;

            /**
             * Offset for memory requests. Used to shift the trace
             * away from the CPU address space.
             */
            Addr addrOffset;

            /**
             * Set to true when the trace replay for one instance of
             * state is complete.
             */
            bool traceComplete;

            /**
             * Used to store the Tick when the next generate should
             * occur. It is to remove a transaction as soon as we
             * enter the state.
             */
            Tick oldEmitTime;
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
            : QueuedMasterPort(name, &_owner, queue), queue(_owner, *this),
              owner(_owner)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt);

      private:

        MasterPacketQueue queue;

        // Owner of the port
        TrafficGen& owner;

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

#endif //__MEM_TRAFFIC_GEN_HH__
