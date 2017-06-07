/*
 * Copyright (c) 2013 - 2016 ARM Limited
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
 * Authors: Radhika Jagtap
 *          Andreas Hansson
 *          Thomas Grass
 */

#ifndef __CPU_TRACE_TRACE_CPU_HH__
#define __CPU_TRACE_TRACE_CPU_HH__

#include <array>
#include <cstdint>
#include <queue>
#include <set>
#include <unordered_map>

#include "arch/registers.hh"
#include "base/statistics.hh"
#include "cpu/base.hh"
#include "debug/TraceCPUData.hh"
#include "debug/TraceCPUInst.hh"
#include "params/TraceCPU.hh"
#include "proto/inst_dep_record.pb.h"
#include "proto/packet.pb.h"
#include "proto/protoio.hh"
#include "sim/sim_events.hh"

/**
 * The trace cpu replays traces generated using the elastic trace probe
 * attached to the O3 CPU model. The elastic trace is an execution trace with
 * register data dependencies and ordering dependencies annotated to it. The
 * trace cpu also replays a fixed timestamp fetch trace that is also generated
 * by the elastic trace probe. This trace cpu model aims at achieving faster
 * simulation compared to the detailed cpu model and good correlation when the
 * same trace is used for playback on different memory sub-systems.
 *
 * The TraceCPU inherits from BaseCPU so some virtual methods need to be
 * defined. It has two port subclasses inherited from MasterPort for
 * instruction and data ports. It issues the memory requests deducing the
 * timing from the trace and without performing real execution of micro-ops. As
 * soon as the last dependency for an instruction is complete, its
 * computational delay, also provided in the input trace is added. The
 * dependency-free nodes are maintained in a list, called 'ReadyList', ordered
 * by ready time. Instructions which depend on load stall until the responses
 * for read requests are received thus achieving elastic replay. If the
 * dependency is not found when adding a new node, it is assumed complete.
 * Thus, if this node is found to be completely dependency-free its issue time
 * is calculated and it is added to the ready list immediately. This is
 * encapsulated in the subclass ElasticDataGen.
 *
 * If ready nodes are issued in an unconstrained way there can be more nodes
 * outstanding which results in divergence in timing compared to the O3CPU.
 * Therefore, the Trace CPU also models hardware resources. A sub-class to
 * model hardware resources contains the maximum sizes of load buffer, store
 * buffer and ROB. If resources are not available, the node is not issued. Such
 * nodes that are pending issue are held in the 'depFreeQueue' structure.
 *
 * Modeling the ROB size in the Trace CPU as a resource limitation is arguably
 * the most important parameter of all resources. The ROB occupancy is
 * estimated using the newly added field 'robNum'. We need to use ROB number as
 * sequence number is at times much higher due to squashing and trace replay is
 * focused on correct path modeling.
 *
 * A map called 'inFlightNodes' is added to track nodes that are not only in
 * the readyList but also load nodes that are executed (and thus removed from
 * readyList) but are not complete. ReadyList handles what and when to execute
 * next node while the inFlightNodes is used for resource modelling. The oldest
 * ROB number is updated when any node occupies the ROB or when an entry in the
 * ROB is released. The ROB occupancy is equal to the difference in the ROB
 * number of the newly dependency-free node and the oldest ROB number in
 * flight.
 *
 * If no node depends on a non load/store node then there is no reason to
 * track it in the dependency graph. We filter out such nodes but count them
 * and add a weight field to the subsequent node that we do include in the
 * trace. The weight field is used to model ROB occupancy during replay.
 *
 * The depFreeQueue is chosen to be FIFO so that child nodes which are in
 * program order get pushed into it in that order and thus issued in program
 * order, like in the O3CPU. This is also why the dependents is made a
 * sequential container, std::set to std::vector. We only check head of the
 * depFreeQueue as nodes are issued in order and blocking on head models that
 * better than looping the entire queue. An alternative choice would be to
 * inspect top N pending nodes where N is the issue-width. This is left for
 * future as the timing correlation looks good as it is.
 *
 * At the start of an execution event, first we attempt to issue such pending
 * nodes by checking if appropriate resources have become available. If yes, we
 * compute the execute tick with respect to the time then. Then we proceed to
 * complete nodes from the readyList.
 *
 * When a read response is received, sometimes a dependency on it that was
 * supposed to be released when it was issued is still not released. This
 * occurs because the dependent gets added to the graph after the read was
 * sent. So the check is made less strict and the dependency is marked complete
 * on read response instead of insisting that it should have been removed on
 * read sent.
 *
 * There is a check for requests spanning two cache lines as this condition
 * triggers an assert fail in the L1 cache. If it does then truncate the size
 * to access only until the end of that line and ignore the remainder.
 * Strictly-ordered requests are skipped and the dependencies on such requests
 * are handled by simply marking them complete immediately.
 *
 * A CountedExitEvent that contains a static int belonging to the Trace CPU
 * class as a down counter is used to implement multi Trace CPU simulation
 * exit.
 */

class TraceCPU : public BaseCPU
{

  public:
    TraceCPU(TraceCPUParams *params);
    ~TraceCPU();

    void init();

    /**
     * This is a pure virtual function in BaseCPU. As we don't know how many
     * insts are in the trace but only know how how many micro-ops are we
     * cannot count this stat.
     *
     * @return 0
     */
    Counter totalInsts() const
    {
        return 0;
    }

    /**
     * Return totalOps as the number of committed micro-ops plus the
     * speculatively issued loads that are modelled in the TraceCPU replay.
     *
     * @return number of micro-ops i.e. nodes in the elastic data generator
     */
    Counter totalOps() const
    {
        return numOps.value();
    }

    /*
     * Set the no. of ops when elastic data generator completes executing a
     * node.
     */
    void updateNumOps(uint64_t rob_num);

    /* Pure virtual function in BaseCPU. Do nothing. */
    void wakeup(ThreadID tid = 0)
    {
        return;
    }

    /*
     * When resuming from checkpoint in FS mode, the TraceCPU takes over from
     * the old cpu. This function overrides the takeOverFrom() function in the
     * BaseCPU. It unbinds the ports of the old CPU and binds the ports of the
     * TraceCPU.
     */
    void takeOverFrom(BaseCPU *oldCPU);

    /**
     * When instruction cache port receives a retry, schedule event
     * icacheNextEvent.
     */
    void icacheRetryRecvd();

    /**
     * When data cache port receives a retry, schedule event
     * dcacheNextEvent.
     */
    void dcacheRetryRecvd();

    /**
     * When data cache port receives a response, this calls the dcache
     * generator method handle to complete the load writeback.
     *
     * @param pkt Pointer to packet received
     */
    void dcacheRecvTimingResp(PacketPtr pkt);

    /**
     * Schedule event dcacheNextEvent at the given tick
     *
     * @param when Tick at which to schedule event
     */
    void schedDcacheNextEvent(Tick when);

  protected:

    /**
     * IcachePort class that interfaces with L1 Instruction Cache.
     */
    class IcachePort : public MasterPort
    {
      public:
        /** Default constructor. */
        IcachePort(TraceCPU* _cpu)
            : MasterPort(_cpu->name() + ".icache_port", _cpu),
                         owner(_cpu)
        { }

      public:
        /**
         * Receive the timing reponse and simply delete the packet since
         * instruction fetch requests are issued as per the timing in the trace
         * and responses are ignored.
         *
         * @param pkt Pointer to packet received
         * @return true
         */
        bool recvTimingResp(PacketPtr pkt);

        /**
         * Required functionally but do nothing.
         *
         * @param pkt Pointer to packet received
         */
        void recvTimingSnoopReq(PacketPtr pkt) { }

        /**
         * Handle a retry signalled by the cache if instruction read failed in
         * the first attempt.
         */
        void recvReqRetry();

      private:
        TraceCPU* owner;
    };

    /**
     * DcachePort class that interfaces with L1 Data Cache.
     */
    class DcachePort : public MasterPort
    {

      public:
        /** Default constructor. */
        DcachePort(TraceCPU* _cpu)
            : MasterPort(_cpu->name() + ".dcache_port", _cpu),
                         owner(_cpu)
        { }

      public:

        /**
         * Receive the timing reponse and call dcacheRecvTimingResp() method
         * of the dcacheGen to handle completing the load
         *
         * @param pkt Pointer to packet received
         * @return true
         */
        bool recvTimingResp(PacketPtr pkt);

        /**
         * Required functionally but do nothing.
         *
         * @param pkt Pointer to packet received
         */
        void recvTimingSnoopReq(PacketPtr pkt)
        { }

        /**
         * Required functionally but do nothing.
         *
         * @param pkt Pointer to packet received
         */
        void recvFunctionalSnoop(PacketPtr pkt)
        { }

        /**
         * Handle a retry signalled by the cache if data access failed in the
         * first attempt.
         */
        void recvReqRetry();

        /**
         * Required functionally.
         *
         * @return true since we have to snoop
         */
        bool isSnooping() const { return true; }

      private:
        TraceCPU* owner;
    };

    /** Port to connect to L1 instruction cache. */
    IcachePort icachePort;

    /** Port to connect to L1 data cache. */
    DcachePort dcachePort;

    /** Master id for instruction read requests. */
    const MasterID instMasterID;

    /** Master id for data read and write requests. */
    const MasterID dataMasterID;

    /** File names for input instruction and data traces. */
    std::string instTraceFile, dataTraceFile;

    /**
     * Generator to read protobuf trace containing memory requests at fixed
     * timestamps, perform flow control and issue memory requests. If L1 cache
     * port sends packet succesfully, determine the tick to send the next
     * packet else wait for retry from cache.
     */
    class FixedRetryGen
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

            /** Potential request flags to use */
            Request::FlagsType flags;

            /** Instruction PC */
            Addr pc;

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

            // Input file stream for the protobuf trace
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
            bool read(TraceElement* element);
        };

        public:
        /* Constructor */
        FixedRetryGen(TraceCPU& _owner, const std::string& _name,
                   MasterPort& _port, MasterID master_id,
                   const std::string& trace_file)
            : owner(_owner),
              port(_port),
              masterID(master_id),
              trace(trace_file),
              genName(owner.name() + ".fixedretry" + _name),
              retryPkt(nullptr),
              delta(0),
              traceComplete(false)
        {
        }

        /**
         * Called from TraceCPU init(). Reads the first message from the
         * input trace file and returns the send tick.
         *
         * @return Tick when first packet must be sent
         */
        Tick init();

        /**
         * This tries to send current or retry packet and returns true if
         * successfull. It calls nextExecute() to read next message.
         *
         * @return bool true if packet is sent successfully
         */
        bool tryNext();

        /** Returns name of the FixedRetryGen instance. */
        const std::string& name() const { return genName; }

        /**
         * Creates a new request assigning the request parameters passed by the
         * arguments. Calls the port's sendTimingReq() and returns true if
         * the packet was sent succesfully. It is called by tryNext()
         *
         * @param addr address of request
         * @param size size of request
         * @param cmd if it is a read or write request
         * @param flags associated request flags
         * @param pc instruction PC that generated the request
         *
         * @return true if packet was sent successfully
         */
        bool send(Addr addr, unsigned size, const MemCmd& cmd,
              Request::FlagsType flags, Addr pc);

        /** Exit the FixedRetryGen. */
        void exit();

        /**
         * Reads a line of the trace file. Returns the tick
         * when the next request should be generated. If the end
         * of the file has been reached, it returns false.
         *
         * @return bool false id end of file has been reached
         */
        bool nextExecute();

        /**
         * Returns the traceComplete variable which is set when end of the
         * input trace file is reached.
         *
         * @return bool true if traceComplete is set, false otherwise.
         */
        bool isTraceComplete() { return traceComplete; }

        int64_t tickDelta() { return delta; }

        void regStats();

      private:

        /** Reference of the TraceCPU. */
        TraceCPU& owner;

        /** Reference of the port to be used to issue memory requests. */
        MasterPort& port;

        /** MasterID used for the requests being sent. */
        const MasterID masterID;

        /** Input stream used for reading the input trace file. */
        InputStream trace;

        /** String to store the name of the FixedRetryGen. */
        std::string genName;

        /** PacketPtr used to store the packet to retry. */
        PacketPtr retryPkt;

        /**
         * Stores the difference in the send ticks of the current and last
         * packets. Keeping this signed to check overflow to a negative value
         * which will be caught by assert(delta > 0)
         */
        int64_t delta;

        /**
         * Set to true when end of trace is reached.
         */
        bool traceComplete;

        /** Store an element read from the trace to send as the next packet. */
        TraceElement currElement;

        /** Stats for instruction accesses replayed. */
        Stats::Scalar numSendAttempted;
        Stats::Scalar numSendSucceeded;
        Stats::Scalar numSendFailed;
        Stats::Scalar numRetrySucceeded;
        /** Last simulated tick by the FixedRetryGen */
        Stats::Scalar instLastTick;

    };

    /**
     * The elastic data memory request generator to read protobuf trace
     * containing execution trace annotated with data and ordering
     * dependencies. It deduces the time at which to send a load/store request
     * by tracking the dependencies. It attempts to send a memory request for a
     * load/store without performing real execution of micro-ops. If L1 cache
     * port sends packet succesfully, the generator checks which instructions
     * became dependency free as a result of this and schedules an event
     * accordingly. If it fails to send the packet, it waits for a retry from
     * the cache.
     */
    class ElasticDataGen
    {

      private:

        /** Node sequence number type. */
        typedef uint64_t NodeSeqNum;

        /** Node ROB number type. */
        typedef uint64_t NodeRobNum;

        typedef ProtoMessage::InstDepRecord::RecordType RecordType;
        typedef ProtoMessage::InstDepRecord Record;

        /**
         * The struct GraphNode stores an instruction in the trace file. The
         * format of the trace file favours constructing a dependency graph of
         * the execution and this struct is used to encapsulate the request
         * data as well as pointers to its dependent GraphNodes.
         */
        class GraphNode {

          public:
            /**
             * The maximum no. of ROB dependencies. There can be at most 2
             * order dependencies which could exist for a store. For a load
             * and comp node there can be at most one order dependency.
             */
            static const uint8_t maxRobDep = 2;

            /** Typedef for the array containing the ROB dependencies */
            typedef std::array<NodeSeqNum, maxRobDep> RobDepArray;

            /** Typedef for the array containing the register dependencies */
            typedef std::array<NodeSeqNum, TheISA::MaxInstSrcRegs> RegDepArray;

            /** Instruction sequence number */
            NodeSeqNum seqNum;

            /** ROB occupancy number */
            NodeRobNum robNum;

           /** Type of the node corresponding to the instruction modelled by it */
            RecordType type;

            /** The address for the request if any */
            Addr physAddr;

            /** The virtual address for the request if any */
            Addr virtAddr;

            /** The address space id which is set if the virtual address is set */
            uint32_t asid;

            /** Size of request if any */
            uint32_t size;

            /** Request flags if any */
            Request::Flags flags;

            /** Instruction PC */
            Addr pc;

            /** Array of order dependencies. */
            RobDepArray robDep;

            /** Number of order dependencies */
            uint8_t numRobDep;

            /** Computational delay */
            uint64_t compDelay;

            /**
             * Array of register dependencies (incoming) if any. Maximum number
             * of source registers used to set maximum size of the array
             */
            RegDepArray regDep;

            /** Number of register dependencies */
            uint8_t numRegDep;

            /**
             * A vector of nodes dependent (outgoing) on this node. A
             * sequential container is chosen because when dependents become
             * free, they attempt to issue in program order.
             */
            std::vector<GraphNode *> dependents;

            /** Is the node a load */
            bool isLoad() const { return (type == Record::LOAD); }

            /** Is the node a store */
            bool isStore() const { return (type == Record::STORE); }

            /** Is the node a compute (non load/store) node */
            bool isComp() const { return (type == Record::COMP); }

            /** Initialize register dependency array to all zeroes */
            void clearRegDep();

            /** Initialize register dependency array to all zeroes */
            void clearRobDep();

            /** Remove completed instruction from register dependency array */
            bool removeRegDep(NodeSeqNum reg_dep);

            /** Remove completed instruction from order dependency array */
            bool removeRobDep(NodeSeqNum rob_dep);

            /** Check for all dependencies on completed inst */
            bool removeDepOnInst(NodeSeqNum done_seq_num);

            /** Return true if node has a request which is strictly ordered */
            bool isStrictlyOrdered() const {
                return (flags.isSet(Request::STRICT_ORDER));
            }
            /**
             * Write out element in trace-compatible format using debug flag
             * TraceCPUData.
             */
            void writeElementAsTrace() const;

            /** Return string specifying the type of the node */
            std::string typeToStr() const;
        };

        /** Struct to store a ready-to-execute node and its execution tick. */
        struct ReadyNode
        {
            /** The sequence number of the ready node */
            NodeSeqNum seqNum;

            /** The tick at which the ready node must be executed */
            Tick execTick;
        };

        /**
         * The HardwareResource class models structures that hold the in-flight
         * nodes. When a node becomes dependency free, first check if resources
         * are available to issue it.
         */
        class HardwareResource
        {
          public:
            /**
             * Constructor that initializes the sizes of the structures.
             *
             * @param max_rob size of the Reorder Buffer
             * @param max_stores size of Store Buffer
             * @param max_loads size of Load Buffer
             */
            HardwareResource(uint16_t max_rob, uint16_t max_stores,
                                uint16_t max_loads);

            /**
             * Occupy appropriate structures for an issued node.
             *
             * @param node_ptr pointer to the issued node
             */
            void occupy(const GraphNode* new_node);

            /**
             * Release appropriate structures for a completed node.
             *
             * @param node_ptr pointer to the completed node
             */
            void release(const GraphNode* done_node);

            /** Release store buffer entry for a completed store */
            void releaseStoreBuffer();

            /**
             * Check if structures required to issue a node are free.
             *
             * @param node_ptr pointer to the node ready to issue
             * @return true if resources are available
             */
            bool isAvailable(const GraphNode* new_node) const;

            /**
             * Check if there are any outstanding requests, i.e. requests for
             * which we are yet to receive a response.
             *
             * @return true if there is at least one read or write request
             *      outstanding
             */
            bool awaitingResponse() const;

            /** Print resource occupancy for debugging */
            void printOccupancy();

          private:
            /**
             * The size of the ROB used to throttle the max. number of in-flight
             * nodes.
             */
            const uint16_t sizeROB;

            /**
             * The size of store buffer. This is used to throttle the max. number
             * of in-flight stores.
             */
            const uint16_t sizeStoreBuffer;

            /**
             * The size of load buffer. This is used to throttle the max. number
             * of in-flight loads.
             */
            const uint16_t sizeLoadBuffer;

            /**
             * A map from the sequence number to the ROB number of the in-
             * flight nodes. This includes all nodes that are in the readyList
             * plus the loads for which a request has been sent which are not
             * present in the readyList. But such loads are not yet complete
             * and thus occupy resources. We need to query the oldest in-flight
             * node and since a map container keeps all its keys sorted using
             * the less than criterion, the first element is the in-flight node
             * with the least sequence number, i.e. the oldest in-flight node.
             */
            std::map<NodeSeqNum, NodeRobNum> inFlightNodes;

            /** The ROB number of the oldest in-flight node */
            NodeRobNum oldestInFlightRobNum;

            /** Number of ready loads for which request may or may not be sent */
            uint16_t numInFlightLoads;

            /** Number of ready stores for which request may or may not be sent */
            uint16_t numInFlightStores;
        };

        /**
         * The InputStream encapsulates a trace file and the
         * internal buffers and populates GraphNodes based on
         * the input.
         */
        class InputStream
        {

          private:

            /** Input file stream for the protobuf trace */
            ProtoInputStream trace;

            /**
             * A multiplier for the compute delays in the trace to modulate
             * the Trace CPU frequency either up or down. The Trace CPU's
             * clock domain frequency must also be set to match the expected
             * result of frequency scaling.
             */
            const double timeMultiplier;

            /** Count of committed ops read from trace plus the filtered ops */
            uint64_t microOpCount;

            /**
             * The window size that is read from the header of the protobuf
             * trace and used to process the dependency trace
             */
            uint32_t windowSize;
          public:

            /**
             * Create a trace input stream for a given file name.
             *
             * @param filename Path to the file to read from
             * @param time_multiplier used to scale the compute delays
             */
            InputStream(const std::string& filename,
                        const double time_multiplier);

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
             * @param size of register dependency array stored in the element
             * @return True if an element could be read successfully
             */
            bool read(GraphNode* element);

            /** Get window size from trace */
            uint32_t getWindowSize() const { return windowSize; }

            /** Get number of micro-ops modelled in the TraceCPU replay */
            uint64_t getMicroOpCount() const { return microOpCount; }
        };

        public:
        /* Constructor */
        ElasticDataGen(TraceCPU& _owner, const std::string& _name,
                   MasterPort& _port, MasterID master_id,
                   const std::string& trace_file, TraceCPUParams *params)
            : owner(_owner),
              port(_port),
              masterID(master_id),
              trace(trace_file, 1.0 / params->freqMultiplier),
              genName(owner.name() + ".elastic" + _name),
              retryPkt(nullptr),
              traceComplete(false),
              nextRead(false),
              execComplete(false),
              windowSize(trace.getWindowSize()),
              hwResource(params->sizeROB, params->sizeStoreBuffer,
                         params->sizeLoadBuffer)
        {
            DPRINTF(TraceCPUData, "Window size in the trace is %d.\n",
                    windowSize);
        }

        /**
         * Called from TraceCPU init(). Reads the first message from the
         * input trace file and returns the send tick.
         *
         * @return Tick when first packet must be sent
         */
        Tick init();

        /**
         * Adjust traceOffset based on what TraceCPU init() determines on
         * comparing the offsets in the fetch request and elastic traces.
         *
         * @param trace_offset trace offset set by comparing both traces
         */
        void adjustInitTraceOffset(Tick& offset);

        /** Returns name of the ElasticDataGen instance. */
        const std::string& name() const { return genName; }

        /** Exit the ElasticDataGen. */
        void exit();

        /**
         * Reads a line of the trace file. Returns the tick when the next
         * request should be generated. If the end of the file has been
         * reached, it returns false.
         *
         * @return bool false if end of file has been reached else true
         */
        bool readNextWindow();

        /**
         * Iterate over the dependencies of a new node and add the new node
         * to the list of dependents of the parent node.
         *
         * @param   new_node    new node to add to the graph
         * @tparam  dep_array   the dependency array of type rob or register,
         *                      that is to be iterated, and may get modified
         * @param   num_dep     the number of dependencies set in the array
         *                      which may get modified during iteration
         */
        template<typename T> void addDepsOnParent(GraphNode *new_node,
                                                    T& dep_array,
                                                    uint8_t& num_dep);

        /**
         * This is the main execute function which consumes nodes from the
         * sorted readyList. First attempt to issue the pending dependency-free
         * nodes held in the depFreeQueue. Insert the ready-to-issue nodes into
         * the readyList. Then iterate through the readyList and when a node
         * has its execute tick equal to curTick(), execute it. If the node is
         * a load or a store call executeMemReq() and if it is neither, simply
         * mark it complete.
         */
        void execute();

        /**
         * Creates a new request for a load or store assigning the request
         * parameters. Calls the port's sendTimingReq() and returns a packet
         * if the send failed so that it can be saved for a retry.
         *
         * @param node_ptr pointer to the load or store node to be executed
         *
         * @return packet pointer if the request failed and nullptr if it was
         *          sent successfully
         */
        PacketPtr executeMemReq(GraphNode* node_ptr);

        /**
         * Add a ready node to the readyList. When inserting, ensure the nodes
         * are sorted in ascending order of their execute ticks.
         *
         * @param seq_num seq. num of ready node
         * @param exec_tick the execute tick of the ready node
         */
        void addToSortedReadyList(NodeSeqNum seq_num, Tick exec_tick);

        /** Print readyList for debugging using debug flag TraceCPUData. */
        void printReadyList();

        /**
         * When a load writeback is received, that is when the load completes,
         * release the dependents on it. This is called from the dcache port
         * recvTimingResp().
         */
        void completeMemAccess(PacketPtr pkt);

        /**
         * Returns the execComplete variable which is set when the last
         * node is executed.
         *
         * @return bool true if execComplete is set, false otherwise.
         */
        bool isExecComplete() const { return execComplete; }

        /**
         * Attempts to issue a node once the node's source dependencies are
         * complete. If resources are available then add it to the readyList,
         * otherwise the node is not issued and is stored in depFreeQueue
         * until resources become available.
         *
         * @param node_ptr pointer to node to be issued
         * @param first true if this is the first attempt to issue this node
         * @return true if node was added to readyList
         */
        bool checkAndIssue(const GraphNode* node_ptr, bool first = true);

        /** Get number of micro-ops modelled in the TraceCPU replay */
        uint64_t getMicroOpCount() const { return trace.getMicroOpCount(); }

        void regStats();

      private:

        /** Reference of the TraceCPU. */
        TraceCPU& owner;

        /** Reference of the port to be used to issue memory requests. */
        MasterPort& port;

        /** MasterID used for the requests being sent. */
        const MasterID masterID;

        /** Input stream used for reading the input trace file. */
        InputStream trace;

        /** String to store the name of the FixedRetryGen. */
        std::string genName;

        /** PacketPtr used to store the packet to retry. */
        PacketPtr retryPkt;

        /** Set to true when end of trace is reached. */
        bool traceComplete;

        /** Set to true when the next window of instructions need to be read */
        bool nextRead;

        /** Set true when execution of trace is complete */
        bool execComplete;

        /**
         * Window size within which to check for dependencies. Its value is
         * made equal to the window size used to generate the trace which is
         * recorded in the trace header. The dependency graph must be
         * populated enough such that when a node completes, its potential
         * child node must be found and the dependency removed before the
         * completed node itself is removed. Thus as soon as the graph shrinks
         * to become smaller than this window, we read in the next window.
         */
        const uint32_t windowSize;

        /**
         * Hardware resources required to contain in-flight nodes and to
         * throttle issuing of new nodes when resources are not available.
         */
        HardwareResource hwResource;

        /** Store the depGraph of GraphNodes */
        std::unordered_map<NodeSeqNum, GraphNode*> depGraph;

        /**
         * Queue of dependency-free nodes that are pending issue because
         * resources are not available. This is chosen to be FIFO so that
         * dependent nodes which become free in program order get pushed
         * into the queue in that order. Thus nodes are more likely to
         * issue in program order.
         */
        std::queue<const GraphNode*> depFreeQueue;

        /** List of nodes that are ready to execute */
        std::list<ReadyNode> readyList;

        /** Stats for data memory accesses replayed. */
        Stats::Scalar maxDependents;
        Stats::Scalar maxReadyListSize;
        Stats::Scalar numSendAttempted;
        Stats::Scalar numSendSucceeded;
        Stats::Scalar numSendFailed;
        Stats::Scalar numRetrySucceeded;
        Stats::Scalar numSplitReqs;
        Stats::Scalar numSOLoads;
        Stats::Scalar numSOStores;
        /** Tick when ElasticDataGen completes execution */
        Stats::Scalar dataLastTick;
    };

    /** Instance of FixedRetryGen to replay instruction read requests. */
    FixedRetryGen icacheGen;

    /** Instance of ElasticDataGen to replay data read and write requests. */
    ElasticDataGen dcacheGen;

    /**
     * This is the control flow that uses the functionality of the icacheGen to
     * replay the trace. It calls tryNext(). If it returns true then next event
     * is scheduled at curTick() plus delta. If it returns false then delta is
     * ignored and control is brought back via recvRetry().
     */
    void schedIcacheNext();

    /**
     * This is the control flow that uses the functionality of the dcacheGen to
     * replay the trace. It calls execute(). It checks if execution is complete
     * and schedules an event to exit simulation accordingly.
     */
    void schedDcacheNext();

    /** Event for the control flow method schedIcacheNext() */
    EventFunctionWrapper icacheNextEvent;

    /** Event for the control flow method schedDcacheNext() */
    EventFunctionWrapper dcacheNextEvent;

    /** This is called when either generator finishes executing from the trace */
    void checkAndSchedExitEvent();

    /** Set to true when one of the generators finishes replaying its trace. */
    bool oneTraceComplete;

    /**
     * This stores the time offset in the trace, which is taken away from
     * the ready times of requests. This is specially useful because the time
     * offset can be very large if the traces are generated from the middle of
     * a program.
     */
    Tick traceOffset;

    /**
     * Number of Trace CPUs in the system used as a shared variable and passed
     * to the CountedExitEvent event used for counting down exit events.  It is
     * incremented in the constructor call so that the total is arrived at
     * automatically.
     */
    static int numTraceCPUs;

   /**
    * A CountedExitEvent which when serviced decrements the counter. A sim
    * exit event is scheduled when the counter equals zero, that is all
    * instances of Trace CPU have had their execCompleteEvent serviced.
    */
    CountedExitEvent *execCompleteEvent;

    /**
     * Exit when any one Trace CPU completes its execution. If this is
     * configured true then the execCompleteEvent is not scheduled.
     */
    const bool enableEarlyExit;

    /**
      * Interval of committed instructions specified by the user at which a
      * progress info message is printed
      */
    const uint64_t progressMsgInterval;

    /*
     * The progress msg threshold is kept updated to the next multiple of the
     * progress msg interval. As soon as the threshold is reached, an info
     * message is printed.
     */
    uint64_t progressMsgThreshold;

    Stats::Scalar numSchedDcacheEvent;
    Stats::Scalar numSchedIcacheEvent;

    /** Stat for number of simulated micro-ops. */
    Stats::Scalar numOps;
    /** Stat for the CPI. This is really cycles per micro-op and not inst. */
    Stats::Formula cpi;

  public:

    /** Used to get a reference to the icache port. */
    MasterPort &getInstPort() { return icachePort; }

    /** Used to get a reference to the dcache port. */
    MasterPort &getDataPort() { return dcachePort; }

    void regStats();
};
#endif // __CPU_TRACE_TRACE_CPU_HH__
