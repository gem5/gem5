/*
 * Copyright (c) 2013 - 2015 ARM Limited
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
 * @file This file describes a trace component which is a cpu probe listener
 * used to generate elastic cpu traces. It registers listeners to probe points
 * in the fetch, rename, iew and commit stages of the O3CPU. It processes the
 * dependency graph of the cpu execution and writes out a protobuf trace. It
 * also generates a protobuf trace of the instruction fetch requests.
 */

#ifndef __CPU_O3_PROBE_ELASTIC_TRACE_HH__
#define __CPU_O3_PROBE_ELASTIC_TRACE_HH__

#include <set>
#include <unordered_map>
#include <utility>

#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/impl.hh"
#include "mem/request.hh"
#include "params/ElasticTrace.hh"
#include "proto/inst_dep_record.pb.h"
#include "proto/packet.pb.h"
#include "proto/protoio.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"

/**
 * The elastic trace is a type of probe listener and listens to probe points
 * in multiple stages of the O3CPU. The notify method is called on a probe
 * point typically when an instruction successfully progresses through that
 * stage.
 *
 * As different listener methods mapped to the different probe points execute,
 * relevant information about the instruction, e.g. timestamps and register
 * accesses, are captured and stored in temporary data structures. When the
 * instruction progresses through the commit stage, the timing as well as
 * dependency information about the instruction is finalised and encapsulated in
 * a struct called TraceInfo. TraceInfo objects are collected in a list instead
 * of writing them out to the trace file one a time. This is required as the
 * trace is processed in chunks to evaluate order dependencies and computational
 * delay in case an instruction does not have any register dependencies. By this
 * we achieve a simpler algorithm during replay because every record in the
 * trace can be hooked onto a record in its past. The trace is written out as
 * a protobuf format output file.
 *
 * The output trace can be read in and played back by the TraceCPU.
 */
class ElasticTrace : public ProbeListenerObject
{

  public:
    typedef typename O3CPUImpl::DynInstPtr DynInstPtr;
    typedef typename O3CPUImpl::DynInstConstPtr DynInstConstPtr;
    typedef typename std::pair<InstSeqNum, PhysRegIndex> SeqNumRegPair;

    /** Trace record types corresponding to instruction node types */
    typedef ProtoMessage::InstDepRecord::RecordType RecordType;
    typedef ProtoMessage::InstDepRecord Record;

    /** Constructor */
    ElasticTrace(const ElasticTraceParams *params);

    /**
     * Register the probe listeners that is the methods called on a probe point
     * notify() call.
     */
    void regProbeListeners();

    /** Register all listeners. */
    void regEtraceListeners();

    /** Returns the name of the trace probe listener. */
    const std::string name() const;

    /**
     * Process any outstanding trace records, flush them out to the protobuf
     * output streams and delete the streams at simulation exit.
     */
    void flushTraces();

    /**
     * Take the fields of the request class object that are relevant to create
     * an instruction fetch request. It creates a protobuf message containing
     * the request fields and writes it to instTraceStream.
     *
     * @param req pointer to the fetch request
     */
    void fetchReqTrace(const RequestPtr &req);

    /**
     * Populate the execute timestamp field in an InstExecInfo object for an
     * instruction in flight.
     *
     * @param dyn_inst pointer to dynamic instruction in flight
     */
    void recordExecTick(const DynInstConstPtr& dyn_inst);

    /**
     * Populate the timestamp field in an InstExecInfo object for an
     * instruction in flight when it is execution is complete and it is ready
     * to commit.
     *
     * @param dyn_inst pointer to dynamic instruction in flight
     */
    void recordToCommTick(const DynInstConstPtr& dyn_inst);

    /**
     * Record a Read After Write physical register dependency if there has
     * been a write to the source register and update the physical register
     * map. For this look up the physRegDepMap with this instruction as the
     * writer of its destination register. If the dependency falls outside the
     * window it is assumed as already complete. Duplicate entries are avoided.
     *
     * @param dyn_inst pointer to dynamic instruction in flight
     */
    void updateRegDep(const DynInstConstPtr& dyn_inst);

    /**
     * When an instruction gets squashed the destination register mapped to it
     * is freed up in the rename stage. Remove the register entry from the
     * physRegDepMap as well to avoid dependencies on squashed instructions.
     *
     * @param inst_reg_pair pair of inst. sequence no. and the register
     */
    void removeRegDepMapEntry(const SeqNumRegPair &inst_reg_pair);

    /**
     * Add an instruction that is at the head of the ROB and is squashed only
     * if it is a load and a request was sent for it.
     *
     * @param head_inst pointer to dynamic instruction to be squashed
     */
    void addSquashedInst(const DynInstConstPtr& head_inst);

    /**
     * Add an instruction that is at the head of the ROB and is committed.
     *
     * @param head_inst pointer to dynamic instruction to be committed
     */
    void addCommittedInst(const DynInstConstPtr& head_inst);

    /** Register statistics for the elastic trace. */
    void regStats();

    /** Event to trigger registering this listener for all probe points. */
    EventFunctionWrapper regEtraceListenersEvent;

  private:
    /**
     * Used for checking the first window for processing and writing of
     * dependency trace. At the start of the program there can be dependency-
     * free instructions and such cases are handled differently.
     */
    bool firstWin;

    /**
     * @defgroup InstExecInfo Struct for storing information before an
     * instruction reaches the commit stage, e.g. execute timestamp.
     */
    struct InstExecInfo
    {
        /**
         * @ingroup InstExecInfo
         * @{
         */
        /** Timestamp when instruction was first processed by execute stage */
        Tick executeTick;
        /**
         * Timestamp when instruction execution is completed in execute stage
         * and instruction is marked as ready to commit
         */
        Tick toCommitTick;
        /**
         * Set of instruction sequence numbers that this instruction depends on
         * due to Read After Write data dependency based on physical register.
         */
        std::set<InstSeqNum> physRegDepSet;
        /** @} */

        /** Constructor */
        InstExecInfo()
          : executeTick(MaxTick),
            toCommitTick(MaxTick)
        { }
    };

    /**
     * Temporary store of InstExecInfo objects. Later on when an instruction
     * is processed for commit or retire, if it is chosen to be written to
     * the output trace then this information is looked up using the instruction
     * sequence number as the key. If it is not chosen then the entry for it in
     * the store is cleared.
     */
    std::unordered_map<InstSeqNum, InstExecInfo*> tempStore;

    /**
     * The last cleared instruction sequence number used to free up the memory
     * allocated in the temporary store.
     */
    InstSeqNum lastClearedSeqNum;

    /**
     * Map for recording the producer of a physical register to check Read
     * After Write dependencies. The key is the renamed physical register and
     * the value is the instruction sequence number of its last producer.
     */
    std::unordered_map<PhysRegIndex, InstSeqNum> physRegDepMap;

    /**
     * @defgroup TraceInfo Struct for a record in the instruction dependency
     * trace. All information required to process and calculate the
     * computational delay is stored in TraceInfo objects. The memory request
     * fields for a load or store instruction are also included here. Note
     * that the structure TraceInfo does not store pointers to children
     * or parents. The dependency trace is maintained as an ordered collection
     * of records for writing to the output trace and not as a tree data
     * structure.
     */
    struct TraceInfo
    {
        /**
         * @ingroup TraceInfo
         * @{
         */
        /* Instruction sequence number. */
        InstSeqNum instNum;
        /** The type of trace record for the instruction node */
        RecordType type;
        /* Tick when instruction was in execute stage. */
        Tick executeTick;
        /* Tick when instruction was marked ready and sent to commit stage. */
        Tick toCommitTick;
        /* Tick when instruction was committed. */
        Tick commitTick;
        /* If instruction was committed, as against squashed. */
        bool commit;
        /* List of order dependencies. */
        std::list<InstSeqNum> robDepList;
        /* List of physical register RAW dependencies. */
        std::list<InstSeqNum> physRegDepList;
        /**
         * Computational delay after the last dependent inst. completed.
         * A value of -1 which means instruction has no dependencies.
         */
        int64_t compDelay;
        /* Number of dependents. */
        uint32_t numDepts;
        /* The instruction PC for a load, store or non load/store. */
        Addr pc;
        /* Request flags in case of a load/store instruction */
        Request::FlagsType reqFlags;
        /* Request physical address in case of a load/store instruction */
        Addr physAddr;
        /* Request virtual address in case of a load/store instruction */
        Addr virtAddr;
        /* Request size in case of a load/store instruction */
        unsigned size;
        /** Default Constructor */
        TraceInfo()
          : type(Record::INVALID)
        { }
        /** Is the record a load */
        bool isLoad() const { return (type == Record::LOAD); }
        /** Is the record a store */
        bool isStore() const { return (type == Record::STORE); }
        /** Is the record a fetch triggering an Icache request */
        bool isComp() const { return (type == Record::COMP); }
        /** Return string specifying the type of the node */
        const std::string& typeToStr() const;
        /** @} */

        /**
         * Get the execute tick of the instruction.
         *
         * @return Tick when instruction was executed
         */
        Tick getExecuteTick() const;
    };

    /**
     * The instruction dependency trace containing TraceInfo objects. The
     * container implemented is sequential as dependencies obey commit
     * order (program order). For example, if B is dependent on A then B must
     * be committed after A. Thus records are updated with dependency
     * information and written to the trace in commit order. This ensures that
     * when a graph is reconstructed from the  trace during replay, all the
     * dependencies are stored in the graph before  the dependent itself is
     * added. This facilitates creating a tree data structure during replay,
     * i.e. adding children as records are read from the trace in an efficient
     * manner.
     */
    std::vector<TraceInfo*> depTrace;

    /**
     * Map where the instruction sequence number is mapped to the pointer to
     * the TraceInfo object.
     */
    std::unordered_map<InstSeqNum, TraceInfo*> traceInfoMap;

    /** Typedef of iterator to the instruction dependency trace. */
    typedef typename std::vector<TraceInfo*>::iterator depTraceItr;

    /** Typedef of the reverse iterator to the instruction dependency trace. */
    typedef typename std::reverse_iterator<depTraceItr> depTraceRevItr;

    /**
     * The maximum distance for a dependency and is set by a top level
     * level parameter. It must be equal to or greater than the number of
     * entries in the ROB. This variable is used as the length of the sliding
     * window for processing the dependency trace.
     */
    uint32_t depWindowSize;

    /** Protobuf output stream for data dependency trace */
    ProtoOutputStream* dataTraceStream;

    /** Protobuf output stream for instruction fetch trace. */
    ProtoOutputStream* instTraceStream;

    /** Number of instructions after which to enable tracing. */
    const InstSeqNum startTraceInst;

    /**
     * Whther the elastic trace listener has been registered for all probes.
     *
     * When enabling tracing after a specified number of instructions have
     * committed, check this to prevent re-registering the listener.
     */
    bool allProbesReg;

    /** Whether to trace virtual addresses for memory requests. */
    const bool traceVirtAddr;

    /** Pointer to the O3CPU that is this listener's parent a.k.a. manager */
    FullO3CPU<O3CPUImpl>* cpu;

    /**
     * Add a record to the dependency trace depTrace which is a sequential
     * container. A record is inserted per committed instruction and in the same
     * order as the order in which instructions are committed.
     *
     * @param head_inst     Pointer to the instruction which is head of the
     *                      ROB and ready to commit
     * @param exec_info_ptr Pointer to InstExecInfo for that instruction
     * @param commit        True if instruction is committed, false if squashed
     */
    void addDepTraceRecord(const DynInstConstPtr& head_inst,
                           InstExecInfo* exec_info_ptr, bool commit);

    /**
     * Clear entries in the temporary store of execution info objects to free
     * allocated memory until the present instruction being added to the trace.
     *
     * @param head_inst pointer to dynamic instruction
     */
    void clearTempStoreUntil(const DynInstConstPtr& head_inst);

    /**
     * Calculate the computational delay between an instruction and a
     * subsequent instruction that has an ROB (order) dependency on it
     *
     * @param past_record   Pointer to instruction
     *
     * @param new_record    Pointer to subsequent instruction having an ROB
     *                      dependency on the instruction pointed to by
     *                      past_record
     */
    void compDelayRob(TraceInfo* past_record, TraceInfo* new_record);

    /**
     * Calculate the computational delay between an instruction and a
     * subsequent instruction that has a Physical Register (data) dependency on
     * it.
     *
     * @param past_record   Pointer to instruction
     *
     * @param new_record    Pointer to subsequent instruction having a Physical
     *                      Register dependency on the instruction pointed to
     *                      by past_record
     */
    void compDelayPhysRegDep(TraceInfo* past_record, TraceInfo* new_record);

    /**
     * Write out given number of records to the trace starting with the first
     * record in depTrace and iterating through the trace in sequence. A
     * record is deleted after it is written.
     *
     * @param num_to_write Number of records to write to the trace
     */
    void writeDepTrace(uint32_t num_to_write);

    /**
     * Reverse iterate through the graph, search for a store-after-store or
     * store-after-load dependency and update the new node's Rob dependency list.
     *
     * If a dependency is found, then call the assignRobDep() method that
     * updates the store with the dependency information. This function is only
     * called when a new store node is added to the trace.
     *
     * @param new_record    pointer to new store record
     * @param find_load_not_store true for searching store-after-load and false
     *                          for searching store-after-store dependency
     */
    void updateCommitOrderDep(TraceInfo* new_record, bool find_load_not_store);

    /**
     * Reverse iterate through the graph, search for an issue order dependency
     * for a new node and update the new node's Rob dependency list.
     *
     * If a dependency is found, call the assignRobDep() method that updates
     * the node with its dependency information. This function is called in
     * case a new node to be added to the trace is dependency-free or its
     * dependency got discarded because the dependency was outside the window.
     *
     * @param new_record    pointer to new record to be added to the trace
     */
    void updateIssueOrderDep(TraceInfo* new_record);

    /**
     * The new_record has an order dependency on a past_record, thus update the
     * new record's Rob dependency list and increment the number of dependents
     * of the past record.
     *
     * @param new_record    pointer to new record
     * @param past_record   pointer to record that new_record has a rob
     *                      dependency on
     */
    void assignRobDep(TraceInfo* past_record, TraceInfo* new_record);

    /**
     * Check if past record is a store sent earlier than the execute tick.
     *
     * @param past_record   pointer to past store
     * @param execute_tick  tick with which to compare past store's commit tick
     *
     * @return true if past record is store sent earlier
     */
    bool hasStoreCommitted(TraceInfo* past_record, Tick execute_tick) const;

    /**
     * Check if past record is a load that completed earlier than the execute
     * tick.
     *
     * @param past_record   pointer to past load
     * @param execute_tick  tick with which to compare past load's complete
     *                      tick
     *
     * @return true if past record is load completed earlier
     */
    bool hasLoadCompleted(TraceInfo* past_record, Tick execute_tick) const;

    /**
     * Check if past record is a load sent earlier than the execute tick.
     *
     * @param past_record   pointer to past load
     * @param execute_tick  tick with which to compare past load's send tick
     *
     * @return true if past record is load sent earlier
     */
    bool hasLoadBeenSent(TraceInfo* past_record, Tick execute_tick) const;

    /**
     * Check if past record is a comp node that completed earlier than the
     * execute tick.
     *
     * @param past_record   pointer to past comp node
     * @param execute_tick  tick with which to compare past comp node's
     *                      completion tick
     *
     * @return true if past record is comp completed earlier
     */
    bool hasCompCompleted(TraceInfo* past_record, Tick execute_tick) const;

    /** Number of register dependencies recorded during tracing */
    Stats::Scalar numRegDep;

    /**
     * Number of stores that got assigned a commit order dependency
     * on a past load/store.
     */
    Stats::Scalar numOrderDepStores;

    /**
     * Number of load insts that got assigned an issue order dependency
     * because they were dependency-free.
     */
    Stats::Scalar numIssueOrderDepLoads;

    /**
     * Number of store insts that got assigned an issue order dependency
     * because they were dependency-free.
     */
    Stats::Scalar numIssueOrderDepStores;

    /**
     * Number of non load/store insts that got assigned an issue order
     * dependency because they were dependency-free.
     */
    Stats::Scalar numIssueOrderDepOther;

    /** Number of filtered nodes */
    Stats::Scalar numFilteredNodes;

    /** Maximum number of dependents on any instruction */
    Stats::Scalar maxNumDependents;

    /**
     * Maximum size of the temporary store mostly useful as a check that it is
     * not growing
     */
    Stats::Scalar maxTempStoreSize;

    /**
     * Maximum size of the map that holds the last writer to a physical
     * register.
     * */
    Stats::Scalar maxPhysRegDepMapSize;

};
#endif//__CPU_O3_PROBE_ELASTIC_TRACE_HH__
