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

#include "cpu/o3/probe/elastic_trace.hh"

#include "base/callback.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/reg_class.hh"
#include "debug/ElasticTrace.hh"
#include "mem/packet.hh"

namespace gem5
{

namespace o3
{

ElasticTrace::ElasticTrace(const ElasticTraceParams &params)
    :  ProbeListenerObject(params),
       regEtraceListenersEvent([this]{ regEtraceListeners(); }, name()),
       firstWin(true),
       lastClearedSeqNum(0),
       depWindowSize(params.depWindowSize),
       dataTraceStream(nullptr),
       instTraceStream(nullptr),
       startTraceInst(params.startTraceInst),
       allProbesReg(false),
       traceVirtAddr(params.traceVirtAddr),
       stats(this)
{
    cpu = dynamic_cast<CPU *>(params.manager);

    fatal_if(!cpu, "Manager of %s is not of type O3CPU and thus does not "\
                "support dependency tracing.\n", name());

    fatal_if(depWindowSize == 0, "depWindowSize parameter must be non-zero. "\
                "Recommended size is 3x ROB size in the O3CPU.\n");

    fatal_if(cpu->numThreads > 1, "numThreads = %i, %s supports tracing for"\
                "single-threaded workload only", cpu->numThreads, name());
    // Initialize the protobuf output stream
    fatal_if(params.instFetchTraceFile == "", "Assign instruction fetch "\
                "trace file path to instFetchTraceFile");
    fatal_if(params.dataDepTraceFile == "", "Assign data dependency "\
                "trace file path to dataDepTraceFile");
    std::string filename = simout.resolve(name() + "." +
                                            params.instFetchTraceFile);
    instTraceStream = new ProtoOutputStream(filename);
    filename = simout.resolve(name() + "." + params.dataDepTraceFile);
    dataTraceStream = new ProtoOutputStream(filename);
    // Create a protobuf message for the header and write it to the stream
    ProtoMessage::PacketHeader inst_pkt_header;
    inst_pkt_header.set_obj_id(name());
    inst_pkt_header.set_tick_freq(sim_clock::Frequency);
    instTraceStream->write(inst_pkt_header);
    // Create a protobuf message for the header and write it to
    // the stream
    ProtoMessage::InstDepRecordHeader data_rec_header;
    data_rec_header.set_obj_id(name());
    data_rec_header.set_tick_freq(sim_clock::Frequency);
    data_rec_header.set_window_size(depWindowSize);
    dataTraceStream->write(data_rec_header);
    // Register a callback to flush trace records and close the output streams.
    registerExitCallback([this]() {  flushTraces(); });
}

void
ElasticTrace::regProbeListeners()
{
    inform("@%llu: regProbeListeners() called, startTraceInst = %llu",
        curTick(), startTraceInst);
    if (startTraceInst == 0) {
        // If we want to start tracing from the start of the simulation,
        // register all elastic trace probes now.
        regEtraceListeners();
    } else {
        // Schedule an event to register all elastic trace probes when
        // specified no. of instructions are committed.
        cpu->getContext(0)->scheduleInstCountEvent(
                &regEtraceListenersEvent, startTraceInst);
    }
}

void
ElasticTrace::regEtraceListeners()
{
    assert(!allProbesReg);
    inform("@%llu: No. of instructions committed = %llu, registering elastic"
        " probe listeners", curTick(), cpu->numSimulatedInsts());
    // Create new listeners: provide method to be called upon a notify() for
    // each probe point.
    listeners.push_back(new ProbeListenerArg<ElasticTrace, RequestPtr>(this,
                        "FetchRequest", &ElasticTrace::fetchReqTrace));
    listeners.push_back(new ProbeListenerArg<ElasticTrace,
            DynInstConstPtr>(this, "Execute",
                &ElasticTrace::recordExecTick));
    listeners.push_back(new ProbeListenerArg<ElasticTrace,
            DynInstConstPtr>(this, "ToCommit",
                &ElasticTrace::recordToCommTick));
    listeners.push_back(new ProbeListenerArg<ElasticTrace,
            DynInstConstPtr>(this, "Rename",
                &ElasticTrace::updateRegDep));
    listeners.push_back(new ProbeListenerArg<ElasticTrace, SeqNumRegPair>(this,
                        "SquashInRename", &ElasticTrace::removeRegDepMapEntry));
    listeners.push_back(new ProbeListenerArg<ElasticTrace,
            DynInstConstPtr>(this, "Squash",
                &ElasticTrace::addSquashedInst));
    listeners.push_back(new ProbeListenerArg<ElasticTrace,
            DynInstConstPtr>(this, "Commit",
                &ElasticTrace::addCommittedInst));
    allProbesReg = true;
}

void
ElasticTrace::fetchReqTrace(const RequestPtr &req)
{

    DPRINTFR(ElasticTrace, "Fetch Req %i,(%lli,%lli,%lli),%i,%i,%lli\n",
             (MemCmd::ReadReq),
             req->getPC(), req->getVaddr(), req->getPaddr(),
             req->getFlags(), req->getSize(), curTick());

    // Create a protobuf message including the request fields necessary to
    // recreate the request in the TraceCPU.
    ProtoMessage::Packet inst_fetch_pkt;
    inst_fetch_pkt.set_tick(curTick());
    inst_fetch_pkt.set_cmd(MemCmd::ReadReq);
    inst_fetch_pkt.set_pc(req->getPC());
    inst_fetch_pkt.set_flags(req->getFlags());
    inst_fetch_pkt.set_addr(req->getPaddr());
    inst_fetch_pkt.set_size(req->getSize());
    // Write the message to the stream.
    instTraceStream->write(inst_fetch_pkt);
}

void
ElasticTrace::recordExecTick(const DynInstConstPtr& dyn_inst)
{

    // In a corner case, a retired instruction is propagated backward to the
    // IEW instruction queue to handle some side-channel information. But we
    // must not process an instruction again. So we test the sequence number
    // against the lastClearedSeqNum and skip adding the instruction for such
    // corner cases.
    if (dyn_inst->seqNum <= lastClearedSeqNum) {
        DPRINTFR(ElasticTrace, "[sn:%lli] Ignoring in execute as instruction \
        has already retired (mostly squashed)", dyn_inst->seqNum);
        // Do nothing as program has proceeded and this inst has been
        // propagated backwards to handle something.
        return;
    }

    DPRINTFR(ElasticTrace, "[sn:%lli] Execute Tick = %i\n", dyn_inst->seqNum,
                curTick());
    // Either the execution info object will already exist if this
    // instruction had a register dependency recorded in the rename probe
    // listener before entering execute stage or it will not exist and will
    // need to be created here.
    InstExecInfo* exec_info_ptr;
    auto itr_exec_info = tempStore.find(dyn_inst->seqNum);
    if (itr_exec_info != tempStore.end()) {
        exec_info_ptr = itr_exec_info->second;
    } else {
        exec_info_ptr = new InstExecInfo;
        tempStore[dyn_inst->seqNum] = exec_info_ptr;
    }

    exec_info_ptr->executeTick = curTick();
    stats.maxTempStoreSize = std::max(tempStore.size(),
                                (std::size_t)stats.maxTempStoreSize.value());
}

void
ElasticTrace::recordToCommTick(const DynInstConstPtr& dyn_inst)
{
    // If tracing has just been enabled then the instruction at this stage of
    // execution is far enough that we cannot gather info about its past like
    // the tick it started execution. Simply return until we see an instruction
    // that is found in the tempStore.
    auto itr_exec_info = tempStore.find(dyn_inst->seqNum);
    if (itr_exec_info == tempStore.end()) {
        DPRINTFR(ElasticTrace, "recordToCommTick: [sn:%lli] Not in temp store,"
                    " skipping.\n", dyn_inst->seqNum);
        return;
    }

    DPRINTFR(ElasticTrace, "[sn:%lli] To Commit Tick = %i\n", dyn_inst->seqNum,
                curTick());
    InstExecInfo* exec_info_ptr = itr_exec_info->second;
    exec_info_ptr->toCommitTick = curTick();

}

void
ElasticTrace::updateRegDep(const DynInstConstPtr& dyn_inst)
{
    // Get the sequence number of the instruction
    InstSeqNum seq_num = dyn_inst->seqNum;

    assert(dyn_inst->seqNum > lastClearedSeqNum);

    // Since this is the first probe activated in the pipeline, create
    // a new execution info object to track this instruction as it
    // progresses through the pipeline.
    InstExecInfo* exec_info_ptr = new InstExecInfo;
    tempStore[seq_num] = exec_info_ptr;

    // Loop through the source registers and look up the dependency map. If
    // the source register entry is found in the dependency map, add a
    // dependency on the last writer.
    int8_t max_regs = dyn_inst->numSrcRegs();
    for (int src_idx = 0; src_idx < max_regs; src_idx++) {

        const RegId& src_reg = dyn_inst->srcRegIdx(src_idx);
        if (!src_reg.is(MiscRegClass) && !src_reg.is(InvalidRegClass)) {
            // Get the physical register index of the i'th source register.
            PhysRegIdPtr phys_src_reg = dyn_inst->renamedSrcIdx(src_idx);
            DPRINTFR(ElasticTrace, "[sn:%lli] Check map for src reg"
                     " %i (%s)\n", seq_num,
                     phys_src_reg->flatIndex(), phys_src_reg->className());
            auto itr_writer = physRegDepMap.find(phys_src_reg->flatIndex());
            if (itr_writer != physRegDepMap.end()) {
                InstSeqNum last_writer = itr_writer->second;
                // Additionally the dependency distance is kept less than the
                // window size parameter to limit the memory allocation to
                // nodes in the graph. If the window were tending to infinite
                // we would have to load a large number of node objects during
                // replay.
                if (seq_num - last_writer < depWindowSize) {
                    // Record a physical register dependency.
                    exec_info_ptr->physRegDepSet.insert(last_writer);
                }
            }

        }

    }

    // Loop through the destination registers of this instruction and update
    // the physical register dependency map for last writers to registers.
    max_regs = dyn_inst->numDestRegs();
    for (int dest_idx = 0; dest_idx < max_regs; dest_idx++) {
        // For data dependency tracking the register must be an int, float or
        // CC register and not a Misc register.
        const RegId& dest_reg = dyn_inst->destRegIdx(dest_idx);
        if (!dest_reg.is(MiscRegClass) && !dest_reg.is(InvalidRegClass)) {
            // Get the physical register index of the i'th destination
            // register.
            PhysRegIdPtr phys_dest_reg =
                dyn_inst->renamedDestIdx(dest_idx);
            DPRINTFR(ElasticTrace, "[sn:%lli] Update map for dest reg"
                     " %i (%s)\n", seq_num, phys_dest_reg->flatIndex(),
                     dest_reg.className());
            physRegDepMap[phys_dest_reg->flatIndex()] = seq_num;
        }
    }
    stats.maxPhysRegDepMapSize = std::max(physRegDepMap.size(),
                            (std::size_t)stats.maxPhysRegDepMapSize.value());
}

void
ElasticTrace::removeRegDepMapEntry(const SeqNumRegPair &inst_reg_pair)
{
    DPRINTFR(ElasticTrace, "Remove Map entry for Reg %i\n",
            inst_reg_pair.second);
    auto itr_regdep_map = physRegDepMap.find(inst_reg_pair.second);
    if (itr_regdep_map != physRegDepMap.end())
        physRegDepMap.erase(itr_regdep_map);
}

void
ElasticTrace::addSquashedInst(const DynInstConstPtr& head_inst)
{
    // If the squashed instruction was squashed before being processed by
    // execute stage then it will not be in the temporary store. In this case
    // do nothing and return.
    auto itr_exec_info = tempStore.find(head_inst->seqNum);
    if (itr_exec_info == tempStore.end())
        return;

    // If there is a squashed load for which a read request was
    // sent before it got squashed then add it to the trace.
    DPRINTFR(ElasticTrace, "Attempt to add squashed inst [sn:%lli]\n",
                head_inst->seqNum);
    // Get pointer to the execution info object corresponding to the inst.
    InstExecInfo* exec_info_ptr = itr_exec_info->second;
    if (head_inst->isLoad() && exec_info_ptr->executeTick != MaxTick &&
        exec_info_ptr->toCommitTick != MaxTick &&
        head_inst->hasRequest() &&
        head_inst->getFault() == NoFault) {
        // Add record to depTrace with commit parameter as false.
        addDepTraceRecord(head_inst, exec_info_ptr, false);
    }
    // As the information contained is no longer needed, remove the execution
    // info object from the temporary store.
    clearTempStoreUntil(head_inst);
}

void
ElasticTrace::addCommittedInst(const DynInstConstPtr& head_inst)
{
    DPRINTFR(ElasticTrace, "Attempt to add committed inst [sn:%lli]\n",
                head_inst->seqNum);

    // Add the instruction to the depTrace.
    if (!head_inst->isNop()) {

        // If tracing has just been enabled then the instruction at this stage
        // of execution is far enough that we cannot gather info about its past
        // like the tick it started execution. Simply return until we see an
        // instruction that is found in the tempStore.
        auto itr_temp_store = tempStore.find(head_inst->seqNum);
        if (itr_temp_store == tempStore.end()) {
            DPRINTFR(ElasticTrace, "addCommittedInst: [sn:%lli] Not in temp "
                "store, skipping.\n", head_inst->seqNum);
            return;
        }

        // Get pointer to the execution info object corresponding to the inst.
        InstExecInfo* exec_info_ptr = itr_temp_store->second;
        assert(exec_info_ptr->executeTick != MaxTick);
        assert(exec_info_ptr->toCommitTick != MaxTick);

        // Check if the instruction had a fault, if it predicated false and
        // thus previous register values were restored or if it was a
        // load/store that did not have a request (e.g. when the size of the
        // request is zero). In all these cases the instruction is set as
        // executed and is picked up by the commit probe listener. But a
        // request is not issued and registers are not written. So practically,
        // skipping these should not hurt as execution would not stall on them.
        // Alternatively, these could be included merely as a compute node in
        // the graph. Removing these for now. If correlation accuracy needs to
        // be improved in future these can be turned into comp nodes at the
        // cost of bigger traces.
        if (head_inst->getFault() != NoFault) {
            DPRINTF(ElasticTrace, "%s [sn:%lli] has faulted so "
                    "skip adding it to the trace\n",
                    (head_inst->isMemRef() ? "Load/store" : "Comp inst."),
                    head_inst->seqNum);
        } else if (head_inst->isMemRef() && !head_inst->hasRequest()) {
            DPRINTF(ElasticTrace, "Load/store [sn:%lli]  has no request so "
                    "skip adding it to the trace\n", head_inst->seqNum);
        } else if (!head_inst->readPredicate()) {
            DPRINTF(ElasticTrace, "%s [sn:%lli] is predicated false so "
                    "skip adding it to the trace\n",
                    (head_inst->isMemRef() ? "Load/store" : "Comp inst."),
                    head_inst->seqNum);
        } else {
            // Add record to depTrace with commit parameter as true.
            addDepTraceRecord(head_inst, exec_info_ptr, true);
        }
    }
    // As the information contained is no longer needed, remove the execution
    // info object from the temporary store.
    clearTempStoreUntil(head_inst);
}

void
ElasticTrace::addDepTraceRecord(const DynInstConstPtr& head_inst,
                                InstExecInfo* exec_info_ptr, bool commit)
{
    // Create a record to assign dynamic intruction related fields.
    TraceInfo* new_record = new TraceInfo;
    // Add to map for sequence number look up to retrieve the TraceInfo pointer
    traceInfoMap[head_inst->seqNum] = new_record;

    // Assign fields from the instruction
    new_record->instNum = head_inst->seqNum;
    new_record->commit = commit;
    new_record->type = head_inst->isLoad() ? Record::LOAD :
                        (head_inst->isStore() ? Record::STORE :
                        Record::COMP);

    // Assign fields for creating a request in case of a load/store
    new_record->reqFlags = head_inst->memReqFlags;
    new_record->virtAddr = head_inst->effAddr;
    new_record->physAddr = head_inst->physEffAddr;
    // Currently the tracing does not support split requests.
    new_record->size = head_inst->effSize;
    new_record->pc = head_inst->pcState().instAddr();

    // Assign the timing information stored in the execution info object
    new_record->executeTick = exec_info_ptr->executeTick;
    new_record->toCommitTick = exec_info_ptr->toCommitTick;
    new_record->commitTick = curTick();

    // Assign initial values for number of dependents and computational delay
    new_record->numDepts = 0;
    new_record->compDelay = -1;

    // The physical register dependency set of the first instruction is
    // empty. Since there are no records in the depTrace at this point, the
    // case of adding an ROB dependency by using a reverse iterator is not
    // applicable. Thus, populate the fields of the record corresponding to the
    // first instruction and return.
    if (depTrace.empty()) {
        // Store the record in depTrace.
        depTrace.push_back(new_record);
        DPRINTF(ElasticTrace, "Added first inst record %lli to DepTrace.\n",
                new_record->instNum);
        return;
    }

    // Clear register dependencies for squashed loads as they may be dependent
    // on squashed instructions and we do not add those to the trace.
    if (head_inst->isLoad() && !commit) {
         (exec_info_ptr->physRegDepSet).clear();
    }

    // Assign the register dependencies stored in the execution info object
    std::set<InstSeqNum>::const_iterator dep_set_it;
    for (dep_set_it = (exec_info_ptr->physRegDepSet).begin();
         dep_set_it != (exec_info_ptr->physRegDepSet).end();
         ++dep_set_it) {
        auto trace_info_itr = traceInfoMap.find(*dep_set_it);
        if (trace_info_itr != traceInfoMap.end()) {
            // The register dependency is valid. Assign it and calculate
            // computational delay
            new_record->physRegDepList.push_back(*dep_set_it);
            DPRINTF(ElasticTrace, "Inst %lli has register dependency on "
                    "%lli\n", new_record->instNum, *dep_set_it);
            TraceInfo* reg_dep = trace_info_itr->second;
            reg_dep->numDepts++;
            compDelayPhysRegDep(reg_dep, new_record);
            ++stats.numRegDep;
        } else {
            // The instruction that this has a register dependency on was
            // not added to the trace because of one of the following
            // 1. it was an instruction that had a fault
            // 2. it was an instruction that was predicated false and
            // previous register values were restored
            // 3. it was load/store that did not have a request (e.g. when
            // the size of the request is zero but this may not be a fault)
            // In all these cases the instruction is set as executed and is
            // picked up by the commit probe listener. But a request is not
            // issued and registers are not written to in these cases.
            DPRINTF(ElasticTrace, "Inst %lli has register dependency on "
                    "%lli is skipped\n",new_record->instNum, *dep_set_it);
        }
    }

    // Check for and assign an ROB dependency in addition to register
    // dependency before adding the record to the trace.
    // As stores have to commit in order a store is dependent on the last
    // committed load/store. This is recorded in the ROB dependency.
    if (head_inst->isStore()) {
        // Look up store-after-store order dependency
        updateCommitOrderDep(new_record, false);
        // Look up store-after-load order dependency
        updateCommitOrderDep(new_record, true);
    }

    // In case a node is dependency-free or its dependency got discarded
    // because it was outside the window, it is marked ready in the ROB at the
    // time of issue. A request is sent as soon as possible. To model this, a
    // node is assigned an issue order dependency on a committed instruction
    // that completed earlier than it. This is done to avoid the problem of
    // determining the issue times of such dependency-free nodes during replay
    // which could lead to too much parallelism, thinking conservatively.
    if (new_record->robDepList.empty() && new_record->physRegDepList.empty()) {
        updateIssueOrderDep(new_record);
    }

    // Store the record in depTrace.
    depTrace.push_back(new_record);
    DPRINTF(ElasticTrace, "Added %s inst %lli to DepTrace.\n",
            (commit ? "committed" : "squashed"), new_record->instNum);

    // To process the number of records specified by depWindowSize in the
    // forward direction, the depTrace must have twice as many records
    // to check for dependencies.
    if (depTrace.size() == 2 * depWindowSize) {

        DPRINTF(ElasticTrace, "Writing out trace...\n");

        // Write out the records which have been processed to the trace
        // and remove them from the depTrace.
        writeDepTrace(depWindowSize);

        // After the first window, writeDepTrace() must check for valid
        // compDelay.
        firstWin = false;
    }
}

void
ElasticTrace::updateCommitOrderDep(TraceInfo* new_record,
                                    bool find_load_not_store)
{
    assert(new_record->isStore());
    // Iterate in reverse direction to search for the last committed
    // load/store that completed earlier than the new record
    depTraceRevItr from_itr(depTrace.end());
    depTraceRevItr until_itr(depTrace.begin());
    TraceInfo* past_record = *from_itr;
    uint32_t num_go_back = 0;

    // The execution time of this store is when it is sent, that is committed
    Tick execute_tick = curTick();
    // Search for store-after-load or store-after-store order dependency
    while (num_go_back < depWindowSize && from_itr != until_itr) {
        if (find_load_not_store) {
            // Check if previous inst is a load completed earlier by comparing
            // with execute tick
            if (hasLoadCompleted(past_record, execute_tick)) {
                // Assign rob dependency and calculate the computational delay
                assignRobDep(past_record, new_record);
                ++stats.numRegDep;
                return;
            }
        } else {
            // Check if previous inst is a store sent earlier by comparing with
            // execute tick
            if (hasStoreCommitted(past_record, execute_tick)) {
                // Assign rob dependency and calculate the computational delay
                assignRobDep(past_record, new_record);
                ++stats.numRegDep;
                return;
            }
        }
        ++from_itr;
        past_record = *from_itr;
        ++num_go_back;
    }
}

void
ElasticTrace::updateIssueOrderDep(TraceInfo* new_record)
{
    // Interate in reverse direction to search for the last committed
    // record that completed earlier than the new record
    depTraceRevItr from_itr(depTrace.end());
    depTraceRevItr until_itr(depTrace.begin());
    TraceInfo* past_record = *from_itr;

    uint32_t num_go_back = 0;
    Tick execute_tick = 0;

    if (new_record->isLoad()) {
        // The execution time of a load is when a request is sent
        execute_tick = new_record->executeTick;
        ++stats.numIssueOrderDepLoads;
    } else if (new_record->isStore()) {
        // The execution time of a store is when it is sent, i.e. committed
        execute_tick = curTick();
        ++stats.numIssueOrderDepStores;
    } else {
        // The execution time of a non load/store is when it completes
        execute_tick = new_record->toCommitTick;
        ++stats.numIssueOrderDepOther;
    }

    // We search if this record has an issue order dependency on a past record.
    // Once we find it, we update both the new record and the record it depends
    // on and return.
    while (num_go_back < depWindowSize && from_itr != until_itr) {
        // Check if a previous inst is a load sent earlier, or a store sent
        // earlier, or a comp inst completed earlier by comparing with execute
        // tick
        if (hasLoadBeenSent(past_record, execute_tick) ||
            hasStoreCommitted(past_record, execute_tick) ||
            hasCompCompleted(past_record, execute_tick)) {
            // Assign rob dependency and calculate the computational delay
            assignRobDep(past_record, new_record);
            return;
        }
        ++from_itr;
        past_record = *from_itr;
        ++num_go_back;
    }
}

void
ElasticTrace::assignRobDep(TraceInfo* past_record, TraceInfo* new_record) {
    DPRINTF(ElasticTrace, "%s %lli has ROB dependency on %lli\n",
            new_record->typeToStr(), new_record->instNum,
            past_record->instNum);
    // Add dependency on past record
    new_record->robDepList.push_back(past_record->instNum);
    // Update new_record's compute delay with respect to the past record
    compDelayRob(past_record, new_record);
    // Increment number of dependents of the past record
    ++(past_record->numDepts);
    // Update stat to log max number of dependents
    stats.maxNumDependents = std::max(past_record->numDepts,
                                (uint32_t)stats.maxNumDependents.value());
}

bool
ElasticTrace::hasStoreCommitted(TraceInfo* past_record,
                                    Tick execute_tick) const
{
    return (past_record->isStore() && past_record->commitTick <= execute_tick);
}

bool
ElasticTrace::hasLoadCompleted(TraceInfo* past_record,
                                    Tick execute_tick) const
{
    return(past_record->isLoad() && past_record->commit &&
                past_record->toCommitTick <= execute_tick);
}

bool
ElasticTrace::hasLoadBeenSent(TraceInfo* past_record,
                                Tick execute_tick) const
{
    // Check if previous inst is a load sent earlier than this
    return (past_record->isLoad() && past_record->commit &&
        past_record->executeTick <= execute_tick);
}

bool
ElasticTrace::hasCompCompleted(TraceInfo* past_record,
                                    Tick execute_tick) const
{
    return(past_record->isComp() && past_record->toCommitTick <= execute_tick);
}

void
ElasticTrace::clearTempStoreUntil(const DynInstConstPtr& head_inst)
{
    // Clear from temp store starting with the execution info object
    // corresponding the head_inst and continue clearing by decrementing the
    // sequence number until the last cleared sequence number.
    InstSeqNum temp_sn = (head_inst->seqNum);
    while (temp_sn > lastClearedSeqNum) {
        auto itr_exec_info = tempStore.find(temp_sn);
        if (itr_exec_info != tempStore.end()) {
            InstExecInfo* exec_info_ptr = itr_exec_info->second;
            // Free allocated memory for the info object
            delete exec_info_ptr;
            // Remove entry from temporary store
            tempStore.erase(itr_exec_info);
        }
        temp_sn--;
    }
    // Update the last cleared sequence number to that of the head_inst
    lastClearedSeqNum = head_inst->seqNum;
}

void
ElasticTrace::compDelayRob(TraceInfo* past_record, TraceInfo* new_record)
{
    // The computation delay is the delay between the completion tick of the
    // inst. pointed to by past_record and the execution tick of its dependent
    // inst. pointed to by new_record.
    int64_t comp_delay = -1;
    Tick execution_tick = 0, completion_tick = 0;

    DPRINTF(ElasticTrace, "Seq num %lli has ROB dependency on seq num %lli.\n",
            new_record->instNum, past_record->instNum);

    // Get the tick when the node is executed as per the modelling of
    // computation delay
    execution_tick = new_record->getExecuteTick();

    if (past_record->isLoad()) {
        if (new_record->isStore()) {
            completion_tick = past_record->toCommitTick;
        } else {
            completion_tick = past_record->executeTick;
        }
    } else if (past_record->isStore()) {
        completion_tick = past_record->commitTick;
    } else if (past_record->isComp()){
        completion_tick = past_record->toCommitTick;
    }
    assert(execution_tick >= completion_tick);
    comp_delay = execution_tick - completion_tick;

    DPRINTF(ElasticTrace, "Computational delay is %lli - %lli = %lli\n",
            execution_tick, completion_tick, comp_delay);

    // Assign the computational delay with respect to the dependency which
    // completes the latest.
    if (new_record->compDelay == -1)
        new_record->compDelay = comp_delay;
    else
        new_record->compDelay = std::min(comp_delay, new_record->compDelay);
    DPRINTF(ElasticTrace, "Final computational delay = %lli.\n",
            new_record->compDelay);
}

void
ElasticTrace::compDelayPhysRegDep(TraceInfo* past_record,
                                    TraceInfo* new_record)
{
    // The computation delay is the delay between the completion tick of the
    // inst. pointed to by past_record and the execution tick of its dependent
    // inst. pointed to by new_record.
    int64_t comp_delay = -1;
    Tick execution_tick = 0, completion_tick = 0;

    DPRINTF(ElasticTrace, "Seq. num %lli has register dependency on seq. num"
            " %lli.\n", new_record->instNum, past_record->instNum);

    // Get the tick when the node is executed as per the modelling of
    // computation delay
    execution_tick = new_record->getExecuteTick();

    // When there is a physical register dependency on an instruction, the
    // completion tick of that instruction is when it wrote to the register,
    // that is toCommitTick. In case, of a store updating a destination
    // register, this is approximated to commitTick instead
    if (past_record->isStore()) {
        completion_tick = past_record->commitTick;
    } else {
        completion_tick = past_record->toCommitTick;
    }
    assert(execution_tick >= completion_tick);
    comp_delay = execution_tick - completion_tick;
    DPRINTF(ElasticTrace, "Computational delay is %lli - %lli = %lli\n",
            execution_tick, completion_tick, comp_delay);

    // Assign the computational delay with respect to the dependency which
    // completes the latest.
    if (new_record->compDelay == -1)
        new_record->compDelay = comp_delay;
    else
        new_record->compDelay = std::min(comp_delay, new_record->compDelay);
    DPRINTF(ElasticTrace, "Final computational delay = %lli.\n",
            new_record->compDelay);
}

Tick
ElasticTrace::TraceInfo::getExecuteTick() const
{
    if (isLoad()) {
        // Execution tick for a load instruction is when the request was sent,
        // that is executeTick.
        return executeTick;
    } else if (isStore()) {
        // Execution tick for a store instruction is when the request was sent,
        // that is commitTick.
        return commitTick;
    } else {
        // Execution tick for a non load/store instruction is when the register
        // value was written to, that is commitTick.
        return toCommitTick;
    }
}

void
ElasticTrace::writeDepTrace(uint32_t num_to_write)
{
    // Write the trace with fields as follows:
    // Instruction sequence number
    // If instruction was a load
    // If instruction was a store
    // If instruction has addr
    // If instruction has size
    // If instruction has flags
    // List of order dependencies - optional, repeated
    // Computational delay with respect to last completed dependency
    // List of physical register RAW dependencies - optional, repeated
    // Weight of a node equal to no. of filtered nodes before it - optional
    uint16_t num_filtered_nodes = 0;
    depTraceItr dep_trace_itr(depTrace.begin());
    depTraceItr dep_trace_itr_start = dep_trace_itr;
    while (num_to_write > 0) {
        TraceInfo* temp_ptr = *dep_trace_itr;
        assert(temp_ptr->type != Record::INVALID);
        // If no node dependends on a comp node then there is no reason to
        // track the comp node in the dependency graph. We filter out such
        // nodes but count them and add a weight field to the subsequent node
        // that we do include in the trace.
        if (!temp_ptr->isComp() || temp_ptr->numDepts != 0) {
            DPRINTFR(ElasticTrace, "Instruction with seq. num %lli "
                     "is as follows:\n", temp_ptr->instNum);
            if (temp_ptr->isLoad() || temp_ptr->isStore()) {
                DPRINTFR(ElasticTrace, "\tis a %s\n", temp_ptr->typeToStr());
                DPRINTFR(ElasticTrace, "\thas a request with phys addr %i, "
                         "size %i, flags %i\n", temp_ptr->physAddr,
                         temp_ptr->size, temp_ptr->reqFlags);
            } else {
                 DPRINTFR(ElasticTrace, "\tis a %s\n", temp_ptr->typeToStr());
            }
            if (firstWin && temp_ptr->compDelay == -1) {
                if (temp_ptr->isLoad()) {
                    temp_ptr->compDelay = temp_ptr->executeTick;
                } else if (temp_ptr->isStore()) {
                    temp_ptr->compDelay = temp_ptr->commitTick;
                } else {
                    temp_ptr->compDelay = temp_ptr->toCommitTick;
                }
            }
            assert(temp_ptr->compDelay != -1);
            DPRINTFR(ElasticTrace, "\thas computational delay %lli\n",
                     temp_ptr->compDelay);

            // Create a protobuf message for the dependency record
            ProtoMessage::InstDepRecord dep_pkt;
            dep_pkt.set_seq_num(temp_ptr->instNum);
            dep_pkt.set_type(temp_ptr->type);
            dep_pkt.set_pc(temp_ptr->pc);
            if (temp_ptr->isLoad() || temp_ptr->isStore()) {
                dep_pkt.set_flags(temp_ptr->reqFlags);
                dep_pkt.set_p_addr(temp_ptr->physAddr);
                // If tracing of virtual addresses is enabled, set the optional
                // field for it
                if (traceVirtAddr)
                    dep_pkt.set_v_addr(temp_ptr->virtAddr);
                dep_pkt.set_size(temp_ptr->size);
            }
            dep_pkt.set_comp_delay(temp_ptr->compDelay);
            if (temp_ptr->robDepList.empty()) {
                DPRINTFR(ElasticTrace, "\thas no order (rob) dependencies\n");
            }
            while (!temp_ptr->robDepList.empty()) {
                DPRINTFR(ElasticTrace, "\thas order (rob) dependency on %lli\n",
                         temp_ptr->robDepList.front());
                dep_pkt.add_rob_dep(temp_ptr->robDepList.front());
                temp_ptr->robDepList.pop_front();
            }
            if (temp_ptr->physRegDepList.empty()) {
                DPRINTFR(ElasticTrace, "\thas no register dependencies\n");
            }
            while (!temp_ptr->physRegDepList.empty()) {
                DPRINTFR(ElasticTrace, "\thas register dependency on %lli\n",
                         temp_ptr->physRegDepList.front());
                dep_pkt.add_reg_dep(temp_ptr->physRegDepList.front());
                temp_ptr->physRegDepList.pop_front();
            }
            if (num_filtered_nodes != 0) {
                // Set the weight of this node as the no. of filtered nodes
                // between this node and the last node that we wrote to output
                // stream. The weight will be used during replay to model ROB
                // occupancy of filtered nodes.
                dep_pkt.set_weight(num_filtered_nodes);
                num_filtered_nodes = 0;
            }
            // Write the message to the protobuf output stream
            dataTraceStream->write(dep_pkt);
        } else {
            // Don't write the node to the trace but note that we have filtered
            // out a node.
            ++stats.numFilteredNodes;
            ++num_filtered_nodes;
        }
        dep_trace_itr++;
        traceInfoMap.erase(temp_ptr->instNum);
        delete temp_ptr;
        num_to_write--;
    }
    depTrace.erase(dep_trace_itr_start, dep_trace_itr);
}

ElasticTrace::ElasticTraceStats::ElasticTraceStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numRegDep, statistics::units::Count::get(),
               "Number of register dependencies recorded during tracing"),
      ADD_STAT(numOrderDepStores, statistics::units::Count::get(),
               "Number of commit order (rob) dependencies for a store "
               "recorded on a past load/store during tracing"),
      ADD_STAT(numIssueOrderDepLoads, statistics::units::Count::get(),
               "Number of loads that got assigned issue order dependency "
               "because they were dependency-free"),
      ADD_STAT(numIssueOrderDepStores, statistics::units::Count::get(),
               "Number of stores that got assigned issue order dependency "
               "because they were dependency-free"),
      ADD_STAT(numIssueOrderDepOther, statistics::units::Count::get(),
               "Number of non load/store insts that got assigned issue order "
               "dependency because they were dependency-free"),
      ADD_STAT(numFilteredNodes, statistics::units::Count::get(),
               "No. of nodes filtered out before writing the output trace"),
      ADD_STAT(maxNumDependents, statistics::units::Count::get(),
               "Maximum number or dependents on any instruction"),
      ADD_STAT(maxTempStoreSize, statistics::units::Count::get(),
               "Maximum size of the temporary store during the run"),
      ADD_STAT(maxPhysRegDepMapSize, statistics::units::Count::get(),
               "Maximum size of register dependency map")
{
}

const std::string&
ElasticTrace::TraceInfo::typeToStr() const
{
    return Record::RecordType_Name(type);
}

void
ElasticTrace::flushTraces()
{
    // Write to trace all records in the depTrace.
    writeDepTrace(depTrace.size());
    // Delete the stream objects
    delete dataTraceStream;
    delete instTraceStream;
}

} // namespace o3
} // namespace gem5
