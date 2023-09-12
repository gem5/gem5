/*
 * Copyright (c) 2013 - 2016, 2023 Arm Limited
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

#include "cpu/trace/trace_cpu.hh"

#include "base/compiler.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

// Declare and initialize the static counter for number of trace CPUs.
int TraceCPU::numTraceCPUs = 0;

TraceCPU::TraceCPU(const TraceCPUParams &params)
    :   ClockedObject(params),
        cacheLineSize(params.system->cacheLineSize()),
        icachePort(this),
        dcachePort(this),
        instRequestorID(params.system->getRequestorId(this, "inst")),
        dataRequestorID(params.system->getRequestorId(this, "data")),
        instTraceFile(params.instTraceFile),
        dataTraceFile(params.dataTraceFile),
        icacheGen(*this, ".iside", icachePort, instRequestorID, instTraceFile),
        dcacheGen(*this, ".dside", dcachePort, dataRequestorID, dataTraceFile,
                  params),
        icacheNextEvent([this]{ schedIcacheNext(); }, name()),
        dcacheNextEvent([this]{ schedDcacheNext(); }, name()),
        oneTraceComplete(false),
        traceOffset(0),
        execCompleteEvent(nullptr),
        enableEarlyExit(params.enableEarlyExit),
        progressMsgInterval(params.progressMsgInterval),
        progressMsgThreshold(params.progressMsgInterval), traceStats(this)
{
    // Increment static counter for number of Trace CPUs.
    ++TraceCPU::numTraceCPUs;

    // Check that the python parameters for sizes of ROB, store buffer and
    // load buffer do not overflow the corresponding C++ variables.
    fatal_if(params.sizeROB > UINT16_MAX,
             "ROB size set to %d exceeds the max. value of %d.",
             params.sizeROB, UINT16_MAX);
    fatal_if(params.sizeStoreBuffer > UINT16_MAX,
             "ROB size set to %d exceeds the max. value of %d.",
             params.sizeROB, UINT16_MAX);
    fatal_if(params.sizeLoadBuffer > UINT16_MAX,
             "Load buffer size set to %d exceeds the max. value of %d.",
                params.sizeLoadBuffer, UINT16_MAX);
}

void
TraceCPU::updateNumOps(uint64_t rob_num)
{
    traceStats.numOps = rob_num;
    if (progressMsgInterval != 0 &&
         traceStats.numOps.value() >= progressMsgThreshold) {
        inform("%s: %i insts committed\n", name(), progressMsgThreshold);
        progressMsgThreshold += progressMsgInterval;
    }
}

void
TraceCPU::init()
{
    DPRINTF(TraceCPUInst, "Instruction fetch request trace file is \"%s\".\n",
            instTraceFile);
    DPRINTF(TraceCPUData, "Data memory request trace file is \"%s\".\n",
            dataTraceFile);

    ClockedObject::init();

    // Get the send tick of the first instruction read request
    Tick first_icache_tick = icacheGen.init();

    // Get the send tick of the first data read/write request
    Tick first_dcache_tick = dcacheGen.init();

    // Set the trace offset as the minimum of that in both traces
    traceOffset = std::min(first_icache_tick, first_dcache_tick);
    inform("%s: Time offset (tick) found as min of both traces is %lli.",
            name(), traceOffset);

    // Schedule next icache and dcache event by subtracting the offset
    schedule(icacheNextEvent, first_icache_tick - traceOffset);
    schedule(dcacheNextEvent, first_dcache_tick - traceOffset);

    // Adjust the trace offset for the dcache generator's ready nodes
    // We don't need to do this for the icache generator as it will
    // send its first request at the first event and schedule subsequent
    // events using a relative tick delta
    dcacheGen.adjustInitTraceOffset(traceOffset);

    // If the Trace CPU simulation is configured to exit on any one trace
    // completion then we don't need a counted event to count down all Trace
    // CPUs in the system. If not then instantiate a counted event.
    if (!enableEarlyExit) {
        // The static counter for number of Trace CPUs is correctly set at
        // this point so create an event and pass it.
        execCompleteEvent = new CountedExitEvent("end of all traces reached.",
                                                 numTraceCPUs);
    }

}

void
TraceCPU::schedIcacheNext()
{
    DPRINTF(TraceCPUInst, "IcacheGen event.\n");

    // Try to send the current packet or a retry packet if there is one
    bool sched_next = icacheGen.tryNext();
    // If packet sent successfully, schedule next event
    if (sched_next) {
        DPRINTF(TraceCPUInst,
                "Scheduling next icacheGen event at %d.\n",
                curTick() + icacheGen.tickDelta());
        schedule(icacheNextEvent, curTick() + icacheGen.tickDelta());
        ++traceStats.numSchedIcacheEvent;
    } else {
        // check if traceComplete. If not, do nothing because sending failed
        // and next event will be scheduled via RecvRetry()
        if (icacheGen.isTraceComplete()) {
            // If this is the first trace to complete, set the variable. If it
            // is already set then both traces are complete to exit sim.
            checkAndSchedExitEvent();
        }
    }
    return;
}

void
TraceCPU::schedDcacheNext()
{
    DPRINTF(TraceCPUData, "DcacheGen event.\n");

    // Update stat for numCycles
    traceStats.numCycles = clockEdge() / clockPeriod();

    dcacheGen.execute();
    if (dcacheGen.isExecComplete()) {
        checkAndSchedExitEvent();
    }
}

void
TraceCPU::checkAndSchedExitEvent()
{
    if (!oneTraceComplete) {
        oneTraceComplete = true;
    } else {
        // Schedule event to indicate execution is complete as both
        // instruction and data access traces have been played back.
        inform("%s: Execution complete.", name());
        // If the replay is configured to exit early, that is when any one
        // execution is complete then exit immediately and return. Otherwise,
        // schedule the counted exit that counts down completion of each Trace
        // CPU.
        if (enableEarlyExit) {
            exitSimLoop("End of trace reached");
        } else {
            schedule(*execCompleteEvent, curTick());
        }
    }
}
 TraceCPU::TraceStats::TraceStats(TraceCPU *trace) :
    statistics::Group(trace),
    ADD_STAT(numSchedDcacheEvent, statistics::units::Count::get(),
             "Number of events scheduled to trigger data request generator"),
    ADD_STAT(numSchedIcacheEvent, statistics::units::Count::get(),
             "Number of events scheduled to trigger instruction request "
             "generator"),
    ADD_STAT(numOps, statistics::units::Count::get(),
             "Number of micro-ops simulated by the Trace CPU"),
    ADD_STAT(cpi, statistics::units::Rate<
                    statistics::units::Cycle, statistics::units::Count>::get(),
             "Cycles per micro-op used as a proxy for CPI",
             trace->traceStats.numCycles / numOps)
{
    cpi.precision(6);
}

TraceCPU::ElasticDataGen::
ElasticDataGenStatGroup::ElasticDataGenStatGroup(statistics::Group *parent,
                                                 const std::string& _name) :
    statistics::Group(parent, _name.c_str()),
    ADD_STAT(maxDependents, statistics::units::Count::get(),
             "Max number of dependents observed on a node"),
    ADD_STAT(maxReadyListSize, statistics::units::Count::get(),
             "Max size of the ready list observed"),
    ADD_STAT(numSendAttempted, statistics::units::Count::get(),
             "Number of first attempts to send a request"),
    ADD_STAT(numSendSucceeded, statistics::units::Count::get(),
             "Number of successful first attempts"),
    ADD_STAT(numSendFailed, statistics::units::Count::get(),
             "Number of failed first attempts"),
    ADD_STAT(numRetrySucceeded, statistics::units::Count::get(),
             "Number of successful retries"),
    ADD_STAT(numSplitReqs, statistics::units::Count::get(),
             "Number of split requests"),
    ADD_STAT(numSOLoads, statistics::units::Count::get(),
             "Number of strictly ordered loads"),
    ADD_STAT(numSOStores, statistics::units::Count::get(),
             "Number of strictly ordered stores"),
    ADD_STAT(dataLastTick, statistics::units::Tick::get(),
             "Last tick simulated from the elastic data trace")
{
}

Tick
TraceCPU::ElasticDataGen::init()
{
    DPRINTF(TraceCPUData, "Initializing data memory request generator "
            "DcacheGen: elastic issue with retry.\n");

    panic_if(!readNextWindow(),
            "Trace has %d elements. It must have at least %d elements.",
            depGraph.size(), 2 * windowSize);
    DPRINTF(TraceCPUData, "After 1st read, depGraph size:%d.\n",
            depGraph.size());

    panic_if(!readNextWindow(),
            "Trace has %d elements. It must have at least %d elements.",
            depGraph.size(), 2 * windowSize);
    DPRINTF(TraceCPUData, "After 2st read, depGraph size:%d.\n",
            depGraph.size());

    // Print readyList
    if (debug::TraceCPUData) {
        printReadyList();
    }
    auto free_itr = readyList.begin();
    DPRINTF(TraceCPUData,
            "Execute tick of the first dependency free node %lli is %d.\n",
            free_itr->seqNum, free_itr->execTick);
    // Return the execute tick of the earliest ready node so that an event
    // can be scheduled to call execute()
    return (free_itr->execTick);
}

void
TraceCPU::ElasticDataGen::adjustInitTraceOffset(Tick& offset)
{
    for (auto& free_node : readyList) {
        free_node.execTick -= offset;
    }
}

void
TraceCPU::ElasticDataGen::exit()
{
    trace.reset();
}

bool
TraceCPU::ElasticDataGen::readNextWindow()
{
    // Read and add next window
    DPRINTF(TraceCPUData, "Reading next window from file.\n");

    if (traceComplete) {
        // We are at the end of the file, thus we have no more records.
        // Return false.
        return false;
    }

    DPRINTF(TraceCPUData, "Start read: Size of depGraph is %d.\n",
            depGraph.size());

    uint32_t num_read = 0;
    while (num_read != windowSize) {

        // Create a new graph node
        GraphNode* new_node = new GraphNode;

        // Read the next line to get the next record. If that fails then end of
        // trace has been reached and traceComplete needs to be set in addition
        // to returning false.
        if (!trace.read(new_node)) {
            DPRINTF(TraceCPUData, "\tTrace complete!\n");
            traceComplete = true;
            return false;
        }

        // Annotate the ROB dependencies of the new node onto the parent nodes.
        addDepsOnParent(new_node, new_node->robDep);
        // Annotate the register dependencies of the new node onto the parent
        // nodes.
        addDepsOnParent(new_node, new_node->regDep);

        num_read++;
        // Add to map
        depGraph[new_node->seqNum] = new_node;
        if (new_node->robDep.empty() && new_node->regDep.empty()) {
            // Source dependencies are already complete, check if resources
            // are available and issue. The execution time is approximated
            // to current time plus the computational delay.
            checkAndIssue(new_node);
        }
    }

    DPRINTF(TraceCPUData, "End read: Size of depGraph is %d.\n",
            depGraph.size());
    return true;
}

template<typename T>
void
TraceCPU::ElasticDataGen::addDepsOnParent(GraphNode *new_node, T& dep_list)
{
    auto dep_it = dep_list.begin();
    while (dep_it != dep_list.end()) {
        // We look up the valid dependency, i.e. the parent of this node
        auto parent_itr = depGraph.find(*dep_it);
        if (parent_itr != depGraph.end()) {
            // If the parent is found, it is yet to be executed. Append a
            // pointer to the new node to the dependents list of the parent
            // node.
            parent_itr->second->dependents.push_back(new_node);
            auto num_depts = parent_itr->second->dependents.size();
            elasticStats.maxDependents = std::max<double>(num_depts,
                                        elasticStats.maxDependents.value());
            dep_it++;
        } else {
            // The dependency is not found in the graph. So consider
            // the execution of the parent is complete, i.e. remove this
            // dependency.
            dep_it = dep_list.erase(dep_it);
        }
    }
}

void
TraceCPU::ElasticDataGen::execute()
{
    DPRINTF(TraceCPUData, "Execute start occupancy:\n");
    DPRINTFR(TraceCPUData, "\tdepGraph = %d, readyList = %d, "
            "depFreeQueue = %d ,", depGraph.size(), readyList.size(),
            depFreeQueue.size());
    hwResource.printOccupancy();

    // Read next window to make sure that dependents of all dep-free nodes
    // are in the depGraph
    if (nextRead) {
        readNextWindow();
        nextRead = false;
    }

    // First attempt to issue the pending dependency-free nodes held
    // in depFreeQueue. If resources have become available for a node,
    // then issue it, i.e. add the node to readyList.
    while (!depFreeQueue.empty()) {
        if (checkAndIssue(depFreeQueue.front(), false)) {
            DPRINTF(TraceCPUData,
                    "Removing from depFreeQueue: seq. num %lli.\n",
                    (depFreeQueue.front())->seqNum);
            depFreeQueue.pop();
        } else {
            break;
        }
    }
    // Proceed to execute from readyList
    auto graph_itr = depGraph.begin();
    auto free_itr = readyList.begin();
    // Iterate through readyList until the next free node has its execute
    // tick later than curTick or the end of readyList is reached
    while (free_itr->execTick <= curTick() && free_itr != readyList.end()) {

        // Get pointer to the node to be executed
        graph_itr = depGraph.find(free_itr->seqNum);
        assert(graph_itr != depGraph.end());
        GraphNode* node_ptr = graph_itr->second;

        // If there is a retryPkt send that else execute the load
        if (retryPkt) {
            // The retryPkt must be the request that was created by the
            // first node in the readyList.
            if (retryPkt->req->getReqInstSeqNum() != node_ptr->seqNum) {
                panic("Retry packet's seqence number does not match "
                      "the first node in the readyList.\n");
            }
            if (port.sendTimingReq(retryPkt)) {
                ++elasticStats.numRetrySucceeded;
                retryPkt = nullptr;
            }
        } else if (node_ptr->isLoad() || node_ptr->isStore()) {
            // If there is no retryPkt, attempt to send a memory request in
            // case of a load or store node. If the send fails, executeMemReq()
            // returns a packet pointer, which we save in retryPkt. In case of
            // a comp node we don't do anything and simply continue as if the
            // execution of the comp node succedded.
            retryPkt = executeMemReq(node_ptr);
        }
        // If the retryPkt or a new load/store node failed, we exit from here
        // as a retry from cache will bring the control to execute(). The
        // first node in readyList then, will be the failed node.
        if (retryPkt) {
            break;
        }

        // Proceed to remove dependencies for the successfully executed node.
        // If it is a load which is not strictly ordered and we sent a
        // request for it successfully, we do not yet mark any register
        // dependencies complete. But as per dependency modelling we need
        // to mark ROB dependencies of load and non load/store nodes which
        // are based on successful sending of the load as complete.
        if (node_ptr->isLoad() && !node_ptr->isStrictlyOrdered()) {
            // If execute succeeded mark its dependents as complete
            DPRINTF(TraceCPUData,
                    "Node seq. num %lli sent. Waking up dependents..\n",
                    node_ptr->seqNum);

            auto child_itr = (node_ptr->dependents).begin();
            while (child_itr != (node_ptr->dependents).end()) {
                // ROB dependency of a store on a load must not be removed
                // after load is sent but after response is received
                if (!(*child_itr)->isStore() &&
                    (*child_itr)->removeRobDep(node_ptr->seqNum)) {

                    // Check if the child node has become dependency free
                    if ((*child_itr)->robDep.empty() &&
                        (*child_itr)->regDep.empty()) {

                        // Source dependencies are complete, check if
                        // resources are available and issue
                        checkAndIssue(*child_itr);
                    }
                    // Remove this child for the sent load and point to new
                    // location of the element following the erased element
                    child_itr = node_ptr->dependents.erase(child_itr);
                } else {
                    // This child is not dependency-free, point to the next
                    // child
                    child_itr++;
                }
            }
        } else {
            // If it is a strictly ordered load mark its dependents as complete
            // as we do not send a request for this case. If it is a store or a
            // comp node we also mark all its dependents complete.
            DPRINTF(TraceCPUData, "Node seq. num %lli done. Waking"
                    " up dependents..\n", node_ptr->seqNum);

            for (auto child : node_ptr->dependents) {
                // If the child node is dependency free removeDepOnInst()
                // returns true.
                if (child->removeDepOnInst(node_ptr->seqNum)) {
                    // Source dependencies are complete, check if resources
                    // are available and issue
                    checkAndIssue(child);
                }
            }
        }

        // After executing the node, remove from readyList and delete node.
        readyList.erase(free_itr);
        // If it is a cacheable load which was sent, don't delete
        // just yet.  Delete it in completeMemAccess() after the
        // response is received. If it is an strictly ordered
        // load, it was not sent and all dependencies were simply
        // marked complete. Thus it is safe to delete it. For
        // stores and non load/store nodes all dependencies were
        // marked complete so it is safe to delete it.
        if (!node_ptr->isLoad() || node_ptr->isStrictlyOrdered()) {
            // Release all resources occupied by the completed node
            hwResource.release(node_ptr);
            // clear the dynamically allocated set of dependents
            (node_ptr->dependents).clear();
            // Update the stat for numOps simulated
            owner.updateNumOps(node_ptr->robNum);
            // delete node
            delete node_ptr;
            // remove from graph
            depGraph.erase(graph_itr);
        }
        // Point to first node to continue to next iteration of while loop
        free_itr = readyList.begin();
    } // end of while loop

    // Print readyList, sizes of queues and resource status after updating
    if (debug::TraceCPUData) {
        printReadyList();
        DPRINTF(TraceCPUData, "Execute end occupancy:\n");
        DPRINTFR(TraceCPUData, "\tdepGraph = %d, readyList = %d, "
                "depFreeQueue = %d ,", depGraph.size(), readyList.size(),
                depFreeQueue.size());
        hwResource.printOccupancy();
    }

    if (retryPkt) {
        DPRINTF(TraceCPUData, "Not scheduling an event as expecting a retry"
                "event from the cache for seq. num %lli.\n",
                retryPkt->req->getReqInstSeqNum());
        return;
    }
    // If the size of the dependency graph is less than the dependency window
    // then read from the trace file to populate the graph next time we are in
    // execute.
    if (depGraph.size() < windowSize && !traceComplete)
        nextRead = true;

    // If cache is not blocked, schedule an event for the first execTick in
    // readyList else retry from cache will schedule the event. If the ready
    // list is empty then check if the next pending node has resources
    // available to issue. If yes, then schedule an event for the next cycle.
    if (!readyList.empty()) {
        Tick next_event_tick = std::max(readyList.begin()->execTick,
                                        curTick());
        DPRINTF(TraceCPUData, "Attempting to schedule @%lli.\n",
                next_event_tick);
        owner.schedDcacheNextEvent(next_event_tick);
    } else if (readyList.empty() && !depFreeQueue.empty() &&
                hwResource.isAvailable(depFreeQueue.front())) {
        DPRINTF(TraceCPUData, "Attempting to schedule @%lli.\n",
                owner.clockEdge(Cycles(1)));
        owner.schedDcacheNextEvent(owner.clockEdge(Cycles(1)));
    }

    // If trace is completely read, readyList is empty and depGraph is empty,
    // set execComplete to true
    if (depGraph.empty() && readyList.empty() && traceComplete &&
        !hwResource.awaitingResponse()) {
        DPRINTF(TraceCPUData, "\tExecution Complete!\n");
        execComplete = true;
        elasticStats.dataLastTick = curTick();
    }
}

PacketPtr
TraceCPU::ElasticDataGen::executeMemReq(GraphNode* node_ptr)
{
    DPRINTF(TraceCPUData, "Executing memory request %lli (phys addr %d, "
            "virt addr %d, pc %#x, size %d, flags %d).\n",
            node_ptr->seqNum, node_ptr->physAddr, node_ptr->virtAddr,
            node_ptr->pc, node_ptr->size, node_ptr->flags);

    // If the request is strictly ordered, do not send it. Just return nullptr
    // as if it was succesfully sent.
    if (node_ptr->isStrictlyOrdered()) {
        node_ptr->isLoad() ? ++elasticStats.numSOLoads :
             ++elasticStats.numSOStores;
        DPRINTF(TraceCPUData, "Skipping strictly ordered request %lli.\n",
                node_ptr->seqNum);
        return nullptr;
    }

    // Check if the request spans two cache lines as this condition triggers
    // an assert fail in the L1 cache. If it does then truncate the size to
    // access only until the end of that line and ignore the remainder. The
    // stat counting this is useful to keep a check on how frequently this
    // happens. If required the code could be revised to mimick splitting such
    // a request into two.
    unsigned blk_size = owner.cacheLineSize;
    Addr blk_offset = (node_ptr->physAddr & (Addr)(blk_size - 1));
    if (!(blk_offset + node_ptr->size <= blk_size)) {
        node_ptr->size = blk_size - blk_offset;
        ++elasticStats.numSplitReqs;
    }

    // Create a request and the packet containing request
    auto req = std::make_shared<Request>(
        node_ptr->physAddr, node_ptr->size, node_ptr->flags, requestorId);
    req->setReqInstSeqNum(node_ptr->seqNum);

    // If this is not done it triggers assert in L1 cache for invalid contextId
    req->setContext(ContextID(0));

    req->setPC(node_ptr->pc);
    // If virtual address is valid, set the virtual address field
    // of the request.
    if (node_ptr->virtAddr != 0) {
        req->setVirt(node_ptr->virtAddr, node_ptr->size,
                     node_ptr->flags, requestorId, node_ptr->pc);
        req->setPaddr(node_ptr->physAddr);
        req->setReqInstSeqNum(node_ptr->seqNum);
    }

    PacketPtr pkt;
    uint8_t* pkt_data = new uint8_t[req->getSize()];
    if (node_ptr->isLoad()) {
        pkt = Packet::createRead(req);
    } else {
        pkt = Packet::createWrite(req);
        memset(pkt_data, 0xA, req->getSize());
    }
    pkt->dataDynamic(pkt_data);

    // Call RequestPort method to send a timing request for this packet
    bool success = port.sendTimingReq(pkt);
    ++elasticStats.numSendAttempted;

    if (!success) {
        // If it fails, return the packet to retry when a retry is signalled by
        // the cache
        ++elasticStats.numSendFailed;
        DPRINTF(TraceCPUData, "Send failed. Saving packet for retry.\n");
        return pkt;
    } else {
        // It is succeeds, return nullptr
        ++elasticStats.numSendSucceeded;
        return nullptr;
    }
}

bool
TraceCPU::ElasticDataGen::checkAndIssue(const GraphNode* node_ptr, bool first)
{
    // Assert the node is dependency-free
    assert(node_ptr->robDep.empty() && node_ptr->regDep.empty());

    // If this is the first attempt, print a debug message to indicate this.
    if (first) {
        DPRINTFR(TraceCPUData, "\t\tseq. num %lli(%s) with rob num %lli is now"
                " dependency free.\n", node_ptr->seqNum, node_ptr->typeToStr(),
                node_ptr->robNum);
    }

    // Check if resources are available to issue the specific node
    if (hwResource.isAvailable(node_ptr)) {
        // If resources are free only then add to readyList
        DPRINTFR(TraceCPUData, "\t\tResources available for seq. num %lli. "
                "Adding to readyList, occupying resources.\n",
                node_ptr->seqNum);
        // Compute the execute tick by adding the compute delay for the node
        // and add the ready node to the ready list
        addToSortedReadyList(node_ptr->seqNum,
                             owner.clockEdge() + node_ptr->compDelay);
        // Account for the resources taken up by this issued node.
        hwResource.occupy(node_ptr);
        return true;
    } else {
        if (first) {
            // Although dependencies are complete, resources are not available.
            DPRINTFR(TraceCPUData, "\t\tResources unavailable for seq. num "
                    "%lli. Adding to depFreeQueue.\n", node_ptr->seqNum);
            depFreeQueue.push(node_ptr);
        } else {
            DPRINTFR(TraceCPUData, "\t\tResources unavailable for seq. num "
                    "%lli. Still pending issue.\n", node_ptr->seqNum);
        }
        return false;
    }
}

void
TraceCPU::ElasticDataGen::completeMemAccess(PacketPtr pkt)
{
    // Release the resources for this completed node.
    if (pkt->isWrite()) {
        // Consider store complete.
        hwResource.releaseStoreBuffer();
        // If it is a store response then do nothing since we do not model
        // dependencies on store completion in the trace. But if we were
        // blocking execution due to store buffer fullness, we need to schedule
        // an event and attempt to progress.
    } else {
        // If it is a load response then release the dependents waiting on it.
        // Get pointer to the completed load
        auto graph_itr = depGraph.find(pkt->req->getReqInstSeqNum());
        assert(graph_itr != depGraph.end());
        GraphNode* node_ptr = graph_itr->second;

        // Release resources occupied by the load
        hwResource.release(node_ptr);

        DPRINTF(TraceCPUData, "Load seq. num %lli response received. Waking up"
                " dependents..\n", node_ptr->seqNum);

        for (auto child : node_ptr->dependents) {
            if (child->removeDepOnInst(node_ptr->seqNum)) {
                checkAndIssue(child);
            }
        }

        // clear the dynamically allocated set of dependents
        (node_ptr->dependents).clear();
        // Update the stat for numOps completed
        owner.updateNumOps(node_ptr->robNum);
        // delete node
        delete node_ptr;
        // remove from graph
        depGraph.erase(graph_itr);
    }

    if (debug::TraceCPUData) {
        printReadyList();
    }

    // If the size of the dependency graph is less than the dependency window
    // then read from the trace file to populate the graph next time we are in
    // execute.
    if (depGraph.size() < windowSize && !traceComplete)
        nextRead = true;

    // If not waiting for retry, attempt to schedule next event
    if (!retryPkt) {
        // We might have new dep-free nodes in the list which will have execute
        // tick greater than or equal to curTick. But a new dep-free node might
        // have its execute tick earlier. Therefore, attempt to reschedule. It
        // could happen that the readyList is empty and we got here via a
        // last remaining response. So, either the trace is complete or there
        // are pending nodes in the depFreeQueue. The checking is done in the
        // execute() control flow, so schedule an event to go via that flow.
        Tick next_event_tick = readyList.empty() ? owner.clockEdge(Cycles(1)) :
            std::max(readyList.begin()->execTick, owner.clockEdge(Cycles(1)));
        DPRINTF(TraceCPUData, "Attempting to schedule @%lli.\n",
                next_event_tick);
        owner.schedDcacheNextEvent(next_event_tick);
    }
}

void
TraceCPU::ElasticDataGen::addToSortedReadyList(NodeSeqNum seq_num,
                                               Tick exec_tick)
{
    ReadyNode ready_node;
    ready_node.seqNum = seq_num;
    ready_node.execTick = exec_tick;

    // Iterator to readyList
    auto itr = readyList.begin();

    // If the readyList is empty, simply insert the new node at the beginning
    // and return
    if (itr == readyList.end()) {
        readyList.insert(itr, ready_node);
        elasticStats.maxReadyListSize =
            std::max<double>(readyList.size(),
                             elasticStats.maxReadyListSize.value());
        return;
    }

    // If the new node has its execution tick equal to the first node in the
    // list then go to the next node. If the first node in the list failed
    // to execute, its position as the first is thus maintained.
    if (retryPkt) {
        if (retryPkt->req->getReqInstSeqNum() == itr->seqNum)
            itr++;
    }

    // Increment the iterator and compare the node pointed to by it to the new
    // node till the position to insert the new node is found.
    bool found = false;
    while (!found && itr != readyList.end()) {
        // If the execution tick of the new node is less than the node then
        // this is the position to insert
        if (exec_tick < itr->execTick) {
            found = true;
        // If the execution tick of the new node is equal to the node then
        // sort in ascending order of sequence numbers
        } else if (exec_tick == itr->execTick) {
            // If the sequence number of the new node is less than the node
            // then this is the position to insert
            if (seq_num < itr->seqNum) {
                found = true;
            // Else go to next node
            } else {
                itr++;
            }
        } else {
            // If the execution tick of the new node is greater than the node
            // then go to the next node.
            itr++;
        }
    }
    readyList.insert(itr, ready_node);
    // Update the stat for max size reached of the readyList
    elasticStats.maxReadyListSize = std::max<double>(readyList.size(),
                                        elasticStats.maxReadyListSize.value());
}

void
TraceCPU::ElasticDataGen::printReadyList()
{
    auto itr = readyList.begin();
    if (itr == readyList.end()) {
        DPRINTF(TraceCPUData, "readyList is empty.\n");
        return;
    }
    DPRINTF(TraceCPUData, "Printing readyList:\n");
    while (itr != readyList.end()) {
        auto graph_itr = depGraph.find(itr->seqNum);
        [[maybe_unused]] GraphNode* node_ptr = graph_itr->second;
        DPRINTFR(TraceCPUData, "\t%lld(%s), %lld\n", itr->seqNum,
            node_ptr->typeToStr(), itr->execTick);
        itr++;
    }
}

TraceCPU::ElasticDataGen::HardwareResource::HardwareResource(
        uint16_t max_rob, uint16_t max_stores, uint16_t max_loads) :
    sizeROB(max_rob),
    sizeStoreBuffer(max_stores),
    sizeLoadBuffer(max_loads),
    oldestInFlightRobNum(UINT64_MAX),
    numInFlightLoads(0),
    numInFlightStores(0)
{}

void
TraceCPU::ElasticDataGen::HardwareResource::occupy(const GraphNode* new_node)
{
    // Occupy ROB entry for the issued node
    // Merely maintain the oldest node, i.e. numerically least robNum by saving
    // it in the variable oldestInFLightRobNum.
    inFlightNodes[new_node->seqNum] = new_node->robNum;
    oldestInFlightRobNum = inFlightNodes.begin()->second;

    // Occupy Load/Store Buffer entry for the issued node if applicable
    if (new_node->isLoad()) {
        ++numInFlightLoads;
    } else if (new_node->isStore()) {
        ++numInFlightStores;
    } // else if it is a non load/store node, no buffer entry is occupied

    printOccupancy();
}

void
TraceCPU::ElasticDataGen::HardwareResource::release(const GraphNode* done_node)
{
    assert(!inFlightNodes.empty());
    DPRINTFR(TraceCPUData,
            "\tClearing done seq. num %d from inFlightNodes..\n",
            done_node->seqNum);

    assert(inFlightNodes.find(done_node->seqNum) != inFlightNodes.end());
    inFlightNodes.erase(done_node->seqNum);

    if (inFlightNodes.empty()) {
        // If we delete the only in-flight node and then the
        // oldestInFlightRobNum is set to it's initialized (max) value.
        oldestInFlightRobNum = UINT64_MAX;
    } else {
        // Set the oldest in-flight node rob number equal to the first node in
        // the inFlightNodes since that will have the numerically least value.
        oldestInFlightRobNum = inFlightNodes.begin()->second;
    }

    DPRINTFR(TraceCPUData,
            "\tCleared. inFlightNodes.size() = %d, "
            "oldestInFlightRobNum = %d\n", inFlightNodes.size(),
            oldestInFlightRobNum);

    // A store is considered complete when a request is sent, thus ROB entry is
    // freed. But it occupies an entry in the Store Buffer until its response
    // is received. A load is considered complete when a response is received,
    // thus both ROB and Load Buffer entries can be released.
    if (done_node->isLoad()) {
        assert(numInFlightLoads != 0);
        --numInFlightLoads;
    }
    // For normal writes, we send the requests out and clear a store buffer
    // entry on response. For writes which are strictly ordered, for e.g.
    // writes to device registers, we do that within release() which is called
    // when node is executed and taken off from readyList.
    if (done_node->isStore() && done_node->isStrictlyOrdered()) {
        releaseStoreBuffer();
    }
}

void
TraceCPU::ElasticDataGen::HardwareResource::releaseStoreBuffer()
{
    assert(numInFlightStores != 0);
    --numInFlightStores;
}

bool
TraceCPU::ElasticDataGen::HardwareResource::isAvailable(
        const GraphNode* new_node) const
{
    uint16_t num_in_flight_nodes;
    if (inFlightNodes.empty()) {
        num_in_flight_nodes = 0;
        DPRINTFR(TraceCPUData, "\t\tChecking resources to issue seq. num %lli:"
                " #in-flight nodes = 0", new_node->seqNum);
    } else if (new_node->robNum > oldestInFlightRobNum) {
        // This is the intuitive case where new dep-free node is younger
        // instruction than the oldest instruction in-flight. Thus we make sure
        // in_flight_nodes does not overflow.
        num_in_flight_nodes = new_node->robNum - oldestInFlightRobNum;
        DPRINTFR(TraceCPUData, "\t\tChecking resources to issue seq. num %lli:"
                " #in-flight nodes = %d - %d =  %d", new_node->seqNum,
                new_node->robNum, oldestInFlightRobNum, num_in_flight_nodes);
    } else {
        // This is the case where an instruction older than the oldest in-
        // flight instruction becomes dep-free. Thus we must have already
        // accounted for the entry in ROB for this new dep-free node.
        // Immediately after this check returns true, oldestInFlightRobNum will
        // be updated in occupy(). We simply let this node issue now.
        num_in_flight_nodes = 0;
        DPRINTFR(TraceCPUData, "\t\tChecking resources to issue seq. num %lli:"
                " new oldestInFlightRobNum = %d, #in-flight nodes ignored",
                new_node->seqNum, new_node->robNum);
    }
    DPRINTFR(TraceCPUData, ", LQ = %d/%d, SQ  = %d/%d.\n",
            numInFlightLoads, sizeLoadBuffer,
            numInFlightStores, sizeStoreBuffer);
    // Check if resources are available to issue the specific node
    if (num_in_flight_nodes >= sizeROB) {
        return false;
    }
    if (new_node->isLoad() && numInFlightLoads >= sizeLoadBuffer) {
        return false;
    }
    if (new_node->isStore() && numInFlightStores >= sizeStoreBuffer) {
        return false;
    }
    return true;
}

bool
TraceCPU::ElasticDataGen::HardwareResource::awaitingResponse() const
{
    // Return true if there is at least one read or write request in flight
    return (numInFlightStores != 0 || numInFlightLoads != 0);
}

void
TraceCPU::ElasticDataGen::HardwareResource::printOccupancy()
{
    DPRINTFR(TraceCPUData, "oldestInFlightRobNum = %d, "
            "LQ = %d/%d, SQ  = %d/%d.\n",
            oldestInFlightRobNum,
            numInFlightLoads, sizeLoadBuffer,
            numInFlightStores, sizeStoreBuffer);
}

TraceCPU::FixedRetryGen::FixedRetryGenStatGroup::FixedRetryGenStatGroup(
        statistics::Group *parent, const std::string& _name) :
    statistics::Group(parent, _name.c_str()),
    ADD_STAT(numSendAttempted, statistics::units::Count::get(),
             "Number of first attempts to send a request"),
    ADD_STAT(numSendSucceeded, statistics::units::Count::get(),
             "Number of successful first attempts"),
    ADD_STAT(numSendFailed, statistics::units::Count::get(),
             "Number of failed first attempts"),
    ADD_STAT(numRetrySucceeded, statistics::units::Count::get(),
             "Number of successful retries"),
    ADD_STAT(instLastTick, statistics::units::Tick::get(),
             "Last tick simulated from the fixed inst trace")
{

}

Tick
TraceCPU::FixedRetryGen::init()
{
    DPRINTF(TraceCPUInst, "Initializing instruction fetch request generator"
            " IcacheGen: fixed issue with retry.\n");

    if (nextExecute()) {
        DPRINTF(TraceCPUInst, "\tFirst tick = %d.\n", currElement.tick);
        return currElement.tick;
    } else {
        panic("Read of first message in the trace failed.\n");
        return MaxTick;
    }
}

bool
TraceCPU::FixedRetryGen::tryNext()
{
    // If there is a retry packet, try to send it
    if (retryPkt) {
        DPRINTF(TraceCPUInst, "Trying to send retry packet.\n");

        if (!port.sendTimingReq(retryPkt)) {
            // Still blocked! This should never occur.
            DPRINTF(TraceCPUInst, "Retry packet sending failed.\n");
            return false;
        }
        ++fixedStats.numRetrySucceeded;
    } else {
        DPRINTF(TraceCPUInst, "Trying to send packet for currElement.\n");

        // try sending current element
        assert(currElement.isValid());

        ++fixedStats.numSendAttempted;

        if (!send(currElement.addr, currElement.blocksize,
                    currElement.cmd, currElement.flags, currElement.pc)) {
            DPRINTF(TraceCPUInst, "currElement sending failed.\n");
            ++fixedStats.numSendFailed;
            // return false to indicate not to schedule next event
            return false;
        } else {
            ++fixedStats.numSendSucceeded;
        }
    }
    // If packet was sent successfully, either retryPkt or currElement, return
    // true to indicate to schedule event at current Tick plus delta. If packet
    // was sent successfully and there is no next packet to send, return false.
    DPRINTF(TraceCPUInst, "Packet sent successfully, trying to read next "
        "element.\n");
    retryPkt = nullptr;
    // Read next element into currElement, currElement gets cleared so save the
    // tick to calculate delta
    Tick last_tick = currElement.tick;
    if (nextExecute()) {
        assert(currElement.tick >= last_tick);
        delta = currElement.tick - last_tick;
    }
    return !traceComplete;
}

void
TraceCPU::FixedRetryGen::exit()
{
    trace.reset();
}

bool
TraceCPU::FixedRetryGen::nextExecute()
{
    if (traceComplete)
        // We are at the end of the file, thus we have no more messages.
        // Return false.
        return false;


    //Reset the currElement to the default values
    currElement.clear();

    // Read the next line to get the next message. If that fails then end of
    // trace has been reached and traceComplete needs to be set in addition
    // to returning false. If successful then next message is in currElement.
    if (!trace.read(&currElement)) {
        traceComplete = true;
        fixedStats.instLastTick = curTick();
        return false;
    }

    DPRINTF(TraceCPUInst, "inst fetch: %c addr %d pc %#x size %d tick %d\n",
            currElement.cmd.isRead() ? 'r' : 'w',
            currElement.addr,
            currElement.pc,
            currElement.blocksize,
            currElement.tick);

    return true;
}

bool
TraceCPU::FixedRetryGen::send(Addr addr, unsigned size, const MemCmd& cmd,
        Request::FlagsType flags, Addr pc)
{

    // Create new request
    auto req = std::make_shared<Request>(addr, size, flags, requestorId);
    req->setPC(pc);

    // If this is not done it triggers assert in L1 cache for invalid contextId
    req->setContext(ContextID(0));

    // Embed it in a packet
    PacketPtr pkt = new Packet(req, cmd);

    uint8_t* pkt_data = new uint8_t[req->getSize()];
    pkt->dataDynamic(pkt_data);

    if (cmd.isWrite()) {
        memset(pkt_data, 0xA, req->getSize());
    }

    // Call RequestPort method to send a timing request for this packet
    bool success = port.sendTimingReq(pkt);
    if (!success) {
        // If it fails, save the packet to retry when a retry is signalled by
        // the cache
        retryPkt = pkt;
    }
    return success;
}

void
TraceCPU::icacheRetryRecvd()
{
    // Schedule an event to go through the control flow in the same tick as
    // retry is received
    DPRINTF(TraceCPUInst, "Icache retry received. Scheduling next IcacheGen"
            " event @%lli.\n", curTick());
    schedule(icacheNextEvent, curTick());
}

void
TraceCPU::dcacheRetryRecvd()
{
    // Schedule an event to go through the execute flow in the same tick as
    // retry is received
    DPRINTF(TraceCPUData, "Dcache retry received. Scheduling next DcacheGen"
            " event @%lli.\n", curTick());
    schedule(dcacheNextEvent, curTick());
}

void
TraceCPU::schedDcacheNextEvent(Tick when)
{
    if (!dcacheNextEvent.scheduled()) {
        DPRINTF(TraceCPUData, "Scheduling next DcacheGen event at %lli.\n",
                when);
        schedule(dcacheNextEvent, when);
        ++traceStats.numSchedDcacheEvent;
    } else if (when < dcacheNextEvent.when()) {
        DPRINTF(TraceCPUData, "Re-scheduling next dcache event from %lli"
                " to %lli.\n", dcacheNextEvent.when(), when);
        reschedule(dcacheNextEvent, when);
    }

}

Port &
TraceCPU::getPort(const std::string &if_name, PortID idx)
{
    // Get the right port based on name. This applies to all the
    // subclasses of the base CPU and relies on their implementation
    // of getDataPort and getInstPort.
    if (if_name == "dcache_port")
        return getDataPort();
    else if (if_name == "icache_port")
        return getInstPort();
    else
        return ClockedObject::getPort(if_name, idx);
}

bool
TraceCPU::IcachePort::recvTimingResp(PacketPtr pkt)
{
    // All responses on the instruction fetch side are ignored. Simply delete
    // the packet to free allocated memory
    delete pkt;

    return true;
}

void
TraceCPU::IcachePort::recvReqRetry()
{
    owner->icacheRetryRecvd();
}

void
TraceCPU::dcacheRecvTimingResp(PacketPtr pkt)
{
    DPRINTF(TraceCPUData, "Received timing response from Dcache.\n");
    dcacheGen.completeMemAccess(pkt);
}

bool
TraceCPU::DcachePort::recvTimingResp(PacketPtr pkt)
{
    // Handle the responses for data memory requests which is done inside the
    // elastic data generator
    owner->dcacheRecvTimingResp(pkt);
    // After processing the response delete the packet to free
    // memory
    delete pkt;

    return true;
}

void
TraceCPU::DcachePort::recvReqRetry()
{
    owner->dcacheRetryRecvd();
}

TraceCPU::ElasticDataGen::InputStream::InputStream(
        const std::string& filename, const double time_multiplier) :
    trace(filename),
    timeMultiplier(time_multiplier),
    microOpCount(0)
{
    // Create a protobuf message for the header and read it from the stream
    ProtoMessage::InstDepRecordHeader header_msg;
    if (!trace.read(header_msg)) {
        panic("Failed to read packet header from %s\n", filename);

        if (header_msg.tick_freq() != sim_clock::Frequency) {
            panic("Trace %s was recorded with a different tick frequency %d\n",
                  header_msg.tick_freq());
        }
    } else {
        // Assign window size equal to the field in the trace that was recorded
        // when the data dependency trace was captured in the o3cpu model
        windowSize = header_msg.window_size();
    }
}

void
TraceCPU::ElasticDataGen::InputStream::reset()
{
    trace.reset();
}

bool
TraceCPU::ElasticDataGen::InputStream::read(GraphNode* element)
{
    ProtoMessage::InstDepRecord pkt_msg;
    if (trace.read(pkt_msg)) {
        // Required fields
        element->seqNum = pkt_msg.seq_num();
        element->type = pkt_msg.type();
        // Scale the compute delay to effectively scale the Trace CPU frequency
        element->compDelay = pkt_msg.comp_delay() * timeMultiplier;

        // Repeated field robDepList
        element->robDep.clear();
        for (int i = 0; i < (pkt_msg.rob_dep()).size(); i++) {
            element->robDep.push_back(pkt_msg.rob_dep(i));
        }

        // Repeated field
        element->regDep.clear();
        for (int i = 0; i < (pkt_msg.reg_dep()).size(); i++) {
            // There is a possibility that an instruction has both, a register
            // and order dependency on an instruction. In such a case, the
            // register dependency is omitted
            bool duplicate = false;
            for (auto &dep: element->robDep) {
                duplicate |= (pkt_msg.reg_dep(i) == dep);
            }
            if (!duplicate)
                element->regDep.push_back(pkt_msg.reg_dep(i));
        }

        // Optional fields
        if (pkt_msg.has_p_addr())
            element->physAddr = pkt_msg.p_addr();
        else
            element->physAddr = 0;

        if (pkt_msg.has_v_addr())
            element->virtAddr = pkt_msg.v_addr();
        else
            element->virtAddr = 0;

        if (pkt_msg.has_size())
            element->size = pkt_msg.size();
        else
            element->size = 0;

        if (pkt_msg.has_flags())
            element->flags = pkt_msg.flags();
        else
            element->flags = 0;

        if (pkt_msg.has_pc())
            element->pc = pkt_msg.pc();
        else
            element->pc = 0;

        // ROB occupancy number
        ++microOpCount;
        if (pkt_msg.has_weight()) {
            microOpCount += pkt_msg.weight();
        }
        element->robNum = microOpCount;
        return true;
    }

    // We have reached the end of the file
    return false;
}

bool
TraceCPU::ElasticDataGen::GraphNode::removeRegDep(NodeSeqNum reg_dep)
{
    for (auto it = regDep.begin(); it != regDep.end(); it++) {
        if (*it == reg_dep) {
            // If register dependency is found, erase it.
            regDep.erase(it);
            DPRINTFR(TraceCPUData,
                    "\tFor %lli: Marking register dependency %lli done.\n",
                    seqNum, reg_dep);
            return true;
        }
    }

    // Return false if the dependency is not found
    return false;
}

bool
TraceCPU::ElasticDataGen::GraphNode::removeRobDep(NodeSeqNum rob_dep)
{
    for (auto it = robDep.begin(); it != robDep.end(); it++) {
        if (*it == rob_dep) {
            // If the rob dependency is found, erase it.
            robDep.erase(it);
            DPRINTFR(TraceCPUData,
                    "\tFor %lli: Marking ROB dependency %lli done.\n",
                    seqNum, rob_dep);
            return true;
        }
    }
    return false;
}

bool
TraceCPU::ElasticDataGen::GraphNode::removeDepOnInst(NodeSeqNum done_seq_num)
{
    // If it is an rob dependency then remove it
    if (!removeRobDep(done_seq_num)) {
        // If it is not an rob dependency then it must be a register dependency
        // If the register dependency is not found, it violates an assumption
        // and must be caught by assert.
        [[maybe_unused]] bool regdep_found = removeRegDep(done_seq_num);
        assert(regdep_found);
    }
    // Return true if the node is dependency free
    return robDep.empty() && regDep.empty();
}

void
TraceCPU::ElasticDataGen::GraphNode::writeElementAsTrace() const
{
#if TRACING_ON
    DPRINTFR(TraceCPUData, "%lli", seqNum);
    DPRINTFR(TraceCPUData, ",%s", typeToStr());
    if (isLoad() || isStore()) {
        DPRINTFR(TraceCPUData, ",%i", physAddr);
        DPRINTFR(TraceCPUData, ",%i", size);
        DPRINTFR(TraceCPUData, ",%i", flags);
    }
    DPRINTFR(TraceCPUData, ",%lli", compDelay);
    DPRINTFR(TraceCPUData, "robDep:");
    for (auto &dep: robDep) {
        DPRINTFR(TraceCPUData, ",%lli", dep);
    }
    DPRINTFR(TraceCPUData, "regDep:");
    for (auto &dep: regDep) {
        DPRINTFR(TraceCPUData, ",%lli", dep);
    }
    auto child_itr = dependents.begin();
    DPRINTFR(TraceCPUData, "dependents:");
    while (child_itr != dependents.end()) {
        DPRINTFR(TraceCPUData, ":%lli", (*child_itr)->seqNum);
        child_itr++;
    }

    DPRINTFR(TraceCPUData, "\n");
#endif // TRACING_ON
}

std::string
TraceCPU::ElasticDataGen::GraphNode::typeToStr() const
{
    return Record::RecordType_Name(type);
}

TraceCPU::FixedRetryGen::InputStream::InputStream(const std::string& filename)
    : trace(filename)
{
    // Create a protobuf message for the header and read it from the stream
    ProtoMessage::PacketHeader header_msg;
    if (!trace.read(header_msg)) {
        panic("Failed to read packet header from %s\n", filename);

        if (header_msg.tick_freq() != sim_clock::Frequency) {
            panic("Trace %s was recorded with a different tick frequency %d\n",
                  header_msg.tick_freq());
        }
    }
}

void
TraceCPU::FixedRetryGen::InputStream::reset()
{
    trace.reset();
}

bool
TraceCPU::FixedRetryGen::InputStream::read(TraceElement* element)
{
    ProtoMessage::Packet pkt_msg;
    if (trace.read(pkt_msg)) {
        element->cmd = pkt_msg.cmd();
        element->addr = pkt_msg.addr();
        element->blocksize = pkt_msg.size();
        element->tick = pkt_msg.tick();
        element->flags = pkt_msg.has_flags() ? pkt_msg.flags() : 0;
        element->pc = pkt_msg.has_pc() ? pkt_msg.pc() : 0;
        return true;
    }

    // We have reached the end of the file
    return false;
}

} // namespace gem5
