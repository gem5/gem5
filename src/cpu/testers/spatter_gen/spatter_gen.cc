/*
* Copyright (c) 2024 The Regents of The University of California
* All rights reserved.
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

#include "cpu/testers/spatter_gen/spatter_gen.hh"

#include "base/cprintf.hh"
#include "debug/SpatterGen.hh"
#include "debug/SpatterKernel.hh"
#include "enums/SpatterKernelType.hh"
#include "enums/SpatterProcessingMode.hh"
#include "mem/packet.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

using enums::SpatterKernelTypeStrings;
using enums::SpatterProcessingMode;

SpatterGen::SpatterGen(const Params& params):
    ClockedObject(params),
    state(SpatterGenState::RUNNING),
    requestorId(params.system->getRequestorId(this)),
    numPendingMemRequests(0),
    stats(this),
    mode(params.processing_mode),
    port(this, name() + ".port"),
    intRegFileSize(params.int_regfile_size), intRegUsed(0),
    fpRegFileSize(params.fp_regfile_size), fpRegUsed(0),
    requestGenLatency(params.request_gen_latency),
    requestGenRate(params.request_gen_rate),
    firstGeneratorAvailableTime(0),
    nextGenEvent([this](){ processNextGenEvent(); }, name() + ".GenEvent"),
    requestBufferEntries(params.request_buffer_entries),
    requestBuffer(clockPeriod()),
    sendRate(params.send_rate),
    firstPortAvailableTime(0),
    nextSendEvent([this](){ processNextSendEvent(); }, name() + ".SendEvent"),
    receiveBuffer(clockPeriod())
{
    fatal_if(fpRegFileSize < requestBufferEntries,
            "fp_regfile_size should be >= request_buffer_entries."
            "if request_buffer_entries is bigger than fp_regfile_size,"
            "it may result in inaccuracies in your simulation."
            "Ideally: fp_regfile_size >> request_buffer_entries."
    );
    generatorBusyUntil.resize(requestGenRate, 0);
    portBusyUntil.resize(sendRate, 0);
}

Port&
SpatterGen::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
SpatterGen::startup()
{
    scheduleNextGenEvent(curTick());
}

void
SpatterGen::SpatterGenPort::sendPacket(PacketPtr pkt)
{
    panic_if(blocked(), "Should never try to send if port is blocked.");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
        DPRINTF(
            SpatterGen,
            "%s: Port blocked when sending %s.\n",
            __func__, pkt->print()
        );
    }
}

void
SpatterGen::SpatterGenPort::recvReqRetry()
{
    DPRINTF(SpatterGen, "%s: Port received a ReqRetry.\n", __func__);
    panic_if(
            blockedPacket == nullptr,
            "Received reqRetry with no blocked packet."
            );
    if (!sendTimingReq(blockedPacket)) {
        DPRINTF(
            SpatterGen,
            "%s: Port blocked when sending %s.\n",
            __func__, blockedPacket->print()
        );
    } else {
        blockedPacket = nullptr;
        owner->recvReqRetry();
    }
}

void
SpatterGen::recvReqRetry()
{
    if (nextSendEvent.pending()) {
        nextSendEvent.wake();
        scheduleNextSendEvent(nextCycle());
    }
}

bool
SpatterGen::SpatterGenPort::recvTimingResp(PacketPtr pkt) {
    return owner->recvTimingResp(pkt);
}

bool
SpatterGen::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SpatterGen, "%s: Received pkt: %s.\n", __func__, pkt->print());
    assert(pkt->isResponse());

    // record trip time.
    SpatterAccess* spatter_access = pkt->findNextSenderState<SpatterAccess>();
    Tick trip_time = (curTick() - requestDepartureTime[pkt->req]);
    requestDepartureTime.erase(pkt->req);
    spatter_access->recordTripTime(trip_time);

    int trips_left = spatter_access->tripsLeft();
    assert(trips_left >= 0);
    if (trips_left > 0) {
        stats.numIndexReads++;
        stats.indexBytesRead += pkt->getSize();
        stats.totalIndexReadLatency += trip_time;

        stats.indexAccessLatency.sample(trip_time);
        receiveBuffer.push(spatter_access, curTick());
    } else {
        stats.valueAccessLatency.sample(trip_time);
        stats.totalIndirectAccessLatency.sample(
                                            spatter_access->tripTimeSoFar()
                                            );
        if (spatter_access->type() == SpatterKernelType::gather) {
            stats.numValueReads++;
            stats.valueBytesRead += pkt->getSize();
            stats.totalValueReadLatency += trip_time;
        } else if (spatter_access->type() == SpatterKernelType::scatter) {
            stats.numValueWrites++;
            stats.valueBytesWritten += pkt->getSize();
            stats.totalValueWriteLatency += trip_time;
        } else {
            panic("Unknown kernel type.");
        }
        // CAUTION: We're going to decrement fpRegUsed here,
        // it could cause inaccuracies if processNextGenEvent
        // is called after recvTimingResp on the same tick.
        // i.e. we might end up releasing a register on the same
        // cycle that we are allocating it.
        // it's probably not going to ever be an issue since
        // fpRegFileSize is probably >> requestBufferEntries
        // i.e. the chances of running out of fp registers is low because
        // we do not simulate parts of the pipeline that back things up into
        // fp registers, e.g. functional units of ALU.
        fpRegUsed--;
        delete spatter_access;
    }

    // delete the pkt since we don't need it anymore.
    delete pkt;

    if (!nextGenEvent.pending()) {
        scheduleNextGenEvent(nextCycle());
    }

    numPendingMemRequests--;
    checkForSimExit();
    return true;
}

void
SpatterGen::addKernel(
    uint32_t id, uint32_t delta, uint32_t count,
    SpatterKernelType type,
    size_t index_size, Addr base_index_addr,
    size_t value_size, Addr base_value_addr,
    const std::vector<uint32_t>& indices
)
{
    DPRINTF(
        SpatterGen,
        "%s: Adding kernel with id: %d, delta: %d, count: %d, type: %s.\n",
        __func__, id, delta, count, SpatterKernelTypeStrings[type]
    );
    SpatterKernel new_kernel(
                            requestorId,
                            id, delta, count, type,
                            index_size, base_index_addr,
                            value_size, base_value_addr
                            );
    new_kernel.setIndices(indices);
    kernels.push(new_kernel);
}

void
SpatterGen::proceedPastSyncPoint()
{
    assert(mode == SpatterProcessingMode::synchronous);
    assert(state == SpatterGenState::WAITING);
    state = SpatterGenState::RUNNING;
    scheduleNextGenEvent(nextCycle());
}

void
SpatterGen::checkForSimExit()
{
    bool no_pending = numPendingMemRequests == 0;
    bool no_queued = requestBuffer.empty();
    int avail_int_regs = intRegFileSize - intRegUsed;
    int avail_fp_regs = fpRegFileSize - fpRegUsed;
    bool can_do_init = initAccessOk(avail_int_regs, avail_fp_regs, curTick());
    bool can_do_mid = interAccessOk(avail_int_regs, avail_fp_regs, curTick());
    bool can_do_ult = ultAccessOk(avail_int_regs, avail_fp_regs, curTick());
    if (!can_do_init && !can_do_mid && !can_do_ult && no_pending && no_queued)
    {
        assert((
                (mode == SpatterProcessingMode::synchronous) &&
                (state == SpatterGenState::DRAINING)
                ) ||
                mode == SpatterProcessingMode::asynchronous
            );
        state = SpatterGenState::WAITING;
        exitSimLoop(
            csprintf("%s received all expected responses.", name()),
            0,
            nextCycle()
        );
    }
}

bool
SpatterGen::initAccessOk(int int_regs, int fp_regs, Tick when) const
{
    bool have_int_reg = int_regs > 0;
    // for mode == SpatterProcessingMode::asynchronous state will always be
    // SpatterGenState::RUNNING. we don't have to do checks for mode.
    // for mode == SpatterProcessingMode::synchronous, if state is
    // SpatterGenState::DRAINING or SpatterGenState::WAITING
    // we can't initiate any new indirect accesses.
    bool have_kernel = !kernels.empty() && (state == SpatterGenState::RUNNING);
    return have_kernel && have_int_reg;
}

bool
SpatterGen::interAccessOk(int int_regs, int fp_regs, Tick when) const
{
    bool have_int_reg = int_regs > 0;
    bool have_index = receiveBuffer.hasReady(when);
    bool mid_idx = have_index && (receiveBuffer.front()->tripsLeft() > 1);
    return mid_idx && have_int_reg;
}

bool
SpatterGen::ultAccessOk(int int_regs, int fp_regs, Tick when) const
{
    bool have_fp_reg = fp_regs > 0;
    bool have_index = receiveBuffer.hasReady(when);
    bool val_idx = have_index && (receiveBuffer.front()->tripsLeft() == 1);
    return val_idx && have_fp_reg;
}

void
SpatterGen::scheduleNextGenEvent(Tick when)
{
    int avail_int_regs = intRegFileSize - intRegUsed;
    int avail_fp_regs = fpRegFileSize - fpRegUsed;
    bool have_work = initAccessOk(avail_int_regs, avail_fp_regs, curTick()) ||
                    interAccessOk(avail_int_regs, avail_fp_regs, curTick()) ||
                    ultAccessOk(avail_int_regs, avail_fp_regs, curTick());
    Tick schedule_tick = std::max(when, firstGeneratorAvailableTime);
    if (have_work && (!nextGenEvent.scheduled())) {
        schedule(nextGenEvent, schedule_tick);
        firstGeneratorAvailableTime = MaxTick;
    }
}

void
SpatterGen::processNextGenEvent()
{
    assert(!nextGenEvent.pending());
    int req_buf_before = requestBuffer.size();
    // track changes to intRegUsed in this variable and apply it
    // at the end of the for loop. This way if we free a register
    // in the for loop, other iterations of the for loop won't
    // observe this change. This matches what happens in real h/w.
    int int_used_now = 0;
    // track this independently to prevent different iterations inside
    // for loop observing change to h/w resources, i.e we can't rely
    // intRegFileSize - intRegUsed to see if we have registers to allocate
    // since they don't change until after the for loop
    int int_regs_now = intRegFileSize - intRegUsed;
    // same explanation as int_used_now
    int fp_used_now = 0;
    // same explanation as int_regs_now
    int fp_regs_now = fpRegFileSize - fpRegUsed;
    for (int i = 0; i < requestGenRate; i++) {
        if (generatorBusyUntil[i] > curTick()) {
            DPRINTF(
                SpatterGen,
                "%s: AGU[%d] is busy this cycle.\n", __func__, i
            );
            continue;
        }
        if (!(requestBuffer.size() < requestBufferEntries)) {
            // if no space left in the requestBuffer sleep
            // whoever pops from requestBuffer wakes us up.
            nextGenEvent.sleep();
            break;
        }
        // Now we know that AGU[i] is available and there is room
        // in the requestBuffer to put the packet.
        if (ultAccessOk(int_regs_now, fp_regs_now, curTick())) {
            // occupy one fp register
            fp_regs_now--;
            fp_used_now++;
            // make AGU busy for the next requestGenLatency cycles.
            generatorBusyUntil[i] = clockEdge(Cycles(requestGenLatency));

            // create a new packet to access
            SpatterAccess* spatter_access = receiveBuffer.front();
            PacketPtr pkt = spatter_access->nextPacket();
            pkt->pushSenderState(spatter_access);

            // push to requestBuffer
            requestBuffer.push(pkt, curTick());
            DPRINTF(
                SpatterGen,
                "%s: Pushed pkt: %s to requestBuffer.\n",
                __func__, pkt->print()
            );

            // now deallocate resources for reading the index
            int_used_now--;
            receiveBuffer.pop();
        } else if (interAccessOk(int_regs_now, fp_regs_now, curTick())) {
            // occupy one int register
            int_regs_now--;
            int_used_now++;
            // make AGU busy for the next requestGenLatency cycles.
            generatorBusyUntil[i] = clockEdge(Cycles(requestGenLatency));

            // create a new packet to access
            SpatterAccess* spatter_access = receiveBuffer.front();
            PacketPtr pkt = spatter_access->nextPacket();
            pkt->pushSenderState(spatter_access);

            // push to requestBuffer
            requestBuffer.push(pkt, curTick());
            DPRINTF(
                SpatterGen,
                "%s: Pushed pkt: %s to requestBuffer.\n",
                __func__, pkt->print()
            );

            // now deallocate resources for reading the index
            int_used_now--;
            receiveBuffer.pop();
        } else if (initAccessOk(int_regs_now, fp_regs_now, curTick())) {
            // occupy one int register
            int_regs_now--;
            int_used_now++;
            generatorBusyUntil[i] = clockEdge(Cycles(requestGenLatency));

            SpatterKernel& front = kernels.front();
            SpatterAccess* spatter_access = front.nextSpatterAccess();
            PacketPtr pkt = spatter_access->nextPacket();
            pkt->pushSenderState(spatter_access);

            requestBuffer.push(pkt, curTick());
            DPRINTF(
                SpatterGen,
                "%s: Pushed pkt: %s to requestBuffer.\n",
                __func__, pkt->print()
            );

            if (front.done()) {
                DPRINTF(
                    SpatterKernel,
                    "%s: Done with kernel %d type: %s.\n",
                    __func__, front.id(),
                    SpatterKernelTypeStrings[front.type()]
                );
                kernels.pop();
                // If we're processing synchronously we now have to stop
                // making intial accesses and wait everyone to receive
                // all expected responses.
                if (mode == SpatterProcessingMode::synchronous) {
                    state = SpatterGenState::DRAINING;
                }
            }
        } else {
            //
            DPRINTF(
                SpatterGen,
                "%s: Nothing more could be done this cycle.\n", __func__
                );
            DPRINTF(SpatterGen, "%s: Here is h/w status report: "
                "{KERNELS_REMAIN: %d, INDEXES_REMAIN: %d, INT_REG_USED: %d, "
                "FP_REG_USED: %d, REQ_BUFF_SIZE: %d}.\n",
                __func__, kernels.size(), receiveBuffer.size(),
                intRegUsed, fpRegUsed, requestBuffer.size());
            break;
        }
    }

    // update firstGeneratorAvailableTime after making all changes.
    for (int i = 0; i < requestGenRate; i++) {
        generatorBusyUntil[i] = std::max(generatorBusyUntil[i], nextCycle());
        firstGeneratorAvailableTime = std::min(
                                            firstGeneratorAvailableTime,
                                            generatorBusyUntil[i]
                                            );
    }

    // now that we have simulated all the work of this cycle, we can
    // apply the deltas to the h/w resources.
    intRegUsed += int_used_now;
    fpRegUsed += fp_used_now;

    bool did_work = (requestBuffer.size() - req_buf_before) > 0;
    if (did_work && (!nextSendEvent.pending())) {
        scheduleNextSendEvent(nextCycle());
    }

    if (!nextGenEvent.pending()) {
        scheduleNextGenEvent(firstGeneratorAvailableTime);
    }
}

void
SpatterGen::scheduleNextSendEvent(Tick when)
{
    bool have_work = !requestBuffer.empty();
    Tick schedule_tick = std::max(when, firstPortAvailableTime);
    if (have_work && (!nextSendEvent.scheduled())) {
        schedule(nextSendEvent, schedule_tick);
        firstPortAvailableTime = MaxTick;
    }
}

void
SpatterGen::processNextSendEvent()
{
    int req_buf_before = requestBuffer.size();
    for (int i = 0; i < sendRate; i++) {
        if (portBusyUntil[i] > curTick()) {
            DPRINTF(
                SpatterGen,
                "%s: Port[%d] is busy this cycle.\n", __func__, i
            );
            continue;
        }
        if (requestBuffer.empty()) {
            DPRINTF(
                SpatterGen,
                "%s: No packets to send this cycle.\n", __func__
            );
            break;
        }
        if (!requestBuffer.hasReady(curTick())) {
            DPRINTF(
                SpatterGen,
                "%s: Packet at front of requestBuffer not ready this cycle.\n",
                __func__
            );
            break;
        }
        PacketPtr pkt = requestBuffer.front();
        DPRINTF(
            SpatterGen,
            "%s: Sending pkt: %s to port[%d].\n",
            __func__, pkt->print(), i
        );
        // NOTE: We assume the port will be busy for 1 cycle.
        portBusyUntil[i] = clockEdge(Cycles(1));
        port.sendPacket(pkt);
        requestBuffer.pop();
        // increase numPendingMemRequests
        numPendingMemRequests++;
        // record packet departure time
        requestDepartureTime[pkt->req] = curTick();
        // Now if we put the port in blocked state no point in continuing
        // the loop. also no point in scheduling nextSendEvent.
        if (port.blocked()) {
            nextSendEvent.sleep();
            break;
        }
    }
    // update firstPortAvailableTime after making all changes.
    for (int i = 0; i < sendRate; i++) {
        // if the port was not used this cycle, it's busy until nextCycle().
        portBusyUntil[i] = std::max(portBusyUntil[i], nextCycle());
        firstPortAvailableTime = std::min(
                                        firstPortAvailableTime,
                                        portBusyUntil[i]
                                        );
    }

    bool did_work = (req_buf_before - requestBuffer.size()) > 0;
    if (did_work && nextGenEvent.pending()) {
        // since this event might open up space for output of nextGenEvent,
        // it should wake it up if nextGenEvent is asleep.
        nextGenEvent.wake();
        scheduleNextGenEvent(nextCycle());
    }

    if (!nextSendEvent.pending()) {
        scheduleNextSendEvent(nextCycle());
    }
}

SpatterGen::SpatterGenStats::SpatterGenStats(SpatterGen* spatter_gen):
    statistics::Group(spatter_gen), spatterGen(spatter_gen),
    ADD_STAT(numIndexReads, statistics::units::Count::get(),
        "Number of reads from the indexer array."),
    ADD_STAT(indexBytesRead, statistics::units::Byte::get(),
        "Number of bytes read from the indexer array."),
    ADD_STAT(totalIndexReadLatency, statistics::units::Tick::get(),
        "Total latency for reading from the indexer array."),
    ADD_STAT(numValueReads, statistics::units::Count::get(),
        "Number of reads from the values array."),
    ADD_STAT(numValueWrites, statistics::units::Count::get(),
        "Number of writes to the values array."),
    ADD_STAT(valueBytesRead, statistics::units::Byte::get(),
        "Number of bytes read from the values array."),
    ADD_STAT(valueBytesWritten, statistics::units::Byte::get(),
        "Number of bytes written to the values array."),
    ADD_STAT(totalValueReadLatency, statistics::units::Tick::get(),
        "Total latency for reading from the values array."),
    ADD_STAT(totalValueWriteLatency, statistics::units::Tick::get(),
        "Total latency for writing to the values array."),
    ADD_STAT(indexAccessLatency, statistics::units::Tick::get(),
        "Distribution of latency for accessing the indexer array."),
    ADD_STAT(valueAccessLatency, statistics::units::Tick::get(),
        "Distribution of latency for accessing the values array."),
    ADD_STAT(totalIndirectAccessLatency, statistics::units::Tick::get(),
        "Distribution of total latency for indirect accesses.")
{}

void
SpatterGen::SpatterGenStats::regStats()
{
    using namespace statistics;
    indexAccessLatency.init(8);
    valueAccessLatency.init(16);
    totalIndirectAccessLatency.init(16);
}

} // namespace gem5
