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
 *
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Sascha Bischoff
 */
#include "cpu/testers/traffic_gen/base.hh"

#include <sstream>

#include "base/intmath.hh"
#include "base/random.hh"
#include "config/have_protobuf.hh"
#include "cpu/testers/traffic_gen/base_gen.hh"
#include "cpu/testers/traffic_gen/dram_gen.hh"
#include "cpu/testers/traffic_gen/dram_rot_gen.hh"
#include "cpu/testers/traffic_gen/exit_gen.hh"
#include "cpu/testers/traffic_gen/idle_gen.hh"
#include "cpu/testers/traffic_gen/linear_gen.hh"
#include "cpu/testers/traffic_gen/random_gen.hh"
#include "cpu/testers/traffic_gen/stream_gen.hh"
#include "debug/Checkpoint.hh"
#include "debug/TrafficGen.hh"
#include "params/BaseTrafficGen.hh"
#include "sim/sim_exit.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

#if HAVE_PROTOBUF
#include "cpu/testers/traffic_gen/trace_gen.hh"
#endif


using namespace std;

BaseTrafficGen::BaseTrafficGen(const BaseTrafficGenParams* p)
    : ClockedObject(p),
      system(p->system),
      elasticReq(p->elastic_req),
      progressCheck(p->progress_check),
      noProgressEvent([this]{ noProgress(); }, name()),
      nextTransitionTick(0),
      nextPacketTick(0),
      maxOutstandingReqs(p->max_outstanding_reqs),
      port(name() + ".port", *this),
      retryPkt(NULL),
      retryPktTick(0), blockedWaitingResp(false),
      updateEvent([this]{ update(); }, name()),
      masterID(system->getMasterId(this)),
      streamGenerator(StreamGen::create(p))
{
}

BaseTrafficGen::~BaseTrafficGen()
{
}

Port &
BaseTrafficGen::getPort(const string &if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
BaseTrafficGen::init()
{
    ClockedObject::init();

    if (!port.isConnected())
        fatal("The port of %s is not connected!\n", name());
}

DrainState
BaseTrafficGen::drain()
{
    if (!updateEvent.scheduled()) {
        // no event has been scheduled yet (e.g. switched from atomic mode)
        return DrainState::Drained;
    }

    if (retryPkt == NULL) {
        // shut things down
        nextPacketTick = MaxTick;
        nextTransitionTick = MaxTick;
        deschedule(updateEvent);
        return DrainState::Drained;
    } else {
        return DrainState::Draining;
    }
}

void
BaseTrafficGen::serialize(CheckpointOut &cp) const
{
    warn("%s serialization does not keep all traffic generator"
         " internal state\n", name());

    DPRINTF(Checkpoint, "Serializing BaseTrafficGen\n");

    // save ticks of the graph event if it is scheduled
    Tick nextEvent = updateEvent.scheduled() ? updateEvent.when() : 0;

    DPRINTF(TrafficGen, "Saving nextEvent=%llu\n", nextEvent);

    SERIALIZE_SCALAR(nextEvent);

    SERIALIZE_SCALAR(nextTransitionTick);

    SERIALIZE_SCALAR(nextPacketTick);
}

void
BaseTrafficGen::unserialize(CheckpointIn &cp)
{
    warn("%s serialization does not restore all traffic generator"
         " internal state\n", name());

    // restore scheduled events
    Tick nextEvent;
    UNSERIALIZE_SCALAR(nextEvent);
    if (nextEvent != 0)
        schedule(updateEvent, nextEvent);

    UNSERIALIZE_SCALAR(nextTransitionTick);

    UNSERIALIZE_SCALAR(nextPacketTick);
}

void
BaseTrafficGen::update()
{
    // shift our progress-tracking event forward
    reschedule(noProgressEvent, curTick() + progressCheck, true);

    // if we have reached the time for the next state transition, then
    // perform the transition
    if (curTick() >= nextTransitionTick) {
        transition();
    } else {
        assert(curTick() >= nextPacketTick);
        // get the next packet and try to send it
        PacketPtr pkt = activeGenerator->getNextPacket();

        // If generating stream/substream IDs are enabled,
        // try to pick and assign them to the new packet
        if (streamGenerator) {
            auto sid = streamGenerator->pickStreamID();
            auto ssid = streamGenerator->pickSubStreamID();

            pkt->req->setStreamId(sid);

            if (streamGenerator->ssidValid()) {
                pkt->req->setSubStreamId(ssid);
            }
        }

        // suppress packets that are not destined for a memory, such as
        // device accesses that could be part of a trace
        if (pkt && system->isMemAddr(pkt->getAddr())) {
            numPackets++;
            // Only attempts to send if not blocked by pending responses
            blockedWaitingResp = allocateWaitingRespSlot(pkt);
            if (blockedWaitingResp || !port.sendTimingReq(pkt)) {
                retryPkt = pkt;
                retryPktTick = curTick();
            }
        } else if (pkt) {
            DPRINTF(TrafficGen, "Suppressed packet %s 0x%x\n",
                    pkt->cmdString(), pkt->getAddr());

            ++numSuppressed;
            if (!(static_cast<int>(numSuppressed.value()) % 10000))
                warn("%s suppressed %d packets with non-memory addresses\n",
                     name(), numSuppressed.value());

            delete pkt;
            pkt = nullptr;
        }
    }

    // if we are waiting for a retry or for a response, do not schedule any
    // further events, in the case of a transition or a successful send, go
    // ahead and determine when the next update should take place
    if (retryPkt == NULL) {
        nextPacketTick = activeGenerator->nextPacketTick(elasticReq, 0);
        scheduleUpdate();
    }
}

void
BaseTrafficGen::transition()
{
    if (activeGenerator)
        activeGenerator->exit();

    activeGenerator = nextGenerator();

    if (activeGenerator) {
        const Tick duration = activeGenerator->duration;
        if (duration != MaxTick && duration != 0) {
            // we could have been delayed and not transitioned on the
            // exact tick when we were supposed to (due to back
            // pressure when sending a packet)
            nextTransitionTick = curTick() + duration;
        } else {
            nextTransitionTick = MaxTick;
        }

        activeGenerator->enter();
        nextPacketTick = activeGenerator->nextPacketTick(elasticReq, 0);
    } else {
        nextPacketTick = MaxTick;
        nextTransitionTick = MaxTick;
        assert(!updateEvent.scheduled());
    }
}

void
BaseTrafficGen::scheduleUpdate()
{
    // Has the generator run out of work? In that case, force a
    // transition if a transition period hasn't been configured.
    while (activeGenerator &&
           nextPacketTick == MaxTick && nextTransitionTick == MaxTick) {
        transition();
    }

    if (!activeGenerator)
        return;

    // schedule next update event based on either the next execute
    // tick or the next transition, which ever comes first
    const Tick nextEventTick = std::min(nextPacketTick, nextTransitionTick);

    DPRINTF(TrafficGen, "Next event scheduled at %lld\n", nextEventTick);

    // The next transition tick may be in the past if there was a
    // retry, so ensure that we don't schedule anything in the past.
    schedule(updateEvent, std::max(curTick(), nextEventTick));
}

void
BaseTrafficGen::start()
{
    transition();
    scheduleUpdate();
}

void
BaseTrafficGen::recvReqRetry()
{
    DPRINTF(TrafficGen, "Received retry\n");
    numRetries++;
    retryReq();
}

void
BaseTrafficGen::retryReq()
{
    assert(retryPkt != NULL);
    assert(retryPktTick != 0);
    assert(!blockedWaitingResp);

    // attempt to send the packet, and if we are successful start up
    // the machinery again
    if (port.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
        // remember how much delay was incurred due to back-pressure
        // when sending the request, we also use this to derive
        // the tick for the next packet
        Tick delay = curTick() - retryPktTick;
        retryPktTick = 0;
        retryTicks += delay;

        if (drainState() != DrainState::Draining) {
            // packet is sent, so find out when the next one is due
            nextPacketTick = activeGenerator->nextPacketTick(elasticReq,
                                                             delay);
            scheduleUpdate();
        } else {
            // shut things down
            nextPacketTick = MaxTick;
            nextTransitionTick = MaxTick;
            signalDrainDone();
        }
    }
}

void
BaseTrafficGen::noProgress()
{
    fatal("BaseTrafficGen %s spent %llu ticks without making progress",
          name(), progressCheck);
}

void
BaseTrafficGen::regStats()
{
    ClockedObject::regStats();

    // Initialise all the stats
    using namespace Stats;

    numPackets
        .name(name() + ".numPackets")
        .desc("Number of packets generated");

    numSuppressed
        .name(name() + ".numSuppressed")
        .desc("Number of suppressed packets to non-memory space");

    numRetries
        .name(name() + ".numRetries")
        .desc("Number of retries");

    retryTicks
        .name(name() + ".retryTicks")
        .desc("Time spent waiting due to back-pressure (ticks)");

    bytesRead
        .name(name() + ".bytesRead")
        .desc("Number of bytes read");

    bytesWritten
        .name(name() + ".bytesWritten")
        .desc("Number of bytes written");

    totalReadLatency
        .name(name() + ".totalReadLatency")
        .desc("Total latency of read requests");

    totalWriteLatency
        .name(name() + ".totalWriteLatency")
        .desc("Total latency of write requests");

    totalReads
        .name(name() + ".totalReads")
        .desc("Total num of reads");

    totalWrites
        .name(name() + ".totalWrites")
        .desc("Total num of writes");

    avgReadLatency
        .name(name() + ".avgReadLatency")
        .desc("Avg latency of read requests");
    avgReadLatency = totalReadLatency / totalReads;

    avgWriteLatency
        .name(name() + ".avgWriteLatency")
        .desc("Avg latency of write requests");
    avgWriteLatency = totalWriteLatency / totalWrites;

    readBW
        .name(name() + ".readBW")
        .desc("Read bandwidth in bytes/s");
    readBW = bytesRead / simSeconds;

    writeBW
        .name(name() + ".writeBW")
        .desc("Write bandwidth in bytes/s");
    writeBW = bytesWritten / simSeconds;

}

std::shared_ptr<BaseGen>
BaseTrafficGen::createIdle(Tick duration)
{
    return std::shared_ptr<BaseGen>(new IdleGen(*this, masterID, duration));
}

std::shared_ptr<BaseGen>
BaseTrafficGen::createExit(Tick duration)
{
    return std::shared_ptr<BaseGen>(new ExitGen(*this, masterID, duration));
}

std::shared_ptr<BaseGen>
BaseTrafficGen::createLinear(Tick duration,
                             Addr start_addr, Addr end_addr, Addr blocksize,
                             Tick min_period, Tick max_period,
                             uint8_t read_percent, Addr data_limit)
{
    return std::shared_ptr<BaseGen>(new LinearGen(*this, masterID,
                                                  duration, start_addr,
                                                  end_addr, blocksize,
                                                  system->cacheLineSize(),
                                                  min_period, max_period,
                                                  read_percent, data_limit));
}

std::shared_ptr<BaseGen>
BaseTrafficGen::createRandom(Tick duration,
                             Addr start_addr, Addr end_addr, Addr blocksize,
                             Tick min_period, Tick max_period,
                             uint8_t read_percent, Addr data_limit)
{
    return std::shared_ptr<BaseGen>(new RandomGen(*this, masterID,
                                                  duration, start_addr,
                                                  end_addr, blocksize,
                                                  system->cacheLineSize(),
                                                  min_period, max_period,
                                                  read_percent, data_limit));
}

std::shared_ptr<BaseGen>
BaseTrafficGen::createDram(Tick duration,
                           Addr start_addr, Addr end_addr, Addr blocksize,
                           Tick min_period, Tick max_period,
                           uint8_t read_percent, Addr data_limit,
                           unsigned int num_seq_pkts, unsigned int page_size,
                           unsigned int nbr_of_banks_DRAM,
                           unsigned int nbr_of_banks_util,
                           unsigned int addr_mapping,
                           unsigned int nbr_of_ranks)
{
    return std::shared_ptr<BaseGen>(new DramGen(*this, masterID,
                                                duration, start_addr,
                                                end_addr, blocksize,
                                                system->cacheLineSize(),
                                                min_period, max_period,
                                                read_percent, data_limit,
                                                num_seq_pkts, page_size,
                                                nbr_of_banks_DRAM,
                                                nbr_of_banks_util,
                                                addr_mapping,
                                                nbr_of_ranks));
}

std::shared_ptr<BaseGen>
BaseTrafficGen::createDramRot(Tick duration,
                              Addr start_addr, Addr end_addr, Addr blocksize,
                              Tick min_period, Tick max_period,
                              uint8_t read_percent, Addr data_limit,
                              unsigned int num_seq_pkts,
                              unsigned int page_size,
                              unsigned int nbr_of_banks_DRAM,
                              unsigned int nbr_of_banks_util,
                              unsigned int addr_mapping,
                              unsigned int nbr_of_ranks,
                              unsigned int max_seq_count_per_rank)
{
    return std::shared_ptr<BaseGen>(new DramRotGen(*this, masterID,
                                                   duration, start_addr,
                                                   end_addr, blocksize,
                                                   system->cacheLineSize(),
                                                   min_period, max_period,
                                                   read_percent, data_limit,
                                                   num_seq_pkts, page_size,
                                                   nbr_of_banks_DRAM,
                                                   nbr_of_banks_util,
                                                   addr_mapping,
                                                   nbr_of_ranks,
                                                   max_seq_count_per_rank));
}

std::shared_ptr<BaseGen>
BaseTrafficGen::createTrace(Tick duration,
                            const std::string& trace_file, Addr addr_offset)
{
#if HAVE_PROTOBUF
    return std::shared_ptr<BaseGen>(
        new TraceGen(*this, masterID, duration, trace_file, addr_offset));
#else
    panic("Can't instantiate trace generation without Protobuf support!\n");
#endif
}

bool
BaseTrafficGen::recvTimingResp(PacketPtr pkt)
{
    auto iter = waitingResp.find(pkt->req);

    panic_if(iter == waitingResp.end(), "%s: "
            "Received unexpected response [%s reqPtr=%x]\n",
               pkt->print(), pkt->req);

    assert(iter->second <= curTick());

    if (pkt->isWrite()) {
        ++totalWrites;
        bytesWritten += pkt->req->getSize();
        totalWriteLatency += curTick() - iter->second;
    } else {
        ++totalReads;
        bytesRead += pkt->req->getSize();
        totalReadLatency += curTick() - iter->second;
    }

    waitingResp.erase(iter);

    delete pkt;

    // Sends up the request if we were blocked
    if (blockedWaitingResp) {
        blockedWaitingResp = false;
        retryReq();
    }

    return true;
}
