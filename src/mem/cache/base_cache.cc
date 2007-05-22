/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definition of BaseCache functions.
 */

#include "cpu/base.hh"
#include "cpu/smt.hh"
#include "mem/cache/base_cache.hh"
#include "mem/cache/miss/mshr.hh"

using namespace std;

BaseCache::CachePort::CachePort(const std::string &_name, BaseCache *_cache)
    : Port(_name, _cache), cache(_cache), otherPort(NULL)
{
    blocked = false;
    waitingOnRetry = false;
}


BaseCache::BaseCache(const std::string &name, Params &params)
    : MemObject(name),
      blocked(0), blockedSnoop(0),
      blkSize(params.blkSize),
      missCount(params.maxMisses), drainEvent(NULL)
{
}


void
BaseCache::CachePort::recvStatusChange(Port::Status status)
{
    if (status == Port::RangeChange) {
        otherPort->sendStatusChange(Port::RangeChange);
    }
}

int
BaseCache::CachePort::deviceBlockSize()
{
    return cache->getBlockSize();
}

bool
BaseCache::CachePort::checkFunctional(PacketPtr pkt)
{
    //Check storage here first
    list<PacketPtr>::iterator i = drainList.begin();
    list<PacketPtr>::iterator iend = drainList.end();
    bool notDone = true;
    while (i != iend && notDone) {
        PacketPtr target = *i;
        // If the target contains data, and it overlaps the
        // probed request, need to update data
        if (target->intersect(pkt)) {
            DPRINTF(Cache, "Functional %s access to blk_addr %x intersects a drain\n",
                    pkt->cmdString(), pkt->getAddr() & ~(cache->getBlockSize() - 1));
            notDone = fixPacket(pkt, target);
        }
        i++;
    }
    //Also check the response not yet ready to be on the list
    std::list<std::pair<Tick,PacketPtr> >::iterator j = transmitList.begin();
    std::list<std::pair<Tick,PacketPtr> >::iterator jend = transmitList.end();

    while (j != jend && notDone) {
        PacketPtr target = j->second;
        // If the target contains data, and it overlaps the
        // probed request, need to update data
        if (target->intersect(pkt)) {
            DPRINTF(Cache, "Functional %s access to blk_addr %x intersects a response\n",
                    pkt->cmdString(), pkt->getAddr() & ~(cache->getBlockSize() - 1));
            notDone = fixDelayedResponsePacket(pkt, target);
        }
        j++;
    }
    return notDone;
}

void
BaseCache::CachePort::checkAndSendFunctional(PacketPtr pkt)
{
    bool notDone = checkFunctional(pkt);
    if (notDone)
        sendFunctional(pkt);
}


void
BaseCache::CachePort::respond(PacketPtr pkt, Tick time)
{
    assert(time >= curTick);
    if (pkt->needsResponse()) {
        if (transmitList.empty()) {
            assert(!responseEvent->scheduled());
            responseEvent->schedule(time);
            transmitList.push_back(std::pair<Tick,PacketPtr>(time,pkt));
            return;
        }

        // something is on the list and this belongs at the end
        if (time >= transmitList.back().first) {
            transmitList.push_back(std::pair<Tick,PacketPtr>(time,pkt));
            return;
        }
        // Something is on the list and this belongs somewhere else
        std::list<std::pair<Tick,PacketPtr> >::iterator i =
            transmitList.begin();
        std::list<std::pair<Tick,PacketPtr> >::iterator end =
            transmitList.end();
        bool done = false;

        while (i != end && !done) {
            if (time < i->first) {
                if (i == transmitList.begin()) {
                    //Inserting at begining, reschedule
                    responseEvent->reschedule(time);
                }
                transmitList.insert(i,std::pair<Tick,PacketPtr>(time,pkt));
                done = true;
            }
            i++;
        }
    }
    else {
        assert(0);
        // this code was on the cpuSidePort only... do we still need it?
        if (pkt->cmd != MemCmd::UpgradeReq)
        {
            delete pkt->req;
            delete pkt;
        }
    }
}

bool
BaseCache::CachePort::drainResponse()
{
    DPRINTF(CachePort,
            "%s attempting to send a retry for response (%i waiting)\n",
            name(), drainList.size());
    //We have some responses to drain first
    PacketPtr pkt = drainList.front();
    if (sendTiming(pkt)) {
        drainList.pop_front();
        DPRINTF(CachePort, "%s sucessful in sending a retry for"
                "response (%i still waiting)\n", name(), drainList.size());
        if (!drainList.empty() || isBusRequested()) {

            DPRINTF(CachePort, "%s has more responses/requests\n", name());
            return false;
        }
    } else {
        waitingOnRetry = true;
        DPRINTF(CachePort, "%s now waiting on a retry\n", name());
    }
    return true;
}


bool
BaseCache::CachePort::recvRetryCommon()
{
    assert(waitingOnRetry);
    waitingOnRetry = false;
    if (!drainList.empty()) {
        if (!drainResponse()) {
            // more responses to drain... re-request bus
            scheduleRequestEvent(curTick + 1);
        }
        // Check if we're done draining once this list is empty
        if (drainList.empty()) {
            cache->checkDrain();
        }
        return true;
    }
    return false;
}


void
BaseCache::CachePort::setBlocked()
{
    assert(!blocked);
    DPRINTF(Cache, "Cache Blocking\n");
    blocked = true;
    //Clear the retry flag
    mustSendRetry = false;
}

void
BaseCache::CachePort::clearBlocked()
{
    assert(blocked);
    DPRINTF(Cache, "Cache Unblocking\n");
    blocked = false;
    if (mustSendRetry)
    {
        DPRINTF(Cache, "Cache Sending Retry\n");
        mustSendRetry = false;
        sendRetry();
    }
}


void
BaseCache::init()
{
    if (!cpuSidePort || !memSidePort)
        panic("Cache not hooked up on both sides\n");
    cpuSidePort->sendStatusChange(Port::RangeChange);
}

void
BaseCache::regStats()
{
    using namespace Stats;

    // Hit statistics
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const string &cstr = cmd.toString();

        hits[access_idx]
            .init(maxThreadsPerCPU)
            .name(name() + "." + cstr + "_hits")
            .desc("number of " + cstr + " hits")
            .flags(total | nozero | nonan)
            ;
    }

    demandHits
        .name(name() + ".demand_hits")
        .desc("number of demand (read+write) hits")
        .flags(total)
        ;
    demandHits = hits[MemCmd::ReadReq] + hits[MemCmd::WriteReq];

    overallHits
        .name(name() + ".overall_hits")
        .desc("number of overall hits")
        .flags(total)
        ;
    overallHits = demandHits + hits[MemCmd::SoftPFReq] + hits[MemCmd::HardPFReq]
        + hits[MemCmd::Writeback];

    // Miss statistics
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const string &cstr = cmd.toString();

        misses[access_idx]
            .init(maxThreadsPerCPU)
            .name(name() + "." + cstr + "_misses")
            .desc("number of " + cstr + " misses")
            .flags(total | nozero | nonan)
            ;
    }

    demandMisses
        .name(name() + ".demand_misses")
        .desc("number of demand (read+write) misses")
        .flags(total)
        ;
    demandMisses = misses[MemCmd::ReadReq] + misses[MemCmd::WriteReq];

    overallMisses
        .name(name() + ".overall_misses")
        .desc("number of overall misses")
        .flags(total)
        ;
    overallMisses = demandMisses + misses[MemCmd::SoftPFReq] +
        misses[MemCmd::HardPFReq] + misses[MemCmd::Writeback];

    // Miss latency statistics
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const string &cstr = cmd.toString();

        missLatency[access_idx]
            .init(maxThreadsPerCPU)
            .name(name() + "." + cstr + "_miss_latency")
            .desc("number of " + cstr + " miss cycles")
            .flags(total | nozero | nonan)
            ;
    }

    demandMissLatency
        .name(name() + ".demand_miss_latency")
        .desc("number of demand (read+write) miss cycles")
        .flags(total)
        ;
    demandMissLatency = missLatency[MemCmd::ReadReq] + missLatency[MemCmd::WriteReq];

    overallMissLatency
        .name(name() + ".overall_miss_latency")
        .desc("number of overall miss cycles")
        .flags(total)
        ;
    overallMissLatency = demandMissLatency + missLatency[MemCmd::SoftPFReq] +
        missLatency[MemCmd::HardPFReq];

    // access formulas
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const string &cstr = cmd.toString();

        accesses[access_idx]
            .name(name() + "." + cstr + "_accesses")
            .desc("number of " + cstr + " accesses(hits+misses)")
            .flags(total | nozero | nonan)
            ;

        accesses[access_idx] = hits[access_idx] + misses[access_idx];
    }

    demandAccesses
        .name(name() + ".demand_accesses")
        .desc("number of demand (read+write) accesses")
        .flags(total)
        ;
    demandAccesses = demandHits + demandMisses;

    overallAccesses
        .name(name() + ".overall_accesses")
        .desc("number of overall (read+write) accesses")
        .flags(total)
        ;
    overallAccesses = overallHits + overallMisses;

    // miss rate formulas
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const string &cstr = cmd.toString();

        missRate[access_idx]
            .name(name() + "." + cstr + "_miss_rate")
            .desc("miss rate for " + cstr + " accesses")
            .flags(total | nozero | nonan)
            ;

        missRate[access_idx] = misses[access_idx] / accesses[access_idx];
    }

    demandMissRate
        .name(name() + ".demand_miss_rate")
        .desc("miss rate for demand accesses")
        .flags(total)
        ;
    demandMissRate = demandMisses / demandAccesses;

    overallMissRate
        .name(name() + ".overall_miss_rate")
        .desc("miss rate for overall accesses")
        .flags(total)
        ;
    overallMissRate = overallMisses / overallAccesses;

    // miss latency formulas
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const string &cstr = cmd.toString();

        avgMissLatency[access_idx]
            .name(name() + "." + cstr + "_avg_miss_latency")
            .desc("average " + cstr + " miss latency")
            .flags(total | nozero | nonan)
            ;

        avgMissLatency[access_idx] =
            missLatency[access_idx] / misses[access_idx];
    }

    demandAvgMissLatency
        .name(name() + ".demand_avg_miss_latency")
        .desc("average overall miss latency")
        .flags(total)
        ;
    demandAvgMissLatency = demandMissLatency / demandMisses;

    overallAvgMissLatency
        .name(name() + ".overall_avg_miss_latency")
        .desc("average overall miss latency")
        .flags(total)
        ;
    overallAvgMissLatency = overallMissLatency / overallMisses;

    blocked_cycles.init(NUM_BLOCKED_CAUSES);
    blocked_cycles
        .name(name() + ".blocked_cycles")
        .desc("number of cycles access was blocked")
        .subname(Blocked_NoMSHRs, "no_mshrs")
        .subname(Blocked_NoTargets, "no_targets")
        ;


    blocked_causes.init(NUM_BLOCKED_CAUSES);
    blocked_causes
        .name(name() + ".blocked")
        .desc("number of cycles access was blocked")
        .subname(Blocked_NoMSHRs, "no_mshrs")
        .subname(Blocked_NoTargets, "no_targets")
        ;

    avg_blocked
        .name(name() + ".avg_blocked_cycles")
        .desc("average number of cycles each access was blocked")
        .subname(Blocked_NoMSHRs, "no_mshrs")
        .subname(Blocked_NoTargets, "no_targets")
        ;

    avg_blocked = blocked_cycles / blocked_causes;

    fastWrites
        .name(name() + ".fast_writes")
        .desc("number of fast writes performed")
        ;

    cacheCopies
        .name(name() + ".cache_copies")
        .desc("number of cache copies performed")
        ;

}

unsigned int
BaseCache::drain(Event *de)
{
    // Set status
    if (!canDrain()) {
        drainEvent = de;

        changeState(SimObject::Draining);
        return 1;
    }

    changeState(SimObject::Drained);
    return 0;
}
