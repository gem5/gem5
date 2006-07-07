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

#include "mem/cache/base_cache.hh"
#include "cpu/smt.hh"
#include "cpu/base.hh"

using namespace std;

BaseCache::CachePort::CachePort(const std::string &_name, BaseCache *_cache,
                                bool _isCpuSide)
    : Port(_name), cache(_cache), isCpuSide(_isCpuSide)
{
    blocked = false;
    //Start ports at null if more than one is created we should panic
    //cpuSidePort = NULL;
    //memSidePort = NULL;
}

void
BaseCache::CachePort::recvStatusChange(Port::Status status)
{
    cache->recvStatusChange(status, isCpuSide);
}

void
BaseCache::CachePort::getDeviceAddressRanges(AddrRangeList &resp,
                                       AddrRangeList &snoop)
{
    cache->getAddressRanges(resp, snoop, isCpuSide);
}

int
BaseCache::CachePort::deviceBlockSize()
{
    return cache->getBlockSize();
}

bool
BaseCache::CachePort::recvTiming(Packet *pkt)
{
    return cache->doTimingAccess(pkt, this, isCpuSide);
}

Tick
BaseCache::CachePort::recvAtomic(Packet *pkt)
{
    return cache->doAtomicAccess(pkt, isCpuSide);
}

void
BaseCache::CachePort::recvFunctional(Packet *pkt)
{
    cache->doFunctionalAccess(pkt, isCpuSide);
}

void
BaseCache::CachePort::setBlocked()
{
    blocked = true;
}

void
BaseCache::CachePort::clearBlocked()
{
    blocked = false;
}

BaseCache::CacheEvent::CacheEvent(CachePort *_cachePort)
    : Event(&mainEventQueue, CPU_Tick_Pri), cachePort(_cachePort)
{
    this->setFlags(AutoDelete);
    pkt = NULL;
}

BaseCache::CacheEvent::CacheEvent(CachePort *_cachePort, Packet *_pkt)
    : Event(&mainEventQueue, CPU_Tick_Pri), cachePort(_cachePort), pkt(_pkt)
{
    this->setFlags(AutoDelete);
}

void
BaseCache::CacheEvent::process()
{
    if (!pkt)
    {
        if (!cachePort->isCpuSide)
            pkt = cachePort->cache->getPacket();
        else
            pkt = cachePort->cache->getCoherencePacket();
        bool success = cachePort->sendTiming(pkt);
        cachePort->cache->sendResult(pkt, success);
        return;
    }
    //Know the packet to send, no need to mark in service (must succed)
    bool success = cachePort->sendTiming(pkt);
    assert(success);
}

const char *
BaseCache::CacheEvent::description()
{
    return "timing event\n";
}

Port*
BaseCache::getPort(const std::string &if_name, int idx)
{
    if (if_name == "")
    {
        if(cpuSidePort == NULL)
            cpuSidePort = new CachePort(name() + "-cpu_side_port", this, true);
        return cpuSidePort;
    }
    else if (if_name == "functional")
    {
        if(cpuSidePort == NULL)
            cpuSidePort = new CachePort(name() + "-cpu_side_port", this, true);
        return cpuSidePort;
    }
    else if (if_name == "cpu_side")
    {
        if(cpuSidePort == NULL)
            cpuSidePort = new CachePort(name() + "-cpu_side_port", this, true);
        return cpuSidePort;
    }
    else if (if_name == "mem_side")
    {
        if (memSidePort != NULL)
            panic("Already have a mem side for this cache\n");
        memSidePort = new CachePort(name() + "-mem_side_port", this, false);
        return memSidePort;
    }
    else panic("Port name %s unrecognized\n", if_name);
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
    Request temp_req((Addr) NULL, 4, 0);
    Packet::Command temp_cmd = Packet::ReadReq;
    Packet temp_pkt(&temp_req, temp_cmd, 0);  //@todo FIx command strings so this isn't neccessary
    temp_pkt.allocate(); //Temp allocate, all need data

    using namespace Stats;

    // Hit statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

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
    demandHits = hits[Packet::ReadReq] + hits[Packet::WriteReq];

    overallHits
        .name(name() + ".overall_hits")
        .desc("number of overall hits")
        .flags(total)
        ;
    overallHits = demandHits + hits[Packet::SoftPFReq] + hits[Packet::HardPFReq]
        + hits[Packet::Writeback];

    // Miss statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

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
    demandMisses = misses[Packet::ReadReq] + misses[Packet::WriteReq];

    overallMisses
        .name(name() + ".overall_misses")
        .desc("number of overall misses")
        .flags(total)
        ;
    overallMisses = demandMisses + misses[Packet::SoftPFReq] +
        misses[Packet::HardPFReq] + misses[Packet::Writeback];

    // Miss latency statistics
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

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
    demandMissLatency = missLatency[Packet::ReadReq] + missLatency[Packet::WriteReq];

    overallMissLatency
        .name(name() + ".overall_miss_latency")
        .desc("number of overall miss cycles")
        .flags(total)
        ;
    overallMissLatency = demandMissLatency + missLatency[Packet::SoftPFReq] +
        missLatency[Packet::HardPFReq];

    // access formulas
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

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
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

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
    for (int access_idx = 0; access_idx < NUM_MEM_CMDS; ++access_idx) {
        Packet::Command cmd = (Packet::Command)access_idx;
        const string &cstr = temp_pkt.cmdIdxToString(cmd);

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
