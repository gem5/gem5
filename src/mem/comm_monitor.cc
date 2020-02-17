/*
 * Copyright (c) 2012-2013, 2015, 2018-2019 ARM Limited
 * Copyright (c) 2016 Google Inc.
 * Copyright (c) 2017, Centre National de la Recherche Scientifique
 * All rights reserved.
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

#include "mem/comm_monitor.hh"

#include "base/trace.hh"
#include "debug/CommMonitor.hh"
#include "sim/stats.hh"

CommMonitor::CommMonitor(Params* params)
    : SimObject(params),
      masterPort(name() + "-master", *this),
      slavePort(name() + "-slave", *this),
      samplePeriodicEvent([this]{ samplePeriodic(); }, name()),
      samplePeriodTicks(params->sample_period),
      samplePeriod(params->sample_period / SimClock::Float::s),
      stats(this, params)
{
    DPRINTF(CommMonitor,
            "Created monitor %s with sample period %d ticks (%f ms)\n",
            name(), samplePeriodTicks, samplePeriod * 1E3);
}

CommMonitor*
CommMonitorParams::create()
{
    return new CommMonitor(this);
}

void
CommMonitor::init()
{
    // make sure both sides of the monitor are connected
    if (!slavePort.isConnected() || !masterPort.isConnected())
        fatal("Communication monitor is not connected on both sides.\n");
}

void
CommMonitor::regProbePoints()
{
    ppPktReq.reset(new ProbePoints::Packet(getProbeManager(), "PktRequest"));
    ppPktResp.reset(new ProbePoints::Packet(getProbeManager(), "PktResponse"));
}

Port &
CommMonitor::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "master") {
        return masterPort;
    } else if (if_name == "slave") {
        return slavePort;
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

void
CommMonitor::recvFunctional(PacketPtr pkt)
{
    masterPort.sendFunctional(pkt);
}

void
CommMonitor::recvFunctionalSnoop(PacketPtr pkt)
{
    slavePort.sendFunctionalSnoop(pkt);
}

CommMonitor::MonitorStats::MonitorStats(Stats::Group *parent,
                                        const CommMonitorParams *params)
    : Stats::Group(parent),

      disableBurstLengthHists(params->disable_burst_length_hists),
      ADD_STAT(readBurstLengthHist,
               "Histogram of burst lengths of transmitted packets"),
      ADD_STAT(writeBurstLengthHist,
               "Histogram of burst lengths of transmitted packets"),

      disableBandwidthHists(params->disable_bandwidth_hists),
      readBytes(0),
      ADD_STAT(readBandwidthHist,
               "Histogram of read bandwidth per sample period (bytes/s)"),
      ADD_STAT(totalReadBytes, "Number of bytes read"),
      ADD_STAT(averageReadBandwidth, "Average read bandwidth (bytes/s)",
               totalReadBytes / simSeconds),

      writtenBytes(0),
      ADD_STAT(writeBandwidthHist, "Histogram of write bandwidth (bytes/s)"),
      ADD_STAT(totalWrittenBytes, "Number of bytes written"),
      ADD_STAT(averageWriteBandwidth, "Average write bandwidth (bytes/s)",
               totalWrittenBytes / simSeconds),

      disableLatencyHists(params->disable_latency_hists),
      ADD_STAT(readLatencyHist, "Read request-response latency"),
      ADD_STAT(writeLatencyHist, "Write request-response latency"),

      disableITTDists(params->disable_itt_dists),
      ADD_STAT(ittReadRead, "Read-to-read inter transaction time"),
      ADD_STAT(ittWriteWrite , "Write-to-write inter transaction time"),
      ADD_STAT(ittReqReq, "Request-to-request inter transaction time"),
      timeOfLastRead(0), timeOfLastWrite(0), timeOfLastReq(0),

      disableOutstandingHists(params->disable_outstanding_hists),
      ADD_STAT(outstandingReadsHist, "Outstanding read transactions"),
      outstandingReadReqs(0),
      ADD_STAT(outstandingWritesHist, "Outstanding write transactions"),
      outstandingWriteReqs(0),

      disableTransactionHists(params->disable_transaction_hists),
      ADD_STAT(readTransHist,
               "Histogram of read transactions per sample period"),
      readTrans(0),
      ADD_STAT(writeTransHist,
               "Histogram of write transactions per sample period"),
      writeTrans(0),

      disableAddrDists(params->disable_addr_dists),
      readAddrMask(params->read_addr_mask),
      writeAddrMask(params->write_addr_mask),
      ADD_STAT(readAddrDist, "Read address distribution"),
      ADD_STAT(writeAddrDist, "Write address distribution")
{
    using namespace Stats;

    readBurstLengthHist
        .init(params->burst_length_bins)
        .flags(disableBurstLengthHists ? nozero : pdf);

    writeBurstLengthHist
        .init(params->burst_length_bins)
        .flags(disableBurstLengthHists ? nozero : pdf);

    // Stats based on received responses
    readBandwidthHist
        .init(params->bandwidth_bins)
        .flags(disableBandwidthHists ? nozero : pdf);

    averageReadBandwidth
        .flags(disableBandwidthHists ? nozero : pdf);

    totalReadBytes
        .flags(disableBandwidthHists ? nozero : pdf);

    // Stats based on successfully sent requests
    writeBandwidthHist
        .init(params->bandwidth_bins)
        .flags(disableBandwidthHists ? (pdf | nozero) : pdf);

    averageWriteBandwidth
        .flags(disableBandwidthHists ? nozero : pdf);

    totalWrittenBytes
        .flags(disableBandwidthHists ? nozero : pdf);


    readLatencyHist
        .init(params->latency_bins)
        .flags(disableLatencyHists ? nozero : pdf);

    writeLatencyHist
        .init(params->latency_bins)
        .flags(disableLatencyHists ? nozero : pdf);

    ittReadRead
        .init(1, params->itt_max_bin, params->itt_max_bin /
              params->itt_bins)
        .flags(disableITTDists ? nozero : pdf);

    ittWriteWrite
        .init(1, params->itt_max_bin, params->itt_max_bin /
              params->itt_bins)
        .flags(disableITTDists ? nozero : pdf);

    ittReqReq
        .init(1, params->itt_max_bin, params->itt_max_bin /
              params->itt_bins)
        .flags(disableITTDists ? nozero : pdf);

    outstandingReadsHist
        .init(params->outstanding_bins)
        .flags(disableOutstandingHists ? nozero : pdf);

    outstandingWritesHist
        .init(params->outstanding_bins)
        .flags(disableOutstandingHists ? nozero : pdf);

    readTransHist
        .init(params->transaction_bins)
        .flags(disableTransactionHists ? nozero : pdf);

    writeTransHist
        .init(params->transaction_bins)
        .flags(disableTransactionHists ? nozero : pdf);

    readAddrDist
        .init(0)
        .flags(disableAddrDists ? nozero : pdf);

    writeAddrDist
        .init(0)
        .flags(disableAddrDists ? nozero : pdf);
}

void
CommMonitor::MonitorStats::updateReqStats(
    const ProbePoints::PacketInfo& pkt_info, bool is_atomic,
    bool expects_response)
{
    if (pkt_info.cmd.isRead()) {
        // Increment number of observed read transactions
        if (!disableTransactionHists)
            ++readTrans;

        // Get sample of burst length
        if (!disableBurstLengthHists)
            readBurstLengthHist.sample(pkt_info.size);

        // Sample the masked address
        if (!disableAddrDists)
            readAddrDist.sample(pkt_info.addr & readAddrMask);

        if (!disableITTDists) {
            // Sample value of read-read inter transaction time
            if (timeOfLastRead != 0)
                ittReadRead.sample(curTick() - timeOfLastRead);
            timeOfLastRead = curTick();

            // Sample value of req-req inter transaction time
            if (timeOfLastReq != 0)
                ittReqReq.sample(curTick() - timeOfLastReq);
            timeOfLastReq = curTick();
        }
        if (!is_atomic && !disableOutstandingHists && expects_response)
            ++outstandingReadReqs;

    } else if (pkt_info.cmd.isWrite()) {
        // Same as for reads
        if (!disableTransactionHists)
            ++writeTrans;

        if (!disableBurstLengthHists)
            writeBurstLengthHist.sample(pkt_info.size);

        // Update the bandwidth stats on the request
        if (!disableBandwidthHists) {
            writtenBytes += pkt_info.size;
            totalWrittenBytes += pkt_info.size;
        }

        // Sample the masked write address
        if (!disableAddrDists)
            writeAddrDist.sample(pkt_info.addr & writeAddrMask);

        if (!disableITTDists) {
            // Sample value of write-to-write inter transaction time
            if (timeOfLastWrite != 0)
                ittWriteWrite.sample(curTick() - timeOfLastWrite);
            timeOfLastWrite = curTick();

            // Sample value of req-to-req inter transaction time
            if (timeOfLastReq != 0)
                ittReqReq.sample(curTick() - timeOfLastReq);
            timeOfLastReq = curTick();
        }

        if (!is_atomic && !disableOutstandingHists && expects_response)
            ++outstandingWriteReqs;
    }
}

void
CommMonitor::MonitorStats::updateRespStats(
    const ProbePoints::PacketInfo& pkt_info, Tick latency, bool is_atomic)
{
    if (pkt_info.cmd.isRead()) {
        // Decrement number of outstanding read requests
        if (!is_atomic && !disableOutstandingHists) {
            assert(outstandingReadReqs != 0);
            --outstandingReadReqs;
        }

        if (!disableLatencyHists)
            readLatencyHist.sample(latency);

        // Update the bandwidth stats based on responses for reads
        if (!disableBandwidthHists) {
            readBytes += pkt_info.size;
            totalReadBytes += pkt_info.size;
        }

    } else if (pkt_info.cmd.isWrite()) {
        // Decrement number of outstanding write requests
        if (!is_atomic && !disableOutstandingHists) {
            assert(outstandingWriteReqs != 0);
            --outstandingWriteReqs;
        }

        if (!disableLatencyHists)
            writeLatencyHist.sample(latency);
    }
}

Tick
CommMonitor::recvAtomic(PacketPtr pkt)
{
    const bool expects_response(pkt->needsResponse() &&
                                !pkt->cacheResponding());
    ProbePoints::PacketInfo req_pkt_info(pkt);
    ppPktReq->notify(req_pkt_info);

    const Tick delay(masterPort.sendAtomic(pkt));

    stats.updateReqStats(req_pkt_info, true, expects_response);
    if (expects_response)
        stats.updateRespStats(req_pkt_info, delay, true);

    // Some packets, such as WritebackDirty, don't need response.
    assert(pkt->isResponse() || !expects_response);
    ProbePoints::PacketInfo resp_pkt_info(pkt);
    ppPktResp->notify(resp_pkt_info);
    return delay;
}

Tick
CommMonitor::recvAtomicSnoop(PacketPtr pkt)
{
    return slavePort.sendAtomicSnoop(pkt);
}

bool
CommMonitor::recvTimingReq(PacketPtr pkt)
{
    // should always see a request
    assert(pkt->isRequest());

    // Store relevant fields of packet, because packet may be modified
    // or even deleted when sendTiming() is called.
    const ProbePoints::PacketInfo pkt_info(pkt);

    const bool expects_response(pkt->needsResponse() &&
                                !pkt->cacheResponding());

    // If a cache miss is served by a cache, a monitor near the memory
    // would see a request which needs a response, but this response
    // would not come back from the memory. Therefore we additionally
    // have to check the cacheResponding flag
    if (expects_response && !stats.disableLatencyHists) {
        pkt->pushSenderState(new CommMonitorSenderState(curTick()));
    }

    // Attempt to send the packet
    bool successful = masterPort.sendTimingReq(pkt);

    // If not successful, restore the sender state
    if (!successful && expects_response && !stats.disableLatencyHists) {
        delete pkt->popSenderState();
    }

    if (successful) {
        ppPktReq->notify(pkt_info);
    }

    if (successful) {
        DPRINTF(CommMonitor, "Forwarded %s request\n", pkt->isRead() ? "read" :
                pkt->isWrite() ? "write" : "non read/write");
        stats.updateReqStats(pkt_info, false, expects_response);
    }
    return successful;
}

bool
CommMonitor::recvTimingResp(PacketPtr pkt)
{
    // should always see responses
    assert(pkt->isResponse());

    // Store relevant fields of packet, because packet may be modified
    // or even deleted when sendTiming() is called.
    const ProbePoints::PacketInfo pkt_info(pkt);

    Tick latency = 0;
    CommMonitorSenderState* received_state =
        dynamic_cast<CommMonitorSenderState*>(pkt->senderState);

    if (!stats.disableLatencyHists) {
        // Restore initial sender state
        if (received_state == NULL)
            panic("Monitor got a response without monitor sender state\n");

        // Restore the sate
        pkt->senderState = received_state->predecessor;
    }

    // Attempt to send the packet
    bool successful = slavePort.sendTimingResp(pkt);

    if (!stats.disableLatencyHists) {
        // If packet successfully send, sample value of latency,
        // afterwards delete sender state, otherwise restore state
        if (successful) {
            latency = curTick() - received_state->transmitTime;
            DPRINTF(CommMonitor, "Latency: %d\n", latency);
            delete received_state;
        } else {
            // Don't delete anything and let the packet look like we
            // did not touch it
            pkt->senderState = received_state;
        }
    }

    if (successful) {
        ppPktResp->notify(pkt_info);
        DPRINTF(CommMonitor, "Received %s response\n", pkt->isRead() ? "read" :
                pkt->isWrite() ?  "write" : "non read/write");
        stats.updateRespStats(pkt_info, latency, false);
    }
    return successful;
}

void
CommMonitor::recvTimingSnoopReq(PacketPtr pkt)
{
    slavePort.sendTimingSnoopReq(pkt);
}

bool
CommMonitor::recvTimingSnoopResp(PacketPtr pkt)
{
    return masterPort.sendTimingSnoopResp(pkt);
}

void
CommMonitor::recvRetrySnoopResp()
{
    slavePort.sendRetrySnoopResp();
}

bool
CommMonitor::isSnooping() const
{
    // check if the connected master port is snooping
    return slavePort.isSnooping();
}

AddrRangeList
CommMonitor::getAddrRanges() const
{
    // get the address ranges of the connected slave port
    return masterPort.getAddrRanges();
}

void
CommMonitor::recvReqRetry()
{
    slavePort.sendRetryReq();
}

void
CommMonitor::recvRespRetry()
{
    masterPort.sendRetryResp();
}

bool
CommMonitor::tryTiming(PacketPtr pkt)
{
    return masterPort.tryTiming(pkt);
}

void
CommMonitor::recvRangeChange()
{
    slavePort.sendRangeChange();
}

void
CommMonitor::samplePeriodic()
{
    // the periodic stats update runs on the granularity of sample
    // periods, but in combination with this there may also be a
    // external resets and dumps of the stats (through schedStatEvent)
    // causing the stats themselves to capture less than a sample
    // period

    // only capture if we have not reset the stats during the last
    // sample period
    if (simTicks.value() >= samplePeriodTicks) {
        if (!stats.disableTransactionHists) {
            stats.readTransHist.sample(stats.readTrans);
            stats.writeTransHist.sample(stats.writeTrans);
        }

        if (!stats.disableBandwidthHists) {
            stats.readBandwidthHist.sample(stats.readBytes / samplePeriod);
            stats.writeBandwidthHist.sample(stats.writtenBytes / samplePeriod);
        }

        if (!stats.disableOutstandingHists) {
            stats.outstandingReadsHist.sample(stats.outstandingReadReqs);
            stats.outstandingWritesHist.sample(stats.outstandingWriteReqs);
        }
    }

    // reset the sampled values
    stats.readTrans = 0;
    stats.writeTrans = 0;

    stats.readBytes = 0;
    stats.writtenBytes = 0;

    schedule(samplePeriodicEvent, curTick() + samplePeriodTicks);
}

void
CommMonitor::startup()
{
    schedule(samplePeriodicEvent, curTick() + samplePeriodTicks);
}
