/*
 * Copyright (c) 2012-2013 ARM Limited
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
 */

#ifndef __MEM_COMM_MONITOR_HH__
#define __MEM_COMM_MONITOR_HH__

#include "base/statistics.hh"
#include "base/time.hh"
#include "mem/mem_object.hh"
#include "params/CommMonitor.hh"
#include "proto/protoio.hh"
#include "sim/system.hh"

/**
 * The communication monitor is a MemObject which can monitor statistics of
 * the communication happening between two ports in the memory system.
 *
 * Currently the following stats are implemented: Histograms of read/write
 * transactions, read/write burst lengths, read/write bandwidth,
 * outstanding read/write requests, read latency and inter transaction time
 * (read-read, write-write, read/write-read/write). Furthermore it allows
 * to capture the number of accesses to an address over time ("heat map").
 * All stats can be disabled from Python.
 */
class CommMonitor : public MemObject
{

  public:

    /** Parameters of communication monitor */
    typedef CommMonitorParams Params;
    const Params* params() const
    { return reinterpret_cast<const Params*>(_params); }

    /**
     * Constructor based on the Python params
     *
     * @param params Python parameters
     */
    CommMonitor(Params* params);

    /** Destructor */
    ~CommMonitor() {}

    /**
     * Callback to flush and close all open output streams on exit. If
     * we were calling the destructor it could be done there.
     */
    void closeStreams();

    virtual BaseMasterPort& getMasterPort(const std::string& if_name,
                                          PortID idx = InvalidPortID);

    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID);

    virtual void init();

    /** Register statistics */
    void regStats();

  private:

    /**
     * Sender state class for the monitor so that we can annotate
     * packets with a transmit time and receive time.
     */
    class CommMonitorSenderState : public Packet::SenderState
    {

      public:

        /**
         * Construct a new sender state and store the time so we can
         * calculate round-trip latency.
         *
         * @param _transmitTime Time of packet transmission
         */
        CommMonitorSenderState(Tick _transmitTime)
            : transmitTime(_transmitTime)
        { }

        /** Destructor */
        ~CommMonitorSenderState() { }

        /** Tick when request is transmitted */
        Tick transmitTime;

    };

    /**
     * This is the master port of the communication monitor. All recv
     * functions call a function in CommMonitor, where the
     * send function of the slave port is called. Besides this, these
     * functions can also perform actions for capturing statistics.
     */
    class MonitorMasterPort : public MasterPort
    {

      public:

        MonitorMasterPort(const std::string& _name, CommMonitor& _mon)
            : MasterPort(_name, &_mon), mon(_mon)
        { }

      protected:

        void recvFunctionalSnoop(PacketPtr pkt)
        {
            mon.recvFunctionalSnoop(pkt);
        }

        Tick recvAtomicSnoop(PacketPtr pkt)
        {
            return mon.recvAtomicSnoop(pkt);
        }

        bool recvTimingResp(PacketPtr pkt)
        {
            return mon.recvTimingResp(pkt);
        }

        void recvTimingSnoopReq(PacketPtr pkt)
        {
            mon.recvTimingSnoopReq(pkt);
        }

        void recvRangeChange()
        {
            mon.recvRangeChange();
        }

        bool isSnooping() const
        {
            return mon.isSnooping();
        }

        void recvRetry()
        {
            mon.recvRetryMaster();
        }

      private:

        CommMonitor& mon;

    };

    /** Instance of master port, facing the memory side */
    MonitorMasterPort masterPort;

    /**
     * This is the slave port of the communication monitor. All recv
     * functions call a function in CommMonitor, where the
     * send function of the master port is called. Besides this, these
     * functions can also perform actions for capturing statistics.
     */
    class MonitorSlavePort : public SlavePort
    {

      public:

        MonitorSlavePort(const std::string& _name, CommMonitor& _mon)
            : SlavePort(_name, &_mon), mon(_mon)
        { }

      protected:

        void recvFunctional(PacketPtr pkt)
        {
            mon.recvFunctional(pkt);
        }

        Tick recvAtomic(PacketPtr pkt)
        {
            return mon.recvAtomic(pkt);
        }

        bool recvTimingReq(PacketPtr pkt)
        {
            return mon.recvTimingReq(pkt);
        }

        bool recvTimingSnoopResp(PacketPtr pkt)
        {
            return mon.recvTimingSnoopResp(pkt);
        }

        AddrRangeList getAddrRanges() const
        {
            return mon.getAddrRanges();
        }

        void recvRetry()
        {
            mon.recvRetrySlave();
        }

      private:

        CommMonitor& mon;

    };

    /** Instance of slave port, i.e. on the CPU side */
    MonitorSlavePort slavePort;

    void recvFunctional(PacketPtr pkt);

    void recvFunctionalSnoop(PacketPtr pkt);

    Tick recvAtomic(PacketPtr pkt);

    Tick recvAtomicSnoop(PacketPtr pkt);

    bool recvTimingReq(PacketPtr pkt);

    bool recvTimingResp(PacketPtr pkt);

    void recvTimingSnoopReq(PacketPtr pkt);

    bool recvTimingSnoopResp(PacketPtr pkt);

    AddrRangeList getAddrRanges() const;

    bool isSnooping() const;

    void recvRetryMaster();

    void recvRetrySlave();

    void recvRangeChange();

    void periodicTraceDump();

    /** Stats declarations, all in a struct for convenience. */
    struct MonitorStats
    {

        /** Disable flag for burst length historgrams **/
        bool disableBurstLengthHists;

        /** Histogram of read burst lengths */
        Stats::Histogram readBurstLengthHist;

        /** Histogram of write burst lengths */
        Stats::Histogram writeBurstLengthHist;

        /** Disable flag for the bandwidth histograms */
        bool disableBandwidthHists;

        /**
         * Histogram for read bandwidth per sample window. The
         * internal counter is an unsigned int rather than a stat.
         */
        unsigned int readBytes;
        Stats::Histogram readBandwidthHist;
        Stats::Formula averageReadBW;
        Stats::Scalar totalReadBytes;

        /**
         * Histogram for write bandwidth per sample window. The
         * internal counter is an unsigned int rather than a stat.
         */
        unsigned int writtenBytes;
        Stats::Histogram writeBandwidthHist;
        Stats::Formula averageWriteBW;
        Stats::Scalar totalWrittenBytes;

        /** Disable flag for latency histograms. */
        bool disableLatencyHists;

        /** Histogram of read request-to-response latencies */
        Stats::Histogram readLatencyHist;

        /** Histogram of write request-to-response latencies */
        Stats::Histogram writeLatencyHist;

        /** Disable flag for ITT distributions. */
        bool disableITTDists;

        /**
         * Inter transaction time (ITT) distributions. There are
         * histograms of the time between two read, write or arbitrary
         * accesses. The time of a request is the tick at which the
         * request is forwarded by the monitor.
         */
        Stats::Distribution ittReadRead;
        Stats::Distribution ittWriteWrite;
        Stats::Distribution ittReqReq;
        Tick timeOfLastRead;
        Tick timeOfLastWrite;
        Tick timeOfLastReq;

        /** Disable flag for outstanding histograms. */
        bool disableOutstandingHists;

        /**
         * Histogram of outstanding read requests. Counter for
         * outstanding read requests is an unsigned integer because
         * it should not be reset when stats are reset.
         */
        Stats::Histogram outstandingReadsHist;
        unsigned int outstandingReadReqs;

        /**
         * Histogram of outstanding write requests. Counter for
         * outstanding write requests is an unsigned integer because
         * it should not be reset when stats are reset.
         */
        Stats::Histogram outstandingWritesHist;
        unsigned int outstandingWriteReqs;

        /** Disable flag for transaction histograms. */
        bool disableTransactionHists;

        /** Histogram of number of read transactions per time bin */
        Stats::Histogram readTransHist;
        unsigned int readTrans;

        /** Histogram of number of timing write transactions per time bin */
        Stats::Histogram writeTransHist;
        unsigned int writeTrans;

        /** Disable flag for address distributions. */
        bool disableAddrDists;

        /**
         * Histogram of number of read accesses to addresses over
         * time.
         */
        Stats::SparseHistogram readAddrDist;

        /**
         * Histogram of number of write accesses to addresses over
         * time.
         */
        Stats::SparseHistogram writeAddrDist;

        /**
         * Create the monitor stats and initialise all the members
         * that are not statistics themselves, but used to control the
         * stats or track values during a sample period.
         */
        MonitorStats(const CommMonitorParams* params) :
            disableBurstLengthHists(params->disable_burst_length_hists),
            disableBandwidthHists(params->disable_bandwidth_hists),
            readBytes(0), writtenBytes(0),
            disableLatencyHists(params->disable_latency_hists),
            disableITTDists(params->disable_itt_dists),
            timeOfLastRead(0), timeOfLastWrite(0), timeOfLastReq(0),
            disableOutstandingHists(params->disable_outstanding_hists),
            outstandingReadReqs(0), outstandingWriteReqs(0),
            disableTransactionHists(params->disable_transaction_hists),
            readTrans(0), writeTrans(0),
            disableAddrDists(params->disable_addr_dists)
        { }

    };

    /** This function is called periodically at the end of each time bin */
    void samplePeriodic();

    /** Schedule the first periodic event */
    void startup();

    /** Periodic event called at the end of each simulation time bin */
    EventWrapper<CommMonitor, &CommMonitor::samplePeriodic> samplePeriodicEvent;

    /** Length of simulation time bin*/
    Tick samplePeriodTicks;
    Time samplePeriod;

    /** Address mask for sources of read accesses to be captured */
    Addr readAddrMask;

    /** Address mask for sources of write accesses to be captured */
    Addr writeAddrMask;

    /** Instantiate stats */
    MonitorStats stats;

    /** Output stream for a potential trace. */
    ProtoOutputStream* traceStream;

    /** The system in which the monitor lives */
    System *system;
};

#endif //__MEM_COMM_MONITOR_HH__
