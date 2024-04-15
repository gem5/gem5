/*
 * Copyright (c) 2012-2014 ARM Limited
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

#ifndef __MEM_MEM_CHECKER_MONITOR_HH__
#define __MEM_MEM_CHECKER_MONITOR_HH__

#include "base/statistics.hh"
#include "mem/mem_checker.hh"
#include "params/MemCheckerMonitor.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

/**
 * Implements a MemChecker monitor, to be inserted between two ports.
 */
class MemCheckerMonitor : public SimObject
{
  public:
    /** Parameters of memchecker monitor */
    using Params = MemCheckerMonitorParams;

    /**
     * Constructor based on the Python params
     *
     * @param params Python parameters
     */
    MemCheckerMonitor(const Params &params);

    /** Destructor */
    ~MemCheckerMonitor();

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void init() override;

  private:
    struct MemCheckerMonitorSenderState : public Packet::SenderState
    {
        MemCheckerMonitorSenderState(MemChecker::Serial _serial)
            : serial(_serial)
        {}

        MemChecker::Serial serial;
    };

    /**
     * This is the request port of the communication monitor. All recv
     * functions call a function in MemCheckerMonitor, where the
     * send function of the response port is called. Besides this, these
     * functions can also perform actions for capturing statistics.
     */
    class MonitorRequestPort : public RequestPort
    {
      public:
        MonitorRequestPort(const std::string &_name, MemCheckerMonitor &_mon)
            : RequestPort(_name), mon(_mon)
        {}

      protected:
        void
        recvFunctionalSnoop(PacketPtr pkt)
        {
            mon.recvFunctionalSnoop(pkt);
        }

        Tick
        recvAtomicSnoop(PacketPtr pkt)
        {
            return mon.recvAtomicSnoop(pkt);
        }

        bool
        recvTimingResp(PacketPtr pkt)
        {
            return mon.recvTimingResp(pkt);
        }

        void
        recvTimingSnoopReq(PacketPtr pkt)
        {
            mon.recvTimingSnoopReq(pkt);
        }

        void
        recvRangeChange()
        {
            mon.recvRangeChange();
        }

        bool
        isSnooping() const
        {
            return mon.isSnooping();
        }

        void
        recvReqRetry()
        {
            mon.recvReqRetry();
        }

      private:
        MemCheckerMonitor &mon;
    };

    /** Instance of request port, facing the memory side */
    MonitorRequestPort memSidePort;

    /**
     * This is the response port of the communication monitor. All recv
     * functions call a function in MemCheckerMonitor, where the
     * send function of the request port is called. Besides this, these
     * functions can also perform actions for capturing statistics.
     */
    class MonitorResponsePort : public ResponsePort
    {
      public:
        MonitorResponsePort(const std::string &_name, MemCheckerMonitor &_mon)
            : ResponsePort(_name), mon(_mon)
        {}

      protected:
        void
        recvFunctional(PacketPtr pkt)
        {
            mon.recvFunctional(pkt);
        }

        Tick
        recvAtomic(PacketPtr pkt)
        {
            return mon.recvAtomic(pkt);
        }

        bool
        recvTimingReq(PacketPtr pkt)
        {
            return mon.recvTimingReq(pkt);
        }

        bool
        recvTimingSnoopResp(PacketPtr pkt)
        {
            return mon.recvTimingSnoopResp(pkt);
        }

        AddrRangeList
        getAddrRanges() const
        {
            return mon.getAddrRanges();
        }

        void
        recvRespRetry()
        {
            mon.recvRespRetry();
        }

      private:
        MemCheckerMonitor &mon;
    };

    /** Instance of response port, i.e. on the CPU side */
    MonitorResponsePort cpuSidePort;

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

    void recvReqRetry();

    void recvRespRetry();

    void recvRangeChange();

    bool warnOnly;

    MemChecker *memchecker;
};

} // namespace gem5

#endif //__MEM_MEM_CHECKER_MONITOR_HH__
