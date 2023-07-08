/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2009-2010 Advanced Micro Devices, Inc.
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

#ifndef __CPU_DIRECTEDTEST_RUBYDIRECTEDTESTER_HH__
#define __CPU_DIRECTEDTEST_RUBYDIRECTEDTESTER_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "params/RubyDirectedTester.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class DirectedGenerator;

class RubyDirectedTester : public ClockedObject
{
  public:
    class CpuPort : public RequestPort
    {
      private:
        RubyDirectedTester *tester;

      public:
        CpuPort(const std::string &_name, RubyDirectedTester *_tester,
                PortID _id)
            : RequestPort(_name, _id), tester(_tester)
        {}

      protected:
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void recvReqRetry()
        { panic("%s does not expect a retry\n", name()); }
    };

    typedef RubyDirectedTesterParams Params;
    RubyDirectedTester(const Params &p);
    ~RubyDirectedTester();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    RequestPort* getCpuPort(int idx);

    void init() override;

    void wakeup();

    void incrementCycleCompletions() { m_requests_completed++; }

    void printStats(std::ostream& out) const {}
    void clearStats() {}
    void printConfig(std::ostream& out) const {}

    void print(std::ostream& out) const;

  protected:
    EventFunctionWrapper directedStartEvent;

  private:
    void hitCallback(ruby::NodeID proc, Addr addr);

    void checkForDeadlock();

    // Private copy constructor and assignment operator
    RubyDirectedTester(const RubyDirectedTester& obj);
    RubyDirectedTester& operator=(const RubyDirectedTester& obj);

    uint64_t m_requests_completed;
    std::vector<RequestPort*> ports;
    uint64_t m_requests_to_complete;
    DirectedGenerator* generator;
};

} // namespace gem5

#endif // __CPU_DIRECTEDTEST_RUBYDIRECTEDTESTER_HH__
