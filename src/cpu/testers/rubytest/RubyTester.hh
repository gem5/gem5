/*
 * Copyright (c) 2013 ARM Limited
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
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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

#ifndef __CPU_RUBYTEST_RUBYTESTER_HH__
#define __CPU_RUBYTEST_RUBYTESTER_HH__

#include <iostream>
#include <string>
#include <vector>

#include "cpu/testers/rubytest/CheckTable.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "params/RubyTester.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class RubyTester : public ClockedObject
{
  public:
    class CpuPort : public RequestPort
    {
      private:
        RubyTester *tester;
        // index for m_last_progress_vector and hitCallback
        PortID globalIdx;

      public:
        //
        // Currently, each instatiation of the RubyTester::CpuPort supports
        // only instruction or data requests, not both.  However, for those
        // RubyPorts that support both types of requests, separate InstOnly
        // and DataOnly CpuPorts will map to that RubyPort

        CpuPort(const std::string &_name, RubyTester *_tester, PortID _id,
                PortID _index)
            : RequestPort(_name, _id), tester(_tester),
              globalIdx(_index)
        {}

      protected:
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void recvReqRetry()
        { panic("%s does not expect a retry\n", name()); }
    };

    struct SenderState : public Packet::SenderState
    {
        ruby::SubBlock subBlock;

        SenderState(Addr addr, int size, int cl_size)
            : subBlock(addr, size, cl_size)
        {}

    };

    typedef RubyTesterParams Params;
    RubyTester(const Params &p);
    ~RubyTester();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    bool isInstOnlyCpuPort(int idx);
    bool isInstDataCpuPort(int idx);

    RequestPort* getReadableCpuPort(int idx);
    RequestPort* getWritableCpuPort(int idx);

    void init() override;

    void wakeup();

    void incrementCheckCompletions() { m_checks_completed++; }

    void printStats(std::ostream& out) const {}
    void clearStats() {}
    void printConfig(std::ostream& out) const {}

    void print(std::ostream& out) const;
    bool getCheckFlush() { return m_check_flush; }

    void updateProgress(int idx, Addr address, Cycles current_time);
    void eraseProgress(int idx, Addr address);

    RequestorID requestorId() { return _requestorId; }
  protected:
    EventFunctionWrapper checkStartEvent;

    RequestorID _requestorId;

  private:
    void hitCallback(ruby::NodeID proc, ruby::SubBlock* data);

    void checkForDeadlock();

    // Private copy constructor and assignment operator
    RubyTester(const RubyTester& obj);
    RubyTester& operator=(const RubyTester& obj);

    CheckTable* m_checkTable_ptr;
    std::vector<std::unordered_map<Addr, Cycles>> m_last_progress_vector;

    int m_num_cpus;
    uint64_t m_checks_completed;
    std::vector<RequestPort*> writePorts;
    std::vector<RequestPort*> readPorts;
    uint64_t m_checks_to_complete;
    int m_deadlock_threshold;
    int m_num_writers;
    int m_num_readers;
    int m_wakeup_frequency;
    bool m_check_flush;
    int m_num_inst_only_ports;
    int m_num_inst_data_ports;
};

inline std::ostream&
operator<<(std::ostream& out, const RubyTester& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace gem5

#endif // __CPU_RUBYTEST_RUBYTESTER_HH__
