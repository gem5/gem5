/*
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

#include "base/misc.hh"
#include "cpu/testers/rubytest/Check.hh"
#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/RubyTest.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/system/System.hh"
#include "sim/sim_exit.hh"

RubyTester::RubyTester(const Params *p)
  : MemObject(p), checkStartEvent(this),
    m_checks_to_complete(p->checks_to_complete),
    m_deadlock_threshold(p->deadlock_threshold),
    m_wakeup_frequency(p->wakeup_frequency),
    m_check_flush(p->check_flush)
{
    m_checks_completed = 0;

    // add the check start event to the event queue
    schedule(checkStartEvent, 1);
}

RubyTester::~RubyTester()
{
    delete m_checkTable_ptr;
    for (int i = 0; i < ports.size(); i++)
        delete ports[i];
}

void
RubyTester::init()
{
    assert(ports.size() > 0);

    m_last_progress_vector.resize(ports.size());
    for (int i = 0; i < m_last_progress_vector.size(); i++) {
        m_last_progress_vector[i] = 0;
    }

    m_num_cpu_sequencers = ports.size();

    m_checkTable_ptr = new CheckTable(m_num_cpu_sequencers, this);
}

Port *
RubyTester::getPort(const std::string &if_name, int idx)
{
    if (if_name != "cpuPort") {
        panic("RubyTester::getPort: unknown port %s requested", if_name);
    }

    if (idx >= (int)ports.size()) {
        ports.resize(idx + 1);
    }

    if (ports[idx] != NULL) {
        panic("RubyTester::getPort: port %d already assigned", idx);
    }

    CpuPort *port = new CpuPort(csprintf("%s-port%d", name(), idx), this, idx);

    ports[idx] = port;
    return port;
}

Tick
RubyTester::CpuPort::recvAtomic(PacketPtr pkt)
{
    panic("RubyTester::CpuPort::recvAtomic() not implemented!\n");
    return 0;
}

bool
RubyTester::CpuPort::recvTiming(PacketPtr pkt)
{
    // retrieve the subblock and call hitCallback
    RubyTester::SenderState* senderState =
        safe_cast<RubyTester::SenderState*>(pkt->senderState);
    SubBlock* subblock = senderState->subBlock;
    assert(subblock != NULL);

    // pop the sender state from the packet
    pkt->senderState = senderState->saved;

    tester->hitCallback(idx, subblock);

    // Now that the tester has completed, delete the senderState
    // (includes sublock) and the packet, then return
    delete senderState;
    delete pkt->req;
    delete pkt;
    return true;
}

Port*
RubyTester::getCpuPort(int idx)
{
    assert(idx >= 0 && idx < ports.size());

    return ports[idx];
}

void
RubyTester::hitCallback(NodeID proc, SubBlock* data)
{
    // Mark that we made progress
    m_last_progress_vector[proc] = g_eventQueue_ptr->getTime();

    DPRINTF(RubyTest, "completed request for proc: %d\n", proc);
    DPRINTF(RubyTest, "addr: 0x%x, size: %d, data: ",
            data->getAddress(), data->getSize());
    for (int byte = 0; byte < data->getSize(); byte++) {
        DPRINTF(RubyTest, "%d", data->getByte(byte));
    }
    DPRINTF(RubyTest, "\n");

    // This tells us our store has 'completed' or for a load gives us
    // back the data to make the check
    Check* check_ptr = m_checkTable_ptr->getCheck(data->getAddress());
    assert(check_ptr != NULL);
    check_ptr->performCallback(proc, data);
}

void
RubyTester::wakeup()
{
    if (m_checks_completed < m_checks_to_complete) {
        // Try to perform an action or check
        Check* check_ptr = m_checkTable_ptr->getRandomCheck();
        assert(check_ptr != NULL);
        check_ptr->initiate();

        checkForDeadlock();

        schedule(checkStartEvent, curTick() + m_wakeup_frequency);
    } else {
        exitSimLoop("Ruby Tester completed");
    }
}

void
RubyTester::checkForDeadlock()
{
    int size = m_last_progress_vector.size();
    Time current_time = g_eventQueue_ptr->getTime();
    for (int processor = 0; processor < size; processor++) {
        if ((current_time - m_last_progress_vector[processor]) >
                m_deadlock_threshold) {
            panic("Deadlock detected: current_time: %d last_progress_time: %d "
                  "difference:  %d processor: %d\n",
                  current_time, m_last_progress_vector[processor],
                  current_time - m_last_progress_vector[processor], processor);
        }
    }
}

void
RubyTester::print(std::ostream& out) const
{
    out << "[RubyTester]" << std::endl;
}

RubyTester *
RubyTesterParams::create()
{
    return new RubyTester(this);
}
