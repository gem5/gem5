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

#include "cpu/testers/directedtest/SeriesRequestGenerator.hh"

#include "base/random.hh"
#include "base/trace.hh"
#include "cpu/testers/directedtest/DirectedGenerator.hh"
#include "cpu/testers/directedtest/RubyDirectedTester.hh"
#include "debug/DirectedTest.hh"

SeriesRequestGenerator::SeriesRequestGenerator(const Params *p)
    : DirectedGenerator(p),
      m_addr_increment_size(p->addr_increment_size),
      m_percent_writes(p->percent_writes)
{
    m_status = SeriesRequestGeneratorStatus_Thinking;
    m_active_node = 0;
    m_address = 0x0;
}

SeriesRequestGenerator::~SeriesRequestGenerator()
{
}

bool
SeriesRequestGenerator::initiate()
{
    DPRINTF(DirectedTest, "initiating request\n");
    assert(m_status == SeriesRequestGeneratorStatus_Thinking);

    MasterPort* port = m_directed_tester->getCpuPort(m_active_node);

    Request::Flags flags;

    // For simplicity, requests are assumed to be 1 byte-sized
    RequestPtr req = std::make_shared<Request>(m_address, 1, flags, masterId);

    Packet::Command cmd;
    bool do_write = (random_mt.random(0, 100) < m_percent_writes);
    if (do_write) {
        cmd = MemCmd::WriteReq;
    } else {
        cmd = MemCmd::ReadReq;
    }

    PacketPtr pkt = new Packet(req, cmd);
    pkt->allocate();

    if (port->sendTimingReq(pkt)) {
        DPRINTF(DirectedTest, "initiating request - successful\n");
        m_status = SeriesRequestGeneratorStatus_Request_Pending;
        return true;
    } else {
        // If the packet did not issue, must delete
        // Note: No need to delete the data, the packet destructor
        // will delete it
        delete pkt;

        DPRINTF(DirectedTest, "failed to initiate request - sequencer not ready\n");
        return false;
    }
}

void
SeriesRequestGenerator::performCallback(uint32_t proc, Addr address)
{
    assert(m_active_node == proc);
    assert(m_address == address);
    assert(m_status == SeriesRequestGeneratorStatus_Request_Pending);

    m_status = SeriesRequestGeneratorStatus_Thinking;
    m_active_node++;
    if (m_active_node == m_num_cpus) {
        //
        // Cycle of requests completed, increment cycle completions and restart
        // at cpu zero
        //
        m_directed_tester->incrementCycleCompletions();
        m_address += m_addr_increment_size;
        m_active_node = 0;
    }
}

SeriesRequestGenerator *
SeriesRequestGeneratorParams::create()
{
    return new SeriesRequestGenerator(this);
}
