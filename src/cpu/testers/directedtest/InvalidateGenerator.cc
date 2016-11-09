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

#include "cpu/testers/directedtest/InvalidateGenerator.hh"

#include "base/trace.hh"
#include "cpu/testers/directedtest/DirectedGenerator.hh"
#include "cpu/testers/directedtest/RubyDirectedTester.hh"
#include "debug/DirectedTest.hh"

InvalidateGenerator::InvalidateGenerator(const Params *p)
    : DirectedGenerator(p)
{
    //
    // First, issue loads to bring the block into S state
    //
    m_status = InvalidateGeneratorStatus_Load_Waiting;
    m_active_read_node = 0;
    m_active_inv_node = 0;
    m_address = 0x0;
    m_addr_increment_size = p->addr_increment_size;
}

InvalidateGenerator::~InvalidateGenerator()
{
}

bool
InvalidateGenerator::initiate()
{
    MasterPort* port;
    Request::Flags flags;
    PacketPtr pkt;
    Packet::Command cmd;

    // For simplicity, requests are assumed to be 1 byte-sized
    Request *req = new Request(m_address, 1, flags, masterId);

    //
    // Based on the current state, issue a load or a store
    //
    if (m_status == InvalidateGeneratorStatus_Load_Waiting) {
        DPRINTF(DirectedTest, "initiating read\n");
        cmd = MemCmd::ReadReq;
        port = m_directed_tester->getCpuPort(m_active_read_node);
        pkt = new Packet(req, cmd);
    } else if (m_status == InvalidateGeneratorStatus_Inv_Waiting) {
        DPRINTF(DirectedTest, "initiating invalidating write\n");
        cmd = MemCmd::WriteReq;
        port = m_directed_tester->getCpuPort(m_active_inv_node);
        pkt = new Packet(req, cmd);
    } else {
        panic("initiate was unexpectedly called\n");
    }
    pkt->allocate();

    if (port->sendTimingReq(pkt)) {
        DPRINTF(DirectedTest, "initiating request - successful\n");
        if (m_status == InvalidateGeneratorStatus_Load_Waiting) {
            m_status = InvalidateGeneratorStatus_Load_Pending;
        } else {
            m_status = InvalidateGeneratorStatus_Inv_Pending;
        }
        return true;
    } else {
        // If the packet did not issue, must delete
        // Note: No need to delete the data, the packet destructor
        // will delete it
        delete pkt->req;
        delete pkt;

        DPRINTF(DirectedTest, "failed to issue request - sequencer not ready\n");
        return false;
    }
}

void
InvalidateGenerator::performCallback(uint32_t proc, Addr address)
{
    assert(m_address == address);

    if (m_status == InvalidateGeneratorStatus_Load_Pending) {
        assert(m_active_read_node == proc);
        m_active_read_node++;
        //
        // Once all cpus have the block in S state, issue the invalidate
        //
        if (m_active_read_node == m_num_cpus) {
            m_status = InvalidateGeneratorStatus_Inv_Waiting;
            m_active_read_node = 0;
        } else {
            m_status = InvalidateGeneratorStatus_Load_Waiting;
        }
    } else if (m_status == InvalidateGeneratorStatus_Inv_Pending) {
        assert(m_active_inv_node == proc);
        m_active_inv_node++;
        if (m_active_inv_node == m_num_cpus) {
            m_address += m_addr_increment_size;
            m_active_inv_node = 0;
        }
        //
        // Invalidate completed, send that info to the tester and restart
        // the cycle
        //
        m_directed_tester->incrementCycleCompletions();
        m_status = InvalidateGeneratorStatus_Load_Waiting;
    }

}

InvalidateGenerator *
InvalidateGeneratorParams::create()
{
    return new InvalidateGenerator(this);
}
