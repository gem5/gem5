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

#include "cpu/testers/rubytest/Check.hh"

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/RubyTest.hh"
#include "mem/ruby/common/SubBlock.hh"

typedef RubyTester::SenderState SenderState;

Check::Check(Addr address, Addr pc, int _num_writers, int _num_readers,
             RubyTester* _tester)
    : m_num_writers(_num_writers), m_num_readers(_num_readers),
      m_tester_ptr(_tester)
{
    m_status = TesterStatus_Idle;

    pickValue();
    pickInitiatingNode();
    changeAddress(address);
    m_pc = pc;
    m_access_mode = RubyAccessMode(random_mt.random(0,
                                                    RubyAccessMode_NUM - 1));
    m_store_count = 0;
}

void
Check::initiate()
{
    DPRINTF(RubyTest, "initiating\n");
    debugPrint();

    // currently no protocols support prefetches
    if (false && (random_mt.random(0, 0xf) == 0)) {
        initiatePrefetch(); // Prefetch from random processor
    }

        if (m_tester_ptr->getCheckFlush() && (random_mt.random(0, 0xff) == 0)) {
        initiateFlush(); // issue a Flush request from random processor
    }

    if (m_status == TesterStatus_Idle) {
        initiateAction();
    } else if (m_status == TesterStatus_Ready) {
        initiateCheck();
    } else {
        // Pending - do nothing
        DPRINTF(RubyTest,
                "initiating action/check - failed: action/check is pending\n");
    }
}

void
Check::initiatePrefetch()
{
    DPRINTF(RubyTest, "initiating prefetch\n");

    int index = random_mt.random(0, m_num_readers - 1);
    MasterPort* port = m_tester_ptr->getReadableCpuPort(index);

    Request::Flags flags;
    flags.set(Request::PREFETCH);

    Packet::Command cmd;

    // 1 in 8 chance this will be an exclusive prefetch
    if (random_mt.random(0, 0x7) != 0) {
        cmd = MemCmd::ReadReq;

        // if necessary, make the request an instruction fetch
        if (m_tester_ptr->isInstOnlyCpuPort(index) ||
            (m_tester_ptr->isInstDataCpuPort(index) &&
             (random_mt.random(0, 0x1)))) {
            flags.set(Request::INST_FETCH);
        }
    } else {
        cmd = MemCmd::WriteReq;
        flags.set(Request::PF_EXCLUSIVE);
    }

    // Prefetches are assumed to be 0 sized
    Request *req = new Request(m_address, 0, flags,
            m_tester_ptr->masterId(), curTick(), m_pc);
    req->setContext(index);

    PacketPtr pkt = new Packet(req, cmd);
    // despite the oddity of the 0 size (questionable if this should
    // even be allowed), a prefetch is still a read and as such needs
    // a place to store the result
    uint8_t *data = new uint8_t[1];
    pkt->dataDynamic(data);

    // push the subblock onto the sender state.  The sequencer will
    // update the subblock on the return
    pkt->senderState = new SenderState(m_address, req->getSize());

    if (port->sendTimingReq(pkt)) {
        DPRINTF(RubyTest, "successfully initiated prefetch.\n");
    } else {
        // If the packet did not issue, must delete
        delete pkt->senderState;
        delete pkt->req;
        delete pkt;

        DPRINTF(RubyTest,
                "prefetch initiation failed because Port was busy.\n");
    }
}

void
Check::initiateFlush()
{

    DPRINTF(RubyTest, "initiating Flush\n");

    int index = random_mt.random(0, m_num_writers - 1);
    MasterPort* port = m_tester_ptr->getWritableCpuPort(index);

    Request::Flags flags;

    Request *req = new Request(m_address, CHECK_SIZE, flags,
            m_tester_ptr->masterId(), curTick(), m_pc);

    Packet::Command cmd;

    cmd = MemCmd::FlushReq;

    PacketPtr pkt = new Packet(req, cmd);

    // push the subblock onto the sender state.  The sequencer will
    // update the subblock on the return
    pkt->senderState = new SenderState(m_address, req->getSize());

    if (port->sendTimingReq(pkt)) {
        DPRINTF(RubyTest, "initiating Flush - successful\n");
    }
}

void
Check::initiateAction()
{
    DPRINTF(RubyTest, "initiating Action\n");
    assert(m_status == TesterStatus_Idle);

    int index = random_mt.random(0, m_num_writers - 1);
    MasterPort* port = m_tester_ptr->getWritableCpuPort(index);

    Request::Flags flags;

    // Create the particular address for the next byte to be written
    Addr writeAddr(m_address + m_store_count);

    // Stores are assumed to be 1 byte-sized
    Request *req = new Request(writeAddr, 1, flags, m_tester_ptr->masterId(),
                               curTick(), m_pc);

    req->setContext(index);
    Packet::Command cmd;

    // 1 out of 8 chance, issue an atomic rather than a write
    // if ((random() & 0x7) == 0) {
    //     cmd = MemCmd::SwapReq;
    // } else {
    cmd = MemCmd::WriteReq;
    // }

    PacketPtr pkt = new Packet(req, cmd);
    uint8_t *writeData = new uint8_t[1];
    *writeData = m_value + m_store_count;
    pkt->dataDynamic(writeData);

    DPRINTF(RubyTest, "Seq write: index %d data 0x%x check 0x%x\n", index,
            *(pkt->getConstPtr<uint8_t>()), *writeData);

    // push the subblock onto the sender state.  The sequencer will
    // update the subblock on the return
    pkt->senderState = new SenderState(writeAddr, req->getSize());

    if (port->sendTimingReq(pkt)) {
        DPRINTF(RubyTest, "initiating action - successful\n");
        DPRINTF(RubyTest, "status before action update: %s\n",
                (TesterStatus_to_string(m_status)).c_str());
        m_status = TesterStatus_Action_Pending;
        DPRINTF(RubyTest, "Check %#x, State=Action_Pending\n", m_address);
    } else {
        // If the packet did not issue, must delete
        // Note: No need to delete the data, the packet destructor
        // will delete it
        delete pkt->senderState;
        delete pkt->req;
        delete pkt;

        DPRINTF(RubyTest, "failed to initiate action - sequencer not ready\n");
    }

    DPRINTF(RubyTest, "status after action update: %s\n",
            (TesterStatus_to_string(m_status)).c_str());
}

void
Check::initiateCheck()
{
    DPRINTF(RubyTest, "Initiating Check\n");
    assert(m_status == TesterStatus_Ready);

    int index = random_mt.random(0, m_num_readers - 1);
    MasterPort* port = m_tester_ptr->getReadableCpuPort(index);

    Request::Flags flags;

    // If necessary, make the request an instruction fetch
    if (m_tester_ptr->isInstOnlyCpuPort(index) ||
        (m_tester_ptr->isInstDataCpuPort(index) &&
         (random_mt.random(0, 0x1)))) {
        flags.set(Request::INST_FETCH);
    }

    // Checks are sized depending on the number of bytes written
    Request *req = new Request(m_address, CHECK_SIZE, flags,
                               m_tester_ptr->masterId(), curTick(), m_pc);

    req->setContext(index);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    uint8_t *dataArray = new uint8_t[CHECK_SIZE];
    pkt->dataDynamic(dataArray);

    DPRINTF(RubyTest, "Seq read: index %d\n", index);

    // push the subblock onto the sender state.  The sequencer will
    // update the subblock on the return
    pkt->senderState = new SenderState(m_address, req->getSize());

    if (port->sendTimingReq(pkt)) {
        DPRINTF(RubyTest, "initiating check - successful\n");
        DPRINTF(RubyTest, "status before check update: %s\n",
                TesterStatus_to_string(m_status).c_str());
        m_status = TesterStatus_Check_Pending;
        DPRINTF(RubyTest, "Check %#x, State=Check_Pending\n", m_address);
    } else {
        // If the packet did not issue, must delete
        // Note: No need to delete the data, the packet destructor
        // will delete it
        delete pkt->senderState;
        delete pkt->req;
        delete pkt;

        DPRINTF(RubyTest, "failed to initiate check - cpu port not ready\n");
    }

    DPRINTF(RubyTest, "status after check update: %s\n",
            TesterStatus_to_string(m_status).c_str());
}

void
Check::performCallback(NodeID proc, SubBlock* data, Cycles curTime)
{
    Addr address = data->getAddress();

    // This isn't exactly right since we now have multi-byte checks
    //  assert(getAddress() == address);

    assert(makeLineAddress(m_address) == makeLineAddress(address));
    assert(data != NULL);

    DPRINTF(RubyTest, "RubyTester Callback\n");
    debugPrint();

    if (m_status == TesterStatus_Action_Pending) {
        DPRINTF(RubyTest, "Action callback write value: %d, currently %d\n",
                (m_value + m_store_count), data->getByte(0));
        // Perform store one byte at a time
        data->setByte(0, (m_value + m_store_count));
        m_store_count++;
        if (m_store_count == CHECK_SIZE) {
            m_status = TesterStatus_Ready;
            DPRINTF(RubyTest, "Check %#x, State=Ready\n", m_address);
        } else {
            m_status = TesterStatus_Idle;
            DPRINTF(RubyTest, "Check %#x, State=Idle store_count: %d\n",
                    m_address, m_store_count);
        }
        DPRINTF(RubyTest, "Action callback return data now %d\n",
                data->getByte(0));
    } else if (m_status == TesterStatus_Check_Pending) {
        DPRINTF(RubyTest, "Check callback\n");
        // Perform load/check
        for (int byte_number=0; byte_number<CHECK_SIZE; byte_number++) {
            if (uint8_t(m_value + byte_number) != data->getByte(byte_number)) {
                panic("Action/check failure: proc: %d address: %#x data: %s "
                      "byte_number: %d m_value+byte_number: %d byte: %d %s"
                      "Time: %d\n",
                      proc, address, data, byte_number,
                      (int)m_value + byte_number,
                      (int)data->getByte(byte_number), *this, curTime);
            }
        }
        DPRINTF(RubyTest, "Action/check success\n");
        debugPrint();

        // successful check complete, increment complete
        m_tester_ptr->incrementCheckCompletions();

        m_status = TesterStatus_Idle;
        DPRINTF(RubyTest, "Check %#x, State=Idle\n", m_address);
        pickValue();

    } else {
        panic("Unexpected TesterStatus: %s proc: %d data: %s m_status: %s "
              "time: %d\n", *this, proc, data, m_status, curTime);
    }

    DPRINTF(RubyTest, "proc: %d, Address: 0x%x\n", proc,
            makeLineAddress(m_address));
    DPRINTF(RubyTest, "Callback done\n");
    debugPrint();
}

void
Check::changeAddress(Addr address)
{
    assert(m_status == TesterStatus_Idle || m_status == TesterStatus_Ready);
    m_status = TesterStatus_Idle;
    m_address = address;
    DPRINTF(RubyTest, "Check %#x, State=Idle\n", m_address);
    m_store_count = 0;
}

void
Check::pickValue()
{
    assert(m_status == TesterStatus_Idle);
    m_value = random_mt.random(0, 0xff); // One byte
    m_store_count = 0;
}

void
Check::pickInitiatingNode()
{
    assert(m_status == TesterStatus_Idle || m_status == TesterStatus_Ready);
    m_status = TesterStatus_Idle;
    m_initiatingNode = (random_mt.random(0, m_num_writers - 1));
    DPRINTF(RubyTest, "Check %#x, State=Idle, picked initiating node %d\n",
            m_address, m_initiatingNode);
    m_store_count = 0;
}

void
Check::print(std::ostream& out) const
{
    out << "["
        << m_address << ", value: "
        << (int)m_value << ", status: "
        << m_status << ", initiating node: "
        << m_initiatingNode << ", store_count: "
        << m_store_count
        << "]" << std::flush;
}

void
Check::debugPrint()
{
    DPRINTF(RubyTest,
        "[%#x, value: %d, status: %s, initiating node: %d, store_count: %d]\n",
        m_address, (int)m_value, TesterStatus_to_string(m_status).c_str(),
        m_initiatingNode, m_store_count);
}
