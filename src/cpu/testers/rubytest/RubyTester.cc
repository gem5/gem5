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

#include "cpu/testers/rubytest/RubyTester.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/testers/rubytest/Check.hh"
#include "debug/RubyTest.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

RubyTester::RubyTester(const Params &p)
  : ClockedObject(p),
    checkStartEvent([this]{ wakeup(); }, "RubyTester tick",
                    false, Event::CPU_Tick_Pri),
    _requestorId(p.system->getRequestorId(this)),
    m_checkTable_ptr(nullptr),
    m_num_cpus(p.num_cpus),
    m_checks_to_complete(p.checks_to_complete),
    m_deadlock_threshold(p.deadlock_threshold),
    m_num_writers(0),
    m_num_readers(0),
    m_wakeup_frequency(p.wakeup_frequency),
    m_check_flush(p.check_flush),
    m_num_inst_only_ports(p.port_cpuInstPort_connection_count),
    m_num_inst_data_ports(p.port_cpuInstDataPort_connection_count)
{
    m_checks_completed = 0;

    //
    // Create the requested inst and data ports and place them on the
    // appropriate read and write port lists.  The reason for the subtle
    // difference between inst and data ports vs. read and write ports is
    // from the tester's perspective, it only needs to know whether a port
    // supports reads (checks) or writes (actions).  Meanwhile, the protocol
    // controllers have data ports (support read and writes) or inst ports
    // (support only reads).
    // Note: the inst ports are the lowest elements of the readPort vector,
    // then the data ports are added to the readPort vector
    //
    int idx = 0;
    for (int i = 0; i < p.port_cpuInstPort_connection_count; ++i) {
        readPorts.push_back(new CpuPort(csprintf("%s-instPort%d", name(), i),
                                        this, i, idx));
        idx++;
    }
    for (int i = 0; i < p.port_cpuInstDataPort_connection_count; ++i) {
        CpuPort *port = new CpuPort(csprintf("%s-instDataPort%d", name(), i),
                                    this, i, idx);
        readPorts.push_back(port);
        writePorts.push_back(port);
        idx++;
    }
    for (int i = 0; i < p.port_cpuDataPort_connection_count; ++i) {
        CpuPort *port = new CpuPort(csprintf("%s-dataPort%d", name(), i),
                                    this, i, idx);
        readPorts.push_back(port);
        writePorts.push_back(port);
        idx++;
    }

    // add the check start event to the event queue
    schedule(checkStartEvent, 1);
}

RubyTester::~RubyTester()
{
    delete m_checkTable_ptr;
    // Only delete the readPorts since the writePorts are just a subset
    for (int i = 0; i < readPorts.size(); i++)
        delete readPorts[i];
}

void
RubyTester::init()
{
    assert(writePorts.size() > 0 && readPorts.size() > 0);

    m_last_progress_vector.resize(m_num_cpus);
    m_num_writers = writePorts.size();
    m_num_readers = readPorts.size();
    assert(m_num_readers == m_num_cpus);

    m_checkTable_ptr = new CheckTable(m_num_writers, m_num_readers, this);
}

Port &
RubyTester::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "cpuInstPort" && if_name != "cpuInstDataPort" &&
        if_name != "cpuDataPort") {
        // pass it along to our super class
        return ClockedObject::getPort(if_name, idx);
    } else {
        if (if_name == "cpuInstPort") {
            if (idx > m_num_inst_only_ports) {
                panic("RubyTester::getPort: unknown inst port %d\n",
                      idx);
            }
            //
            // inst ports map to the lowest readPort elements
            //
            return *readPorts[idx];
        } else if (if_name == "cpuInstDataPort") {
            if (idx > m_num_inst_data_ports) {
                panic("RubyTester::getPort: unknown inst+data port %d\n",
                      idx);
            }
            int read_idx = idx + m_num_inst_only_ports;
            //
            // inst+data ports map to the next readPort elements
            //
            return *readPorts[read_idx];
        } else {
            assert(if_name == "cpuDataPort");
            //
            // data only ports map to the final readPort elements
            //
            if (idx > (static_cast<int>(readPorts.size()) -
                       (m_num_inst_only_ports + m_num_inst_data_ports))) {
                panic("RubyTester::getPort: unknown data port %d\n",
                      idx);
            }
            int read_idx = idx + m_num_inst_only_ports + m_num_inst_data_ports;
            return *readPorts[read_idx];
        }
        // Note: currently the Ruby Tester does not support write only ports
        // but that could easily be added here
    }
}

bool
RubyTester::CpuPort::recvTimingResp(PacketPtr pkt)
{
    // retrieve the subblock and call hitCallback
    RubyTester::SenderState* senderState =
        safe_cast<RubyTester::SenderState*>(pkt->senderState);
    ruby::SubBlock& subblock = senderState->subBlock;

    tester->hitCallback(globalIdx, &subblock);

    // Now that the tester has completed, delete the senderState
    // (includes sublock) and the packet, then return
    delete pkt->senderState;
    delete pkt;
    return true;
}

bool
RubyTester::isInstOnlyCpuPort(int idx)
{
    return idx < m_num_inst_only_ports;
}

bool
RubyTester::isInstDataCpuPort(int idx)
{
    return ((idx >= m_num_inst_only_ports) &&
            (idx < (m_num_inst_only_ports + m_num_inst_data_ports)));
}

RequestPort*
RubyTester::getReadableCpuPort(int idx)
{
    assert(idx >= 0 && idx < readPorts.size());

    return readPorts[idx];
}

RequestPort*
RubyTester::getWritableCpuPort(int idx)
{
    assert(idx >= 0 && idx < writePorts.size());

    return writePorts[idx];
}

void
RubyTester::hitCallback(ruby::NodeID proc, ruby::SubBlock* data)
{
    DPRINTF(RubyTest, "completed request for proc: %d", proc);
    DPRINTFR(RubyTest, " addr: 0x%x, size: %d, data: ",
            data->getAddress(), data->getSize());
    for (int byte = 0; byte < data->getSize(); byte++) {
        DPRINTFR(RubyTest, "%d ", data->getByte(byte));
    }
    DPRINTFR(RubyTest, "\n");

    // This tells us our store has 'completed' or for a load gives us
    // back the data to make the check
    Check* check_ptr = m_checkTable_ptr->getCheck(data->getAddress());
    assert(check_ptr != NULL);
    check_ptr->performCallback(proc, data, curCycle());
}

void
RubyTester::wakeup()
{
    if (m_checks_completed < m_checks_to_complete) {
        // Try to perform an action or check
        Check* check_ptr = m_checkTable_ptr->getRandomCheck();
        assert(check_ptr != NULL);
        check_ptr->initiate(curCycle());

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
    Cycles current_time = curCycle();
    for (int processor = 0; processor < size; processor++) {
        if (m_last_progress_vector[processor].empty()) {
            DPRINTF(RubyTest, "Processor %d is idle now, skip\n", processor);
            continue;
        }
        for (const auto& [addr, last_cycle] :
            m_last_progress_vector[processor]) {
            if ((current_time - last_cycle) > m_deadlock_threshold) {
                panic("Possible deadlock detected:\n"
                      "  Processor: %d\n"
                      "  Addr: %#x\n"
                      "  Current Time: %d\n"
                      "  Last Progress Time: %d\n"
                      "  Difference: %d cycles\n",
                      processor, addr, current_time,
                      last_cycle, current_time - last_cycle);
            }
        }
    }
}

void
RubyTester::updateProgress(int index, Addr address, Cycles current_time){
    m_last_progress_vector[index].emplace(address, current_time);
    DPRINTF(RubyTest, "Update progress: index: %d \
        address: %#x current_time: %d\n",
        index, address, current_time);
}

void
RubyTester::eraseProgress(int index, Addr address){
    auto i = m_last_progress_vector[index].find(address);
    assert(i != m_last_progress_vector[index].end());
    m_last_progress_vector[index].erase(i);
    DPRINTF(RubyTest, "Erase progress: index: %d address: %#x\n",
                index, address);
}

void
RubyTester::print(std::ostream& out) const
{
    out << "[RubyTester]" << std::endl;
}

} // namespace gem5
