/*
 * Copyright (c) 2022 Fraunhofer IESE
 * All rights reserved
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

#include "dramsys_wrapper.hh"

namespace gem5
{

namespace memory
{

DRAMSysWrapper::DRAMSysWrapper(
    sc_core::sc_module_name name,
    DRAMSysConfiguration::Configuration const &config,
    bool recordable,
    AddrRange range) :
    sc_core::sc_module(name),
    dramsys(instantiateDRAMSys(recordable, config)),
    range(range)
{
    tSocket.register_nb_transport_fw(this, &DRAMSysWrapper::nb_transport_fw);
    tSocket.register_transport_dbg(this, &DRAMSysWrapper::transport_dbg);
    iSocket.register_nb_transport_bw(this, &DRAMSysWrapper::nb_transport_bw);
    iSocket.bind(dramsys->tSocket);

    // Register a callback to compensate for the destructor not
    // being called.
    registerExitCallback(
        [this]()
        {
            // Workaround for BUG GEM5-1233
            sc_gem5::Kernel::stop();
        });
}

std::shared_ptr<::DRAMSys>
DRAMSysWrapper::instantiateDRAMSys(
    bool recordable,
    DRAMSysConfiguration::Configuration const &config)
{
    return recordable
        ? std::make_shared<::DRAMSysRecordable>("DRAMSys", config)
        : std::make_shared<::DRAMSys>("DRAMSys", config);
}

tlm::tlm_sync_enum DRAMSysWrapper::nb_transport_fw(
    tlm::tlm_generic_payload &payload,
    tlm::tlm_phase &phase,
    sc_core::sc_time &fwDelay)
{
    // Subtract base address offset
    payload.set_address(payload.get_address() - range.start());

    return iSocket->nb_transport_fw(payload, phase, fwDelay);
}

tlm::tlm_sync_enum DRAMSysWrapper::nb_transport_bw(
    tlm::tlm_generic_payload &payload,
    tlm::tlm_phase &phase,
    sc_core::sc_time &bwDelay)
{
    return tSocket->nb_transport_bw(payload, phase, bwDelay);
}

unsigned int DRAMSysWrapper::transport_dbg(tlm::tlm_generic_payload &trans)
{
    // Subtract base address offset
    trans.set_address(trans.get_address() - range.start());

    return iSocket->transport_dbg(trans);
}

} // namespace memory
} // namespace gem5
