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

#ifndef __MEM_DRAMSYS_WRAPPER_HH__
#define __MEM_DRAMSYS_WRAPPER_HH__

#include <iostream>
#include <memory>

#include "DRAMSys/config/DRAMSysConfiguration.h"
#include "DRAMSys/simulation/DRAMSysRecordable.h"
#include "mem/abstract_mem.hh"
#include "params/DRAMSys.hh"
#include "sim/core.hh"
#include "systemc/core/kernel.hh"
#include "systemc/ext/core/sc_module_name.hh"

#include "systemc/ext/systemc"
#include "systemc/ext/tlm"
#include "systemc/ext/tlm_utils/simple_target_socket.h"
#include "systemc/tlm_port_wrapper.hh"

namespace gem5
{

namespace memory
{

class DRAMSysWrapper : public sc_core::sc_module
{
    friend class DRAMSys;

  public:
    SC_HAS_PROCESS(DRAMSysWrapper);
    DRAMSysWrapper(sc_core::sc_module_name name,
                   ::DRAMSys::Config::Configuration const &config,
                   bool recordable,
                   AddrRange range);

  private:
    static std::shared_ptr<::DRAMSys::DRAMSys>
    instantiateDRAMSys(bool recordable,
        ::DRAMSys::Config::Configuration const &config);

    tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload &payload,
                                       tlm::tlm_phase &phase,
                                       sc_core::sc_time &fwDelay);

    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload &payload,
                                       tlm::tlm_phase &phase,
                                       sc_core::sc_time &bwDelay);

    void b_transport(tlm::tlm_generic_payload &payload,
                     sc_core::sc_time &delay);

    unsigned int transport_dbg(tlm::tlm_generic_payload &trans);

    tlm_utils::simple_initiator_socket<DRAMSysWrapper> iSocket;
    tlm_utils::simple_target_socket<DRAMSysWrapper> tSocket;

    std::shared_ptr<::DRAMSys::DRAMSys> dramsys;

    AddrRange range;
};

} // namespace memory
} // namespace gem5

#endif // __MEM_DRAMSYS_WRAPPER_HH__
