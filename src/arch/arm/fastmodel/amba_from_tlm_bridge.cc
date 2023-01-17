/*
 * Copyright 2019 Google, Inc.
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

#include "arch/arm/fastmodel/amba_from_tlm_bridge.hh"

#include "params/AmbaFromTlmBridge64.hh"
#include "pv_userpayload_extension.h"
#include "systemc/tlm_bridge/sc_ext.hh"

namespace gem5
{

namespace fastmodel
{

AmbaFromTlmBridge64::AmbaFromTlmBridge64(
    const AmbaFromTlmBridge64Params &params,
    const sc_core::sc_module_name& name) :
    amba_pv::amba_pv_from_tlm_bridge<64>(name),
    targetProxy("target_proxy"),
    initiatorProxy("initiator_proxy"),
    tlmWrapper(targetProxy, std::string(name) + ".tlm", -1),
    ambaWrapper(amba_pv_m, std::string(name) + ".amba", -1)
{
    targetProxy.register_b_transport(this, &AmbaFromTlmBridge64::bTransport);
    targetProxy.register_get_direct_mem_ptr(
        this, &AmbaFromTlmBridge64::getDirectMemPtr);
    targetProxy.register_transport_dbg(this, &AmbaFromTlmBridge64::transportDbg);
    initiatorProxy.register_invalidate_direct_mem_ptr(
        this, &AmbaFromTlmBridge64::invalidateDirectMemPtr);
    initiatorProxy(tlm_s);
}

Port &
AmbaFromTlmBridge64::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "tlm") {
        return tlmWrapper;
    } else if (if_name == "amba") {
        return ambaWrapper;
    } else {
        return amba_pv::amba_pv_from_tlm_bridge<64>::gem5_getPort(
                if_name, idx);
    }
}

void
AmbaFromTlmBridge64::bTransport(amba_pv::amba_pv_transaction &trans,
                                sc_core::sc_time &t)
{
    syncControlExtension(trans);
    return initiatorProxy->b_transport(trans, t);
}

bool
AmbaFromTlmBridge64::getDirectMemPtr(amba_pv::amba_pv_transaction &trans,
                                   tlm::tlm_dmi &dmi_data)
{
    return initiatorProxy->get_direct_mem_ptr(trans, dmi_data);
}

unsigned int
AmbaFromTlmBridge64::transportDbg(amba_pv::amba_pv_transaction &trans)
{
    syncControlExtension(trans);
    return initiatorProxy->transport_dbg(trans);
}

void
AmbaFromTlmBridge64::invalidateDirectMemPtr(sc_dt::uint64 start_range,
                                          sc_dt::uint64 end_range)
{
    targetProxy->invalidate_direct_mem_ptr(start_range, end_range);
}

void
AmbaFromTlmBridge64::syncControlExtension(amba_pv::amba_pv_transaction &trans)
{
    Gem5SystemC::ControlExtension *control_ex = nullptr;
    trans.get_extension(control_ex);
    if (!control_ex) {
        return;
    }

    amba_pv::amba_pv_extension *amba_ex = nullptr;
    trans.get_extension(amba_ex);
    if (!amba_ex) {
        return;
    }

    amba_ex->set_privileged(control_ex->isPrivileged());
    amba_ex->set_non_secure(!control_ex->isSecure());
    amba_ex->set_instruction(control_ex->isInstruction());
}

} // namespace fastmodel
} // namespace gem5
