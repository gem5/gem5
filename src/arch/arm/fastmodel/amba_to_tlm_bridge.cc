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

#include "arch/arm/fastmodel/amba_to_tlm_bridge.hh"

#include "base/amo.hh"
#include "params/AmbaToTlmBridge64.hh"
#include "pv/FarAtomicService.h"
#include "pv_userpayload_extension.h"
#include "systemc/tlm_bridge/sc_ext.hh"

namespace gem5
{

namespace {

// According to AbstractMemory::access in mem/abstract_mem.cc, the gem5 memory
// model would set original data into the packet buffer. However, the TLM
// request with atomic operation doesn't carry a valid buffer because the
// resource is all allocated in atomic extension. Preventing from segmentation
// fault, we allocate a random buffer and share it with all atomic transactions
// since we don't really care about the content of it.
uint8_t dummy_buffer[64] = {};

struct FarAtomicOpFunctor : public AtomicOpFunctor
{
    FarAtomicOpFunctor(far_atomic::FarAtomic *_fa) : fa(_fa) {}

    void
    operator() (uint8_t *p) override
    {
        fa->serviceWasFound();
        fa->doAtomicOperation(p);
    }

    AtomicOpFunctor *
    clone() override
    {
        return new FarAtomicOpFunctor(*this);
    }

    far_atomic::FarAtomic *fa;
};

}

namespace fastmodel
{

AmbaToTlmBridge64::AmbaToTlmBridge64(const sc_core::sc_module_name& name) :
    amba_pv::amba_pv_to_tlm_bridge<64>(name),
    targetProxy("target_proxy"),
    initiatorProxy("initiator_proxy"),
    tlmWrapper(initiatorProxy, std::string(name) + ".tlm", -1),
    ambaWrapper(amba_pv_s, std::string(name) + ".amba", -1)
{
    targetProxy.register_b_transport(this, &AmbaToTlmBridge64::bTransport);
    targetProxy.register_get_direct_mem_ptr(
        this, &AmbaToTlmBridge64::getDirectMemPtr);
    targetProxy.register_transport_dbg(this, &AmbaToTlmBridge64::transportDbg);
    initiatorProxy.register_invalidate_direct_mem_ptr(
        this, &AmbaToTlmBridge64::invalidateDirectMemPtr);
    tlm_m(targetProxy);
}

Port &
AmbaToTlmBridge64::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "tlm")
        return tlmWrapper;
    else if (if_name == "amba")
        return ambaWrapper;
    else
        return amba_pv::amba_pv_to_tlm_bridge<64>::gem5_getPort(if_name, idx);
}

void
AmbaToTlmBridge64::bTransport(amba_pv::amba_pv_transaction &trans,
                              sc_core::sc_time &t)
{
    maybeSetupAtomicExtension(trans);
    setupControlExtension(trans);
    return initiatorProxy->b_transport(trans, t);
}

bool
AmbaToTlmBridge64::getDirectMemPtr(amba_pv::amba_pv_transaction &trans,
                                   tlm::tlm_dmi &dmi_data)
{
    return initiatorProxy->get_direct_mem_ptr(trans, dmi_data);
}

unsigned int
AmbaToTlmBridge64::transportDbg(amba_pv::amba_pv_transaction &trans)
{
    return initiatorProxy->transport_dbg(trans);
}

void
AmbaToTlmBridge64::invalidateDirectMemPtr(sc_dt::uint64 start_range,
                                          sc_dt::uint64 end_range)
{
    targetProxy->invalidate_direct_mem_ptr(start_range, end_range);
}

void
AmbaToTlmBridge64::maybeSetupAtomicExtension(
    amba_pv::amba_pv_transaction &trans)
{
    Gem5SystemC::AtomicExtension *atomic_ex = nullptr;
    trans.get_extension(atomic_ex);
    if (atomic_ex)
        return;

    pv_userpayload_extension *user_ex = nullptr;
    trans.get_extension(user_ex);
    if (!user_ex)
        return;

    pv::UserPayloadBase *upb = user_ex->get_user_payload();
    uint32_t appid = upb->get_appID();
    if (appid != pv::UserPayloadBase::SERVICE_REQUEST)
        return;

    std::pair<void *, std::size_t> u_data = user_ex->get_user_data();
    far_atomic::FarAtomic *fa = static_cast<far_atomic::FarAtomic *>(
        u_data.first);

    // Correct the request size manually and give it a dummy buffer preventing
    // from segmentation fault.
    fatal_if(
        fa->getDataValueSizeInBytes() > sizeof(dummy_buffer),
        "atomic operation(%d) is larger than dummy buffer(%d)",
        fa->getDataValueSizeInBytes(), sizeof(dummy_buffer));
    trans.set_data_length(fa->getDataValueSizeInBytes());
    trans.set_data_ptr(dummy_buffer);

    // The return value would store in the extension. We don't need to specify
    // returnRequired here.
    atomic_ex = new Gem5SystemC::AtomicExtension(
        std::make_shared<FarAtomicOpFunctor>(fa), false);
    if (trans.has_mm())
        trans.set_auto_extension(atomic_ex);
    else
        trans.set_extension(atomic_ex);
}

void
AmbaToTlmBridge64::setupControlExtension(amba_pv::amba_pv_transaction &trans)
{
    Gem5SystemC::ControlExtension *control_ex = nullptr;
    trans.get_extension(control_ex);
    if (control_ex) {
        return;
    }

    amba_pv::amba_pv_extension *amba_ex = nullptr;
    trans.get_extension(amba_ex);
    if (!amba_ex) {
        return;
    }

    control_ex = new Gem5SystemC::ControlExtension();

    control_ex->setPrivileged(amba_ex->is_privileged());
    control_ex->setSecure(!amba_ex->is_non_secure());
    control_ex->setInstruction(amba_ex->is_instruction());

    if (trans.has_mm()) {
        trans.set_auto_extension(control_ex);
    } else {
        trans.set_extension(control_ex);
    }
}

} // namespace fastmodel

fastmodel::AmbaToTlmBridge64 *
AmbaToTlmBridge64Params::create() const
{
    return new fastmodel::AmbaToTlmBridge64(name.c_str());
}

} // namespace gem5
