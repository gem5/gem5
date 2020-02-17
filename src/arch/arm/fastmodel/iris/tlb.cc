/*
 * Copyright 2019 Google Inc.
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

#include "arch/arm/fastmodel/iris/tlb.hh"

#include "arch/arm/fastmodel/iris/thread_context.hh"
#include "arch/generic/debugfaults.hh"
#include "params/IrisTLB.hh"
#include "sim/faults.hh"

Fault
Iris::TLB::translateFunctional(
        const RequestPtr &req, ::ThreadContext *tc, Mode mode)
{
    auto *itc = dynamic_cast<Iris::ThreadContext *>(tc);
    panic_if(!itc, "Failed to cast to Iris::ThreadContext *");

    Addr vaddr = req->getVaddr();
    Addr paddr = 0;
    bool success = itc->translateAddress(paddr, vaddr);
    if (!success) {
        return std::make_shared<GenericISA::M5PanicFault>(
                "Failed translation");
    } else {
        req->setPaddr(paddr);
        return NoFault;
    }
}

Fault
Iris::TLB::translateAtomic(
        const RequestPtr &req, ::ThreadContext *tc, Mode mode)
{
    return translateFunctional(req, tc, mode);
}

void
Iris::TLB::translateTiming(const RequestPtr &req, ::ThreadContext *tc,
        Translation *translation, Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

Iris::TLB *
IrisTLBParams::create()
{
    return new Iris::TLB(this);
}
