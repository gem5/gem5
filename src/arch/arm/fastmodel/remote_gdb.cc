/* * Copyright 2022 Google, Inc.
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

#include "arch/arm/fastmodel/remote_gdb.hh"

#include "arch/arm/fastmodel/iris/thread_context.hh"
#include "arch/arm/utility.hh"
#include "base/trace.hh"
#include "debug/GDBAcc.hh"

namespace gem5 {

using namespace ArmISA;

namespace fastmodel {

void
FastmodelRemoteGDB::AArch64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");

    for (int i = 0; i < 31; ++i)
        context->setReg(int_reg::x(i), r.x[i]);
    auto pc_state = context->pcState().as<PCState>();
    pc_state.set(r.pc);
    context->pcState(pc_state);
    context->setMiscRegNoEffect(MISCREG_CPSR, r.cpsr);
    // Update the stack pointer. This should be done after
    // updating CPSR/PSTATE since that might affect how SPX gets
    // mapped.
    context->setReg(int_reg::Spx, r.spx);

    // Remove the vector registers update in FastmodelRemoteGDB since it's not
    // implemented in iris::ThreadContext.
    warn("Skip update vector registers in remotegdb\n");

    context->setMiscRegNoEffect(MISCREG_FPSR, r.fpsr);
    context->setMiscRegNoEffect(MISCREG_FPCR, r.fpcr);
}

FastmodelRemoteGDB::FastmodelRemoteGDB(System *_system,
        ListenSocketConfig _listen_config)
    : gem5::ArmISA::RemoteGDB(_system, _listen_config), regCache64(this)
{
}

bool
FastmodelRemoteGDB::readBlob(Addr vaddr, size_t size, char *data)
{
    auto tc = dynamic_cast<Iris::ThreadContext *>(context());
    panic_if(!tc,
             "FastmodelRemoteGdb can only work on Iris::ThreadContext");
    tc->readMemWithCurrentMsn(vaddr, size, data);
    return true;
}

bool
FastmodelRemoteGDB::writeBlob(Addr vaddr, size_t size, const char *data)
{
    auto tc = dynamic_cast<Iris::ThreadContext *>(context());
    panic_if(!tc,
             "FastmodelRemoteGdb can only work on Iris::ThreadContext");
    tc->writeMemWithCurrentMsn(vaddr, size, data);
    return true;
}

BaseGdbRegCache*
FastmodelRemoteGDB::gdbRegs()
{
    if (inAArch64(context()))
        return &regCache64;
    else
        return &regCache32;
}

}  // namespace fastmodel
}  // namespace gem5
