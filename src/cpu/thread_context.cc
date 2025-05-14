/*
 * Copyright (c) 2012, 2016-2017 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "cpu/thread_context.hh"

#include <vector>

#include "arch/generic/vec_pred_reg.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "debug/Context.hh"
#include "debug/Quiesce.hh"
#include "mem/port.hh"
#include "params/BaseCPU.hh"
#include "sim/full_system.hh"

namespace gem5
{

void
ThreadContext::compare(ThreadContext *one, ThreadContext *two)
{
    const auto &regClasses = one->getIsaPtr()->regClasses();

    DPRINTF(Context, "Comparing thread contexts\n");

    // First loop through the integer registers.
    for (auto &id: *regClasses.at(IntRegClass)) {
        RegVal t1 = one->getReg(id);
        RegVal t2 = two->getReg(id);
        if (t1 != t2)
            panic("Int reg idx %d doesn't match, one: %#x, two: %#x",
                  id.index(), t1, t2);
    }

    // Then loop through the floating point registers.
    for (auto &id: *regClasses.at(FloatRegClass)) {
        RegVal t1 = one->getReg(id);
        RegVal t2 = two->getReg(id);
        if (t1 != t2)
            panic("Float reg idx %d doesn't match, one: %#x, two: %#x",
                  id.index(), t1, t2);
    }

    // Then loop through the vector registers.
    const auto *vec_class = regClasses.at(VecRegClass);
    std::vector<uint8_t> vec1(vec_class->regBytes());
    std::vector<uint8_t> vec2(vec_class->regBytes());
    for (auto &id: *regClasses.at(VecRegClass)) {
        one->getReg(id, vec1.data());
        two->getReg(id, vec2.data());
        if (vec1 != vec2) {
            panic("Vec reg idx %d doesn't match, one: %#x, two: %#x",
                  id.index(), vec_class->valString(vec1.data()),
                  vec_class->valString(vec2.data()));
        }
    }

    // Then loop through the predicate registers.
    const auto *vec_pred_class = regClasses.at(VecPredRegClass);
    std::vector<uint8_t> pred1(vec_pred_class->regBytes());
    std::vector<uint8_t> pred2(vec_pred_class->regBytes());
    for (auto &id: *regClasses.at(VecPredRegClass)) {
        one->getReg(id, pred1.data());
        two->getReg(id, pred2.data());
        if (pred1 != pred2) {
            panic("Pred reg idx %d doesn't match, one: %s, two: %s",
                  id.index(), vec_pred_class->valString(pred1.data()),
                  vec_pred_class->valString(pred2.data()));
        }
    }

    // Then loop through the matrix registers.
    const auto *mat_class = regClasses.at(MatRegClass);
    std::vector<uint8_t> mat1(mat_class->regBytes());
    std::vector<uint8_t> mat2(mat_class->regBytes());
    for (auto &id: *regClasses.at(MatRegClass)) {
        one->getReg(id, mat1.data());
        two->getReg(id, mat2.data());
        if (mat1 != mat2) {
            panic("Mat reg idx %d doesn't match, one: %#x, two: %#x",
                  id.index(), mat_class->valString(mat1.data()),
                  mat_class->valString(mat2.data()));
        }
    }

    for (int i = 0; i < regClasses.at(MiscRegClass)->numRegs(); ++i) {
        RegVal t1 = one->readMiscRegNoEffect(i);
        RegVal t2 = two->readMiscRegNoEffect(i);
        if (t1 != t2)
            panic("Misc reg idx %d doesn't match, one: %#x, two: %#x",
                  i, t1, t2);
    }

    // loop through the Condition Code registers.
    for (auto &id: *regClasses.at(CCRegClass)) {
        RegVal t1 = one->getReg(id);
        RegVal t2 = two->getReg(id);
        if (t1 != t2)
            panic("CC reg idx %d doesn't match, one: %#x, two: %#x",
                  id.index(), t1, t2);
    }
    if (one->pcState() != two->pcState())
        panic("PC state doesn't match.");
    int id1 = one->cpuId();
    int id2 = two->cpuId();
    if (id1 != id2)
        panic("CPU ids don't match, one: %d, two: %d", id1, id2);

    const ContextID cid1 = one->contextId();
    const ContextID cid2 = two->contextId();
    if (cid1 != cid2)
        panic("Context ids don't match, one: %d, two: %d", id1, id2);


}

void
ThreadContext::sendFunctional(PacketPtr pkt)
{
    const auto *port =
        dynamic_cast<const RequestPort *>(&getCpuPtr()->getDataPort());
    assert(port);
    port->sendFunctional(pkt);
}

void
ThreadContext::quiesce()
{
    getSystemPtr()->threads.quiesce(contextId());
}


void
ThreadContext::quiesceTick(Tick resume)
{
    getSystemPtr()->threads.quiesceTick(contextId(), resume);
}

RegVal
ThreadContext::getReg(const RegId &reg) const
{
    RegVal val;
    getReg(reg, &val);
    return val;
}

void
ThreadContext::setReg(const RegId &reg, RegVal val)
{
    setReg(reg, &val);
}

void
serialize(const ThreadContext &tc, CheckpointOut &cp)
{
    for (const auto *reg_class: tc.getIsaPtr()->regClasses()) {
        // MiscRegs are serialized elsewhere.
        if (reg_class->type() == MiscRegClass)
            continue;

        const size_t reg_bytes = reg_class->regBytes();
        const size_t reg_count = reg_class->numRegs();
        const size_t array_bytes = reg_bytes * reg_count;

        auto regs = std::make_unique<uint8_t[]>(array_bytes);
        auto *reg_ptr = regs.get();
        for (const auto &id: *reg_class) {
            tc.getReg(id, reg_ptr);
            reg_ptr += reg_bytes;
        }

        arrayParamOut(cp, std::string("regs.") + reg_class->name(), regs.get(),
                array_bytes);
    }

    tc.pcState().serialize(cp);

    // thread_num and cpu_id are deterministic from the config
}

void
unserialize(ThreadContext &tc, CheckpointIn &cp)
{
    for (const auto *reg_class: tc.getIsaPtr()->regClasses()) {
        // MiscRegs are serialized elsewhere.
        if (reg_class->type() == MiscRegClass)
            continue;

        const size_t reg_bytes = reg_class->regBytes();
        const size_t reg_count = reg_class->numRegs();
        const size_t array_bytes = reg_bytes * reg_count;

        auto regs = std::make_unique<uint8_t[]>(array_bytes);
        arrayParamIn(cp, std::string("regs.") + reg_class->name(), regs.get(),
                array_bytes);

        auto *reg_ptr = regs.get();
        for (const auto &id: *reg_class) {
            tc.setReg(id, reg_ptr);
            reg_ptr += reg_bytes;
        }
    }

    std::unique_ptr<PCStateBase> pc_state(tc.pcState().clone());
    pc_state->unserialize(cp);
    tc.pcState(*pc_state);

    // thread_num and cpu_id are deterministic from the config
}

void
takeOverFrom(ThreadContext &ntc, ThreadContext &otc)
{
    assert(ntc.getProcessPtr() == otc.getProcessPtr());

    ntc.setStatus(otc.status());
    ntc.copyArchRegs(&otc);
    ntc.setContextId(otc.contextId());
    ntc.setThreadId(otc.threadId());

    if (FullSystem)
        assert(ntc.getSystemPtr() == otc.getSystemPtr());

    otc.setStatus(ThreadContext::Halted);
}

} // namespace gem5
