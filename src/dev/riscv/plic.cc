/*
 * Copyright (c) 2021 Huawei International
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

#include "dev/riscv/plic.hh"

#include <algorithm>

#include "cpu/base.hh"
#include "debug/Plic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/Plic.hh"
#include "params/PlicBase.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

Plic::Plic(const Params &params)
    : PlicBase(params),
      system(params.system),
      nSrc(params.n_src),
      registers(params.name, pioAddr, this),
      update([this] { updateOutput(); }, name() + ".update")
{
    fatal_if(params.hart_config != "" && params.n_contexts != 0,
             "the hart_config and n_contexts can't be set simultaneously");

    if (params.n_contexts != 0) {
        initContextFromNContexts(params.n_contexts);
    }
    if (params.hart_config != "") {
        initContextFromHartConfig(params.hart_config);
    }
}

void
Plic::post(int src_id)
{
    // Sanity check
    assert(src_id < nSrc && src_id >= 0);

    // Update pending bit
    int src_index = src_id >> 5;
    int src_offset = src_id & 0x1F;

    uint32_t &pending = registers.pending[src_index].get();
    std::bitset<32> pending_bits(pending);
    pending_bits[src_offset] = 1;
    pending = (uint32_t)pending_bits.to_ulong();

    // Update states
    pendingPriority[src_id] = registers.priority[src_id].get();
    for (int i = 0; i < contextConfigs.size(); i++) {
        bool enabled = bits(registers.enable[i][src_index].get(), src_offset);
        effPriority[i][src_id] = enabled ? pendingPriority[src_id] : 0;
    }
    DPRINTF(Plic, "Int post request - source: %#x, current priority: %#x\n",
            src_id, pendingPriority[src_id]);

    // Propagate output changes
    propagateOutput();
}

void
Plic::clear(int src_id)
{
    // Sanity check
    assert(src_id < nSrc);
    assert(src_id >= 0);

    // Update pending bit
    int src_index = src_id >> 5;
    int src_offset = src_id & 0x1F;
    uint32_t &pending = registers.pending[src_index].get();
    std::bitset<32> pending_bits(pending);
    pending_bits[src_offset] = 0;
    pending = (uint32_t)pending_bits.to_ulong();

    // Update states
    pendingPriority[src_id] = 0;
    for (int i = 0; i < contextConfigs.size(); i++) {
        effPriority[i][src_id] = 0;
    }
    DPRINTF(Plic, "Int clear request - source: %#x, current priority: %#x\n",
            src_id, pendingPriority[src_id]);

    // Propagate output changes
    propagateOutput();
}

Tick
Plic::read(PacketPtr pkt)
{
    // Check for atomic operation
    bool is_atomic = pkt->isAtomicOp() && pkt->cmd == MemCmd::SwapReq;
    DPRINTF(Plic, "Read request - addr: %#x, size: %#x, atomic:%d\n",
            pkt->getAddr(), pkt->getSize(), is_atomic);

    // Perform register read
    registers.read(pkt->getAddr(), pkt->getPtr<void>(), pkt->getSize());

    if (is_atomic) {
        // Perform atomic operation
        (*(pkt->getAtomicOp()))(pkt->getPtr<uint8_t>());
        return write(pkt);
    } else {
        pkt->makeResponse();
        return pioDelay;
    }
}

Tick
Plic::write(PacketPtr pkt)
{
    DPRINTF(Plic, "Write request - addr: %#x, size: %#x\n", pkt->getAddr(),
            pkt->getSize());

    // Perform register write
    registers.write(pkt->getAddr(), pkt->getPtr<void>(), pkt->getSize());

    // Propagate output changes
    propagateOutput();

    // Apply threshold changes
    updateInt();

    pkt->makeResponse();
    return pioDelay;
}

void
Plic::init()
{
    // Number of 32-bit pending registesrs where
    // each bit correspondings to one interrupt source
    nSrc32 = divCeil(nSrc, 32);

    // Setup register bank
    registers.init();

    // Setup internal states
    pendingPriority.resize(nSrc, 0x0);
    for (int i = 0; i < contextConfigs.size(); i++) {
        std::vector<uint32_t> context_priority(nSrc, 0x0);
        effPriority.push_back(context_priority);
    }
    lastID.resize(contextConfigs.size(), 0x0);

    // Setup outputs
    output = PlicOutput{ std::vector<uint32_t>(contextConfigs.size(), 0x0),
                         std::vector<uint32_t>(contextConfigs.size(), 0x0) };

    DPRINTF(Plic,
            "Device init - %d contexts, %d sources, %d pending registers\n",
            contextConfigs.size(), nSrc, nSrc32);

    BasicPioDevice::init();
}

void
Plic::PlicRegisters::init()
{
    using namespace std::placeholders;

    // Calculate reserved space size
    const size_t reserve0_size = pendingStart - plic->nSrc * 4;
    reserved.emplace_back("reserved0", reserve0_size);
    const size_t reserve1_size = enableStart - pendingStart - plic->nSrc32 * 4;
    reserved.emplace_back("reserved1", reserve1_size);
    const size_t reserve2_size = thresholdStart - enableStart -
                                 plic->contextConfigs.size() * enablePadding;
    reserved.emplace_back("reserved2", reserve2_size);
    const size_t reserve3_size =
        plic->pioSize - thresholdStart -
        plic->contextConfigs.size() * thresholdPadding;
    reserved.emplace_back("reserved3", reserve3_size);

    // Sanity check
    assert(plic->pioSize >=
           thresholdStart + plic->contextConfigs.size() * thresholdPadding);
    assert((int)plic->pioSize <= maxBankSize);

    // Calculate hole sizes
    const size_t enable_hole_size = enablePadding - plic->nSrc32 * 4;
    const size_t claim_hole_size = thresholdPadding - 0x8;

    // Initialize registers
    for (int i = 0; i < plic->nSrc; i++) {
        priority.emplace_back(std::string("priority") + std::to_string(i), 0);
    }
    for (int i = 0; i < plic->nSrc32; i++) {
        pending.emplace_back(std::string("pending") + std::to_string(i), 0);
    }
    for (int i = 0; i < plic->contextConfigs.size(); i++) {
        enable.push_back(std::vector<Register32>());
        for (int j = 0; j < plic->nSrc32; j++) {
            enable[i].emplace_back(std::string("enable") + std::to_string(i) +
                                       "_" + std::to_string(j),
                                   0);
        }
        enable_holes.emplace_back(
            std::string("enable_hole") + std::to_string(i), enable_hole_size);

        threshold.emplace_back(std::string("threshold") + std::to_string(i),
                               0);
        claim.emplace_back(std::string("claim") + std::to_string(i), 0);
        claim_holes.emplace_back(std::string("claim_hole") + std::to_string(i),
                                 claim_hole_size);
    }

    // Add registers to bank
    // Priority
    for (int i = 0; i < plic->nSrc; i++) {
        auto write_cb = std::bind(&Plic::writePriority, plic, _1, _2, i);
        priority[i].writer(write_cb);
        addRegister(priority[i]);
    }
    addRegister(reserved[0]);

    // Pending
    for (int i = 0; i < plic->nSrc32; i++) {
        pending[i].readonly();
        addRegister(pending[i]);
    }
    addRegister(reserved[1]);

    // Enable
    for (int i = 0; i < plic->contextConfigs.size(); i++) {
        for (int j = 0; j < plic->nSrc32; j++) {
            auto write_cb = std::bind(&Plic::writeEnable, plic, _1, _2, j, i);
            enable[i][j].writer(write_cb);
            addRegister(enable[i][j]);
        }
        addRegister(enable_holes[i]);
    }
    addRegister(reserved[2]);

    // Threshold and claim
    for (int i = 0; i < plic->contextConfigs.size(); i++) {
        auto threshold_cb = std::bind(&Plic::writeThreshold, plic, _1, _2, i);
        threshold[i].writer(threshold_cb);
        auto read_cb = std::bind(&Plic::readClaim, plic, _1, i);
        auto write_cb = std::bind(&Plic::writeClaim, plic, _1, _2, i);
        claim[i].reader(read_cb).writer(write_cb);
        addRegister(threshold[i]);
        addRegister(claim[i]);
        addRegister(claim_holes[i]);
    }
    addRegister(reserved[3]);
}

void
Plic::writePriority(Register32 &reg, const uint32_t &data, const int src_id)
{
    reg.update(data);

    // Calculate indices
    int src_index = src_id >> 5;
    int src_offset = src_id & 0x1F;

    // Update states
    bool pending = bits(registers.pending[src_index].get(), src_offset);
    pendingPriority[src_id] = pending ? reg.get() : 0;
    for (int i = 0; i < contextConfigs.size(); i++) {
        bool enabled = bits(registers.enable[i][src_index].get(), src_offset);
        effPriority[i][src_id] = enabled ? pendingPriority[src_id] : 0;
    }

    DPRINTF(Plic, "Priority updated - src: %d, val: %d\n", src_id, reg.get());
}

void
Plic::writeEnable(Register32 &reg, const uint32_t &data, const int src32_id,
                  const int context_id)
{
    reg.update(data);

    for (int i = 0; i < 32; i++) {
        int src_id = (src32_id << 5) + i;
        if (src_id < nSrc) {
            effPriority[context_id][src_id] =
                bits(reg.get(), i) ? pendingPriority[src_id] : 0;
        }
    }
    DPRINTF(Plic, "Enable updated - context: %d, src32: %d, val: %#x\n",
            context_id, src32_id, reg.get());
}

void
Plic::writeThreshold(Register32 &reg, const uint32_t &data,
                     const int context_id)
{
    reg.update(data);

    DPRINTF(Plic, "Threshold updated - context: %d, val: %d\n", context_id,
            reg.get());
}

uint32_t
Plic::readClaim(Register32 &reg, const int context_id)
{
    if (lastID[context_id] == 0) {
        // Calculate indices
        uint32_t max_int_id = output.maxID[context_id];
        int src_index = max_int_id >> 5;
        int src_offset = max_int_id & 0x1F;

        // Check pending bits
        if (bits(registers.pending[src_index].get(), src_offset)) {
            lastID[context_id] = max_int_id;
            DPRINTF(Plic, "Claim success - context: %d, interrupt ID: %d\n",
                    context_id, max_int_id);
            clear(max_int_id);
            reg.update(max_int_id);
            return reg.get();
        } else {
            DPRINTF(Plic,
                    "Claim already cleared - context: %d, interrupt ID: %d\n",
                    context_id, max_int_id);
            return 0;
        }
    } else {
        warn("PLIC claim repeated (not completed) - context: %d, last: %d",
             context_id, lastID[context_id]);
        return lastID[context_id];
    }
}

void
Plic::writeClaim(Register32 &reg, const uint32_t &data, const int context_id)
{
    reg.update(data);

    /**
     * Plic spec states that this error should be silently ignored.
     * However, this is not supposed to happen.
     */
    assert(lastID[context_id] == reg.get());
    lastID[context_id] = 0;
    DPRINTF(Plic, "Complete - context: %d, interrupt ID: %d\n", context_id,
            reg.get());
    updateInt();
}

void
Plic::propagateOutput()
{
    // Calculate new output
    PlicOutput new_output{ std::vector<uint32_t>(contextConfigs.size(), 0x0),
                           std::vector<uint32_t>(contextConfigs.size(), 0x0) };
    uint32_t max_id;
    uint32_t max_priority;
    for (int i = 0; i < contextConfigs.size(); i++) {
        max_id = max_element(effPriority[i].begin(), effPriority[i].end()) -
                 effPriority[i].begin();
        max_priority = effPriority[i][max_id];
        new_output.maxID[i] = max_id;
        new_output.maxPriority[i] = max_priority;
    }

    // Add new output to outputQueue
    Tick next_update = curTick() + cyclesToTicks(Cycles(3));
    if (outputQueue.find(next_update) != outputQueue.end()) {
        outputQueue[next_update] = new_output;
    } else {
        outputQueue.insert({ next_update, new_output });
    }

    // Schedule next update event
    if (!update.scheduled()) {
        DPRINTF(Plic, "Update scheduled - tick: %d\n", next_update);
        schedule(update, next_update);
    }
}

void
Plic::initContextFromNContexts(int n_contexts)
{
    contextConfigs.reserve(n_contexts);
    for (uint32_t i = 0; i < (uint32_t)n_contexts; i += 2) {
        contextConfigs.emplace_back((i >> 1), ExceptionCode::INT_EXT_MACHINE);
        contextConfigs.emplace_back((i >> 1), ExceptionCode::INT_EXT_SUPER);
    }
}

void
Plic::initContextFromHartConfig(const std::string &hart_config)
{
    contextConfigs.reserve(hart_config.size());
    uint32_t hart_id = 0;
    for (char c : hart_config) {
        switch (c) {
        case ',':
            hart_id++;
            break;
        case 'M':
            contextConfigs.emplace_back(hart_id,
                                        ExceptionCode::INT_EXT_MACHINE);
            break;
        case 'S':
            contextConfigs.emplace_back(hart_id, ExceptionCode::INT_EXT_SUPER);
            break;
        default:
            fatal("hart_config should not contains the value: %c", c);
            break;
        }
    }
}

void
Plic::updateOutput()
{
    DPRINTF(Plic, "Update triggered\n");
    // Set current output to new output
    output = outputQueue.begin()->second;
    outputQueue.erase(outputQueue.begin()->first);

    // Schedule next update event (if any)
    if (!outputQueue.empty()) {
        DPRINTF(Plic, "Update scheduled - tick: %d\n",
                outputQueue.begin()->first);
        schedule(update, outputQueue.begin()->first);
    }

    updateInt();
}

void
Plic::updateInt()
{
    // Update xEIP lines
    for (int i = 0; i < contextConfigs.size(); i++) {
        auto [thread_id, int_id] = contextConfigs[i];

        auto tc = system->threads[thread_id];
        uint32_t max_id = output.maxID[i];
        uint32_t priority = output.maxPriority[i];
        uint32_t threshold = registers.threshold[i].get();
        if (priority > threshold && max_id > 0 && lastID[i] == 0) {
            DPRINTF(Plic, "Int posted - thread: %d, int id: %d, ", thread_id,
                    int_id);
            DPRINTFR(Plic, "pri: %d, thres: %d\n", priority, threshold);
            tc->getCpuPtr()->postInterrupt(tc->threadId(), int_id, 0);
        } else {
            if (priority > 0) {
                DPRINTF(Plic, "Int filtered - thread: %d, int id: %d, ",
                        thread_id, int_id);
                DPRINTFR(Plic, "pri: %d, thres: %d\n", priority, threshold);
            }
            tc->getCpuPtr()->clearInterrupt(tc->threadId(), int_id, 0);
        }
    }
}

void
Plic::serialize(CheckpointOut &cp) const
{
    int n_outputs = 0;

    for (auto const &reg : registers.pending) {
        paramOut(cp, reg.name(), reg);
    }
    for (auto const &reg : registers.priority) {
        paramOut(cp, reg.name(), reg);
    }
    for (auto const &reg : registers.enable) {
        for (auto const &reg_inner : reg) {
            paramOut(cp, reg_inner.name(), reg_inner);
        }
    }
    for (auto const &reg : registers.threshold) {
        paramOut(cp, reg.name(), reg);
    }
    for (auto const &reg : registers.claim) {
        paramOut(cp, reg.name(), reg);
    }
    for (auto const &it : outputQueue) {
        paramOut(cp, std::string("output_tick") + std::to_string(n_outputs),
                 it.first);
        arrayParamOut(cp, std::string("output_id") + std::to_string(n_outputs),
                      it.second.maxID);
        arrayParamOut(cp,
                      std::string("output_pri") + std::to_string(n_outputs),
                      it.second.maxPriority);
        n_outputs++;
    }
    SERIALIZE_SCALAR(n_outputs);
    SERIALIZE_CONTAINER(output.maxID);
    SERIALIZE_CONTAINER(output.maxPriority);
    SERIALIZE_CONTAINER(pendingPriority);
    for (int i = 0; i < effPriority.size(); i++) {
        arrayParamOut(cp, std::string("effPriority") + std::to_string(i),
                      effPriority[i]);
    }
    SERIALIZE_CONTAINER(lastID);
}

void
Plic::unserialize(CheckpointIn &cp)
{
    int n_outputs;
    UNSERIALIZE_SCALAR(n_outputs);

    for (auto &reg : registers.pending) {
        paramIn(cp, reg.name(), reg);
    }
    for (auto &reg : registers.priority) {
        paramIn(cp, reg.name(), reg);
    }
    for (auto &reg : registers.enable) {
        for (auto &reg_inner : reg) {
            paramIn(cp, reg_inner.name(), reg_inner);
        }
    }
    for (auto &reg : registers.threshold) {
        paramIn(cp, reg.name(), reg);
    }
    for (auto &reg : registers.claim) {
        paramIn(cp, reg.name(), reg);
    }
    for (int i = 0; i < n_outputs; i++) {
        Tick output_tick;
        std::vector<uint32_t> output_id;
        std::vector<uint32_t> output_pri;
        paramIn(cp, std::string("output_tick") + std::to_string(i),
                output_tick);
        arrayParamIn(cp, std::string("output_id") + std::to_string(i),
                     output_id);
        arrayParamIn(cp, std::string("output_pri") + std::to_string(i),
                     output_pri);
        outputQueue[output_tick] = PlicOutput{ output_id, output_pri };
    }
    if (!outputQueue.empty()) {
        schedule(update, outputQueue.begin()->first);
    }
    UNSERIALIZE_CONTAINER(output.maxID);
    UNSERIALIZE_CONTAINER(output.maxPriority);
    UNSERIALIZE_CONTAINER(pendingPriority);
    for (int i = 0; i < effPriority.size(); i++) {
        arrayParamIn(cp, std::string("effPriority") + std::to_string(i),
                     effPriority[i]);
    }
    UNSERIALIZE_CONTAINER(lastID);
    updateInt();
}

} // namespace gem5
