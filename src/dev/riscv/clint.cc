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

#include "dev/riscv/clint.hh"

#include "cpu/base.hh"
#include "debug/Clint.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/Clint.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

Clint::Clint(const Params &params) :
    BasicPioDevice(params, params.pio_size),
    system(params.system),
    nThread(params.num_threads),
    signal(params.name + ".signal", 0, this, INT_RTC),
    reset(params.name + ".reset"),
    resetMtimecmp(params.reset_mtimecmp),
    registers(params.name + ".registers", params.pio_addr, this,
              params.mtimecmp_reset_value)
{
      reset.onChange([this](const bool& new_val){
          if (new_val) {
              doReset();
          }
      });
}

void
Clint::raiseInterruptPin(int id)
{
    // Increment mtime when received RTC signal
    uint64_t& mtime = registers.mtime.get();
    if (id == INT_RTC) {
      mtime++;
    }

    for (int context_id = 0; context_id < nThread; context_id++) {

        auto tc = system->threads[context_id];

        // Update misc reg file
        ISA* isa = dynamic_cast<ISA*>(tc->getIsaPtr());
        if (isa->rvType() == RV32) {
            isa->setMiscRegNoEffect(MISCREG_TIME, bits(mtime, 31, 0));
            isa->setMiscRegNoEffect(MISCREG_TIMEH, bits(mtime, 63, 32));
        } else {
            isa->setMiscRegNoEffect(MISCREG_TIME, mtime);
        }

        // Post timer interrupt
        uint64_t mtimecmp = registers.mtimecmp[context_id].get();
        if (mtime >= mtimecmp) {
            if (mtime == mtimecmp) {
                DPRINTF(Clint,
                    "MTIP posted - thread: %d, mtime: %d, mtimecmp: %d\n",
                    context_id, mtime, mtimecmp);
            }
            tc->getCpuPtr()->postInterrupt(tc->threadId(),
                    ExceptionCode::INT_TIMER_MACHINE, 0);
        } else {
            tc->getCpuPtr()->clearInterrupt(tc->threadId(),
                    ExceptionCode::INT_TIMER_MACHINE, 0);
        }
    }
}

void
Clint::ClintRegisters::init()
{
    using namespace std::placeholders;

    // Calculate reserved space size
    const size_t reserved0_size = mtimecmpStart - clint->nThread * 4;
    reserved.emplace_back("reserved0", reserved0_size);
    const size_t reserved1_size = mtimeStart
        - mtimecmpStart - clint->nThread * 8;
    reserved.emplace_back("reserved1", reserved1_size);

    // Sanity check
    assert((int) clint->pioSize <= maxBankSize);

    // Initialize registers
    for (int i = 0; i < clint->nThread; i++) {
        msip.emplace_back(std::string("msip") + std::to_string(i), 0);
        mtimecmp.emplace_back(
            std::string("mtimecmp") + std::to_string(i), mtimecmpResetValue);
    }

    // Add registers to bank
    for (int i = 0; i < clint->nThread; i++) {
        auto read_cb = std::bind(&Clint::readMSIP, clint, _1, i);
        msip[i].reader(read_cb);
        auto write_cb = std::bind(&Clint::writeMSIP, clint, _1, _2, i);
        msip[i].writer(write_cb);
        addRegister(msip[i]);
    }
    addRegister(reserved[0]);
    for (int i = 0; i < clint->nThread; i++) {
        addRegister(mtimecmp[i]);
    }
    addRegister(reserved[1]);
    mtime.readonly();
    addRegister(mtime);
}

uint32_t
Clint::readMSIP(Register32& reg, const int thread_id)
{
    // To avoid discrepancies if mip is externally set using remote_gdb etc.
    auto tc = system->threads[thread_id];
    RegVal mip = tc->readMiscReg(MISCREG_IP);
    uint32_t msip = bits<uint32_t>(mip, ExceptionCode::INT_SOFTWARE_MACHINE);
    reg.update(msip);
    return reg.get();
};

void
Clint::writeMSIP(Register32& reg, const uint32_t& data, const int thread_id)
{
    reg.update(data);
    assert(data <= 1);
    auto tc = system->threads[thread_id];
    if (data > 0) {
        DPRINTF(Clint, "MSIP posted - thread: %d\n", thread_id);
        tc->getCpuPtr()->postInterrupt(tc->threadId(),
            ExceptionCode::INT_SOFTWARE_MACHINE, 0);
    } else {
        DPRINTF(Clint, "MSIP cleared - thread: %d\n", thread_id);
        tc->getCpuPtr()->clearInterrupt(tc->threadId(),
            ExceptionCode::INT_SOFTWARE_MACHINE, 0);
    }
};

Tick
Clint::read(PacketPtr pkt)
{
    // Check for atomic operation
    bool is_atomic = pkt->isAtomicOp() && pkt->cmd == MemCmd::SwapReq;
    DPRINTF(Clint,
        "Read request - addr: %#x, size: %#x, atomic:%d\n",
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
Clint::write(PacketPtr pkt)
{
    DPRINTF(Clint,
        "Write request - addr: %#x, size: %#x\n",
        pkt->getAddr(), pkt->getSize());

    // Perform register write
    registers.write(pkt->getAddr(), pkt->getPtr<void>(), pkt->getSize());

    pkt->makeResponse();
    return pioDelay;
}

void
Clint::init()
{
    registers.init();
    BasicPioDevice::init();
}

Port &
Clint::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "int_pin")
        return signal;
    else if (if_name == "reset")
        return reset;
    else
        return BasicPioDevice::getPort(if_name, idx);
}

void
Clint::serialize(CheckpointOut &cp) const
{
    for (auto const &reg: registers.msip) {
        paramOut(cp, reg.name(), reg);
    }
    for (auto const &reg: registers.mtimecmp) {
        paramOut(cp, reg.name(), reg);
    }
    paramOut(cp, "mtime", registers.mtime);
}

void
Clint::unserialize(CheckpointIn &cp)
{
    for (auto &reg: registers.msip) {
        paramIn(cp, reg.name(), reg);
    }
    for (auto &reg: registers.mtimecmp) {
        paramIn(cp, reg.name(), reg);
    }
    paramIn(cp, "mtime", registers.mtime);
}

void
Clint::doReset() {
    registers.mtime.reset();
    for (int i = 0; i < nThread; i++) {
        // According to the spec, the mtimecmp is in unknown state
        // Assume we will change the mtimecmp registers to specify value
        // if the mtimecmp registers accept the reset signal.
        if (resetMtimecmp) {
            registers.mtimecmp[i].reset();
        }
        registers.msip[i].reset();
    }
    // We need to update the mtip interrupt bits when reset
    raiseInterruptPin(INT_RESET);
}

} // namespace gem5
