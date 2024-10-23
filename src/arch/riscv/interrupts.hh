/*
 * Copyright (c) 2011 Google
 * Copyright (c) 2024 University of Rostock
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

#ifndef __ARCH_RISCV_INTERRUPT_HH__
#define __ARCH_RISCV_INTERRUPT_HH__

#include <bitset>
#include <memory>

#include "arch/generic/interrupts.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/regs/misc.hh"
#include "base/logging.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"
#include "dev/intpin.hh"
#include "params/RiscvInterrupts.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class BaseCPU;
class ThreadContext;

namespace RiscvISA {

/*
 * This is based on version 1.10 of the RISC-V privileged ISA reference,
 * chapter 3.1.14.
 */
class Interrupts : public BaseInterrupts
{
  private:
    std::bitset<NumInterruptTypes> ip;
    std::bitset<NumInterruptTypes> ie;
    int nmi_cause;

    std::vector<gem5::IntSinkPin<Interrupts>*> localInterruptPins;
  public:
    using Params = RiscvInterruptsParams;

    Interrupts(const Params &p);

    std::bitset<NumInterruptTypes> globalMask() const;

    bool
    checkNonMaskableInterrupt() const
    {
        return tc->readMiscReg(MISCREG_NMIP) & tc->readMiscReg(MISCREG_NMIE);
    }

    bool checkInterrupt(int num) const { return ip[num] && ie[num]; }
    bool checkInterrupts() const override
    {
        return checkNonMaskableInterrupt() || (ip & ie & globalMask()).any();
    }

    Fault getInterrupt() override;

    void updateIntrInfo() override {}

    void post(int int_num, int index) override;

    void clear(int int_num, int index) override;

    void postNMI() { tc->setMiscReg(MISCREG_NMIP, 1); }
    void clearNMI() { tc->setMiscReg(MISCREG_NMIP, 0); }

    void clearAll() override;

    bool isWakeUp() const override
    {
        return checkNonMaskableInterrupt() || (ip & ie).any();
    }

    uint64_t readIP() const { return (uint64_t)ip.to_ulong(); }
    uint64_t readIE() const { return (uint64_t)ie.to_ulong(); }
    void setIP(const uint64_t& val) { ip = val; }
    void setIE(const uint64_t& val) { ie = val; }

    void serialize(CheckpointOut &cp) const override;

    void unserialize(CheckpointIn &cp) override;

    Port &getPort(const std::string &if_name, PortID idx) override;

    void raiseInterruptPin(uint32_t num);
    void lowerInterruptPin(uint32_t num) {};
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_INTERRUPT_HH__
