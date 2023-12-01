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


#include "arch/riscv/interrupts.hh"

namespace gem5
{

namespace RiscvISA
{

Interrupts::Interrupts(const Params &p) : BaseInterrupts(p),
                                          ip(0),
                                          ie(0)
{
    for (uint8_t i = 0;
        i < p.port_local_interrupt_pins_connection_count;
        ++i) {
            uint8_t interruptID = p.local_interrupt_ids[i];
            assert(interruptID >= 16);
            std::string pinName =
                csprintf("%s.local_interrupt_pins[%d]", p.name, i);
            IntSinkPin<Interrupts>* pin =
                new IntSinkPin<Interrupts>(pinName,i, this, interruptID);
            localInterruptPins.push_back(pin);
        }
}


std::bitset<NumInterruptTypes>
Interrupts::globalMask() const
{
    INTERRUPT mask = 0;
    STATUS status = tc->readMiscReg(MISCREG_STATUS);
    MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
    INTERRUPT mideleg = 0;
    if (misa.rvs || misa.rvn) {
        mideleg = tc->readMiscReg(MISCREG_MIDELEG);
    }
    INTERRUPT sideleg = 0;
    if (misa.rvs && misa.rvn) {
        sideleg = tc->readMiscReg(MISCREG_SIDELEG);
    }
    PrivilegeMode prv = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);
    switch (prv) {
        case PRV_U:
            // status.uie is always 0 if misa.rvn is disabled
            if (misa.rvs) {
                mask.local = ~sideleg.local;
                if (status.uie)
                    mask.local = mask.local | sideleg.local;
                mask.mei = (!sideleg.mei) | (sideleg.mei & status.uie);
                mask.mti = (!sideleg.mti) | (sideleg.mti & status.uie);
                mask.msi = (!sideleg.msi) | (sideleg.msi & status.uie);
                mask.sei = (!sideleg.sei) | (sideleg.sei & status.uie);
                mask.sti = (!sideleg.sti) | (sideleg.sti & status.uie);
                mask.ssi = (!sideleg.ssi) | (sideleg.ssi & status.uie);
            } else {
                // According to the RISC-V privilege spec v1.10, if the
                // S privilege mode is not implemented and user-trap
                // support, setting mideleg/medeleg bits will delegate the
                // trap to U-mode trap handler
                mask.local = ~mideleg.local;
                if (status.uie)
                    mask.local = mask.local | mideleg.local;
                mask.mei = (!mideleg.mei) | (mideleg.mei & status.uie);
                mask.mti = (!mideleg.mti) | (mideleg.mti & status.uie);
                mask.msi = (!mideleg.msi) | (mideleg.msi & status.uie);
                mask.sei = mask.sti = mask.ssi = 0;
            }
            if (status.uie)
                mask.uei = mask.uti = mask.usi = 1;
            break;
        case PRV_S:
            mask.local = ~mideleg.local;
            mask.mei = (!mideleg.mei) | (mideleg.mei & status.sie);
            mask.mti = (!mideleg.mti) | (mideleg.mti & status.sie);
            mask.msi = (!mideleg.msi) | (mideleg.msi & status.sie);
            if (status.sie) {
                mask.sei = mask.sti = mask.ssi = 1;
                mask.local = mask.local | mideleg.local;
            }
            mask.uei = mask.uti = mask.usi = 0;
            break;
        case PRV_M:

            if (status.mie) {
                    mask.local = gem5::mask(48);
                    mask.mei = mask.mti = mask.msi = 1;
            }
            mask.sei = mask.sti = mask.ssi = 0;
            mask.uei = mask.uti = mask.usi = 0;
            break;
        default:
            panic("Unknown privilege mode %d.", prv);
            break;
    }

    return std::bitset<NumInterruptTypes>(mask);
}

Fault
Interrupts::getInterrupt()
{
    assert(checkInterrupts());
    if (checkNonMaskableInterrupt())
        return std::make_shared<NonMaskableInterruptFault>();
    std::bitset<NumInterruptTypes> mask = globalMask();
    if (((ISA*) tc->getIsaPtr())->rvType() == RV64) {
        const std::vector<int> interrupt_order {
            INT_LOCAL_63, INT_LOCAL_62, INT_LOCAL_61, INT_LOCAL_60,
            INT_LOCAL_59, INT_LOCAL_58, INT_LOCAL_57, INT_LOCAL_56,
            INT_LOCAL_55, INT_LOCAL_54, INT_LOCAL_53, INT_LOCAL_52,
            INT_LOCAL_51, INT_LOCAL_50, INT_LOCAL_49, INT_LOCAL_48,
            INT_LOCAL_47, INT_LOCAL_46, INT_LOCAL_45, INT_LOCAL_44,
            INT_LOCAL_43, INT_LOCAL_42, INT_LOCAL_41, INT_LOCAL_40,
            INT_LOCAL_39, INT_LOCAL_38, INT_LOCAL_37, INT_LOCAL_36,
            INT_LOCAL_35, INT_LOCAL_34, INT_LOCAL_33, INT_LOCAL_32,
            INT_LOCAL_31, INT_LOCAL_30, INT_LOCAL_29, INT_LOCAL_28,
            INT_LOCAL_27, INT_LOCAL_26, INT_LOCAL_25, INT_LOCAL_24,
            INT_LOCAL_23, INT_LOCAL_22, INT_LOCAL_21, INT_LOCAL_20,
            INT_LOCAL_19, INT_LOCAL_18, INT_LOCAL_17, INT_LOCAL_16,
            INT_EXT_MACHINE, INT_SOFTWARE_MACHINE, INT_TIMER_MACHINE,
            INT_EXT_SUPER, INT_SOFTWARE_SUPER, INT_TIMER_SUPER,
            INT_EXT_USER, INT_SOFTWARE_USER, INT_TIMER_USER
        };
        for (const int &id : interrupt_order) {
            if (checkInterrupt(id) && mask[id]) {
                return std::make_shared<InterruptFault>(id);
            }
        }
    } else if (((ISA*) tc->getIsaPtr())->rvType() == RV32) {
        const std::vector<int> interrupt_order {
            INT_LOCAL_31, INT_LOCAL_30, INT_LOCAL_29, INT_LOCAL_28,
            INT_LOCAL_27, INT_LOCAL_26, INT_LOCAL_25, INT_LOCAL_24,
            INT_LOCAL_23, INT_LOCAL_22, INT_LOCAL_21, INT_LOCAL_20,
            INT_LOCAL_19, INT_LOCAL_18, INT_LOCAL_17, INT_LOCAL_16,
            INT_EXT_MACHINE, INT_SOFTWARE_MACHINE, INT_TIMER_MACHINE,
            INT_EXT_SUPER, INT_SOFTWARE_SUPER, INT_TIMER_SUPER,
            INT_EXT_USER, INT_SOFTWARE_USER, INT_TIMER_USER
        };
        for (const int &id : interrupt_order) {
            if (checkInterrupt(id) && mask[id]) {
                return std::make_shared<InterruptFault>(id);
            }
        }
    }
    return NoFault;
}

void
Interrupts::post(int int_num, int index)
{
    DPRINTF(Interrupt, "Interrupt %d:%d posted\n", int_num, index);
    if (int_num != INT_NMI) {
        ip[int_num] = true;
    } else {
        postNMI();
    }
}

void
Interrupts::clear(int int_num, int index)
{
    DPRINTF(Interrupt, "Interrupt %d:%d cleared\n", int_num, index);
    if (int_num != INT_NMI) {
        ip[int_num] = false;
    } else {
        clearNMI();
    }
}

void
Interrupts::clearAll()
{
    DPRINTF(Interrupt, "All interrupts cleared\n");
    ip = 0;
    clearNMI();
}

void
Interrupts::raiseInterruptPin(uint32_t num)
{
    tc->getCpuPtr()->postInterrupt(tc->threadId(), num, 0);
}

void
Interrupts::serialize(CheckpointOut &cp) const
{
    unsigned long ip_ulong = ip.to_ulong();
    unsigned long ie_ulong = ie.to_ulong();
    SERIALIZE_SCALAR(ip_ulong);
    SERIALIZE_SCALAR(ie_ulong);
}

void
Interrupts::unserialize(CheckpointIn &cp)
{
    unsigned long ip_ulong;
    unsigned long ie_ulong;
    UNSERIALIZE_SCALAR(ip_ulong);
    ip = ip_ulong;
    UNSERIALIZE_SCALAR(ie_ulong);
    ie = ie_ulong;
}

Port &
Interrupts::getPort(const std::string &if_name, PortID idx)
{

    if (if_name == "local_interrupt_pins" && idx < localInterruptPins.size()) {
        return *localInterruptPins[idx];
    } else {
        return BaseInterrupts::getPort(if_name, idx);
    }
}

} // namespace RiscvISA

} // namespace gem5
