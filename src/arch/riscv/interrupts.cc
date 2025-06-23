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
                                          ie(0),
                                          hvip(0),
                                          nmi_cause(p.nmi_cause)
{
    for (uint8_t i = 0;
        i < p.port_local_interrupt_pins_connection_count;
        ++i) {
            uint8_t interruptID = p.local_interrupt_ids[i];
            assert(interruptID <= 47);
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
    if (misa.rvs) {
        mideleg = tc->readMiscReg(MISCREG_MIDELEG);
    }
    PrivilegeMode prv = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);
    switch (prv) {
        case PRV_U:
            if (misa.rvh && virtualizationEnabled(tc)) {
                STATUS vsstatus = tc->readMiscReg(MISCREG_VSSTATUS);
                INTERRUPT hideleg = tc->readMiscReg(MISCREG_HIDELEG);

                mask.local = ~hideleg.local | ~mideleg.local;

                mask.vsei = (~hideleg.vsei)|(hideleg.vsei & vsstatus.sie);
                mask.vssi = (~hideleg.vssi)|(hideleg.vssi & vsstatus.sie);
                mask.vsti = (~hideleg.vsti)|(hideleg.vsti & vsstatus.sie);

                mask.mei = (~mideleg.mei | ~hideleg.mei)
                        |  (mideleg.mei & hideleg.mei & vsstatus.sie);

                mask.mti = (~mideleg.mti | ~hideleg.mti)
                        |  (mideleg.mti & hideleg.mti & vsstatus.sie);

                mask.msi = (~mideleg.msi | ~hideleg.msi)
                        |  (mideleg.msi & hideleg.msi & vsstatus.sie);

                mask.sei = (~mideleg.sei | ~hideleg.sei)
                        |  (mideleg.sei & hideleg.sei & vsstatus.sie);

                mask.sti = (~mideleg.sti | ~hideleg.sti)
                        |  (mideleg.sti & hideleg.sti & vsstatus.sie);

                mask.ssi = (~mideleg.ssi | ~hideleg.ssi)
                        |  (mideleg.ssi & hideleg.ssi & vsstatus.sie);
            } else {
                mask.local = gem5::mask(48);
                mask.mei = 1;
                mask.mti = 1;
                mask.msi = 1;
                mask.sei = 1;
                mask.sti = 1;
                mask.ssi = 1;
            }
            break;
        case PRV_S:
            if (misa.rvh && virtualizationEnabled(tc)) {
                STATUS vsstatus = tc->readMiscReg(MISCREG_VSSTATUS);
                INTERRUPT hideleg = tc->readMiscReg(MISCREG_HIDELEG);

                mask.local = ~hideleg.local | ~mideleg.local;
                if (status.sie) {
                    mask.local = mask.local | (hideleg.local & mideleg.local);
                }

                mask.vsei = (~hideleg.vsei)|(hideleg.vsei & vsstatus.sie);
                mask.vssi = (~hideleg.vssi)|(hideleg.vssi & vsstatus.sie);
                mask.vsti = (~hideleg.vsti)|(hideleg.vsti & vsstatus.sie);

                // status.sie is always 0 if misa.rvn is disabled
                mask.mei = (~mideleg.mei | ~hideleg.mei)
                        |  (mideleg.mei & hideleg.mei & vsstatus.sie);

                mask.mti = (~mideleg.mti | ~hideleg.mti)
                        |  (mideleg.mti & hideleg.mti & vsstatus.sie);

                mask.msi = (~mideleg.msi | ~hideleg.msi)
                        |  (mideleg.msi & hideleg.msi & vsstatus.sie);


                mask.sei = (~mideleg.sei | ~hideleg.sei)
                        |  (mideleg.sei & hideleg.sei & vsstatus.sie);

                mask.sti = (~mideleg.sti | ~hideleg.sti)
                        |  (mideleg.sti & hideleg.sti & vsstatus.sie);

                mask.ssi = (~mideleg.ssi | ~hideleg.ssi)
                        |  (mideleg.ssi & hideleg.ssi & vsstatus.sie);
            } else {
                mask.local = ~mideleg.local;
                mask.mei = (~mideleg.mei) | (mideleg.mei & status.sie);
                mask.mti = (~mideleg.mti) | (mideleg.mti & status.sie);
                mask.msi = (~mideleg.msi) | (mideleg.msi & status.sie);
                if (status.sie) {
                    mask.sei = mask.sti = mask.ssi = 1;
                    mask.local = mask.local | mideleg.local;
                }
            }

            break;
        case PRV_M:
            if (status.mie) {
                mask.local = gem5::mask(48);
                mask.mei = mask.mti = mask.msi = 1;
            }
            mask.sei = mask.sti = mask.ssi = 0;
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
        return std::make_shared<NonMaskableInterruptFault>(nmi_cause);
    std::bitset<NumInterruptTypes> mask = globalMask();
    if (((ISA*) tc->getIsaPtr())->rvType() == RV64) {
        const std::vector<int> interrupt_order {
            INT_LOCAL_47, INT_LOCAL_46, INT_LOCAL_45, INT_LOCAL_44,
            INT_LOCAL_43, INT_LOCAL_42, INT_LOCAL_41, INT_LOCAL_40,
            INT_LOCAL_39, INT_LOCAL_38, INT_LOCAL_37, INT_LOCAL_36,
            INT_LOCAL_35, INT_LOCAL_34, INT_LOCAL_33, INT_LOCAL_32,
            INT_LOCAL_31, INT_LOCAL_30, INT_LOCAL_29, INT_LOCAL_28,
            INT_LOCAL_27, INT_LOCAL_26, INT_LOCAL_25, INT_LOCAL_24,
            INT_LOCAL_23, INT_LOCAL_22, INT_LOCAL_21, INT_LOCAL_20,
            INT_LOCAL_19, INT_LOCAL_18, INT_LOCAL_17, INT_LOCAL_16,
            INT_LOCAL_15, INT_LOCAL_14, INT_LOCAL_13, INT_LOCAL_12,
            INT_LOCAL_11, INT_LOCAL_10, INT_LOCAL_9, INT_LOCAL_8,
            INT_LOCAL_7, INT_LOCAL_6, INT_LOCAL_5, INT_LOCAL_4,
            INT_LOCAL_3, INT_LOCAL_2, INT_LOCAL_1, INT_LOCAL_0,
            //Table 5.1 from riscv-interrupts-1.0-RC3.pdf
            //https://github.com/riscv/riscv-aia
            INT_EXT_MACHINE, INT_SOFTWARE_MACHINE, INT_TIMER_MACHINE,
            INT_EXT_SUPER, INT_SOFTWARE_SUPER, INT_TIMER_SUPER,
            INT_EXT_SUPER_GUEST, INT_EXT_VIRTUAL_SUPER,
            INT_SOFTWARE_VIRTUAL_SUPER, INT_TIMER_VIRTUAL_SUPER
        };
        for (const int &id : interrupt_order) {
            if (checkInterrupt(id) && mask[id]) {
                return std::make_shared<InterruptFault>(id);
            }
        }
    } else if (((ISA*) tc->getIsaPtr())->rvType() == RV32) {
        const std::vector<int> interrupt_order {
            INT_LOCAL_15, INT_LOCAL_14, INT_LOCAL_13, INT_LOCAL_12,
            INT_LOCAL_11, INT_LOCAL_10, INT_LOCAL_9, INT_LOCAL_8,
            INT_LOCAL_7, INT_LOCAL_6, INT_LOCAL_5, INT_LOCAL_4,
            INT_LOCAL_3, INT_LOCAL_2, INT_LOCAL_1, INT_LOCAL_0,
            INT_EXT_MACHINE, INT_SOFTWARE_MACHINE, INT_TIMER_MACHINE,
            INT_EXT_SUPER, INT_SOFTWARE_SUPER, INT_TIMER_SUPER
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
    tc->getCpuPtr()->postInterrupt(tc->threadId(), num + 16, 0);
}

void
Interrupts::serialize(CheckpointOut &cp) const
{
    unsigned long ip_ulong = ip.to_ulong();
    unsigned long hvip_ulong = hvip.to_ulong();
    unsigned long ie_ulong = ie.to_ulong();
    SERIALIZE_SCALAR(ip_ulong);
    SERIALIZE_SCALAR(hvip_ulong);
    SERIALIZE_SCALAR(ie_ulong);
}

void
Interrupts::unserialize(CheckpointIn &cp)
{
    unsigned long ip_ulong;
    unsigned long hvip_ulong;
    unsigned long ie_ulong;
    UNSERIALIZE_SCALAR(ip_ulong);
    ip = ip_ulong;
    UNSERIALIZE_SCALAR(hvip_ulong);
    hvip = hvip_ulong;
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
