/*
 * Copyright (c) 2010, 2017-2018 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#include "dev/arm/kmi.hh"

#include "base/trace.hh"
#include "base/vnc/vncinput.hh"
#include "debug/Pl050.hh"
#include "dev/arm/amba_device.hh"
#include "dev/ps2/device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Pl050::Pl050(const Pl050Params *p)
    : AmbaIntDevice(p, 0x1000), control(0), status(0x43), clkdiv(0),
      rawInterrupts(0),
      ps2(p->ps2)
{
    ps2->hostRegDataAvailable([this]() { this->updateRxInt(); });
}

Tick
Pl050::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    uint32_t data = 0;

    switch (daddr) {
      case kmiCr:
        DPRINTF(Pl050, "Read Commmand: %#x\n", (uint32_t)control);
        data = control;
        break;

      case kmiStat:
        status.rxfull = ps2->hostDataAvailable() ? 1 : 0;
        DPRINTF(Pl050, "Read Status: %#x\n", (uint32_t)status);
        data = status;
        break;

      case kmiData:
        data = ps2->hostDataAvailable() ? ps2->hostRead() : 0;
        updateRxInt();
        DPRINTF(Pl050, "Read Data: %#x\n", (uint32_t)data);
        break;

      case kmiClkDiv:
        data = clkdiv;
        break;

      case kmiISR:
        data = getInterrupt();
        DPRINTF(Pl050, "Read Interrupts: %#x\n", getInterrupt());
        break;

      default:
        if (readId(pkt, ambaId, pioAddr)) {
            // Hack for variable size accesses
            data = pkt->getLE<uint32_t>();
            break;
        }

        warn("Tried to read PL050 at offset %#x that doesn't exist\n", daddr);
        break;
    }

    pkt->setUintX(data, LittleEndianByteOrder);
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Pl050::write(PacketPtr pkt)
{

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;
    const uint32_t data = pkt->getUintX(LittleEndianByteOrder);

    panic_if(pkt->getSize() != 1,
             "PL050: Unexpected write size "
             "(offset: %#x, data: %#x, size: %u)\n",
             daddr, data, pkt->getSize());

    switch (daddr) {
      case kmiCr:
        DPRINTF(Pl050, "Write Commmand: %#x\n", data);
        // Use the update interrupts helper to make sure any interrupt
        // mask changes are handled correctly.
        setControl((uint8_t)data);
        break;

      case kmiData:
        DPRINTF(Pl050, "Write Data: %#x\n", data);
        // Clear the TX interrupt before writing new data.
        setTxInt(false);
        ps2->hostWrite((uint8_t)data);
        // Data is written in 0 time, so raise the TX interrupt again.
        setTxInt(true);
        break;

      case kmiClkDiv:
        clkdiv = (uint8_t)data;
        break;

      default:
        warn("PL050: Unhandled write of %#x to offset %#x\n", data, daddr);
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Pl050::setTxInt(bool value)
{
    InterruptReg ints = rawInterrupts;

    ints.tx = value ? 1 : 0;

    setInterrupts(ints);
}

void
Pl050::updateRxInt()
{
    InterruptReg ints = rawInterrupts;

    ints.rx = ps2->hostDataAvailable() ? 1 : 0;

    setInterrupts(ints);
}

void
Pl050::updateIntCtrl(InterruptReg ints, ControlReg ctrl)
{
    const bool old_pending(getInterrupt());
    control = ctrl;
    rawInterrupts = ints;
    const bool new_pending(getInterrupt());

    if (!old_pending && new_pending) {
        DPRINTF(Pl050, "Generate interrupt: rawInt=%#x ctrl=%#x int=%#x\n",
                rawInterrupts, control, getInterrupt());
        gic->sendInt(intNum);
    } else if (old_pending && !new_pending) {
        DPRINTF(Pl050, "Clear interrupt: rawInt=%#x ctrl=%#x int=%#x\n",
                rawInterrupts, control, getInterrupt());
        gic->clearInt(intNum);
    }
}

Pl050::InterruptReg
Pl050::getInterrupt() const
{
    InterruptReg tmp_interrupt(0);

    tmp_interrupt.tx = rawInterrupts.tx & control.txint_enable;
    tmp_interrupt.rx = rawInterrupts.rx & control.rxint_enable;

    return tmp_interrupt;
}

void
Pl050::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "ctrlreg", control);
    paramOut(cp, "stsreg", status);
    SERIALIZE_SCALAR(clkdiv);
    paramOut(cp, "raw_ints", rawInterrupts);
}

void
Pl050::unserialize(CheckpointIn &cp)
{
    paramIn(cp, "ctrlreg", control);
    paramIn(cp, "stsreg", status);
    UNSERIALIZE_SCALAR(clkdiv);
    paramIn(cp, "raw_ints", rawInterrupts);
}

Pl050 *
Pl050Params::create()
{
    return new Pl050(this);
}
