/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Ali Saidi
 *          William Wang
 */

#include "base/vnc/vncinput.hh"
#include "base/trace.hh"
#include "debug/Pl050.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/kmi.hh"
#include "dev/ps2.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Pl050::Pl050(const Params *p)
    : AmbaIntDevice(p, 0xfff), control(0), status(0x43), clkdiv(0),
      interrupts(0), rawInterrupts(0), ackNext(false), shiftDown(false),
      vnc(p->vnc), driverInitialized(false), intEvent(this)
{
    if (vnc) {
        if (!p->is_mouse)
            vnc->setKeyboard(this);
        else
            vnc->setMouse(this);
    }
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
        if (rxQueue.empty())
            status.rxfull = 0;
        else
            status.rxfull = 1;

        DPRINTF(Pl050, "Read Status: %#x\n", (uint32_t)status);
        data = status;
        break;
      case kmiData:
        if (rxQueue.empty()) {
            data = 0;
        } else {
            data = rxQueue.front();
            rxQueue.pop_front();
        }
        DPRINTF(Pl050, "Read Data: %#x\n", (uint32_t)data);
        updateIntStatus();
        break;
      case kmiClkDiv:
        data = clkdiv;
        break;
      case kmiISR:
        data = interrupts;
        DPRINTF(Pl050, "Read Interrupts: %#x\n", (uint32_t)interrupts);
        break;
      default:
        if (readId(pkt, ambaId, pioAddr)) {
            // Hack for variable size accesses
            data = pkt->get<uint32_t>();
            break;
        }

        warn("Tried to read PL050 at offset %#x that doesn't exist\n", daddr);
        break;
    }

    switch(pkt->getSize()) {
      case 1:
        pkt->set<uint8_t>(data);
        break;
      case 2:
        pkt->set<uint16_t>(data);
        break;
      case 4:
        pkt->set<uint32_t>(data);
        break;
      default:
        panic("KMI read size too big?\n");
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Pl050::write(PacketPtr pkt)
{

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    assert(pkt->getSize() == sizeof(uint8_t));


    switch (daddr) {
      case kmiCr:
        DPRINTF(Pl050, "Write Commmand: %#x\n", (uint32_t)pkt->get<uint8_t>());
        control = pkt->get<uint8_t>();
        updateIntStatus();
        break;
      case kmiData:
        DPRINTF(Pl050, "Write Data: %#x\n", (uint32_t)pkt->get<uint8_t>());
        processCommand(pkt->get<uint8_t>());
        updateIntStatus();
        break;
      case kmiClkDiv:
        clkdiv = pkt->get<uint8_t>();
        break;
      default:
        warn("Tried to write PL050 at offset %#x that doesn't exist\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Pl050::processCommand(uint8_t byte)
{
    using namespace Ps2;

    if (ackNext) {
        ackNext--;
        rxQueue.push_back(Ack);
        updateIntStatus();
        return;
    }

    switch (byte) {
      case Ps2Reset:
        rxQueue.push_back(Ack);
        rxQueue.push_back(SelfTestPass);
        break;
      case SetResolution:
      case SetRate:
      case SetStatusLed:
      case SetScaling1_1:
      case SetScaling1_2:
        rxQueue.push_back(Ack);
        ackNext = 1;
        break;
      case ReadId:
        rxQueue.push_back(Ack);
        if (params()->is_mouse)
            rxQueue.push_back(MouseId);
        else
            rxQueue.push_back(KeyboardId);
        break;
      case TpReadId:
        if (!params()->is_mouse)
            break;
        // We're not a trackpoint device, this should make the probe go away
        rxQueue.push_back(Ack);
        rxQueue.push_back(0);
        rxQueue.push_back(0);
        // fall through
      case Disable:
      case Enable:
      case SetDefaults:
        rxQueue.push_back(Ack);
        break;
      case StatusRequest:
        rxQueue.push_back(Ack);
        rxQueue.push_back(0);
        rxQueue.push_back(2); // default resolution
        rxQueue.push_back(100); // default sample rate
        break;
      case TouchKitId:
        ackNext = 2;
        rxQueue.push_back(Ack);
        rxQueue.push_back(TouchKitId);
        rxQueue.push_back(1);
        rxQueue.push_back('A');

        driverInitialized = true;
        break;
      default:
        panic("Unknown byte received: %d\n", byte);
    }

    updateIntStatus();
}


void
Pl050::updateIntStatus()
{
    if (!rxQueue.empty())
        rawInterrupts.rx = 1;
    else
        rawInterrupts.rx = 0;

    interrupts.tx = rawInterrupts.tx & control.txint_enable;
    interrupts.rx = rawInterrupts.rx & control.rxint_enable;

    DPRINTF(Pl050, "rawInterupts=%#x control=%#x interrupts=%#x\n",
            (uint32_t)rawInterrupts, (uint32_t)control, (uint32_t)interrupts);

    if (interrupts && !intEvent.scheduled())
        schedule(intEvent, curTick() + intDelay);
}

void
Pl050::generateInterrupt()
{

    if (interrupts) {
        gic->sendInt(intNum);
        DPRINTF(Pl050, "Generated interrupt\n");
    }
}

void
Pl050::mouseAt(uint16_t x, uint16_t y, uint8_t buttons)
{
    using namespace Ps2;

    // If the driver hasn't initialized the device yet, no need to try and send
    // it anything. Similarly we can get vnc mouse events orders of maginture
    // faster than m5 can process them. Only queue up two sets mouse movements
    // and don't add more until those are processed.
    if (!driverInitialized || rxQueue.size() > 10)
        return;

    // We shouldn't be here unless a vnc server called us in which case
    // we should have a pointer to it
    assert(vnc);

    // Convert screen coordinates to touchpad coordinates
    uint16_t _x = (2047.0/vnc->videoWidth()) * x;
    uint16_t _y = (2047.0/vnc->videoHeight()) * y;

    rxQueue.push_back(buttons);
    rxQueue.push_back(_x >> 7);
    rxQueue.push_back(_x & 0x7f);
    rxQueue.push_back(_y >> 7);
    rxQueue.push_back(_y & 0x7f);

    updateIntStatus();
}


void
Pl050::keyPress(uint32_t key, bool down)
{
    using namespace Ps2;

    std::list<uint8_t> keys;

    // convert the X11 keysym into ps2 codes
    keySymToPs2(key, down, shiftDown, keys);

    // Insert into our queue of charecters
    rxQueue.splice(rxQueue.end(), keys);
    updateIntStatus();
}

void
Pl050::serialize(CheckpointOut &cp) const
{
    uint8_t ctrlreg = control;
    SERIALIZE_SCALAR(ctrlreg);

    uint8_t stsreg = status;
    SERIALIZE_SCALAR(stsreg);
    SERIALIZE_SCALAR(clkdiv);

    uint8_t ints = interrupts;
    SERIALIZE_SCALAR(ints);

    uint8_t raw_ints = rawInterrupts;
    SERIALIZE_SCALAR(raw_ints);

    SERIALIZE_SCALAR(ackNext);
    SERIALIZE_SCALAR(shiftDown);
    SERIALIZE_SCALAR(driverInitialized);

    SERIALIZE_CONTAINER(rxQueue);
}

void
Pl050::unserialize(CheckpointIn &cp)
{
    uint8_t ctrlreg;
    UNSERIALIZE_SCALAR(ctrlreg);
    control = ctrlreg;

    uint8_t stsreg;
    UNSERIALIZE_SCALAR(stsreg);
    status = stsreg;

    UNSERIALIZE_SCALAR(clkdiv);

    uint8_t ints;
    UNSERIALIZE_SCALAR(ints);
    interrupts = ints;

    uint8_t raw_ints;
    UNSERIALIZE_SCALAR(raw_ints);
    rawInterrupts = raw_ints;

    UNSERIALIZE_SCALAR(ackNext);
    UNSERIALIZE_SCALAR(shiftDown);
    UNSERIALIZE_SCALAR(driverInitialized);

    UNSERIALIZE_CONTAINER(rxQueue);
}



Pl050 *
Pl050Params::create()
{
    return new Pl050(this);
}
