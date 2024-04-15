/*
 * Copyright (c) 2012 ARM Limited
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

#include "dev/i2c/bus.hh"

#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "dev/i2c/device.hh"
#include "mem/packet_access.hh"
#include "sim/serialize.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::map;
using std::vector;

namespace gem5
{

/**
 * 4KB - see e.g.
 * http://infocenter.arm.com/help/topic/com.arm.doc.dui0440b/Bbajihec.html
 */
I2CBus::I2CBus(const I2CBusParams &p)
    : BasicPioDevice(p, 0x1000),
      scl(1),
      sda(1),
      state(IDLE),
      currBit(7),
      i2cAddr(0x00),
      message(0x00)
{
    vector<I2CDevice *> devs = p.devices;

    for (auto d : p.devices) {
        devices[d->i2cAddr()] = d;
    }
}

/**
 * Reads will always be to SB_CONTROLS. The kernel wants to know the state
 * of sda and scl.
 */
Tick
I2CBus::read(PacketPtr pkt)
{
    assert(pkt->getAddr() == pioAddr + SB_CONTROLS);

    pkt->setRaw<uint8_t>((sda << 1) | scl);
    pkt->makeAtomicResponse();
    return pioDelay;
}

/**
 * The default i2c bus driver used by the realview pbx board writes to
 * this device one bit at a time. To facilitate making new i2c devices,
 * i2cBus::write takes care of the low-level details of the i2c protocol.
 * See the I2C Specification [1] for a detailed description of the
 * protocol.
 *
 * [1] - http://www.nxp.com/documents/user_manual/UM10204.pdf
 */
Tick
I2CBus::write(PacketPtr pkt)
{
    assert(pkt->getAddr() == pioAddr + SB_CONTROLS ||
           pkt->getAddr() == pioAddr + SB_CONTROLC);

    updateSignals(pkt);

    // Check if the bus master is starting a new transmission.
    if (isStart(pkt)) {
        state = RECEIVING_ADDR;
        message = 0x00;
        currBit = 7;
        /* Most i2c devices expect something special (e.g., command,
         * register address) in the first byte they receive so they
         * must be notified somehow that this is a new transmission.
         */
        for (auto &d : devices) {
            d.second->i2cStart();
        }
        return pioDelay;
    }

    // Check if the bus master is ending a transmission.
    if (isEnd(pkt)) {
        state = IDLE;
        return pioDelay;
    }

    // Only change state when the clock is transitioning from low to high.
    // This may not perfectly mimic physical i2c devices but the important
    // part is to only do the following once per clock cycle.
    if (isClockSet(pkt)) {
        switch (state) {
        case RECEIVING_ADDR:
            if (currBit >= 0) {
                message |= sda << currBit;
                currBit--;
            } else {
                i2cAddr = message >> 1;
                assert(devices.find(i2cAddr) != devices.end());
                if (message & 0x01) {
                    state = SENDING_DATA;
                    message = devices[i2cAddr]->read();
                } else {
                    state = RECEIVING_DATA;
                    message = 0x00;
                }
                currBit = 7;
                sda = 0; /* Ack */
            }
            break;
        case RECEIVING_DATA:
            if (currBit >= 0) {
                message |= sda << currBit;
                currBit--;
            } else {
                devices[i2cAddr]->write(message);
                message = 0x00;
                currBit = 7;
                sda = 0; /* Ack */
            }
            break;
        case SENDING_DATA:
            if (currBit >= 0) {
                sda = (message >> currBit) & 0x01;
                currBit--;
            } else {
                if (!sda) /* Check for ack from the bus master. */
                    message = devices[i2cAddr]->read();
                currBit = 7;
            }
            break;
        case IDLE:
        default:
            panic("Invalid state on posedge of clock in I2CBus::write.\n");
            break;
        }
    }

    return pioDelay;
}

void
I2CBus::updateSignals(PacketPtr pkt)
{
    uint8_t msg = pkt->getRaw<uint8_t>();
    Addr daddr = pkt->getAddr() - pioAddr;

    switch (daddr) {
    case SB_CONTROLS:
        scl = (msg & 1) ? 1 : scl;
        sda = (msg & 2) ? 1 : sda;
        break;
    case SB_CONTROLC:
        scl = (msg & 1) ? 0 : scl;
        sda = (msg & 2) ? 0 : sda;
        break;
    default:
        break;
    }
}

bool
I2CBus::isClockSet(PacketPtr pkt) const
{
    uint8_t msg = pkt->getRaw<uint8_t>();
    Addr daddr = pkt->getAddr() - pioAddr;
    return daddr == SB_CONTROLS && (msg & 1);
}

bool
I2CBus::isStart(PacketPtr pkt) const
{
    uint8_t msg = pkt->getRaw<uint8_t>();
    Addr daddr = pkt->getAddr() - pioAddr;
    return scl && (msg & 2) && daddr == SB_CONTROLC;
}

bool
I2CBus::isEnd(PacketPtr pkt) const
{
    uint8_t msg = pkt->getRaw<uint8_t>();
    Addr daddr = pkt->getAddr() - pioAddr;
    return scl && (msg & 2) && daddr == SB_CONTROLS;
}

void
I2CBus::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing I2C bus.\n");
    SERIALIZE_SCALAR(scl);
    SERIALIZE_SCALAR(sda);
    SERIALIZE_ENUM(state);
    SERIALIZE_SCALAR(currBit);
    SERIALIZE_SCALAR(i2cAddr);
    SERIALIZE_SCALAR(message);
}

void
I2CBus::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing I2C bus.\n");
    UNSERIALIZE_SCALAR(scl);
    UNSERIALIZE_SCALAR(sda);
    UNSERIALIZE_ENUM(state);
    UNSERIALIZE_SCALAR(currBit);
    UNSERIALIZE_SCALAR(i2cAddr);
    UNSERIALIZE_SCALAR(message);
}

} // namespace gem5
