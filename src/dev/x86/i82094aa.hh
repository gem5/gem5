/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#ifndef __DEV_X86_I82094AA_HH__
#define __DEV_X86_I82094AA_HH__

#include <map>

#include "arch/x86/intmessage.hh"
#include "base/bitunion.hh"
#include "dev/x86/intdev.hh"
#include "dev/intpin.hh"
#include "dev/io_device.hh"
#include "params/I82094AA.hh"

namespace gem5
{

namespace X86ISA
{

class Interrupts;

class I82094AA : public BasicPioDevice
{
  public:
    BitUnion64(RedirTableEntry)
        Bitfield<63, 32> topDW;
        Bitfield<55, 32> topReserved;
        Bitfield<31, 0> bottomDW;
        Bitfield<31, 17> bottomReserved;
        Bitfield<63, 56> dest;
        Bitfield<16> mask;
        Bitfield<15> trigger;
        Bitfield<14> remoteIRR;
        Bitfield<13> polarity;
        Bitfield<12> deliveryStatus;
        Bitfield<11> destMode;
        Bitfield<10, 8> deliveryMode;
        Bitfield<7, 0> vector;
    EndBitUnion(RedirTableEntry)

  protected:
    uint8_t regSel;
    uint8_t initialApicId;
    uint8_t id;
    uint8_t arbId;

    uint64_t lowestPriorityOffset;

    static const uint8_t TableSize = 24;
    // This implementation is based on version 0x11, but 0x14 avoids having
    // to deal with the arbitration and APIC bus guck.
    static const uint8_t APICVersion = 0x14;

    RedirTableEntry redirTable[TableSize];
    bool pinStates[TableSize];

    std::vector<IntSinkPin<I82094AA> *> inputs;

    IntRequestPort<I82094AA> intRequestPort;

    void signalInterrupt(TriggerIntMessage message);

  public:
    using Params = I82094AAParams;

    I82094AA(const Params &p);

    void init() override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void writeReg(uint8_t offset, uint32_t value);
    uint32_t readReg(uint8_t offset);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    bool recvResponse(PacketPtr pkt);

    void requestInterrupt(int line);
    void raiseInterruptPin(int number);
    void lowerInterruptPin(int number);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace X86ISA
} // namespace gem5

#endif //__DEV_X86_SOUTH_BRIDGE_I8254_HH__
