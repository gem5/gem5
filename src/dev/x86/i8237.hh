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

#ifndef __DEV_X86_I8237_HH__
#define __DEV_X86_I8237_HH__

#include <array>

#include "dev/io_device.hh"
#include "dev/reg_bank.hh"
#include "params/I8237.hh"

namespace gem5
{

namespace X86ISA
{

class I8237 : public BasicPioDevice
{
  public:
    using Register = RegisterBankLE::Register8;

  protected:
    Tick latency;
    uint8_t maskVal;
    //XXX These should be serialized.
    uint8_t requestVal;
    uint8_t commandVal;
    uint8_t statusVal;
    uint8_t tempVal;
    bool highByte;

    RegisterBankLE regs;

    struct Channel
    {
        class ChannelAddrReg : public Register
        {
          public:
            ChannelAddrReg(Channel &);
        };

        class ChannelRemainingReg : public Register
        {
          public:
            ChannelRemainingReg(Channel &);
        };

        int number;

        //XXX These should be serialized.
        uint8_t mode = 0x0;

        ChannelAddrReg addrReg;
        ChannelRemainingReg remainingReg;

        Channel(int _num) : number(_num), addrReg(*this), remainingReg(*this)
        {}
    };

    class WriteOnlyReg : public Register
    {
      public:
        WriteOnlyReg(const std::string &new_name, Addr offset);
    };

    std::array<Channel, 4> channels;

    Register statusCommandReg;
    WriteOnlyReg requestReg;
    WriteOnlyReg setMaskBitReg;
    WriteOnlyReg modeReg;
    WriteOnlyReg clearFlipFlopReg;
    Register temporaryMasterClearReg;
    WriteOnlyReg clearMaskReg;
    WriteOnlyReg writeMaskReg;

    void reset();
    void setMaskBit(Register &reg, const uint8_t &command);
    void setRequestBit(Register &reg, const uint8_t &command);

  public:
    using Params = I8237Params;

    I8237(const Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace X86ISA
} // namespace gem5

#endif //__DEV_X86_I8237_HH__
