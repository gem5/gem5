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

#ifndef __DEV_X86_I8042_HH__
#define __DEV_X86_I8042_HH__

#include <deque>

#include "base/bitunion.hh"
#include "dev/intpin.hh"
#include "dev/io_device.hh"
#include "dev/ps2/device.hh"
#include "params/I8042.hh"

namespace gem5
{

namespace X86ISA
{

class I8042 : public PioDevice
{
  protected:
    enum Command
    {
        GetCommandByte = 0x20,
        ReadControllerRamBase = 0x20,
        WriteCommandByte = 0x60,
        WriteControllerRamBase = 0x60,
        CheckForPassword = 0xA4,
        LoadPassword = 0xA5,
        CheckPassword = 0xA6,
        DisableMouse = 0xA7,
        EnableMouse = 0xA8,
        TestMouse = 0xA9,
        SelfTest = 0xAA,
        InterfaceTest = 0xAB,
        DiagnosticDump = 0xAC,
        DisableKeyboard = 0xAD,
        EnableKeyboard = 0xAE,
        ReadInputPort = 0xC0,
        ContinuousPollLow = 0xC1,
        ContinuousPollHigh = 0xC2,
        ReadOutputPort = 0xD0,
        WriteOutputPort = 0xD1,
        WriteKeyboardOutputBuff = 0xD2,
        WriteMouseOutputBuff = 0xD3,
        WriteToMouse = 0xD4,
        DisableA20 = 0xDD,
        EnableA20 = 0xDF,
        ReadTestInputs = 0xE0,
        PulseOutputBitBase = 0xF0,
        SystemReset = 0xFE
    };

    BitUnion8(StatusReg)
        Bitfield<7> parityError;
        Bitfield<6> timeout;
        Bitfield<5> mouseOutputFull;
        Bitfield<4> keyboardUnlocked;
        Bitfield<3> commandLast;
        Bitfield<2> passedSelfTest;
        Bitfield<1> inputFull;
        Bitfield<0> outputFull;
    EndBitUnion(StatusReg)

    BitUnion8(CommandByte)
        Bitfield<6> convertScanCodes;
        Bitfield<5> disableMouse;
        Bitfield<4> disableKeyboard;
        Bitfield<2> passedSelfTest;
        Bitfield<1> mouseFullInt;
        Bitfield<0> keyboardFullInt;
    EndBitUnion(CommandByte)

    Tick latency = 0;
    Addr dataPort = 0;
    Addr commandPort = 0;

    StatusReg statusReg = 0;
    CommandByte commandByte = 0;

    uint8_t dataReg = 0;

    static inline const uint16_t NoCommand = (uint16_t)(-1);
    uint16_t lastCommand = NoCommand;

    std::vector<IntSourcePin<I8042> *> mouseIntPin;
    std::vector<IntSourcePin<I8042> *> keyboardIntPin;

    ps2::Device *mouse = nullptr;
    ps2::Device *keyboard = nullptr;

    void writeData(uint8_t newData, bool mouse=false);
    uint8_t readDataOut();

  public:
    using Params = I8042Params;

    I8042(const Params &p);

    Port &
    getPort(const std::string &if_name, PortID idx=InvalidPortID) override
    {
        if (if_name == "mouse_int_pin")
            return *mouseIntPin.at(idx);
        else if (if_name == "keyboard_int_pin")
            return *keyboardIntPin.at(idx);
        else
            return PioDevice::getPort(if_name, idx);
    }

    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;

    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace X86ISA
} // namespace gem5

#endif //__DEV_X86_I8042_HH__
