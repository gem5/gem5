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
 *
 * Authors: Gabe Black
 */

#ifndef __DEV_X86_I8042_HH__
#define __DEV_X86_I8042_HH__

#include "dev/io_device.hh"
#include "dev/x86/intdev.hh"
#include "params/I8042.hh"

#include <queue>

namespace X86ISA
{

class IntPin;

class I8042 : public BasicPioDevice
{
  protected:
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

    Tick latency;
    Addr dataPort;
    Addr commandPort;

    StatusReg statusReg;
    CommandByte commandByte;

    uint8_t dataReg;

    static const uint16_t NoCommand = (uint16_t)(-1);
    uint16_t lastCommand;

    BitUnion8(MouseStatus)
        Bitfield<6> remote;
        Bitfield<5> enabled;
        Bitfield<4> twoToOne;
        Bitfield<2> leftButton;
        Bitfield<0> rightButton;
    EndBitUnion(MouseStatus)

    IntSourcePin *mouseIntPin;
    std::queue<uint8_t> mouseBuffer;
    uint16_t lastMouseCommand;
    uint8_t mouseResolution;
    uint8_t mouseSampleRate;
    MouseStatus mouseStatus;


    IntSourcePin *keyboardIntPin;
    std::queue<uint8_t> keyboardBuffer;
    uint16_t lastKeyboardCommand;

    bool writeData(uint8_t newData, bool mouse = false);
    void keyboardAck();
    void writeKeyboardData(const uint8_t *data, int size);
    void mouseAck();
    void mouseNack();
    void writeMouseData(const uint8_t *data, int size);
    uint8_t readDataOut();

  public:
    typedef I8042Params Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    I8042(Params *p) : BasicPioDevice(p), latency(p->pio_latency),
            dataPort(p->data_port), commandPort(p->command_port),
            statusReg(0), commandByte(0), dataReg(0), lastCommand(NoCommand),
            mouseIntPin(p->mouse_int_pin), lastMouseCommand(NoCommand),
            keyboardIntPin(p->keyboard_int_pin),
            lastKeyboardCommand(NoCommand)
    {
        statusReg.passedSelfTest = 1;
        statusReg.commandLast = 1;
        statusReg.keyboardUnlocked = 1;

        commandByte.convertScanCodes = 1;
        commandByte.passedSelfTest = 1;
        commandByte.keyboardFullInt = 1;
    }

    void addressRanges(AddrRangeList &range_list);

    Tick read(PacketPtr pkt);

    Tick write(PacketPtr pkt);
};

}; // namespace X86ISA

#endif //__DEV_X86_I8042_HH__
