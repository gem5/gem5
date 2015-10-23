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

#include <deque>

#include "dev/x86/intdev.hh"
#include "dev/io_device.hh"
#include "params/I8042.hh"

namespace X86ISA
{

class IntPin;

class PS2Device
{
  protected:
    std::deque<uint8_t> outBuffer;

    static const uint16_t NoCommand = (uint16_t)(-1);

    uint16_t lastCommand;
    void bufferData(const uint8_t *data, int size);
    void ack();
    void nack();

  public:
    virtual ~PS2Device()
    {};

    PS2Device() : lastCommand(NoCommand)
    {}

    virtual void serialize(const std::string &base, CheckpointOut &cp) const;
    virtual void unserialize(const std::string &base, CheckpointIn &cp);

    bool hasData()
    {
        return !outBuffer.empty();
    }

    uint8_t getData()
    {
        uint8_t data = outBuffer.front();
        outBuffer.pop_front();
        return data;
    }

    virtual bool processData(uint8_t data) = 0;
};

class PS2Mouse : public PS2Device
{
  protected:
    static const uint8_t ID[];

    enum Command
    {
        Scale1to1 = 0xE6,
        Scale2to1 = 0xE7,
        SetResolution = 0xE8,
        GetStatus = 0xE9,
        ReadData = 0xEB,
        ResetWrapMode = 0xEC,
        WrapMode = 0xEE,
        RemoteMode = 0xF0,
        ReadID = 0xF2,
        SampleRate = 0xF3,
        EnableReporting = 0xF4,
        DisableReporting = 0xF5,
        DefaultsAndDisable = 0xF6,
        Resend = 0xFE,
        Reset = 0xFF
    };

    BitUnion8(Status)
        Bitfield<6> remote;
        Bitfield<5> enabled;
        Bitfield<4> twoToOne;
        Bitfield<2> leftButton;
        Bitfield<0> rightButton;
    EndBitUnion(Status)

    Status status;
    uint8_t resolution;
    uint8_t sampleRate;
  public:
    PS2Mouse() : PS2Device(), status(0), resolution(4), sampleRate(100)
    {}

    bool processData(uint8_t data) override;

    void serialize(const std::string &base, CheckpointOut &cp) const override;
    void unserialize(const std::string &base, CheckpointIn &cp) override;
};

class PS2Keyboard : public PS2Device
{
  protected:
    static const uint8_t ID[];

    enum Command
    {
        LEDWrite = 0xED,
        DiagnosticEcho = 0xEE,
        AlternateScanCodes = 0xF0,
        ReadID = 0xF2,
        TypematicInfo = 0xF3,
        Enable = 0xF4,
        Disable = 0xF5,
        DefaultsAndDisable = 0xF6,
        AllKeysToTypematic = 0xF7,
        AllKeysToMakeRelease = 0xF8,
        AllKeysToMake = 0xF9,
        AllKeysToTypematicMakeRelease = 0xFA,
        KeyToTypematic = 0xFB,
        KeyToMakeRelease = 0xFC,
        KeyToMakeOnly = 0xFD,
        Resend = 0xFE,
        Reset = 0xFF
    };

  public:
    bool processData(uint8_t data) override;
};

class I8042 : public BasicPioDevice
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

    Tick latency;
    Addr dataPort;
    Addr commandPort;

    StatusReg statusReg;
    CommandByte commandByte;

    uint8_t dataReg;

    static const uint16_t NoCommand = (uint16_t)(-1);
    uint16_t lastCommand;

    IntSourcePin *mouseIntPin;
    IntSourcePin *keyboardIntPin;

    PS2Mouse mouse;
    PS2Keyboard keyboard;

    void writeData(uint8_t newData, bool mouse = false);
    uint8_t readDataOut();

  public:
    typedef I8042Params Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    I8042(Params *p);

    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;

    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace X86ISA

#endif //__DEV_X86_I8042_HH__
