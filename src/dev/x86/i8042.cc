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

#include "base/bitunion.hh"
#include "dev/x86/i8042.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

// The 8042 has a whopping 32 bytes of internal RAM.
const uint8_t RamSize = 32;
const uint8_t NumOutputBits = 14;
const uint8_t KeyboardID[] = {0xab, 0x83};
const uint8_t MouseID[] = {0x00};
const uint8_t CommandAck = 0xfa;
const uint8_t CommandNack = 0xfe;
const uint8_t BatSuccessful = 0xaa;

enum Port64Command
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

enum Port60Command
{
    MouseScale1to1 = 0xE6,
    MouseScale2to1 = 0xE7,
    SetMouseResolution = 0xE8,
    MouseGetStatus = 0xE9,
    MouseReadData = 0xEB,
    MouseResetWrapMode = 0xEC,
    LEDWrite = 0xED,
    DiagnosticEcho = 0xEE,
    MouseWrapMode = 0xEE,
    AlternateScanCodes = 0xF0,
    MouseRemoteMode = 0xF0,
    ReadKeyboardID = 0xF2,
    ReadMouseID = 0xF2,
    TypematicInfo = 0xF3,
    MouseSampleRate = 0xF3,
    KeyboardEnable = 0xF4,
    MouseEnableReporting = 0xF4,
    KeyboardDisable = 0xF5,
    MouseDisableReporting = 0xF5,
    DefaultsAndDisableKeyboard = 0xF6,
    DefaultsAndDisableMouse = 0xF6,
    AllKeysToTypematic = 0xF7,
    AllKeysToMakeRelease = 0xF8,
    AllKeysToMake = 0xF9,
    AllKeysToTypematicMakeRelease = 0xFA,
    KeyToTypematic = 0xFB,
    KeyToMakeRelease = 0xFC,
    KeyToMakeOnly = 0xFD,
    Resend = 0xFE,
    KeyboardReset = 0xFF,
    MouseReset = 0xFF
};

void
X86ISA::I8042::addressRanges(AddrRangeList &range_list)
{
    range_list.clear();
    range_list.push_back(RangeSize(dataPort, 1));
    range_list.push_back(RangeSize(commandPort, 1));
}

bool
X86ISA::I8042::writeData(uint8_t newData, bool mouse)
{
    if (!statusReg.outputFull) {
        DPRINTF(I8042, "Set data %#02x.\n", newData);
        dataReg = newData;
        statusReg.outputFull = 1;
        statusReg.mouseOutputFull = (mouse ? 1 : 0);
        return true;
    } else {
        return false;
    }
}

void
X86ISA::I8042::keyboardAck()
{
    while (!keyboardBuffer.empty())
        keyboardBuffer.pop();
    writeKeyboardData(&CommandAck, sizeof(CommandAck));
}

void
X86ISA::I8042::writeKeyboardData(const uint8_t *data, int size)
{
    assert(data || size == 0);
    while (size) {
        keyboardBuffer.push(*(data++));
        size--;
    }
    if (writeData(keyboardBuffer.front())) {
        keyboardBuffer.pop();
        if (commandByte.keyboardFullInt) {
            DPRINTF(I8042, "Sending keyboard interrupt.\n");
            keyboardIntPin->raise();
            //XXX This is a hack.
            keyboardIntPin->lower();
        }
    }
}

void
X86ISA::I8042::mouseAck()
{
    while (!mouseBuffer.empty())
        mouseBuffer.pop();
    writeMouseData(&CommandAck, sizeof(CommandAck));
}

void
X86ISA::I8042::mouseNack()
{
    while (!mouseBuffer.empty())
        mouseBuffer.pop();
    writeMouseData(&CommandNack, sizeof(CommandAck));
}

void
X86ISA::I8042::writeMouseData(const uint8_t *data, int size)
{
    assert(data || size == 0);
    while (size) {
        mouseBuffer.push(*(data++));
        size--;
    }
    if (writeData(mouseBuffer.front(), true)) {
        mouseBuffer.pop();
        if (commandByte.mouseFullInt) {
            DPRINTF(I8042, "Sending mouse interrupt.\n");
            mouseIntPin->raise();
            //XXX This is a hack
            mouseIntPin->lower();
        }
    }
}

uint8_t
X86ISA::I8042::readDataOut()
{
    uint8_t data = dataReg;
    statusReg.outputFull = 0;
    statusReg.mouseOutputFull = 0;
    if (!keyboardBuffer.empty()) {
        writeKeyboardData(NULL, 0);
    }
    if (!mouseBuffer.empty()) {
        writeMouseData(NULL, 0);
    }
    return data;
}

Tick
X86ISA::I8042::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr addr = pkt->getAddr();
    if (addr == dataPort) {
        uint8_t data = readDataOut();
        //DPRINTF(I8042, "Read from data port got %#02x.\n", data);
        pkt->set<uint8_t>(data);
    } else if (addr == commandPort) {
        //DPRINTF(I8042, "Read status as %#02x.\n", (uint8_t)statusReg);
        pkt->set<uint8_t>((uint8_t)statusReg);
    } else {
        panic("Read from unrecognized port %#x.\n", addr);
    }
    return latency;
}

Tick
X86ISA::I8042::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr addr = pkt->getAddr();
    uint8_t data = pkt->get<uint8_t>();
    if (addr == dataPort) {
        statusReg.commandLast = 0;
        switch (lastCommand) {
          case NoCommand:
            if (lastKeyboardCommand != NoCommand) {
                switch (lastKeyboardCommand) {
                  case LEDWrite:
                    DPRINTF(I8042, "Setting LEDs: "
                            "caps lock %s, num lock %s, scroll lock %s\n",
                            bits(data, 2) ? "on" : "off",
                            bits(data, 1) ? "on" : "off",
                            bits(data, 0) ? "on" : "off");
                    keyboardAck();
                    lastKeyboardCommand = NoCommand;
                    break;
                  case TypematicInfo:
                    DPRINTF(I8042,
                            "Setting typematic info to %#02x.\n", data);
                    keyboardAck();
                    lastKeyboardCommand = NoCommand;
                    break;
                }
                break;
            }
            DPRINTF(I8042, "Got port 0x60 command %#02x.\n", data);
            switch (data) {
              case LEDWrite:
                DPRINTF(I8042, "Got LED write command.\n");
                keyboardAck();
                lastKeyboardCommand = LEDWrite;
                break;
              case DiagnosticEcho:
                panic("Keyboard diagnostic echo unimplemented.\n");
              case AlternateScanCodes:
                panic("Accessing alternate scan codes unimplemented.\n");
              case ReadKeyboardID:
                DPRINTF(I8042, "Got keyboard read ID command.\n");
                keyboardAck();
                writeKeyboardData((uint8_t *)&KeyboardID, sizeof(KeyboardID));
                break;
              case TypematicInfo:
                DPRINTF(I8042, "Setting typematic info.\n");
                keyboardAck();
                lastKeyboardCommand = TypematicInfo;
                break;
              case KeyboardEnable:
                DPRINTF(I8042, "Enabling the keyboard.\n");
                keyboardAck();
                break;
              case KeyboardDisable:
                DPRINTF(I8042, "Disabling the keyboard.\n");
                keyboardAck();
                break;
              case DefaultsAndDisableKeyboard:
                DPRINTF(I8042, "Disabling and resetting the keyboard.\n");
                keyboardAck();
                break;
              case AllKeysToTypematic:
                panic("Setting all keys to typemantic unimplemented.\n");
              case AllKeysToMakeRelease:
                panic("Setting all keys to make/release unimplemented.\n");
              case AllKeysToMake:
                panic("Setting all keys to make unimplemented.\n");
              case AllKeysToTypematicMakeRelease:
                panic("Setting all keys to "
                        "typematic/make/release unimplemented.\n");
              case KeyToTypematic:
                panic("Setting a key to typematic unimplemented.\n");
              case KeyToMakeRelease:
                panic("Setting a key to make/release unimplemented.\n");
              case KeyToMakeOnly:
                panic("Setting key to make only unimplemented.\n");
              case Resend:
                panic("Keyboard resend unimplemented.\n");
              case KeyboardReset:
                panic("Keyboard reset unimplemented.\n");
              default:
                panic("Unknown keyboard command %#02x.\n", data);
            }
            break;
          case WriteToMouse:
            if (lastMouseCommand != NoCommand) {
                switch(lastMouseCommand) {
                  case SetMouseResolution:
                    DPRINTF(I8042, "Mouse resolution set to %d.\n", data);
                    mouseResolution = data;
                    mouseAck();
                    lastMouseCommand = NoCommand;
                    break;
                  case MouseSampleRate:
                    DPRINTF(I8042, "Mouse sample rate %d samples "
                            "per second.\n", data);
                    mouseSampleRate = data;
                    mouseAck();
                    lastMouseCommand = NoCommand;
                    break;
                  default:
                    panic("Not expecting data for a mouse command.\n");
                }
                break;
            }
            switch (data) {
              case MouseScale1to1:
                DPRINTF(I8042, "Setting mouse scale to 1:1.\n");
                mouseStatus.twoToOne = 0;
                mouseAck();
                break;
              case MouseScale2to1:
                DPRINTF(I8042, "Setting mouse scale to 2:1.\n");
                mouseStatus.twoToOne = 1;
                mouseAck();
                break;
              case SetMouseResolution:
                DPRINTF(I8042, "Setting mouse resolution.\n");
                lastMouseCommand = SetMouseResolution;
                mouseAck();
                break;
              case MouseGetStatus:
                DPRINTF(I8042, "Getting mouse status.\n");
                mouseAck();
                writeMouseData((uint8_t *)&(mouseStatus), 1);
                writeMouseData(&mouseResolution, sizeof(mouseResolution));
                writeMouseData(&mouseSampleRate, sizeof(mouseSampleRate));
                break;
              case MouseReadData:
                panic("Reading mouse data unimplemented.\n");
              case MouseResetWrapMode:
                panic("Resetting mouse wrap mode unimplemented.\n");
              case MouseWrapMode:
                panic("Setting mouse wrap mode unimplemented.\n");
              case MouseRemoteMode:
                panic("Setting mouse remote mode unimplemented.\n");
              case ReadMouseID:
                DPRINTF(I8042, "Mouse ID requested.\n");
                mouseAck();
                writeMouseData(MouseID, sizeof(MouseID));
                break;
              case MouseSampleRate:
                DPRINTF(I8042, "Setting mouse sample rate.\n");
                lastMouseCommand = MouseSampleRate;
                mouseAck();
                break;
              case MouseDisableReporting:
                DPRINTF(I8042, "Disabling data reporting.\n");
                mouseStatus.enabled = 0;
                mouseAck();
                break;
              case MouseEnableReporting:
                DPRINTF(I8042, "Enabling data reporting.\n");
                mouseStatus.enabled = 1;
                mouseAck();
                break;
              case DefaultsAndDisableMouse:
                DPRINTF(I8042, "Disabling and resetting mouse.\n");
                mouseSampleRate = 100;
                mouseResolution = 4;
                mouseStatus.twoToOne = 0;
                mouseStatus.enabled = 0;
                mouseAck();
                break;
              case Resend:
                panic("Keyboard resent unimplemented.\n");
              case MouseReset:
                DPRINTF(I8042, "Resetting the mouse.\n");
                mouseSampleRate = 100;
                mouseResolution = 4;
                mouseStatus.twoToOne = 0;
                mouseStatus.enabled = 0;
                mouseAck();
                writeMouseData(&BatSuccessful, sizeof(BatSuccessful));
                writeMouseData(MouseID, sizeof(MouseID));
                break;
              default:
                warn("Unknown mouse command %#02x.\n", data);
                mouseNack();
                break;
            }
            break;
          case WriteCommandByte:
            commandByte = data;
            DPRINTF(I8042, "Got data %#02x for \"Write "
                    "command byte\" command.\n", data);
            statusReg.passedSelfTest = (uint8_t)commandByte.passedSelfTest;
            break;
          case WriteMouseOutputBuff:
            DPRINTF(I8042, "Got data %#02x for \"Write "
                    "mouse output buffer\" command.\n", data);
            writeMouseData(&data, sizeof(data));
            break;
          default:
            panic("Data written for unrecognized "
                    "command %#02x\n", lastCommand);
        }
        lastCommand = NoCommand;
    } else if (addr == commandPort) {
        DPRINTF(I8042, "Got command %#02x.\n", data);
        statusReg.commandLast = 1;
        // These purposefully leave off the first byte of the controller RAM
        // so it can be handled specially.
        if (data > ReadControllerRamBase &&
                data < ReadControllerRamBase + RamSize) {
            panic("Attempted to use i8042 read controller RAM command to "
                    "get byte %d.\n", data - ReadControllerRamBase);
        } else if (data > WriteControllerRamBase &&
                data < WriteControllerRamBase + RamSize) {
            panic("Attempted to use i8042 read controller RAM command to "
                    "get byte %d.\n", data - ReadControllerRamBase);
        } else if (data >= PulseOutputBitBase &&
                data < PulseOutputBitBase + NumOutputBits) {
            panic("Attempted to use i8042 pulse output bit command to "
                    "to pulse bit %d.\n", data - PulseOutputBitBase);
        }
        switch (data) {
          case GetCommandByte:
            DPRINTF(I8042, "Getting command byte.\n");
            writeData(commandByte);
            break;
          case WriteCommandByte:
            DPRINTF(I8042, "Setting command byte.\n");
            lastCommand = WriteCommandByte;
            break;
          case CheckForPassword:
            panic("i8042 \"Check for password\" command not implemented.\n");
          case LoadPassword:
            panic("i8042 \"Load password\" command not implemented.\n");
          case CheckPassword:
            panic("i8042 \"Check password\" command not implemented.\n");
          case DisableMouse:
            DPRINTF(I8042, "Disabling mouse at controller.\n");
            commandByte.disableMouse = 1;
            break;
          case EnableMouse:
            DPRINTF(I8042, "Enabling mouse at controller.\n");
            commandByte.disableMouse = 0;
            break;
          case TestMouse:
            panic("i8042 \"Test mouse\" command not implemented.\n");
          case SelfTest:
            panic("i8042 \"Self test\" command not implemented.\n");
          case InterfaceTest:
            panic("i8042 \"Interface test\" command not implemented.\n");
          case DiagnosticDump:
            panic("i8042 \"Diagnostic dump\" command not implemented.\n");
          case DisableKeyboard:
            DPRINTF(I8042, "Disabling keyboard at controller.\n");
            commandByte.disableKeyboard = 1;
            break;
          case EnableKeyboard:
            DPRINTF(I8042, "Enabling keyboard at controller.\n");
            commandByte.disableKeyboard = 0;
            break;
          case ReadInputPort:
            panic("i8042 \"Read input port\" command not implemented.\n");
          case ContinuousPollLow:
            panic("i8042 \"Continuous poll low\" command not implemented.\n");
          case ContinuousPollHigh:
            panic("i8042 \"Continuous poll high\" command not implemented.\n");
          case ReadOutputPort:
            panic("i8042 \"Read output port\" command not implemented.\n");
          case WriteOutputPort:
            panic("i8042 \"Write output port\" command not implemented.\n");
          case WriteKeyboardOutputBuff:
            panic("i8042 \"Write keyboard output buffer\" "
                    "command not implemented.\n");
          case WriteMouseOutputBuff:
            DPRINTF(I8042, "Got command to write to mouse output buffer.\n");
            lastCommand = WriteMouseOutputBuff;
            break;
          case WriteToMouse:
            DPRINTF(I8042, "Expecting mouse command.\n");
            lastCommand = WriteToMouse;
            break;
          case DisableA20:
            panic("i8042 \"Disable A20\" command not implemented.\n");
          case EnableA20:
            panic("i8042 \"Enable A20\" command not implemented.\n");
          case ReadTestInputs:
            panic("i8042 \"Read test inputs\" command not implemented.\n");
          case SystemReset:
            panic("i8042 \"System reset\" command not implemented.\n");
          default:
            panic("Write to unknown i8042 "
                    "(keyboard controller) command port.\n");
        }
    } else {
        panic("Write to unrecognized port %#x.\n", addr);
    }
    return latency;
}

X86ISA::I8042 *
I8042Params::create()
{
    return new X86ISA::I8042(this);
}
